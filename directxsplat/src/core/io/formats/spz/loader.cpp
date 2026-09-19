#include "io/formats/spz/loader.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <new>
#include <stdexcept>

#include "miniz.h"
#include "zstd.h"

#include "directxsplat/bounding.h"

namespace directxsplat::io {

namespace {

constexpr uint32_t kSpzMagic = 0x5053474e;
constexpr uint32_t kMaxSpzVersion = 4;
constexpr uint32_t kMinZstdSpzVersion = 4;
constexpr uint32_t kMaxSpzPoints = 10000000;
constexpr size_t kMaxSpzCompressedBytes = 512ull * 1024ull * 1024ull;
constexpr size_t kMaxSpzDecompressedBytes = 1024ull * 1024ull * 1024ull;
constexpr uint64_t kMaxSpzExpandedBytes = 2ull * 1024ull * 1024ull * 1024ull;
constexpr size_t kNgspHeaderBytes = 32;
constexpr size_t kNgspTocEntryBytes = 16;
constexpr float kSpzColorScale = 0.15f;
constexpr float kSqrtHalf = 0.7071067811865476f;

StatusOr<std::vector<uint8_t>> ReadFileBytes(const std::string& path) {
  std::ifstream file(path, std::ios::binary | std::ios::ate);
  if (!file.is_open()) {
    return StatusOr<std::vector<uint8_t>>::Error("failed to open spz file");
  }
  const std::streamoff end = file.tellg();
  if (end <= 0) {
    return StatusOr<std::vector<uint8_t>>::Error("empty spz file");
  }
  if (static_cast<uint64_t>(end) > kMaxSpzCompressedBytes) {
    return StatusOr<std::vector<uint8_t>>::Error("spz file is too large");
  }
  std::vector<uint8_t> data;
  try {
    data.resize(static_cast<size_t>(end));
  } catch (const std::bad_alloc&) {
    return StatusOr<std::vector<uint8_t>>::Error("spz allocation failed");
  } catch (const std::length_error&) {
    return StatusOr<std::vector<uint8_t>>::Error("spz file is too large");
  }
  file.seekg(0, std::ios::beg);
  file.read(reinterpret_cast<char*>(data.data()), static_cast<std::streamsize>(data.size()));
  if (!file) {
    return StatusOr<std::vector<uint8_t>>::Error("failed to read spz file");
  }
  return StatusOr<std::vector<uint8_t>>::Ok(std::move(data));
}

uint16_t ReadLe16(const uint8_t* p) {
  return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}

uint32_t ReadLe32(const uint8_t* p) {
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

uint64_t ReadLe64(const uint8_t* p) {
  return static_cast<uint64_t>(ReadLe32(p)) | (static_cast<uint64_t>(ReadLe32(p + 4)) << 32);
}

bool SkipZeroTerminated(const std::vector<uint8_t>& data, size_t& offset) {
  while (offset < data.size()) {
    if (data[offset++] == 0) {
      return true;
    }
  }
  return false;
}

StatusOr<size_t> FindGzipDeflateOffset(const std::vector<uint8_t>& data) {
  if (data.size() < 18 || data[0] != 0x1f || data[1] != 0x8b || data[2] != 8) {
    return StatusOr<size_t>::Error("invalid spz gzip header");
  }
  const uint8_t flags = data[3];
  if ((flags & 0xe0) != 0) {
    return StatusOr<size_t>::Error("invalid spz gzip flags");
  }
  size_t offset = 10;
  if ((flags & 0x04) != 0) {
    if (offset + 2 > data.size()) {
      return StatusOr<size_t>::Error("truncated spz gzip extra header");
    }
    const uint16_t xlen = ReadLe16(data.data() + offset);
    offset += 2u + xlen;
    if (offset > data.size()) {
      return StatusOr<size_t>::Error("truncated spz gzip extra data");
    }
  }
  if ((flags & 0x08) != 0 && !SkipZeroTerminated(data, offset)) {
    return StatusOr<size_t>::Error("truncated spz gzip file name");
  }
  if ((flags & 0x10) != 0 && !SkipZeroTerminated(data, offset)) {
    return StatusOr<size_t>::Error("truncated spz gzip comment");
  }
  if ((flags & 0x02) != 0) {
    offset += 2;
    if (offset > data.size()) {
      return StatusOr<size_t>::Error("truncated spz gzip crc");
    }
  }
  if (offset + 8 > data.size()) {
    return StatusOr<size_t>::Error("truncated spz gzip payload");
  }
  return StatusOr<size_t>::Ok(offset);
}

StatusOr<std::vector<uint8_t>> DecompressGzip(const std::vector<uint8_t>& data) {
  const auto offsetResult = FindGzipDeflateOffset(data);
  if (!offsetResult.ok()) {
    return StatusOr<std::vector<uint8_t>>::Error(offsetResult.status.message);
  }
  const size_t offset = offsetResult.value;
  const size_t compressedSize = data.size() - offset - 8;

  mz_stream stream{};
  if (mz_inflateInit2(&stream, -MZ_DEFAULT_WINDOW_BITS) != MZ_OK) {
    return StatusOr<std::vector<uint8_t>>::Error("failed to initialize spz inflate");
  }

  std::vector<uint8_t> out;
  const uint32_t trailerSize = ReadLe32(data.data() + data.size() - 4);
  if (trailerSize > 0) {
    if (trailerSize > kMaxSpzDecompressedBytes) {
      mz_inflateEnd(&stream);
      return StatusOr<std::vector<uint8_t>>::Error("spz payload is too large");
    }
    try {
      out.reserve(trailerSize);
    } catch (const std::bad_alloc&) {
      mz_inflateEnd(&stream);
      return StatusOr<std::vector<uint8_t>>::Error("spz allocation failed");
    } catch (const std::length_error&) {
      mz_inflateEnd(&stream);
      return StatusOr<std::vector<uint8_t>>::Error("spz payload is too large");
    }
  }

  stream.next_in = const_cast<unsigned char*>(reinterpret_cast<const unsigned char*>(data.data() + offset));
  stream.avail_in = static_cast<unsigned int>(compressedSize);

  std::array<uint8_t, 32768> buffer{};
  int result = MZ_OK;
  while (result != MZ_STREAM_END) {
    stream.next_out = buffer.data();
    stream.avail_out = static_cast<unsigned int>(buffer.size());
    result = mz_inflate(&stream, MZ_NO_FLUSH);
    if (result != MZ_OK && result != MZ_STREAM_END) {
      mz_inflateEnd(&stream);
      return StatusOr<std::vector<uint8_t>>::Error("failed to decompress spz gzip payload");
    }
    const size_t produced = buffer.size() - stream.avail_out;
    if (produced > kMaxSpzDecompressedBytes - out.size()) {
      mz_inflateEnd(&stream);
      return StatusOr<std::vector<uint8_t>>::Error("spz payload is too large");
    }
    try {
      out.insert(out.end(), buffer.data(), buffer.data() + produced);
    } catch (const std::bad_alloc&) {
      mz_inflateEnd(&stream);
      return StatusOr<std::vector<uint8_t>>::Error("spz allocation failed");
    } catch (const std::length_error&) {
      mz_inflateEnd(&stream);
      return StatusOr<std::vector<uint8_t>>::Error("spz payload is too large");
    }
    if (produced == 0 && result == MZ_OK && stream.avail_in == 0) {
      mz_inflateEnd(&stream);
      return StatusOr<std::vector<uint8_t>>::Error("truncated spz gzip payload");
    }
  }

  mz_inflateEnd(&stream);
  return StatusOr<std::vector<uint8_t>>::Ok(std::move(out));
}

int32_t ShDimForDegree(uint32_t degree) {
  switch (degree) {
    case 0:
      return 0;
    case 1:
      return 3;
    case 2:
      return 8;
    case 3:
      return 15;
    case 4:
      return 24;
    default:
      return -1;
  }
}

float HalfToFloat(uint16_t h) {
  const uint32_t sign = (h & 0x8000u) << 16;
  uint32_t expBits = (h >> 10) & 0x1fu;
  uint32_t mant = h & 0x03ffu;
  uint32_t bits = 0;
  if (expBits == 0) {
    if (mant == 0) {
      bits = sign;
    } else {
      int32_t exp = 1;
      while ((mant & 0x0400u) == 0) {
        mant <<= 1;
        --exp;
      }
      mant &= 0x03ffu;
      bits = sign | (static_cast<uint32_t>(exp + 112) << 23) | (mant << 13);
    }
  } else if (expBits == 31) {
    bits = sign | 0x7f800000u | (mant << 13);
  } else {
    bits = sign | ((expBits + 112u) << 23) | (mant << 13);
  }
  float out = 0.0f;
  std::memcpy(&out, &bits, sizeof(out));
  return out;
}

float SigmoidInv(float y) {
  const float e = std::clamp(y, 1e-6f, 1.0f - 1e-6f);
  return std::log(e / (1.0f - e));
}

float DecodeQuantizedSh(uint8_t v) {
  return (static_cast<float>(v) - 128.0f) / 128.0f;
}

Status ValidateSpzGaussianStorage(uint32_t count) {
  constexpr uint64_t stride = sizeof(Gaussian) + sizeof(Vec3);
  if (static_cast<uint64_t>(count) > kMaxSpzExpandedBytes / stride) {
    return Status::Error("spz scene is too large");
  }
  return Status::Ok();
}

StatusOr<std::vector<uint8_t>> DecompressNgsp(const std::vector<uint8_t>& data) {
  if (data.size() < kNgspHeaderBytes) {
    return StatusOr<std::vector<uint8_t>>::Error("truncated spz v4 header");
  }

  const uint32_t magic = ReadLe32(data.data());
  const uint32_t version = ReadLe32(data.data() + 4);
  const uint32_t count = ReadLe32(data.data() + 8);
  const uint32_t shDegree = data[12];
  const uint8_t numStreams = data[15];
  const uint32_t tocByteOffset = ReadLe32(data.data() + 16);
  if (magic != kSpzMagic) {
    return StatusOr<std::vector<uint8_t>>::Error("invalid spz magic");
  }
  if (version < kMinZstdSpzVersion || version > kMaxSpzVersion) {
    return StatusOr<std::vector<uint8_t>>::Error("unsupported spz version");
  }
  if (count == 0) {
    return StatusOr<std::vector<uint8_t>>::Error("spz scene has zero count");
  }
  if (count > kMaxSpzPoints) {
    return StatusOr<std::vector<uint8_t>>::Error("spz scene has too many splats");
  }
  const Status storageStatus = ValidateSpzGaussianStorage(count);
  if (!storageStatus.ok) {
    return StatusOr<std::vector<uint8_t>>::Error(storageStatus.message);
  }

  const int32_t dim = ShDimForDegree(shDegree);
  if (dim < 0) {
    return StatusOr<std::vector<uint8_t>>::Error("unsupported spz sh degree");
  }
  const size_t n = count;
  const std::array<size_t, 6> streamSizes = {
      n * 9u,
      n,
      n * 3u,
      n * 3u,
      n * 4u,
      n * static_cast<size_t>(dim) * 3u,
  };
  const size_t expectedStreams = streamSizes.back() == 0 ? 5u : 6u;
  if (numStreams != expectedStreams) {
    return StatusOr<std::vector<uint8_t>>::Error("invalid spz v4 stream count");
  }
  if (tocByteOffset < kNgspHeaderBytes || tocByteOffset > data.size()) {
    return StatusOr<std::vector<uint8_t>>::Error("invalid spz v4 toc offset");
  }
  const size_t tocSize = expectedStreams * kNgspTocEntryBytes;
  if (tocSize > data.size() - tocByteOffset) {
    return StatusOr<std::vector<uint8_t>>::Error("truncated spz v4 toc");
  }

  size_t decompressedSize = 16;
  for (size_t i = 0; i < expectedStreams; ++i) {
    if (streamSizes[i] > kMaxSpzDecompressedBytes - decompressedSize) {
      return StatusOr<std::vector<uint8_t>>::Error("spz payload is too large");
    }
    decompressedSize += streamSizes[i];
  }

  std::vector<uint8_t> out;
  try {
    out.resize(decompressedSize);
  } catch (const std::bad_alloc&) {
    return StatusOr<std::vector<uint8_t>>::Error("spz allocation failed");
  } catch (const std::length_error&) {
    return StatusOr<std::vector<uint8_t>>::Error("spz payload is too large");
  }
  std::copy_n(data.data(), 16, out.data());

  size_t compressedOffset = static_cast<size_t>(tocByteOffset) + tocSize;
  size_t decompressedOffset = 16;
  for (size_t i = 0; i < expectedStreams; ++i) {
    const size_t entryOffset = static_cast<size_t>(tocByteOffset) + i * kNgspTocEntryBytes;
    const uint64_t compressedSize64 = ReadLe64(data.data() + entryOffset);
    const uint64_t uncompressedSize64 = ReadLe64(data.data() + entryOffset + 8);
    if (uncompressedSize64 != streamSizes[i]) {
      return StatusOr<std::vector<uint8_t>>::Error("invalid spz v4 stream size");
    }
    if (compressedSize64 > static_cast<uint64_t>(data.size() - compressedOffset)) {
      return StatusOr<std::vector<uint8_t>>::Error("truncated spz v4 stream");
    }
    const size_t compressedSize = static_cast<size_t>(compressedSize64);
    const size_t result = ZSTD_decompress(out.data() + decompressedOffset, streamSizes[i],
                                          data.data() + compressedOffset, compressedSize);
    if (ZSTD_isError(result) || result != streamSizes[i]) {
      return StatusOr<std::vector<uint8_t>>::Error("failed to decompress spz v4 stream");
    }
    compressedOffset += compressedSize;
    decompressedOffset += streamSizes[i];
  }
  if (compressedOffset != data.size()) {
    return StatusOr<std::vector<uint8_t>>::Error("invalid spz v4 payload size");
  }

  return StatusOr<std::vector<uint8_t>>::Ok(std::move(out));
}

Aabb ComputeGaussianBounds(const std::vector<Gaussian>& gaussians) {
  Aabb out{};
  if (gaussians.empty()) {
    return out;
  }
  out.min = gaussians[0].position;
  out.max = gaussians[0].position;
  out.valid = true;
  for (const Gaussian& g : gaussians) {
    out.min = Min(out.min, g.position);
    out.max = Max(out.max, g.position);
  }
  return out;
}

int32_t DecodeSigned24(const uint8_t* p) {
  int32_t value = static_cast<int32_t>(p[0]) | (static_cast<int32_t>(p[1]) << 8) |
                  (static_cast<int32_t>(p[2]) << 16);
  if ((value & 0x800000) != 0) {
    value |= static_cast<int32_t>(0xff000000);
  }
  return value;
}

Vec3 DecodePosition(const uint8_t* p, bool float16, uint32_t fractionalBits) {
  if (float16) {
    return {HalfToFloat(ReadLe16(p + 0)), -HalfToFloat(ReadLe16(p + 2)), -HalfToFloat(ReadLe16(p + 4))};
  }
  const float scale = 1.0f / static_cast<float>(uint32_t{1} << std::min<uint32_t>(fractionalBits, 30u));
  return {
      static_cast<float>(DecodeSigned24(p + 0)) * scale,
      -static_cast<float>(DecodeSigned24(p + 3)) * scale,
      -static_cast<float>(DecodeSigned24(p + 6)) * scale,
  };
}

Quat DecodeQuaternionFirstThree(const uint8_t* r) {
  const float x = (static_cast<float>(r[0]) / 127.5f - 1.0f);
  const float y = -(static_cast<float>(r[1]) / 127.5f - 1.0f);
  const float z = -(static_cast<float>(r[2]) / 127.5f - 1.0f);
  const float w = std::sqrt(std::max(0.0f, 1.0f - (x * x + y * y + z * z)));
  return Normalize({x, y, z, w});
}

Quat DecodeQuaternionSmallestThree(const uint8_t* r) {
  uint32_t packed = ReadLe32(r);
  constexpr uint32_t mask = (1u << 9u) - 1u;
  const int largest = static_cast<int>(packed >> 30);
  std::array<float, 4> q{};
  float sum = 0.0f;
  for (int i = 3; i >= 0; --i) {
    if (i == largest) {
      continue;
    }
    const uint32_t mag = packed & mask;
    const uint32_t neg = (packed >> 9u) & 1u;
    packed >>= 10u;
    float value = kSqrtHalf * static_cast<float>(mag) / static_cast<float>(mask);
    if (neg != 0) {
      value = -value;
    }
    q[i] = value;
    sum += value * value;
  }
  q[largest] = std::sqrt(std::max(0.0f, 1.0f - sum));
  return Normalize({q[0], -q[1], -q[2], q[3]});
}

Status ValidatePayloadLayout(size_t dataSize, uint32_t count, uint32_t version, uint32_t shDegree, size_t& payloadOffset,
                             size_t& positionBytes, size_t& rotationBytes, size_t& shDim) {
  if (count == 0) {
    return Status::Error("spz scene has zero count");
  }
  if (count > kMaxSpzPoints) {
    return Status::Error("spz scene has too many splats");
  }
  Status storageStatus = ValidateSpzGaussianStorage(count);
  if (!storageStatus.ok) {
    return storageStatus;
  }
  if (version < 1 || version > kMaxSpzVersion) {
    return Status::Error("unsupported spz version");
  }
  const int32_t dim = ShDimForDegree(shDegree);
  if (dim < 0) {
    return Status::Error("unsupported spz sh degree");
  }
  positionBytes = version == 1 ? 6u : 9u;
  rotationBytes = version >= 3 ? 4u : 3u;
  shDim = static_cast<size_t>(dim);
  payloadOffset = 16;

  const size_t n = count;
  if (n > std::numeric_limits<size_t>::max() / 128u) {
    return Status::Error("spz scene is too large");
  }
  const size_t required = payloadOffset + n * positionBytes + n + n * 3u + n * 3u + n * rotationBytes +
                          n * shDim * 3u;
  if (required > dataSize) {
    return Status::Error("truncated spz payload");
  }
  return Status::Ok();
}

std::array<float, 24> SpzToRdfShFlip() {
  return {
      -1.0f, -1.0f, 1.0f, -1.0f, 1.0f, 1.0f, -1.0f, 1.0f,
      -1.0f, 1.0f, -1.0f, -1.0f, 1.0f, -1.0f, 1.0f, -1.0f,
      1.0f, -1.0f, 1.0f, 1.0f, -1.0f, 1.0f, -1.0f, -1.0f,
  };
}

}  

StatusOr<GaussianSet> SpzLoader::Load(const std::string& path, const std::string& setName) const try {
  const auto file = ReadFileBytes(path);
  if (!file.ok()) {
    return StatusOr<GaussianSet>::Error(file.status.message);
  }

  const bool isNgsp = file.value.size() >= 4 && ReadLe32(file.value.data()) == kSpzMagic;
  StatusOr<std::vector<uint8_t>> payload;
  if (isNgsp) {
    payload = DecompressNgsp(file.value);
  } else if (file.value.size() >= 2 && file.value[0] == 0x1f && file.value[1] == 0x8b) {
    payload = DecompressGzip(file.value);
  } else {
    return StatusOr<GaussianSet>::Error("unrecognized spz container");
  }
  if (!payload.ok()) {
    return StatusOr<GaussianSet>::Error(payload.status.message);
  }
  if (payload.value.size() < 16) {
    return StatusOr<GaussianSet>::Error("truncated spz header");
  }

  const uint8_t* data = payload.value.data();
  const uint32_t magic = ReadLe32(data + 0);
  const uint32_t version = ReadLe32(data + 4);
  const uint32_t count = ReadLe32(data + 8);
  const uint32_t shDegree = data[12];
  if (magic != kSpzMagic) {
    return StatusOr<GaussianSet>::Error("invalid spz magic");
  }
  if (!isNgsp && version >= kMinZstdSpzVersion) {
    return StatusOr<GaussianSet>::Error("unsupported spz version");
  }

  size_t offset = 0;
  size_t positionBytes = 0;
  size_t rotationBytes = 0;
  size_t shDim = 0;
  const Status layout = ValidatePayloadLayout(payload.value.size(), count, version, shDegree, offset, positionBytes,
                                              rotationBytes, shDim);
  if (!layout.ok) {
    return StatusOr<GaussianSet>::Error(layout.message);
  }

  const uint32_t fractionalBits = data[13];
  const size_t n = count;
  const uint8_t* positions = data + offset;
  offset += n * positionBytes;
  const uint8_t* alphas = data + offset;
  offset += n;
  const uint8_t* colors = data + offset;
  offset += n * 3u;
  const uint8_t* scales = data + offset;
  offset += n * 3u;
  const uint8_t* rotations = data + offset;
  offset += n * rotationBytes;
  const uint8_t* sh = data + offset;

  GaussianSet set{};
  set.name = setName;
  set.gaussians.reserve(n);
  const auto shFlip = SpzToRdfShFlip();

  for (uint32_t i = 0; i < count; ++i) {
    Gaussian g{};
    g.splatId = i;
    g.position = DecodePosition(positions + static_cast<size_t>(i) * positionBytes, version == 1, fractionalBits);
    g.rotation = version >= 3
                     ? DecodeQuaternionSmallestThree(rotations + static_cast<size_t>(i) * rotationBytes)
                     : DecodeQuaternionFirstThree(rotations + static_cast<size_t>(i) * rotationBytes);
    g.scale = {
        std::max(std::exp(std::clamp(static_cast<float>(scales[static_cast<size_t>(i) * 3u + 0]) / 16.0f - 10.0f, -14.0f, 8.0f)), 1e-6f),
        std::max(std::exp(std::clamp(static_cast<float>(scales[static_cast<size_t>(i) * 3u + 1]) / 16.0f - 10.0f, -14.0f, 8.0f)), 1e-6f),
        std::max(std::exp(std::clamp(static_cast<float>(scales[static_cast<size_t>(i) * 3u + 2]) / 16.0f - 10.0f, -14.0f, 8.0f)), 1e-6f),
    };
    g.opacity = SigmoidInv(static_cast<float>(alphas[i]) / 255.0f);
    g.sh[0] = ((static_cast<float>(colors[static_cast<size_t>(i) * 3u + 0]) / 255.0f) - 0.5f) / kSpzColorScale;
    g.sh[16] = ((static_cast<float>(colors[static_cast<size_t>(i) * 3u + 1]) / 255.0f) - 0.5f) / kSpzColorScale;
    g.sh[32] = ((static_cast<float>(colors[static_cast<size_t>(i) * 3u + 2]) / 255.0f) - 0.5f) / kSpzColorScale;

    const size_t shBase = static_cast<size_t>(i) * shDim * 3u;
    const size_t copied = std::min<size_t>(shDim, 15u);
    for (size_t c = 0; c < copied; ++c) {
      const float flip = shFlip[c];
      g.sh[1u + c] = flip * DecodeQuantizedSh(sh[shBase + c * 3u + 0]);
      g.sh[16u + 1u + c] = flip * DecodeQuantizedSh(sh[shBase + c * 3u + 1]);
      g.sh[32u + 1u + c] = flip * DecodeQuantizedSh(sh[shBase + c * 3u + 2]);
    }

    if (!std::isfinite(g.position.x) || !std::isfinite(g.position.y) || !std::isfinite(g.position.z)) {
      continue;
    }
    if (!std::isfinite(g.scale.x) || !std::isfinite(g.scale.y) || !std::isfinite(g.scale.z)) {
      continue;
    }
    set.gaussians.push_back(g);
  }

  if (set.gaussians.empty()) {
    return StatusOr<GaussianSet>::Error("no valid gaussians found");
  }

  set.bounds = ComputeGaussianBounds(set.gaussians);
  return StatusOr<GaussianSet>::Ok(std::move(set));
} catch (const std::bad_alloc&) {
  return StatusOr<GaussianSet>::Error("spz scene allocation failed");
} catch (const std::length_error&) {
  return StatusOr<GaussianSet>::Error("spz scene allocation failed");
}

}

