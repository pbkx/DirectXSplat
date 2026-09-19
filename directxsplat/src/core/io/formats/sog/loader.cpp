#include "io/formats/sog/loader.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include "filesystem.hpp"
#include <fstream>
#include <new>
#include <optional>
#include <stdexcept>

#include <nlohmann/json.hpp>

#include "directxsplat/bounding.h"
#include "io/common/string_util.h"
#include "io/image/wic_image.h"
#include "io/archive/zip_archive.h"

namespace directxsplat::io {

namespace fs = ghc::filesystem;

namespace {

constexpr uint32_t kMaxSogGaussians = 10u * 1024u * 1024u;
constexpr uint64_t kMaxSogExpandedBytes = 2ull * 1024ull * 1024ull * 1024ull;
constexpr uint64_t kMaxSogAssetBytes = 512ull * 1024ull * 1024ull;
constexpr uint64_t kMaxSogTotalAssetBytes = 1024ull * 1024ull * 1024ull;

StatusOr<uint64_t> FileSizeBytes(const fs::path& path) {
  std::ifstream file(path.string(), std::ios::binary | std::ios::ate);
  if (!file.is_open()) {
    return StatusOr<uint64_t>::Error("failed to open file");
  }
  const std::streamoff end = file.tellg();
  if (end < 0) {
    return StatusOr<uint64_t>::Error("failed to read file size");
  }
  return StatusOr<uint64_t>::Ok(static_cast<uint64_t>(end));
}

StatusOr<std::vector<uint8_t>> ReadFileBytes(const fs::path& path) {
  std::ifstream file(path.string(), std::ios::binary);
  if (!file.is_open()) {
    return StatusOr<std::vector<uint8_t>>::Error("failed to open file");
  }
  file.seekg(0, std::ios::end);
  const std::streamoff end = file.tellg();
  if (end < 0 || static_cast<uint64_t>(end) > kMaxSogAssetBytes) {
    return StatusOr<std::vector<uint8_t>>::Error("sog asset is too large");
  }
  const size_t size = static_cast<size_t>(end);
  file.seekg(0, std::ios::beg);
  std::vector<uint8_t> out;
  try {
    out.resize(size);
  } catch (const std::bad_alloc&) {
    return StatusOr<std::vector<uint8_t>>::Error("sog asset allocation failed");
  } catch (const std::length_error&) {
    return StatusOr<std::vector<uint8_t>>::Error("sog asset is too large");
  }
  file.read(reinterpret_cast<char*>(out.data()), static_cast<std::streamsize>(size));
  if (!file) {
    return StatusOr<std::vector<uint8_t>>::Error("failed to read file");
  }
  return StatusOr<std::vector<uint8_t>>::Ok(std::move(out));
}

float Lerp(float a, float b, float t) { return a * (1.0f - t) + b * t; }

Status ValidateSogGaussianStorage(uint32_t count) {
  constexpr uint64_t stride = sizeof(Gaussian) + sizeof(Vec3);
  if (static_cast<uint64_t>(count) > kMaxSogExpandedBytes / stride) {
    return Status::Error("sog scene is too large");
  }
  return Status::Ok();
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

float InverseSymmetricLog(float v) {
  const float a = std::abs(v);
  const float e = std::exp(a) - 1.0f;
  return (v < 0.0f) ? -e : e;
}

float DecodeLogScaleValue(float raw) {
  if (!std::isfinite(raw)) {
    return 1e-4f;
  }
  return std::max(std::exp(std::clamp(raw, -14.0f, 8.0f)), 1e-4f);
}

Quat DecodeSogQuat(uint8_t px, uint8_t py, uint8_t pz, uint8_t tag) {
  if (tag < 252 || tag > 255) {
    return {};
  }
  const uint32_t mode = tag - 252;
  const float sqrt2 = std::sqrt(2.0f);
  const float a = (static_cast<float>(px) / 255.0f * 2.0f - 1.0f) / sqrt2;
  const float b = (static_cast<float>(py) / 255.0f * 2.0f - 1.0f) / sqrt2;
  const float c = (static_cast<float>(pz) / 255.0f * 2.0f - 1.0f) / sqrt2;

  std::array<float, 4> q{0.0f, 0.0f, 0.0f, 0.0f};
  std::array<std::array<uint32_t, 3>, 4> map{std::array<uint32_t, 3>{1, 2, 3}, std::array<uint32_t, 3>{0, 2, 3},
                                             std::array<uint32_t, 3>{0, 1, 3}, std::array<uint32_t, 3>{0, 1, 2}};
  q[map[mode][0]] = a;
  q[map[mode][1]] = b;
  q[map[mode][2]] = c;
  const float t = 1.0f - (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  q[mode] = std::sqrt(std::max(0.0f, t));
  return Normalize({q[1], q[2], q[3], q[0]});
}

float SigmoidInv(float y) {
  const float e = std::clamp(y, 1e-6f, 1.0f - 1e-6f);
  return std::log(e / (1.0f - e));
}

struct SogSource {
  std::function<StatusOr<std::vector<uint8_t>>(const std::string& name)> read;
  std::function<StatusOr<uint64_t>(const std::string& name)> size;
};

Status ValidateSogAssetPath(const std::string& name) {
  if (name.empty()) {
    return Status::Error("invalid sog asset path");
  }
  const fs::path relative(name);
  if (relative.is_absolute() || relative.has_root_name() || relative.has_root_directory()) {
    return Status::Error("sog asset path escapes scene folder");
  }
  for (const auto& part : relative) {
    if (part == "..") {
      return Status::Error("sog asset path escapes scene folder");
    }
  }
  return Status::Ok();
}

StatusOr<fs::path> ResolveSogAssetPath(const fs::path& base, const std::string& name) {
  Status valid = ValidateSogAssetPath(name);
  if (!valid.ok) {
    return StatusOr<fs::path>::Error(valid.message);
  }
  try {
    fs::path canonicalBase = fs::weakly_canonical(base);
    fs::path resolved = fs::weakly_canonical((base / fs::path(name)).lexically_normal());
    std::string baseString = canonicalBase.lexically_normal().generic_string();
    std::string resolvedString = resolved.lexically_normal().generic_string();
#ifdef _WIN32
    std::transform(baseString.begin(), baseString.end(), baseString.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    std::transform(resolvedString.begin(), resolvedString.end(), resolvedString.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
#endif
    while (!baseString.empty() && (baseString.back() == '/' || baseString.back() == '\\')) {
      baseString.pop_back();
    }
    while (!resolvedString.empty() && (resolvedString.back() == '/' || resolvedString.back() == '\\')) {
      resolvedString.pop_back();
    }
    if (resolvedString != baseString &&
        (resolvedString.size() <= baseString.size() || resolvedString.compare(0, baseString.size(), baseString) != 0 ||
         (resolvedString[baseString.size()] != '/' && resolvedString[baseString.size()] != '\\'))) {
      return StatusOr<fs::path>::Error("sog asset path escapes scene folder");
    }
    return StatusOr<fs::path>::Ok(std::move(resolved));
  } catch (const fs::filesystem_error&) {
    return StatusOr<fs::path>::Error("sog filesystem error");
  } catch (const std::exception&) {
    return StatusOr<fs::path>::Error("sog asset path resolution failed");
  }
}

StatusOr<SogSource> CreateSogSource(const std::string& path, std::string& metaRelativePath) {
  const fs::path p(path);
  const std::string lowerExt = ToLower(p.extension().string());

  if (lowerExt == ".sog") {
    auto archive = std::make_shared<ZipArchive>();
    Status st = archive->Open(path);
    if (!st.ok) {
      return StatusOr<SogSource>::Error(st.message);
    }

    metaRelativePath = "meta.json";
    if (!archive->HasEntry(metaRelativePath)) {
      metaRelativePath.clear();
    }

    if (metaRelativePath.empty()) {
      for (const std::string& candidate : {"./meta.json", "scene/meta.json"}) {
        if (archive->HasEntry(candidate)) {
          metaRelativePath = candidate;
          break;
        }
      }
    }
    if (metaRelativePath.empty()) {
      return StatusOr<SogSource>::Error("meta.json not found in sog archive");
    }

    SogSource source{};
    source.read = [archive](const std::string& name) { return archive->ReadEntry(name); };
    source.size = [archive](const std::string& name) { return archive->EntrySize(name); };
    return StatusOr<SogSource>::Ok(std::move(source));
  }

  fs::path metaPath;
  if (fs::is_directory(p)) {
    metaPath = p / "meta.json";
  } else if (ToLower(p.filename().string()) == "meta.json") {
    metaPath = p;
  } else {
    return StatusOr<SogSource>::Error("sog path must be .sog, meta.json, or folder");
  }

  if (!fs::exists(metaPath)) {
    return StatusOr<SogSource>::Error("meta.json not found");
  }

  const fs::path base = metaPath.parent_path();
  metaRelativePath = "meta.json";
  SogSource source{};
  source.read = [base](const std::string& name) -> StatusOr<std::vector<uint8_t>> {
    const auto resolved = ResolveSogAssetPath(base, name);
    if (!resolved.ok()) {
      return StatusOr<std::vector<uint8_t>>::Error(resolved.status.message);
    }
    return ReadFileBytes(resolved.value);
  };
  source.size = [base](const std::string& name) -> StatusOr<uint64_t> {
    const auto resolved = ResolveSogAssetPath(base, name);
    if (!resolved.ok()) {
      return StatusOr<uint64_t>::Error(resolved.status.message);
    }
    return FileSizeBytes(resolved.value);
  };
  return StatusOr<SogSource>::Ok(std::move(source));
}

std::string JoinZipPath(const std::string& base, const std::string& name) {
  if (base.empty()) {
    return name;
  }
  if (base.back() == '/' || base.back() == '\\') {
    return base + name;
  }
  return base + "/" + name;
}

bool HasImagePixels(const DecodedImage& image, uint32_t count) {
  return static_cast<uint64_t>(image.width) * image.height >= count &&
         static_cast<uint64_t>(image.rgba.size()) >= static_cast<uint64_t>(count) * 4u;
}

}  

StatusOr<GaussianSet> SogLoader::Load(const std::string& path, const std::string& setName) const try {
  std::string metaRelativePath;
  const auto sourceResult = CreateSogSource(path, metaRelativePath);
  if (!sourceResult.ok()) {
    return StatusOr<GaussianSet>::Error(sourceResult.status.message);
  }

  std::string metaBase;
  {
    const fs::path mp(metaRelativePath);
    metaBase = mp.has_parent_path() ? mp.parent_path().generic_string() : "";
  }

  uint64_t loadedAssetBytes = 0;
  auto loadAsset = [&](const std::string& name) -> StatusOr<std::vector<uint8_t>> {
    Status valid = ValidateSogAssetPath(name);
    if (!valid.ok) {
      return StatusOr<std::vector<uint8_t>>::Error(valid.message);
    }
    const std::string finalName = metaBase.empty() ? name : JoinZipPath(metaBase, name);
    valid = ValidateSogAssetPath(finalName);
    if (!valid.ok) {
      return StatusOr<std::vector<uint8_t>>::Error(valid.message);
    }
    const auto size = sourceResult.value.size(finalName);
    if (!size.ok()) {
      return StatusOr<std::vector<uint8_t>>::Error(size.status.message);
    }
    if (size.value > kMaxSogAssetBytes || size.value > kMaxSogTotalAssetBytes - loadedAssetBytes) {
      return StatusOr<std::vector<uint8_t>>::Error("sog assets are too large");
    }
    loadedAssetBytes += size.value;
    auto data = sourceResult.value.read(finalName);
    if (!data.ok()) {
      return data;
    }
    if (static_cast<uint64_t>(data.value.size()) > size.value) {
      return StatusOr<std::vector<uint8_t>>::Error("sog assets are too large");
    }
    return data;
  };

  nlohmann::json meta;
  {
    const auto metaBytesResult = loadAsset("meta.json");
    if (!metaBytesResult.ok()) {
      return StatusOr<GaussianSet>::Error(metaBytesResult.status.message);
    }

    try {
      meta = nlohmann::json::parse(metaBytesResult.value);
    } catch (...) {
      return StatusOr<GaussianSet>::Error("invalid sog meta json");
    }
  }

  if (!meta.is_object()) {
    return StatusOr<GaussianSet>::Error("invalid sog meta json");
  }

  const bool legacyV1 = !meta.contains("version");
  if (!legacyV1 && meta.at("version").get<uint32_t>() != 2u) {
    return StatusOr<GaussianSet>::Error("unsupported sog version");
  }

  uint32_t count = 0;
  if (legacyV1) {
    const auto& shape = meta.at("means").at("shape");
    if (!shape.is_array() || shape.size() < 2) {
      return StatusOr<GaussianSet>::Error("invalid sog v1 means shape");
    }
    count = shape.at(0).get<uint32_t>();
  } else {
    count = meta.at("count").get<uint32_t>();
  }
  if (count == 0) {
    return StatusOr<GaussianSet>::Error("sog scene has zero count");
  }
  if (count > kMaxSogGaussians) {
    return StatusOr<GaussianSet>::Error("sog scene has too many splats");
  }
  Status storageStatus = ValidateSogGaussianStorage(count);
  if (!storageStatus.ok) {
    return StatusOr<GaussianSet>::Error(storageStatus.message);
  }

  GaussianSet set{};
  set.name = setName;

  {
    std::vector<Vec3> positions;
    {
      const auto& meansJson = meta.at("means");
      const auto& meansFiles = meansJson.at("files");
      if (!meansFiles.is_array() || meansFiles.size() < 2) {
        return StatusOr<GaussianSet>::Error("invalid sog means files");
      }
      DecodedImage meansL;
      {
        const auto meansLBytes = loadAsset(meansFiles.at(0).get<std::string>());
        if (!meansLBytes.ok()) {
          return StatusOr<GaussianSet>::Error("failed to read sog means");
        }
        auto decoded = DecodeImageFromMemoryWic(meansLBytes.value);
        if (!decoded.ok()) {
          return StatusOr<GaussianSet>::Error("failed to decode sog means images");
        }
        meansL = std::move(decoded.value);
      }
      DecodedImage meansU;
      {
        const auto meansUBytes = loadAsset(meansFiles.at(1).get<std::string>());
        if (!meansUBytes.ok()) {
          return StatusOr<GaussianSet>::Error("failed to read sog means");
        }
        auto decoded = DecodeImageFromMemoryWic(meansUBytes.value);
        if (!decoded.ok()) {
          return StatusOr<GaussianSet>::Error("failed to decode sog means images");
        }
        meansU = std::move(decoded.value);
      }
      if (meansL.width != meansU.width || meansL.height != meansU.height) {
        return StatusOr<GaussianSet>::Error("sog means image dimensions mismatch");
      }
      if (!HasImagePixels(meansL, count) || !HasImagePixels(meansU, count)) {
        return StatusOr<GaussianSet>::Error("sog means image too small");
      }

      std::array<float, 3> mins{};
      std::array<float, 3> maxs{};
      for (uint32_t i = 0; i < 3; ++i) {
        mins[i] = meansJson.at("mins").at(i).get<float>();
        maxs[i] = meansJson.at("maxs").at(i).get<float>();
      }

      positions.resize(count);
      for (uint32_t i = 0; i < count; ++i) {
        const size_t o = static_cast<size_t>(i) * 4;
        const uint16_t qx = static_cast<uint16_t>(meansL.rgba[o + 0]) |
                            static_cast<uint16_t>(static_cast<uint16_t>(meansU.rgba[o + 0]) << 8);
        const uint16_t qy = static_cast<uint16_t>(meansL.rgba[o + 1]) |
                            static_cast<uint16_t>(static_cast<uint16_t>(meansU.rgba[o + 1]) << 8);
        const uint16_t qz = static_cast<uint16_t>(meansL.rgba[o + 2]) |
                            static_cast<uint16_t>(static_cast<uint16_t>(meansU.rgba[o + 2]) << 8);

        const float nx = Lerp(mins[0], maxs[0], static_cast<float>(qx) / 65535.0f);
        const float ny = Lerp(mins[1], maxs[1], static_cast<float>(qy) / 65535.0f);
        const float nz = Lerp(mins[2], maxs[2], static_cast<float>(qz) / 65535.0f);

        positions[i] = {InverseSymmetricLog(nx), InverseSymmetricLog(ny), InverseSymmetricLog(nz)};
      }
    }

    set.gaussians.resize(count);
    for (uint32_t i = 0; i < count; ++i) {
      set.gaussians[i].position = positions[i];
      set.gaussians[i].splatId = i;
    }
  }

  {
    const auto quatFile = meta.at("quats").at("files").at(0).get<std::string>();
    const auto quatBytes = loadAsset(quatFile);
    if (!quatBytes.ok()) {
      return StatusOr<GaussianSet>::Error("failed to read sog quat image");
    }
    const auto quats = DecodeImageFromMemoryWic(quatBytes.value);
    if (!quats.ok()) {
      return StatusOr<GaussianSet>::Error("failed to decode sog quat image");
    }
    if (!HasImagePixels(quats.value, count)) {
      return StatusOr<GaussianSet>::Error("sog quat image too small");
    }

    for (uint32_t i = 0; i < count; ++i) {
      const size_t o = static_cast<size_t>(i) * 4;
      set.gaussians[i].rotation = DecodeSogQuat(quats.value.rgba[o + 0], quats.value.rgba[o + 1],
                                                quats.value.rgba[o + 2], quats.value.rgba[o + 3]);
    }
  }

  {
    const auto& scalesJson = meta.at("scales");
    const auto scaleFile = scalesJson.at("files").at(0).get<std::string>();
    const auto scaleBytes = loadAsset(scaleFile);
    if (!scaleBytes.ok()) {
      return StatusOr<GaussianSet>::Error("failed to read sog scale image");
    }
    const auto scales = DecodeImageFromMemoryWic(scaleBytes.value);
    if (!scales.ok()) {
      return StatusOr<GaussianSet>::Error("failed to decode sog scale image");
    }
    if (!HasImagePixels(scales.value, count)) {
      return StatusOr<GaussianSet>::Error("sog scale image too small");
    }

    if (legacyV1) {
      std::array<float, 3> mins{};
      std::array<float, 3> maxs{};
      for (uint32_t i = 0; i < 3; ++i) {
        mins[i] = scalesJson.at("mins").at(i).get<float>();
        maxs[i] = scalesJson.at("maxs").at(i).get<float>();
      }
      for (uint32_t i = 0; i < count; ++i) {
        const size_t o = static_cast<size_t>(i) * 4;
        set.gaussians[i].scale = {
            DecodeLogScaleValue(Lerp(mins[0], maxs[0], static_cast<float>(scales.value.rgba[o + 0]) / 255.0f)),
            DecodeLogScaleValue(Lerp(mins[1], maxs[1], static_cast<float>(scales.value.rgba[o + 1]) / 255.0f)),
            DecodeLogScaleValue(Lerp(mins[2], maxs[2], static_cast<float>(scales.value.rgba[o + 2]) / 255.0f)),
        };
      }
    } else {
      std::array<float, 256> scaleCodebook{};
      const auto& codebookJson = scalesJson.at("codebook");
      if (codebookJson.size() < 256) {
        return StatusOr<GaussianSet>::Error("invalid sog scale codebook");
      }
      for (size_t i = 0; i < 256; ++i) {
        scaleCodebook[i] = codebookJson.at(i).get<float>();
      }

      for (uint32_t i = 0; i < count; ++i) {
        const size_t o = static_cast<size_t>(i) * 4;
        set.gaussians[i].scale = {
            DecodeLogScaleValue(scaleCodebook[scales.value.rgba[o + 0]]),
            DecodeLogScaleValue(scaleCodebook[scales.value.rgba[o + 1]]),
            DecodeLogScaleValue(scaleCodebook[scales.value.rgba[o + 2]]),
        };
      }
    }
  }

  {
    const auto& sh0Json = meta.at("sh0");
    const auto sh0File = sh0Json.at("files").at(0).get<std::string>();
    const auto sh0Bytes = loadAsset(sh0File);
    if (!sh0Bytes.ok()) {
      return StatusOr<GaussianSet>::Error("failed to read sog sh0 image");
    }
    const auto sh0 = DecodeImageFromMemoryWic(sh0Bytes.value);
    if (!sh0.ok()) {
      return StatusOr<GaussianSet>::Error("failed to decode sog sh0 image");
    }
    if (!HasImagePixels(sh0.value, count)) {
      return StatusOr<GaussianSet>::Error("sog sh0 image too small");
    }

    if (legacyV1) {
      std::array<float, 4> mins{};
      std::array<float, 4> maxs{};
      for (uint32_t i = 0; i < 4; ++i) {
        mins[i] = sh0Json.at("mins").at(i).get<float>();
        maxs[i] = sh0Json.at("maxs").at(i).get<float>();
      }
      for (uint32_t i = 0; i < count; ++i) {
        const size_t o = static_cast<size_t>(i) * 4;
        set.gaussians[i].sh[0] =
            Lerp(mins[0], maxs[0], static_cast<float>(sh0.value.rgba[o + 0]) / 255.0f);
        set.gaussians[i].sh[16] =
            Lerp(mins[1], maxs[1], static_cast<float>(sh0.value.rgba[o + 1]) / 255.0f);
        set.gaussians[i].sh[32] =
            Lerp(mins[2], maxs[2], static_cast<float>(sh0.value.rgba[o + 2]) / 255.0f);
        set.gaussians[i].opacity =
            Lerp(mins[3], maxs[3], static_cast<float>(sh0.value.rgba[o + 3]) / 255.0f);
      }
    } else {
      std::array<float, 256> sh0Codebook{};
      const auto& sh0CodebookJson = sh0Json.at("codebook");
      if (sh0CodebookJson.size() < 256) {
        return StatusOr<GaussianSet>::Error("invalid sog sh0 codebook");
      }
      for (size_t i = 0; i < 256; ++i) {
        sh0Codebook[i] = sh0CodebookJson.at(i).get<float>();
      }

      for (uint32_t i = 0; i < count; ++i) {
        const size_t o = static_cast<size_t>(i) * 4;
        set.gaussians[i].sh[0] = sh0Codebook[sh0.value.rgba[o + 0]];
        set.gaussians[i].sh[16] = sh0Codebook[sh0.value.rgba[o + 1]];
        set.gaussians[i].sh[32] = sh0Codebook[sh0.value.rgba[o + 2]];
        set.gaussians[i].opacity = SigmoidInv(static_cast<float>(sh0.value.rgba[o + 3]) / 255.0f);
      }
    }
  }

  if (meta.contains("shN")) {
    const auto& shN = meta.at("shN");
    const auto& shNFiles = shN.at("files");
    if (!shNFiles.is_array() || shNFiles.size() < 2) {
      return StatusOr<GaussianSet>::Error("invalid sog shN files");
    }

    std::vector<uint16_t> shLabels;
    shLabels.resize(count);
    {
      const auto labelsBytes = loadAsset(shNFiles.at(1).get<std::string>());
      if (!labelsBytes.ok()) {
        return StatusOr<GaussianSet>::Error("failed to read sog shN assets");
      }
      const auto labels = DecodeImageFromMemoryWic(labelsBytes.value);
      if (!labels.ok()) {
        return StatusOr<GaussianSet>::Error("failed to decode sog shN assets");
      }
      if (!HasImagePixels(labels.value, count)) {
        return StatusOr<GaussianSet>::Error("sog shN label image too small");
      }
      for (uint32_t i = 0; i < count; ++i) {
        const size_t lo = static_cast<size_t>(i) * 4;
        shLabels[i] = static_cast<uint16_t>(static_cast<uint32_t>(labels.value.rgba[lo + 0]) |
                                            (static_cast<uint32_t>(labels.value.rgba[lo + 1]) << 8));
      }
    }

    const auto centroidBytes = loadAsset(shNFiles.at(0).get<std::string>());
    if (!centroidBytes.ok()) {
      return StatusOr<GaussianSet>::Error("failed to read sog shN assets");
    }
    const auto centroids = DecodeImageFromMemoryWic(centroidBytes.value);
    if (!centroids.ok()) {
      return StatusOr<GaussianSet>::Error("failed to decode sog shN assets");
    }

    uint32_t coeffs = 0;
    uint32_t paletteCount = 0;
    std::array<float, 256> shNCodebook{};
    float shNMin = 0.0f;
    float shNMax = 0.0f;
    if (legacyV1) {
      if (centroids.value.width == 192u) coeffs = 3;
      if (centroids.value.width == 512u) coeffs = 8;
      if (centroids.value.width == 960u) coeffs = 15;
      if (coeffs == 0) {
        return StatusOr<GaussianSet>::Error("invalid sog v1 shN palette width");
      }
      paletteCount = (centroids.value.width / coeffs) * centroids.value.height;
      shNMin = shN.at("mins").get<float>();
      shNMax = shN.at("maxs").get<float>();
    } else {
      const uint32_t bands = shN.value("bands", 0u);
      if (bands == 1) coeffs = 3;
      if (bands == 2) coeffs = 8;
      if (bands == 3) coeffs = 15;
      paletteCount = shN.value("count", 0u);
      if (coeffs == 0 || paletteCount == 0) {
        return StatusOr<GaussianSet>::Error("invalid sog shN metadata");
      }
      const auto& shNCodebookJson = shN.at("codebook");
      if (shNCodebookJson.size() < 256) {
        return StatusOr<GaussianSet>::Error("invalid sog shN codebook");
      }
      for (size_t i = 0; i < 256; ++i) {
        shNCodebook[i] = shNCodebookJson.at(i).get<float>();
      }
    }

    for (uint32_t i = 0; i < count; ++i) {
      const uint32_t label = shLabels[i];
      if (label >= paletteCount) {
        continue;
      }

      for (uint32_t c = 0; c < coeffs; ++c) {
        const uint32_t u = (label % 64u) * coeffs + c;
        const uint32_t v = label / 64u;
        if (u >= centroids.value.width || v >= centroids.value.height) {
          continue;
        }
        const size_t co = (static_cast<size_t>(v) * centroids.value.width + u) * 4;
        if (legacyV1) {
          set.gaussians[i].sh[1 + c] =
              Lerp(shNMin, shNMax, static_cast<float>(centroids.value.rgba[co + 0]) / 255.0f);
          set.gaussians[i].sh[16 + 1 + c] =
              Lerp(shNMin, shNMax, static_cast<float>(centroids.value.rgba[co + 1]) / 255.0f);
          set.gaussians[i].sh[32 + 1 + c] =
              Lerp(shNMin, shNMax, static_cast<float>(centroids.value.rgba[co + 2]) / 255.0f);
        } else {
          set.gaussians[i].sh[1 + c] = shNCodebook[centroids.value.rgba[co + 0]];
          set.gaussians[i].sh[16 + 1 + c] = shNCodebook[centroids.value.rgba[co + 1]];
          set.gaussians[i].sh[32 + 1 + c] = shNCodebook[centroids.value.rgba[co + 2]];
        }
      }
    }
  }

  set.bounds = ComputeGaussianBounds(set.gaussians);

  return StatusOr<GaussianSet>::Ok(std::move(set));
} catch (const nlohmann::json::exception&) {
  return StatusOr<GaussianSet>::Error("invalid sog meta json");
} catch (const fs::filesystem_error&) {
  return StatusOr<GaussianSet>::Error("sog filesystem error");
} catch (const std::bad_alloc&) {
  return StatusOr<GaussianSet>::Error("sog scene allocation failed");
} catch (const std::length_error&) {
  return StatusOr<GaussianSet>::Error("sog scene allocation failed");
} catch (const std::exception&) {
  return StatusOr<GaussianSet>::Error("sog scene load failed");
}

}
