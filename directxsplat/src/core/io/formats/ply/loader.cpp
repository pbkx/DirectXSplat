#include "io/formats/ply/loader.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <new>
#include <stdexcept>
#include <unordered_map>

#include "io/formats/ply/raw/ply_reader.h"
#include "directxsplat/bounding.h"
#include "io/common/string_util.h"

namespace directxsplat::io {

namespace {

constexpr uint32_t kMaxFastPlyElementRows = 64u * 1024u * 1024u;
constexpr uint32_t kMaxFastPlyChunkRows = 4u * 1024u * 1024u;
constexpr uint64_t kMaxFastPlyExpandedBytes = 4ull * 1024ull * 1024ull * 1024ull;
constexpr size_t kMaxFastPlyHeaderBytes = 4ull * 1024ull * 1024ull;
constexpr size_t kMaxFastPlyHeaderLineBytes = 1024ull * 1024ull;
constexpr size_t kMaxFastPlyHeaderTokens = 16;
constexpr size_t kMaxFastPlyHeaderElements = 4096;
constexpr size_t kMaxFastPlyHeaderProperties = 4096;
constexpr size_t kMaxFastPlyHeaderComments = 4096;

struct FastChunkInfo {
  Vec3 minPosition{};
  Vec3 maxPosition{};
  Vec3 minScale{};
  Vec3 maxScale{};
  Vec3 minColor{};
  Vec3 maxColor{};
  bool hasColor = false;
};

float DecodeLogScaleValue(float raw) {
  if (!std::isfinite(raw)) {
    return 1e-4f;
  }
  return std::max(std::exp(std::clamp(raw, -14.0f, 8.0f)), 1e-4f);
}

float DecodeScaleValue(float raw) {
  if (!std::isfinite(raw)) {
    return 1e-4f;
  }
  if (raw <= 0.0f) {
    return DecodeLogScaleValue(raw);
  }
  return std::max(raw, 1e-4f);
}

int FindProperty(const ply::PlyElement& element, std::string_view name) {
  for (size_t i = 0; i < element.properties.size(); ++i) {
    if (!element.properties[i].isList && element.properties[i].name == name) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

double ReadProp(const ply::PlyElement& element, const std::unordered_map<std::string, int>& indices, uint32_t row,
                const std::string& name, double fallback = 0.0) {
  const auto it = indices.find(name);
  if (it == indices.end()) {
    return fallback;
  }
  return element.scalarColumns[it->second][row];
}

bool HasCompressedSchema(const ply::PlyFile& file) {
  const ply::PlyElement* chunk = ply::FindElement(file, "chunk");
  const ply::PlyElement* vertex = ply::FindElement(file, "vertex");
  if (chunk == nullptr || vertex == nullptr) {
    return false;
  }

  constexpr std::array<const char*, 12> requiredChunk = {
      "min_x", "min_y", "min_z", "max_x", "max_y", "max_z",
      "min_scale_x", "min_scale_y", "min_scale_z", "max_scale_x", "max_scale_y", "max_scale_z",
  };
  for (const char* name : requiredChunk) {
    if (FindProperty(*chunk, name) < 0) {
      return false;
    }
  }

  constexpr std::array<const char*, 4> packedProps = {
      "packed_position", "packed_rotation", "packed_scale", "packed_color",
  };
  for (const char* name : packedProps) {
    if (FindProperty(*vertex, name) < 0) {
      return false;
    }
  }

  return chunk->count > 0 && vertex->count > 0;
}

float Saturate(float v) {
  return std::max(0.0f, std::min(1.0f, v));
}

float Lerp(float a, float b, float t) {
  return a * (1.0f - t) + b * t;
}

float ColorToShDc(float c) {
  return (c - 0.5f) / 0.28209479177387814f;
}

float NormalizeColorValue(double v) {
  if (v > 1.0) {
    return Saturate(static_cast<float>(v / 255.0));
  }
  return Saturate(static_cast<float>(v));
}

float ReadColor(const ply::PlyElement& element, const std::unordered_map<std::string, int>& indices, uint32_t row,
                const std::string& a, const std::string& b, double fallback) {
  const auto ia = indices.find(a);
  if (ia != indices.end()) {
    return NormalizeColorValue(element.scalarColumns[ia->second][row]);
  }
  const auto ib = indices.find(b);
  if (ib != indices.end()) {
    return NormalizeColorValue(element.scalarColumns[ib->second][row]);
  }
  return NormalizeColorValue(fallback);
}

float UnpackUnorm(uint32_t value, uint32_t bits) {
  const uint32_t mask = (1u << bits) - 1u;
  return static_cast<float>(value & mask) / static_cast<float>(mask);
}

Vec3 Unpack111011(uint32_t value) {
  return {
      UnpackUnorm(value >> 21, 11),
      UnpackUnorm(value >> 11, 10),
      UnpackUnorm(value, 11),
  };
}

Vec4 Unpack8888(uint32_t value) {
  return {
      UnpackUnorm(value >> 24, 8),
      UnpackUnorm(value >> 16, 8),
      UnpackUnorm(value >> 8, 8),
      UnpackUnorm(value, 8),
  };
}

Quat UnpackRotation(uint32_t value) {
  const float norm = 1.0f / (std::sqrt(2.0f) * 0.5f);
  const float a = (UnpackUnorm(value >> 20, 10) - 0.5f) * norm;
  const float b = (UnpackUnorm(value >> 10, 10) - 0.5f) * norm;
  const float c = (UnpackUnorm(value, 10) - 0.5f) * norm;
  const float m = std::sqrt(std::max(0.0f, 1.0f - (a * a + b * b + c * c)));
  const uint32_t which = value >> 30;
  std::array<float, 4> rotation{};
  switch (which) {
    case 0:
      rotation = {m, a, b, c};
      break;
    case 1:
      rotation = {a, m, b, c};
      break;
    case 2:
      rotation = {a, b, m, c};
      break;
    default:
      rotation = {a, b, c, m};
      break;
  }

  // Compressed PLY stores rot_0..rot_3 as wxyz; renderer quaternions use xyzw.
  return {rotation[1], rotation[2], rotation[3], rotation[0]};
}

bool UseBlockShLayout(const std::vector<std::string>& comments) {
  for (const auto& c : comments) {
    const std::string lc = ToLower(c);
    if (lc.find("interleaved_sh") != std::string::npos || lc.find("interleaved sh") != std::string::npos) {
      return false;
    }
  }
  return true;
}

void WriteRestSh(std::array<float, kShOrder3CoeffCountTotal>& sh, uint32_t index, uint32_t restCount, float value,
                 bool blockLayout) {
  if (restCount == 0) {
    return;
  }
  const uint32_t coeffPerChannel = restCount / 3;
  if (coeffPerChannel == 0) {
    return;
  }

  uint32_t channel = 0;
  uint32_t coeff = 0;
  if (blockLayout) {
    channel = index / coeffPerChannel;
    coeff = index % coeffPerChannel;
  } else {
    channel = index % 3;
    coeff = index / 3;
  }

  if (channel > 2 || coeff >= 15) {
    return;
  }
  sh[channel * 16 + 1 + coeff] = value;
}

GaussianSet ParseStandardGaussianSet(const ply::PlyFile& file, const std::string& setName,
                                     std::vector<std::string>& warnings) {
  GaussianSet set{};
  set.name = setName;

  const ply::PlyElement* vertex = ply::FindElement(file, "vertex");
  if (vertex == nullptr) {
    warnings.push_back("missing vertex element");
    return set;
  }

  std::unordered_map<std::string, int> propIndices;
  for (size_t i = 0; i < vertex->properties.size(); ++i) {
    if (!vertex->properties[i].isList) {
      propIndices[vertex->properties[i].name] = static_cast<int>(i);
    }
  }

  const int xIndex = FindProperty(*vertex, "x");
  const int yIndex = FindProperty(*vertex, "y");
  const int zIndex = FindProperty(*vertex, "z");
  if (xIndex < 0 || yIndex < 0 || zIndex < 0) {
    warnings.push_back("missing position attributes");
    return set;
  }

  const bool hasScale012 = FindProperty(*vertex, "scale_0") >= 0 && FindProperty(*vertex, "scale_1") >= 0 &&
                           FindProperty(*vertex, "scale_2") >= 0;
  const bool hasScaleXyz = FindProperty(*vertex, "scale_x") >= 0 && FindProperty(*vertex, "scale_y") >= 0 &&
                           FindProperty(*vertex, "scale_z") >= 0;
  const bool hasAnyScale = hasScale012 || hasScaleXyz;
  const bool hasDc = FindProperty(*vertex, "f_dc_0") >= 0 && FindProperty(*vertex, "f_dc_1") >= 0 &&
                     FindProperty(*vertex, "f_dc_2") >= 0;
  const bool hasRgb = (FindProperty(*vertex, "red") >= 0 || FindProperty(*vertex, "r") >= 0) &&
                      (FindProperty(*vertex, "green") >= 0 || FindProperty(*vertex, "g") >= 0) &&
                      (FindProperty(*vertex, "blue") >= 0 || FindProperty(*vertex, "b") >= 0);
  const bool pointCloudFallback = !hasAnyScale && !hasDc && hasRgb;

  std::vector<Vec3> sourcePoints;
  sourcePoints.reserve(vertex->count);
  for (uint32_t i = 0; i < vertex->count; ++i) {
    Vec3 p{
        static_cast<float>(vertex->scalarColumns[xIndex][i]),
        static_cast<float>(vertex->scalarColumns[yIndex][i]),
        static_cast<float>(vertex->scalarColumns[zIndex][i]),
    };
    if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
      sourcePoints.push_back(p);
    }
  }
  const Aabb sourceBounds = ComputeAabb(sourcePoints);
  const float sourceRadius = sourceBounds.valid ? std::max(ComputeAabbRadius(sourceBounds), 1e-3f) : 1.0f;
  const float defaultPointScale =
      std::clamp(sourceRadius / std::max(32.0f, std::sqrt(static_cast<float>(std::max<size_t>(sourcePoints.size(), 1)))) * 0.08f,
                 sourceRadius * 0.00008f, sourceRadius * 0.0015f);

  std::vector<int> restIndices;
  for (int i = 0;; ++i) {
    const int idx = FindProperty(*vertex, "f_rest_" + std::to_string(i));
    if (idx < 0) {
      break;
    }
    restIndices.push_back(idx);
  }
  const uint32_t restCount = static_cast<uint32_t>(restIndices.size());
  const bool blockLayout = UseBlockShLayout(file.comments);

  set.gaussians.reserve(vertex->count);
  std::vector<Vec3> points;
  points.reserve(vertex->count);

  for (uint32_t i = 0; i < vertex->count; ++i) {
    Gaussian g{};
    g.splatId = i;
    g.position = {
        static_cast<float>(vertex->scalarColumns[xIndex][i]),
        static_cast<float>(vertex->scalarColumns[yIndex][i]),
        static_cast<float>(vertex->scalarColumns[zIndex][i]),
    };

    const float sx = static_cast<float>(ReadProp(*vertex, propIndices, i, "scale_0", ReadProp(*vertex, propIndices, i, "scale_x", 0.0)));
    const float sy = static_cast<float>(ReadProp(*vertex, propIndices, i, "scale_1", ReadProp(*vertex, propIndices, i, "scale_y", 0.0)));
    const float sz = static_cast<float>(ReadProp(*vertex, propIndices, i, "scale_2", ReadProp(*vertex, propIndices, i, "scale_z", 0.0)));
    if (hasScale012) {
      g.scale = {
          DecodeLogScaleValue(sx),
          DecodeLogScaleValue(sy),
          DecodeLogScaleValue(sz),
      };
    } else if (!hasAnyScale) {
      g.scale = {defaultPointScale, defaultPointScale, defaultPointScale};
    } else {
      g.scale = {
          DecodeScaleValue(sx),
          DecodeScaleValue(sy),
          DecodeScaleValue(sz),
      };
    }

    g.rotation = Normalize({
        static_cast<float>(ReadProp(*vertex, propIndices, i, "rot_1", 0.0)),
        static_cast<float>(ReadProp(*vertex, propIndices, i, "rot_2", 0.0)),
        static_cast<float>(ReadProp(*vertex, propIndices, i, "rot_3", 0.0)),
        static_cast<float>(ReadProp(*vertex, propIndices, i, "rot_0", 1.0)),
    });

    g.opacity = static_cast<float>(ReadProp(*vertex, propIndices, i, "opacity", pointCloudFallback ? 2.0 : 1.0));

    if (hasDc) {
      g.sh[0] = static_cast<float>(ReadProp(*vertex, propIndices, i, "f_dc_0", 0.0));
      g.sh[16] = static_cast<float>(ReadProp(*vertex, propIndices, i, "f_dc_1", 0.0));
      g.sh[32] = static_cast<float>(ReadProp(*vertex, propIndices, i, "f_dc_2", 0.0));
    } else if (hasRgb) {
      g.sh[0] = ColorToShDc(ReadColor(*vertex, propIndices, i, "red", "r", 127.5));
      g.sh[16] = ColorToShDc(ReadColor(*vertex, propIndices, i, "green", "g", 127.5));
      g.sh[32] = ColorToShDc(ReadColor(*vertex, propIndices, i, "blue", "b", 127.5));
    }

    for (uint32_t r = 0; r < restCount; ++r) {
      const float v = static_cast<float>(vertex->scalarColumns[restIndices[r]][i]);
      WriteRestSh(g.sh, r, restCount, v, blockLayout);
    }

    if (!std::isfinite(g.position.x) || !std::isfinite(g.position.y) || !std::isfinite(g.position.z)) {
      continue;
    }
    if (!std::isfinite(g.scale.x) || !std::isfinite(g.scale.y) || !std::isfinite(g.scale.z)) {
      continue;
    }

    set.gaussians.push_back(g);
    points.push_back(g.position);
  }

  set.bounds = ComputeAabb(points);
  return set;
}

GaussianSet ParseCompressedGaussianSet(const ply::PlyFile& file, const std::string& setName,
                                       std::vector<std::string>& warnings) {
  GaussianSet set{};
  set.name = setName;

  const ply::PlyElement* chunk = ply::FindElement(file, "chunk");
  const ply::PlyElement* vertex = ply::FindElement(file, "vertex");
  if (chunk == nullptr || vertex == nullptr) {
    warnings.push_back("compressed ply missing chunk or vertex elements");
    return set;
  }

  std::unordered_map<std::string, int> chunkIndices;
  for (size_t i = 0; i < chunk->properties.size(); ++i) {
    if (!chunk->properties[i].isList) {
      chunkIndices[chunk->properties[i].name] = static_cast<int>(i);
    }
  }

  std::unordered_map<std::string, int> vertexIndices;
  for (size_t i = 0; i < vertex->properties.size(); ++i) {
    if (!vertex->properties[i].isList) {
      vertexIndices[vertex->properties[i].name] = static_cast<int>(i);
    }
  }

  auto chunkProp = [&](uint32_t row, const char* name) {
    const auto it = chunkIndices.find(name);
    if (it == chunkIndices.end() || row >= chunk->count) {
      return 0.0f;
    }
    return static_cast<float>(chunk->scalarColumns[it->second][row]);
  };

  auto vertexPacked = [&](uint32_t row, const char* name) {
    const auto it = vertexIndices.find(name);
    if (it == vertexIndices.end() || row >= vertex->count) {
      return 0u;
    }
    return static_cast<uint32_t>(vertex->scalarColumns[it->second][row]);
  };

  const bool hasChunkColor = (chunkIndices.count("min_r") > 0 && chunkIndices.count("max_r") > 0 &&
                              chunkIndices.count("min_g") > 0 && chunkIndices.count("max_g") > 0 &&
                              chunkIndices.count("min_b") > 0 && chunkIndices.count("max_b") > 0);

  set.gaussians.reserve(vertex->count);
  std::vector<Vec3> points;
  points.reserve(vertex->count);

  const float shC0 = 0.28209479177387814f;
  for (uint32_t i = 0; i < vertex->count; ++i) {
    const uint32_t chunkIndex = i / 256u;
    if (chunkIndex >= chunk->count) {
      break;
    }

    const Vec3 p = Unpack111011(vertexPacked(i, "packed_position"));
    const Quat r = UnpackRotation(vertexPacked(i, "packed_rotation"));
    const Vec3 s = Unpack111011(vertexPacked(i, "packed_scale"));
    const Vec4 c = Unpack8888(vertexPacked(i, "packed_color"));

    Gaussian g{};
    g.splatId = i;
    g.position = {
        Lerp(chunkProp(chunkIndex, "min_x"), chunkProp(chunkIndex, "max_x"), p.x),
        Lerp(chunkProp(chunkIndex, "min_y"), chunkProp(chunkIndex, "max_y"), p.y),
        Lerp(chunkProp(chunkIndex, "min_z"), chunkProp(chunkIndex, "max_z"), p.z),
    };
    g.rotation = Normalize(r);
    const float sx = Lerp(chunkProp(chunkIndex, "min_scale_x"), chunkProp(chunkIndex, "max_scale_x"), s.x);
    const float sy = Lerp(chunkProp(chunkIndex, "min_scale_y"), chunkProp(chunkIndex, "max_scale_y"), s.y);
    const float sz = Lerp(chunkProp(chunkIndex, "min_scale_z"), chunkProp(chunkIndex, "max_scale_z"), s.z);
    g.scale = {
        DecodeLogScaleValue(sx),
        DecodeLogScaleValue(sy),
        DecodeLogScaleValue(sz),
    };

    float cr = c.x;
    float cg = c.y;
    float cb = c.z;
    if (hasChunkColor) {
      cr = Lerp(chunkProp(chunkIndex, "min_r"), chunkProp(chunkIndex, "max_r"), c.x);
      cg = Lerp(chunkProp(chunkIndex, "min_g"), chunkProp(chunkIndex, "max_g"), c.y);
      cb = Lerp(chunkProp(chunkIndex, "min_b"), chunkProp(chunkIndex, "max_b"), c.z);
    }

    g.sh[0] = (cr - 0.5f) / shC0;
    g.sh[16] = (cg - 0.5f) / shC0;
    g.sh[32] = (cb - 0.5f) / shC0;

    const float alpha = Saturate(c.w);
    const float safe = std::clamp(alpha, 1e-6f, 1.0f - 1e-6f);
    g.opacity = std::log(safe / (1.0f - safe));

    set.gaussians.push_back(g);
    points.push_back(g.position);
  }

  const ply::PlyElement* sh = ply::FindElement(file, "sh");
  if (sh != nullptr && sh->count == vertex->count) {
    uint32_t restCount = 0;
    while (FindProperty(*sh, "f_rest_" + std::to_string(restCount)) >= 0) {
      ++restCount;
    }

    for (uint32_t i = 0; i < set.gaussians.size(); ++i) {
      for (uint32_t r = 0; r < restCount; ++r) {
        const int idx = FindProperty(*sh, "f_rest_" + std::to_string(r));
        if (idx < 0) {
          continue;
        }
        const uint32_t q = static_cast<uint32_t>(std::clamp(sh->scalarColumns[idx][i], 0.0, 255.0));
        float n = 0.0f;
        if (q == 255) {
          n = 1.0f;
        } else if (q > 0) {
          n = (static_cast<float>(q) + 0.5f) / 256.0f;
        }
        const float v = (n - 0.5f) * 8.0f;
        WriteRestSh(set.gaussians[i].sh, r, restCount, v, true);
      }
    }
  }

  set.bounds = ComputeAabb(points);
  return set;
}


struct FastPlyHeader {
  ply::PlyFormat format = ply::PlyFormat::BinaryLittleEndian;
  std::vector<std::string> comments;
  std::vector<ply::PlyElement> elements;
  std::streampos bodyOffset{};
};

StatusOr<bool> ReadFastPlyHeaderLine(std::ifstream& file, std::string& line, size_t& headerBytes) {
  line.clear();
  for (;;) {
    const int ch = file.get();
    if (ch == std::char_traits<char>::eof()) {
      if (file.eof()) {
        return StatusOr<bool>::Ok(!line.empty());
      }
      return StatusOr<bool>::Error("invalid ply header");
    }
    if (headerBytes >= kMaxFastPlyHeaderBytes) {
      return StatusOr<bool>::Error("ply header is too large");
    }
    ++headerBytes;
    if (ch == '\n') {
      return StatusOr<bool>::Ok(true);
    }
    if (line.size() >= kMaxFastPlyHeaderLineBytes) {
      return StatusOr<bool>::Error("ply header line too large");
    }
    line.push_back(static_cast<char>(ch));
  }
}

StatusOr<std::vector<std::string>> SplitSpaces(const std::string& s) {
  std::vector<std::string> out;
  size_t i = 0;
  while (i < s.size()) {
    while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) {
      ++i;
    }
    if (i >= s.size()) {
      break;
    }
    const size_t start = i;
    while (i < s.size() && !std::isspace(static_cast<unsigned char>(s[i]))) {
      ++i;
    }
    if (out.size() >= kMaxFastPlyHeaderTokens) {
      return StatusOr<std::vector<std::string>>::Error("invalid ply header");
    }
    out.emplace_back(s.substr(start, i - start));
  }
  return StatusOr<std::vector<std::string>>::Ok(std::move(out));
}

StatusOr<FastPlyHeader> ReadFastPlyHeader(std::ifstream& file) {
  FastPlyHeader header{};
  std::string line;
  size_t headerBytes = 0;
  auto lineRead = ReadFastPlyHeaderLine(file, line, headerBytes);
  if (!lineRead.ok()) {
    return StatusOr<FastPlyHeader>::Error(lineRead.status.message);
  }
  if (!lineRead.value || Trim(line) != "ply") {
    return StatusOr<FastPlyHeader>::Error("invalid ply magic");
  }

  ply::PlyElement* currentElement = nullptr;
  bool formatSet = false;
  bool headerEnded = false;
  for (;;) {
    lineRead = ReadFastPlyHeaderLine(file, line, headerBytes);
    if (!lineRead.ok()) {
      return StatusOr<FastPlyHeader>::Error(lineRead.status.message);
    }
    if (!lineRead.value) {
      break;
    }
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    const std::string trimmed = Trim(line);
    if (trimmed.empty()) {
      continue;
    }
    if (trimmed == "end_header") {
      headerEnded = true;
      header.bodyOffset = file.tellg();
      break;
    }

    if (trimmed.rfind("comment", 0) == 0 &&
        (trimmed.size() == 7 || std::isspace(static_cast<unsigned char>(trimmed[7])))) {
      if (trimmed.size() > 8) {
        if (header.comments.size() >= kMaxFastPlyHeaderComments) {
          return StatusOr<FastPlyHeader>::Error("ply header is too large");
        }
        header.comments.emplace_back(trimmed.substr(8));
      }
      continue;
    }

    auto tokensResult = SplitSpaces(trimmed);
    if (!tokensResult.ok()) {
      return StatusOr<FastPlyHeader>::Error(tokensResult.status.message);
    }
    const std::vector<std::string>& tokens = tokensResult.value;
    if (tokens.empty()) {
      continue;
    }
    if (tokens[0] == "format") {
      if (tokens.size() < 3) {
        return StatusOr<FastPlyHeader>::Error("invalid format line");
      }
      const std::string f = ToLower(tokens[1]);
      if (f == "binary_little_endian") {
        header.format = ply::PlyFormat::BinaryLittleEndian;
      } else if (f == "ascii") {
        header.format = ply::PlyFormat::Ascii;
      } else {
        return StatusOr<FastPlyHeader>::Error("unsupported ply format");
      }
      formatSet = true;
      continue;
    }

    if (tokens[0] == "element") {
      if (tokens.size() < 3) {
        return StatusOr<FastPlyHeader>::Error("invalid element line");
      }
      uint64_t count = 0;
      try {
        size_t consumed = 0;
        count = std::stoull(tokens[2], &consumed);
        if (consumed != tokens[2].size() || count > std::numeric_limits<uint32_t>::max()) {
          return StatusOr<FastPlyHeader>::Error("invalid element count");
        }
      } catch (...) {
        return StatusOr<FastPlyHeader>::Error("invalid element count");
      }
      if (header.elements.size() >= kMaxFastPlyHeaderElements) {
        return StatusOr<FastPlyHeader>::Error("too many ply elements");
      }
      ply::PlyElement elem{};
      elem.name = tokens[1];
      elem.count = static_cast<uint32_t>(count);
      header.elements.push_back(std::move(elem));
      currentElement = &header.elements.back();
      continue;
    }

    if (tokens[0] == "property") {
      if (currentElement == nullptr) {
        return StatusOr<FastPlyHeader>::Error("property before element");
      }
      ply::PlyProperty prop{};
      if (tokens.size() >= 5 && tokens[1] == "list") {
        prop.isList = true;
        prop.listCountType = ply::ParseScalarType(tokens[2]);
        prop.listValueType = ply::ParseScalarType(tokens[3]);
        prop.name = tokens[4];
        if (ply::ScalarTypeSize(prop.listCountType) == 0 || ply::ScalarTypeSize(prop.listValueType) == 0) {
          return StatusOr<FastPlyHeader>::Error("unsupported scalar type");
        }
      } else if (tokens.size() >= 3) {
        prop.type = ply::ParseScalarType(tokens[1]);
        prop.name = tokens[2];
        if (ply::ScalarTypeSize(prop.type) == 0) {
          return StatusOr<FastPlyHeader>::Error("unsupported scalar type");
        }
      } else {
        return StatusOr<FastPlyHeader>::Error("invalid property line");
      }
      if (currentElement->properties.size() >= kMaxFastPlyHeaderProperties) {
        return StatusOr<FastPlyHeader>::Error("too many ply properties");
      }
      currentElement->properties.push_back(std::move(prop));
      continue;
    }
  }

  if (!formatSet || !headerEnded || header.bodyOffset == std::streampos(-1)) {
    return StatusOr<FastPlyHeader>::Error("invalid ply header");
  }
  return StatusOr<FastPlyHeader>::Ok(std::move(header));
}

StatusOr<uint64_t> StreamPosBytes(std::streampos pos) {
  if (pos == std::streampos(-1)) {
    return StatusOr<uint64_t>::Error("invalid ply body");
  }
  const std::streamoff offset = static_cast<std::streamoff>(pos);
  if (offset < 0) {
    return StatusOr<uint64_t>::Error("invalid ply body");
  }
  return StatusOr<uint64_t>::Ok(static_cast<uint64_t>(offset));
}

StatusOr<uint64_t> FastPlyFileSize(std::ifstream& file) {
  const std::streampos current = file.tellg();
  const auto currentBytes = StreamPosBytes(current);
  if (!currentBytes.ok()) {
    return currentBytes;
  }
  file.seekg(0, std::ios::end);
  if (!file) {
    return StatusOr<uint64_t>::Error("invalid ply body");
  }
  const auto endBytes = StreamPosBytes(file.tellg());
  file.seekg(current);
  if (!file) {
    return StatusOr<uint64_t>::Error("invalid ply body");
  }
  return endBytes;
}

Status ValidateFastPlyCounts(const FastPlyHeader& header) {
  for (const ply::PlyElement& element : header.elements) {
    if (element.count > kMaxFastPlyElementRows) {
      return Status::Error("ply element count too large");
    }
    if (element.name == "chunk" && element.count > kMaxFastPlyChunkRows) {
      return Status::Error("ply element count too large");
    }
  }
  return Status::Ok();
}

Status AddFastPlyExpandedBytes(uint64_t count, uint64_t stride, uint64_t& bytes) {
  if (stride != 0 && count > std::numeric_limits<uint64_t>::max() / stride) {
    return Status::Error("ply expanded data too large");
  }
  const uint64_t added = count * stride;
  if (bytes > std::numeric_limits<uint64_t>::max() - added) {
    return Status::Error("ply expanded data too large");
  }
  bytes += added;
  if (bytes > kMaxFastPlyExpandedBytes) {
    return Status::Error("ply expanded data too large");
  }
  return Status::Ok();
}

Status ValidateFastPlyExpandedStorage(const FastPlyHeader& header) {
  uint64_t bytes = 0;
  for (const ply::PlyElement& element : header.elements) {
    if (element.name == "vertex") {
      Status status = AddFastPlyExpandedBytes(element.count, sizeof(Gaussian) + sizeof(Vec3), bytes);
      if (!status.ok) {
        return status;
      }
    } else if (element.name == "chunk") {
      Status status = AddFastPlyExpandedBytes(element.count, sizeof(FastChunkInfo), bytes);
      if (!status.ok) {
        return status;
      }
    }
  }
  return Status::Ok();
}

Status ValidateFastPlyFixedRows(const FastPlyHeader& header) {
  for (const ply::PlyElement& element : header.elements) {
    for (const ply::PlyProperty& prop : element.properties) {
      if (prop.isList) {
        return Status::Error("unsupported fast ply path");
      }
    }
  }
  return Status::Ok();
}

StatusOr<uint64_t> FastPlyMinimumRowBytes(const ply::PlyElement& element) {
  uint64_t rowBytes = 0;
  for (const ply::PlyProperty& prop : element.properties) {
    const uint64_t size = static_cast<uint64_t>(ply::ScalarTypeSize(prop.type));
    if (size == 0) {
      return StatusOr<uint64_t>::Error("unsupported scalar type");
    }
    if (rowBytes > std::numeric_limits<uint64_t>::max() - size) {
      return StatusOr<uint64_t>::Error("ply body is too large");
    }
    rowBytes += size;
  }
  return StatusOr<uint64_t>::Ok(rowBytes);
}

Status ValidateFastPlyBodyFootprint(const FastPlyHeader& header, uint64_t fileSize) {
  Status countStatus = ValidateFastPlyCounts(header);
  if (!countStatus.ok) {
    return countStatus;
  }
  Status storageStatus = ValidateFastPlyExpandedStorage(header);
  if (!storageStatus.ok) {
    return storageStatus;
  }
  Status fixedRowsStatus = ValidateFastPlyFixedRows(header);
  if (!fixedRowsStatus.ok) {
    return fixedRowsStatus;
  }
  const auto bodyOffset = StreamPosBytes(header.bodyOffset);
  if (!bodyOffset.ok()) {
    return Status::Error(bodyOffset.status.message);
  }
  if (bodyOffset.value > fileSize) {
    return Status::Error("invalid ply body");
  }
  const uint64_t remainingBytes = fileSize - bodyOffset.value;
  uint64_t requiredBytes = 0;
  for (const ply::PlyElement& element : header.elements) {
    const auto rowBytes = FastPlyMinimumRowBytes(element);
    if (!rowBytes.ok()) {
      return Status::Error(rowBytes.status.message);
    }
    if (rowBytes.value != 0 && element.count > std::numeric_limits<uint64_t>::max() / rowBytes.value) {
      return Status::Error("ply body is too large");
    }
    const uint64_t elementBytes = rowBytes.value * static_cast<uint64_t>(element.count);
    if (requiredBytes > std::numeric_limits<uint64_t>::max() - elementBytes) {
      return Status::Error("ply body is too large");
    }
    requiredBytes += elementBytes;
    if (requiredBytes > remainingBytes) {
      return Status::Error("invalid ply body");
    }
  }
  return Status::Ok();
}

template <typename T>
StatusOr<T> ReadPod(std::istream& file) {
  T value{};
  file.read(reinterpret_cast<char*>(&value), sizeof(T));
  if (!file) {
    return StatusOr<T>::Error("binary ply scalar read out of bounds");
  }
  return StatusOr<T>::Ok(value);
}

StatusOr<double> ReadScalarAsDouble(std::istream& file, ply::PlyScalarType type) {
  switch (type) {
    case ply::PlyScalarType::Int8: {
      auto v = ReadPod<int8_t>(file);
      if (!v.ok()) return StatusOr<double>::Error(v.status.message);
      return StatusOr<double>::Ok(static_cast<double>(v.value));
    }
    case ply::PlyScalarType::UInt8: {
      auto v = ReadPod<uint8_t>(file);
      if (!v.ok()) return StatusOr<double>::Error(v.status.message);
      return StatusOr<double>::Ok(static_cast<double>(v.value));
    }
    case ply::PlyScalarType::Int16: {
      auto v = ReadPod<int16_t>(file);
      if (!v.ok()) return StatusOr<double>::Error(v.status.message);
      return StatusOr<double>::Ok(static_cast<double>(v.value));
    }
    case ply::PlyScalarType::UInt16: {
      auto v = ReadPod<uint16_t>(file);
      if (!v.ok()) return StatusOr<double>::Error(v.status.message);
      return StatusOr<double>::Ok(static_cast<double>(v.value));
    }
    case ply::PlyScalarType::Int32: {
      auto v = ReadPod<int32_t>(file);
      if (!v.ok()) return StatusOr<double>::Error(v.status.message);
      return StatusOr<double>::Ok(static_cast<double>(v.value));
    }
    case ply::PlyScalarType::UInt32: {
      auto v = ReadPod<uint32_t>(file);
      if (!v.ok()) return StatusOr<double>::Error(v.status.message);
      return StatusOr<double>::Ok(static_cast<double>(v.value));
    }
    case ply::PlyScalarType::Float32: {
      auto v = ReadPod<float>(file);
      if (!v.ok()) return StatusOr<double>::Error(v.status.message);
      return StatusOr<double>::Ok(static_cast<double>(v.value));
    }
    case ply::PlyScalarType::Float64: {
      auto v = ReadPod<double>(file);
      if (!v.ok()) return StatusOr<double>::Error(v.status.message);
      return StatusOr<double>::Ok(v.value);
    }
    default:
      return StatusOr<double>::Error("unsupported scalar type");
  }
}

Status SkipScalar(std::istream& file, ply::PlyScalarType type) {
  const std::streamoff bytes = static_cast<std::streamoff>(ply::ScalarTypeSize(type));
  if (bytes <= 0) {
    return Status::Error("unsupported scalar type");
  }
  file.seekg(bytes, std::ios::cur);
  if (!file) {
    return Status::Error("binary ply scalar read out of bounds");
  }
  return Status::Ok();
}

Status SkipProperty(std::istream& file, const ply::PlyProperty& prop) {
  if (!prop.isList) {
    return SkipScalar(file, prop.type);
  }
  const auto countValue = ReadScalarAsDouble(file, prop.listCountType);
  if (!countValue.ok()) {
    return Status::Error(countValue.status.message);
  }
  if (!std::isfinite(countValue.value) || countValue.value < 0.0 ||
      countValue.value > static_cast<double>(std::numeric_limits<uint32_t>::max()) ||
      std::floor(countValue.value) != countValue.value) {
    return Status::Error("binary ply list count out of range");
  }
  const uint64_t count = static_cast<uint64_t>(countValue.value);
  const uint64_t itemBytes = static_cast<uint64_t>(ply::ScalarTypeSize(prop.listValueType));
  if (itemBytes == 0 || count > static_cast<uint64_t>(std::numeric_limits<std::streamoff>::max()) / itemBytes) {
    return Status::Error("ply body is too large");
  }
  file.seekg(static_cast<std::streamoff>(count * itemBytes), std::ios::cur);
  if (!file) {
    return Status::Error("binary ply scalar read out of bounds");
  }
  return Status::Ok();
}

std::unordered_map<std::string, int> BuildPropIndex(const ply::PlyElement& element) {
  std::unordered_map<std::string, int> out;
  for (size_t i = 0; i < element.properties.size(); ++i) {
    if (!element.properties[i].isList) {
      out[element.properties[i].name] = static_cast<int>(i);
    }
  }
  return out;
}

bool HasProperty(const std::unordered_map<std::string, int>& indices, const char* name) {
  return indices.find(name) != indices.end();
}

Status SkipElement(std::istream& file, const ply::PlyElement& element) {
  for (uint32_t row = 0; row < element.count; ++row) {
    for (const ply::PlyProperty& prop : element.properties) {
      Status status = SkipProperty(file, prop);
      if (!status.ok) {
        return status;
      }
    }
  }
  return Status::Ok();
}

StatusOr<PlyLoadResult> LoadBinaryStandardPlyFast(std::ifstream& file, const FastPlyHeader& header,
                                                  const std::string& setName) {
  auto vertexIt = std::find_if(header.elements.begin(), header.elements.end(), [](const ply::PlyElement& e) {
    return e.name == "vertex";
  });
  if (vertexIt == header.elements.end()) {
    return StatusOr<PlyLoadResult>::Error("unsupported fast ply path");
  }
  const auto indices = BuildPropIndex(*vertexIt);
  const int xIndex = ply::FindScalarProperty(*vertexIt, "x");
  const int yIndex = ply::FindScalarProperty(*vertexIt, "y");
  const int zIndex = ply::FindScalarProperty(*vertexIt, "z");
  if (xIndex < 0 || yIndex < 0 || zIndex < 0) {
    return StatusOr<PlyLoadResult>::Error("unsupported fast ply path");
  }

  const bool hasScale012 = HasProperty(indices, "scale_0") && HasProperty(indices, "scale_1") && HasProperty(indices, "scale_2");
  const bool hasScaleXyz = HasProperty(indices, "scale_x") && HasProperty(indices, "scale_y") && HasProperty(indices, "scale_z");
  const bool hasAnyScale = hasScale012 || hasScaleXyz;
  const bool hasDc = HasProperty(indices, "f_dc_0") && HasProperty(indices, "f_dc_1") && HasProperty(indices, "f_dc_2");
  const bool hasRgb = (HasProperty(indices, "red") || HasProperty(indices, "r")) &&
                      (HasProperty(indices, "green") || HasProperty(indices, "g")) &&
                      (HasProperty(indices, "blue") || HasProperty(indices, "b"));
  const bool pointCloudFallback = !hasAnyScale && !hasDc && hasRgb;
  if (!hasAnyScale && !hasDc && !hasRgb) {
    return StatusOr<PlyLoadResult>::Error("unsupported fast ply path");
  }

  std::vector<int> restByProperty(vertexIt->properties.size(), -1);
  uint32_t restCount = 0;
  for (;;) {
    const int idx = ply::FindScalarProperty(*vertexIt, "f_rest_" + std::to_string(restCount));
    if (idx < 0) {
      break;
    }
    restByProperty[static_cast<size_t>(idx)] = static_cast<int>(restCount);
    ++restCount;
  }
  const bool blockLayout = UseBlockShLayout(header.comments);

  PlyLoadResult out{};
  out.wasCompressed = false;
  out.set.name = setName;
  out.set.gaussians.reserve(vertexIt->count);
  std::vector<Vec3> points;
  points.reserve(vertexIt->count);

  for (const ply::PlyElement& element : header.elements) {
    if (element.name != "vertex") {
      const Status skipped = SkipElement(file, element);
      if (!skipped.ok) {
        return StatusOr<PlyLoadResult>::Error(skipped.message);
      }
      continue;
    }

    for (uint32_t row = 0; row < element.count; ++row) {
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      double sx = 0.0;
      double sy = 0.0;
      double sz = 0.0;
      double rot0 = 1.0;
      double rot1 = 0.0;
      double rot2 = 0.0;
      double rot3 = 0.0;
      double opacity = pointCloudFallback ? 2.0 : 1.0;
      double red = 127.5;
      double green = 127.5;
      double blue = 127.5;
      bool readRed = false;
      bool readGreen = false;
      bool readBlue = false;
      Gaussian g{};
      g.splatId = row;

      for (size_t pi = 0; pi < element.properties.size(); ++pi) {
        const ply::PlyProperty& prop = element.properties[pi];
        if (prop.isList) {
          const Status skipped = SkipProperty(file, prop);
          if (!skipped.ok) {
            return StatusOr<PlyLoadResult>::Error(skipped.message);
          }
          continue;
        }
        const auto value = ReadScalarAsDouble(file, prop.type);
        if (!value.ok()) {
          return StatusOr<PlyLoadResult>::Error(value.status.message);
        }
        const double v = value.value;
        const std::string& name = prop.name;
        if (name == "x") x = v;
        else if (name == "y") y = v;
        else if (name == "z") z = v;
        else if (name == "scale_0" || name == "scale_x") sx = v;
        else if (name == "scale_1" || name == "scale_y") sy = v;
        else if (name == "scale_2" || name == "scale_z") sz = v;
        else if (name == "rot_0") rot0 = v;
        else if (name == "rot_1") rot1 = v;
        else if (name == "rot_2") rot2 = v;
        else if (name == "rot_3") rot3 = v;
        else if (name == "opacity") opacity = v;
        else if (name == "f_dc_0") g.sh[0] = static_cast<float>(v);
        else if (name == "f_dc_1") g.sh[16] = static_cast<float>(v);
        else if (name == "f_dc_2") g.sh[32] = static_cast<float>(v);
        else if (name == "red" || name == "r") { red = v; readRed = true; }
        else if (name == "green" || name == "g") { green = v; readGreen = true; }
        else if (name == "blue" || name == "b") { blue = v; readBlue = true; }
        if (pi < restByProperty.size() && restByProperty[pi] >= 0) {
          WriteRestSh(g.sh, static_cast<uint32_t>(restByProperty[pi]), restCount, static_cast<float>(v), blockLayout);
        }
      }

      g.position = {static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)};
      if (hasScale012) {
        g.scale = {DecodeLogScaleValue(static_cast<float>(sx)), DecodeLogScaleValue(static_cast<float>(sy)),
                   DecodeLogScaleValue(static_cast<float>(sz))};
      } else if (!hasAnyScale) {
        g.scale = {1.0f, 1.0f, 1.0f};
      } else {
        g.scale = {DecodeScaleValue(static_cast<float>(sx)), DecodeScaleValue(static_cast<float>(sy)),
                   DecodeScaleValue(static_cast<float>(sz))};
      }
      g.rotation = Normalize({static_cast<float>(rot1), static_cast<float>(rot2), static_cast<float>(rot3), static_cast<float>(rot0)});
      g.opacity = static_cast<float>(opacity);
      if (!hasDc && hasRgb) {
        if (!readRed) red = 127.5;
        if (!readGreen) green = 127.5;
        if (!readBlue) blue = 127.5;
        g.sh[0] = ColorToShDc(NormalizeColorValue(red));
        g.sh[16] = ColorToShDc(NormalizeColorValue(green));
        g.sh[32] = ColorToShDc(NormalizeColorValue(blue));
      }

      if (!std::isfinite(g.position.x) || !std::isfinite(g.position.y) || !std::isfinite(g.position.z) ||
          !std::isfinite(g.scale.x) || !std::isfinite(g.scale.y) || !std::isfinite(g.scale.z)) {
        continue;
      }
      out.set.gaussians.push_back(g);
      points.push_back(g.position);
    }
  }

  if (out.set.gaussians.empty()) {
    return StatusOr<PlyLoadResult>::Error("no valid gaussians found");
  }

  if (pointCloudFallback) {
    const Aabb sourceBounds = ComputeAabb(points);
    const float sourceRadius = sourceBounds.valid ? std::max(ComputeAabbRadius(sourceBounds), 1e-3f) : 1.0f;
    const float defaultPointScale = std::clamp(
        sourceRadius / std::max(32.0f, std::sqrt(static_cast<float>(std::max<size_t>(points.size(), 1)))) * 0.08f,
        sourceRadius * 0.00008f, sourceRadius * 0.0015f);
    for (Gaussian& g : out.set.gaussians) {
      g.scale = {defaultPointScale, defaultPointScale, defaultPointScale};
    }
  }

  out.set.bounds = ComputeAabb(points);
  return StatusOr<PlyLoadResult>::Ok(std::move(out));
}

StatusOr<PlyLoadResult> LoadBinaryCompressedPlyFast(std::ifstream& file, const FastPlyHeader& header,
                                                    const std::string& setName) {
  ply::PlyFile layout{};
  layout.format = header.format;
  layout.comments = header.comments;
  layout.elements = header.elements;
  if (!HasCompressedSchema(layout)) {
    return StatusOr<PlyLoadResult>::Error("unsupported fast ply path");
  }

  const ply::PlyElement* chunkHeader = ply::FindElement(layout, "chunk");
  const ply::PlyElement* vertexHeader = ply::FindElement(layout, "vertex");
  if (chunkHeader == nullptr || vertexHeader == nullptr) {
    return StatusOr<PlyLoadResult>::Error("unsupported fast ply path");
  }

  PlyLoadResult out{};
  out.wasCompressed = true;
  out.set.name = setName;
  out.set.gaussians.reserve(vertexHeader->count);
  std::vector<Vec3> points;
  points.reserve(vertexHeader->count);
  std::vector<FastChunkInfo> chunks;

  for (const ply::PlyElement& element : header.elements) {
    if (element.name == "chunk") {
      const auto indices = BuildPropIndex(element);
      const bool hasChunkColor = HasProperty(indices, "min_r") && HasProperty(indices, "max_r") &&
                                HasProperty(indices, "min_g") && HasProperty(indices, "max_g") &&
                                HasProperty(indices, "min_b") && HasProperty(indices, "max_b");
      chunks.resize(element.count);
      for (uint32_t row = 0; row < element.count; ++row) {
        FastChunkInfo info{};
        info.hasColor = hasChunkColor;
        for (size_t pi = 0; pi < element.properties.size(); ++pi) {
          const ply::PlyProperty& prop = element.properties[pi];
          if (prop.isList) {
            const Status skipped = SkipProperty(file, prop);
            if (!skipped.ok) return StatusOr<PlyLoadResult>::Error(skipped.message);
            continue;
          }
          const auto value = ReadScalarAsDouble(file, prop.type);
          if (!value.ok()) return StatusOr<PlyLoadResult>::Error(value.status.message);
          const float v = static_cast<float>(value.value);
          const std::string& name = prop.name;
          if (name == "min_x") info.minPosition.x = v;
          else if (name == "min_y") info.minPosition.y = v;
          else if (name == "min_z") info.minPosition.z = v;
          else if (name == "max_x") info.maxPosition.x = v;
          else if (name == "max_y") info.maxPosition.y = v;
          else if (name == "max_z") info.maxPosition.z = v;
          else if (name == "min_scale_x") info.minScale.x = v;
          else if (name == "min_scale_y") info.minScale.y = v;
          else if (name == "min_scale_z") info.minScale.z = v;
          else if (name == "max_scale_x") info.maxScale.x = v;
          else if (name == "max_scale_y") info.maxScale.y = v;
          else if (name == "max_scale_z") info.maxScale.z = v;
          else if (name == "min_r") info.minColor.x = v;
          else if (name == "min_g") info.minColor.y = v;
          else if (name == "min_b") info.minColor.z = v;
          else if (name == "max_r") info.maxColor.x = v;
          else if (name == "max_g") info.maxColor.y = v;
          else if (name == "max_b") info.maxColor.z = v;
        }
        chunks[row] = info;
      }
      continue;
    }

    if (element.name == "vertex") {
      if (chunks.empty()) {
        return StatusOr<PlyLoadResult>::Error("compressed ply vertex data appeared before chunk data");
      }
      for (uint32_t row = 0; row < element.count; ++row) {
        uint32_t packedPosition = 0;
        uint32_t packedRotation = 0;
        uint32_t packedScale = 0;
        uint32_t packedColor = 0;
        for (const ply::PlyProperty& prop : element.properties) {
          if (prop.isList) {
            const Status skipped = SkipProperty(file, prop);
            if (!skipped.ok) return StatusOr<PlyLoadResult>::Error(skipped.message);
            continue;
          }
          const auto value = ReadScalarAsDouble(file, prop.type);
          if (!value.ok()) return StatusOr<PlyLoadResult>::Error(value.status.message);
          const uint32_t v = static_cast<uint32_t>(value.value);
          if (prop.name == "packed_position") packedPosition = v;
          else if (prop.name == "packed_rotation") packedRotation = v;
          else if (prop.name == "packed_scale") packedScale = v;
          else if (prop.name == "packed_color") packedColor = v;
        }

        const uint32_t chunkIndex = row / 256u;
        if (chunkIndex >= chunks.size()) {
          break;
        }
        const FastChunkInfo& chunk = chunks[chunkIndex];
        const Vec3 p = Unpack111011(packedPosition);
        const Quat r = UnpackRotation(packedRotation);
        const Vec3 s = Unpack111011(packedScale);
        const Vec4 c = Unpack8888(packedColor);

        Gaussian g{};
        g.splatId = row;
        g.position = {Lerp(chunk.minPosition.x, chunk.maxPosition.x, p.x),
                      Lerp(chunk.minPosition.y, chunk.maxPosition.y, p.y),
                      Lerp(chunk.minPosition.z, chunk.maxPosition.z, p.z)};
        g.rotation = Normalize(r);
        g.scale = {DecodeLogScaleValue(Lerp(chunk.minScale.x, chunk.maxScale.x, s.x)),
                   DecodeLogScaleValue(Lerp(chunk.minScale.y, chunk.maxScale.y, s.y)),
                   DecodeLogScaleValue(Lerp(chunk.minScale.z, chunk.maxScale.z, s.z))};
        float cr = c.x;
        float cg = c.y;
        float cb = c.z;
        if (chunk.hasColor) {
          cr = Lerp(chunk.minColor.x, chunk.maxColor.x, c.x);
          cg = Lerp(chunk.minColor.y, chunk.maxColor.y, c.y);
          cb = Lerp(chunk.minColor.z, chunk.maxColor.z, c.z);
        }
        constexpr float shC0 = 0.28209479177387814f;
        g.sh[0] = (cr - 0.5f) / shC0;
        g.sh[16] = (cg - 0.5f) / shC0;
        g.sh[32] = (cb - 0.5f) / shC0;
        const float alpha = Saturate(c.w);
        const float safe = std::clamp(alpha, 1e-6f, 1.0f - 1e-6f);
        g.opacity = std::log(safe / (1.0f - safe));
        out.set.gaussians.push_back(g);
        points.push_back(g.position);
      }
      continue;
    }

    if (element.name == "sh" && element.count == out.set.gaussians.size()) {
      std::vector<int> restByProperty(element.properties.size(), -1);
      uint32_t restCount = 0;
      for (;;) {
        const int idx = ply::FindScalarProperty(element, "f_rest_" + std::to_string(restCount));
        if (idx < 0) break;
        restByProperty[static_cast<size_t>(idx)] = static_cast<int>(restCount);
        ++restCount;
      }
      for (uint32_t row = 0; row < element.count; ++row) {
        for (size_t pi = 0; pi < element.properties.size(); ++pi) {
          const ply::PlyProperty& prop = element.properties[pi];
          if (prop.isList) {
            const Status skipped = SkipProperty(file, prop);
            if (!skipped.ok) return StatusOr<PlyLoadResult>::Error(skipped.message);
            continue;
          }
          const auto value = ReadScalarAsDouble(file, prop.type);
          if (!value.ok()) return StatusOr<PlyLoadResult>::Error(value.status.message);
          if (pi < restByProperty.size() && restByProperty[pi] >= 0) {
            const uint32_t q = static_cast<uint32_t>(std::clamp(value.value, 0.0, 255.0));
            float n = 0.0f;
            if (q == 255) {
              n = 1.0f;
            } else if (q > 0) {
              n = (static_cast<float>(q) + 0.5f) / 256.0f;
            }
            const float v = (n - 0.5f) * 8.0f;
            WriteRestSh(out.set.gaussians[row].sh, static_cast<uint32_t>(restByProperty[pi]), restCount, v, true);
          }
        }
      }
      continue;
    }

    const Status skipped = SkipElement(file, element);
    if (!skipped.ok) {
      return StatusOr<PlyLoadResult>::Error(skipped.message);
    }
  }

  if (out.set.gaussians.empty()) {
    return StatusOr<PlyLoadResult>::Error("no valid gaussians found");
  }
  out.set.bounds = ComputeAabb(points);
  return StatusOr<PlyLoadResult>::Ok(std::move(out));
}

StatusOr<PlyLoadResult> TryLoadBinaryPlyFast(const std::string& path, const std::string& setName) {
  std::ifstream file(path, std::ios::binary);
  if (!file.is_open()) {
    return StatusOr<PlyLoadResult>::Error("failed to open file");
  }

  auto header = ReadFastPlyHeader(file);
  if (!header.ok()) {
    return StatusOr<PlyLoadResult>::Error(header.status.message);
  }
  if (header.value.format != ply::PlyFormat::BinaryLittleEndian) {
    return StatusOr<PlyLoadResult>::Error("unsupported fast ply path");
  }
  const auto fileSize = FastPlyFileSize(file);
  if (!fileSize.ok()) {
    return StatusOr<PlyLoadResult>::Error(fileSize.status.message);
  }
  Status bodyStatus = ValidateFastPlyBodyFootprint(header.value, fileSize.value);
  if (!bodyStatus.ok) {
    return StatusOr<PlyLoadResult>::Error(bodyStatus.message);
  }
  file.seekg(header.value.bodyOffset);
  if (!file) {
    return StatusOr<PlyLoadResult>::Error("invalid ply body");
  }

  auto compressed = LoadBinaryCompressedPlyFast(file, header.value, setName);
  if (compressed.ok() || compressed.status.message != "unsupported fast ply path") {
    return compressed;
  }

  file.clear();
  file.seekg(header.value.bodyOffset);
  if (!file) {
    return StatusOr<PlyLoadResult>::Error("invalid ply body");
  }
  return LoadBinaryStandardPlyFast(file, header.value, setName);
}

}  // namespace

StatusOr<PlyLoadResult> PlyLoader::Load(const std::string& path, const std::string& setName) const try {
  const auto fastResult = TryLoadBinaryPlyFast(path, setName);
  if (fastResult.ok()) {
    return fastResult;
  }
  if (fastResult.status.message != "unsupported fast ply path") {
    return StatusOr<PlyLoadResult>::Error(fastResult.status.message);
  }

  const auto plyResult = ply::ReadPlyFile(path);
  if (!plyResult.ok()) {
    return StatusOr<PlyLoadResult>::Error(plyResult.status.message);
  }

  PlyLoadResult out{};
  out.wasCompressed = HasCompressedSchema(plyResult.value);
  if (out.wasCompressed) {
    out.set = ParseCompressedGaussianSet(plyResult.value, setName, out.warnings);
  } else {
    out.set = ParseStandardGaussianSet(plyResult.value, setName, out.warnings);
  }

  if (out.set.gaussians.empty()) {
    return StatusOr<PlyLoadResult>::Error("no valid gaussians found");
  }

  return StatusOr<PlyLoadResult>::Ok(std::move(out));
} catch (const std::bad_alloc&) {
  return StatusOr<PlyLoadResult>::Error("ply scene allocation failed");
} catch (const std::length_error&) {
  return StatusOr<PlyLoadResult>::Error("ply scene allocation failed");
}

}
