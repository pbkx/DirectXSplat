#include <doctest/doctest.h>

#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

#include "directxsplat/io.h"
#include "directxsplat/scene.h"
#include "io/formats/ply/raw/ply_reader.h"
#include "io/formats/ply/raw/ply_writer.h"
#include "io/image/wic_image.h"

namespace directxsplat {
namespace {

constexpr float kShC0 = 0.28209479177387814f;

std::filesystem::path MakeTempDir(const char* name) {
  const std::filesystem::path dir = std::filesystem::temp_directory_path() / name;
  std::error_code ec;
  std::filesystem::remove_all(dir, ec);
  std::filesystem::create_directories(dir, ec);
  return dir;
}

void WriteText(const std::filesystem::path& path, const std::string& data) {
  std::ofstream file(path, std::ios::binary);
  file << data;
}

template <typename T>
void AppendPod(std::string& out, T value) {
  const auto* bytes = reinterpret_cast<const char*>(&value);
  out.append(bytes, bytes + sizeof(T));
}

void AppendFloat(std::string& out, float value) {
  AppendPod(out, value);
}

void AppendByte(std::string& out, uint8_t value) {
  out.push_back(static_cast<char>(value));
}

std::string RawZstdFrame(std::initializer_list<uint8_t> payload) {
  std::string out;
  AppendByte(out, 0x28u);
  AppendByte(out, 0xB5u);
  AppendByte(out, 0x2Fu);
  AppendByte(out, 0xFDu);
  AppendByte(out, 0x20u);
  AppendByte(out, static_cast<uint8_t>(payload.size()));
  const uint32_t blockHeader = (static_cast<uint32_t>(payload.size()) << 3u) | 1u;
  AppendByte(out, static_cast<uint8_t>(blockHeader & 0xFFu));
  AppendByte(out, static_cast<uint8_t>((blockHeader >> 8u) & 0xFFu));
  AppendByte(out, static_cast<uint8_t>((blockHeader >> 16u) & 0xFFu));
  for (uint8_t value : payload) {
    AppendByte(out, value);
  }
  return out;
}

std::string MakeSpzV4Record() {
  const std::array<std::string, 6> streams{{
      RawZstdFrame({0x00u, 0x01u, 0x00u, 0x00u, 0xFEu, 0xFFu, 0x00u, 0xFDu, 0xFFu}),
      RawZstdFrame({128u}),
      RawZstdFrame({128u, 64u, 255u}),
      RawZstdFrame({160u, 144u, 176u}),
      RawZstdFrame({0x00u, 0x00u, 0x00u, 0xC0u}),
      RawZstdFrame({128u, 128u, 128u, 255u, 0u, 128u, 64u, 192u, 128u}),
  }};
  const std::array<uint64_t, 6> uncompressedSizes{{9u, 1u, 3u, 3u, 4u, 9u}};

  std::string out;
  AppendPod(out, uint32_t{0x5053474Eu});
  AppendPod(out, uint32_t{4u});
  AppendPod(out, uint32_t{1u});
  AppendByte(out, 1u);
  AppendByte(out, 8u);
  AppendByte(out, 0u);
  AppendByte(out, static_cast<uint8_t>(streams.size()));
  AppendPod(out, uint32_t{32u});
  out.append(12u, '\0');
  for (size_t i = 0; i < streams.size(); ++i) {
    AppendPod(out, static_cast<uint64_t>(streams[i].size()));
    AppendPod(out, uncompressedSizes[i]);
  }
  for (const std::string& stream : streams) {
    out.append(stream);
  }
  return out;
}

std::string BinaryPlyHeader(uint32_t count) {
  return std::string("ply\n")
       + "format binary_little_endian 1.0\n"
       + "element vertex " + std::to_string(count) + "\n"
       + "property float x\n"
       + "property float y\n"
       + "property float z\n"
       + "property float scale_x\n"
       + "property float scale_y\n"
       + "property float scale_z\n"
       + "property float rot_0\n"
       + "property float rot_1\n"
       + "property float rot_2\n"
       + "property float rot_3\n"
       + "property float opacity\n"
       + "property uchar red\n"
       + "property uchar green\n"
       + "property uchar blue\n"
       + "end_header\n";
}

void AppendBinaryPlyVertex(std::string& out, Vec3 p, Vec3 scale, float opacity, uint8_t r, uint8_t g, uint8_t b) {
  AppendFloat(out, p.x);
  AppendFloat(out, p.y);
  AppendFloat(out, p.z);
  AppendFloat(out, scale.x);
  AppendFloat(out, scale.y);
  AppendFloat(out, scale.z);
  AppendFloat(out, 1.0f);
  AppendFloat(out, 0.0f);
  AppendFloat(out, 0.0f);
  AppendFloat(out, 0.0f);
  AppendFloat(out, opacity);
  AppendByte(out, r);
  AppendByte(out, g);
  AppendByte(out, b);
}

void WriteSplatRecord(const std::filesystem::path& path) {
  std::string bytes;
  bytes.resize(32u, '\0');
  auto putFloat = [&](size_t offset, float value) {
    const uint32_t bits = std::bit_cast<uint32_t>(value);
    bytes[offset + 0u] = static_cast<char>(bits & 0xFFu);
    bytes[offset + 1u] = static_cast<char>((bits >> 8u) & 0xFFu);
    bytes[offset + 2u] = static_cast<char>((bits >> 16u) & 0xFFu);
    bytes[offset + 3u] = static_cast<char>((bits >> 24u) & 0xFFu);
  };
  putFloat(0u, 1.25f);
  putFloat(4u, -2.5f);
  putFloat(8u, 3.75f);
  putFloat(12u, 0.5f);
  putFloat(16u, 0.75f);
  putFloat(20u, 1.25f);
  bytes[24] = static_cast<char>(255);
  bytes[25] = static_cast<char>(64);
  bytes[26] = static_cast<char>(0);
  bytes[27] = static_cast<char>(200);
  bytes[28] = static_cast<char>(255);
  bytes[29] = static_cast<char>(128);
  bytes[30] = static_cast<char>(128);
  bytes[31] = static_cast<char>(128);
  WriteText(path, bytes);
}

std::string MakeRgbaTga(uint16_t width, uint16_t height, const std::vector<uint8_t>& rgba) {
  std::string out(18u, '\0');
  out[2] = 2;
  out[12] = static_cast<char>(width & 0xFFu);
  out[13] = static_cast<char>((width >> 8u) & 0xFFu);
  out[14] = static_cast<char>(height & 0xFFu);
  out[15] = static_cast<char>((height >> 8u) & 0xFFu);
  out[16] = 32;
  out[17] = 0x28;
  for (size_t i = 0; i + 3u < rgba.size(); i += 4u) {
    AppendByte(out, rgba[i + 2u]);
    AppendByte(out, rgba[i + 1u]);
    AppendByte(out, rgba[i + 0u]);
    AppendByte(out, rgba[i + 3u]);
  }
  return out;
}

void WriteRgbaTga(const std::filesystem::path& path, uint16_t width, uint16_t height,
                  const std::vector<uint8_t>& rgba) {
  WriteText(path, MakeRgbaTga(width, height, rgba));
}

std::string MakeLinearCodebook() {
  std::string out = "[";
  for (uint32_t i = 0; i < 256u; ++i) {
    if (i != 0) {
      out += ',';
    }
    out += std::to_string(static_cast<float>(i) / 255.0f);
  }
  out += ']';
  return out;
}

void WriteMinimalSogV2(const std::filesystem::path& dir, float x) {
  std::filesystem::create_directories(dir);
  const float transformedX = std::copysign(std::log(std::abs(x) + 1.0f), x);
  const std::string encodedX = std::to_string(transformedX);
  const std::string codebook = MakeLinearCodebook();
  WriteText(dir / "meta.json",
            "{"
            "\"version\":2,\"count\":1,"
            "\"means\":{\"mins\":[" + encodedX + ",0,0],\"maxs\":[" + encodedX +
                ",0,0],\"files\":[\"means_l.webp\",\"means_u.webp\"]},"
            "\"scales\":{\"codebook\":" + codebook + ",\"files\":[\"scales.webp\"]},"
            "\"quats\":{\"files\":[\"quats.webp\"]},"
            "\"sh0\":{\"codebook\":" + codebook + ",\"files\":[\"sh0.webp\"]}"
            "}");
  WriteRgbaTga(dir / "means_l.webp", 1u, 1u, {0u, 0u, 0u, 255u});
  WriteRgbaTga(dir / "means_u.webp", 1u, 1u, {0u, 0u, 0u, 255u});
  WriteRgbaTga(dir / "scales.webp", 1u, 1u, {0u, 0u, 0u, 255u});
  WriteRgbaTga(dir / "quats.webp", 1u, 1u, {128u, 128u, 128u, 252u});
  WriteRgbaTga(dir / "sh0.webp", 1u, 1u, {0u, 0u, 0u, 128u});
}

bool IsFiniteSet(const GaussianSet& set) {
  for (const Gaussian& gaussian : set.gaussians) {
    if (!std::isfinite(gaussian.position.x) || !std::isfinite(gaussian.position.y) ||
        !std::isfinite(gaussian.position.z) || !std::isfinite(gaussian.scale.x) ||
        !std::isfinite(gaussian.scale.y) || !std::isfinite(gaussian.scale.z) ||
        !std::isfinite(gaussian.opacity)) {
      return false;
    }
  }
  return true;
}

}

TEST_CASE("scene format detection covers manifest and extension matrix") {
  const std::filesystem::path dir = MakeTempDir("directxsplat_format_matrix");
  std::filesystem::create_directories(dir / "lod_dir");
  std::filesystem::create_directories(dir / "sog_dir");
  WriteText(dir / "lod_dir" / "lod-meta.json", "{}");
  WriteText(dir / "sog_dir" / "meta.json", "{}");

  struct Case {
    std::string path;
    SceneFormat expected;
  };
  const std::array<Case, 9> cases{{
      {(dir / "scene.PLY").string(), SceneFormat::Ply},
      {(dir / "scene.compressed.ply").string(), SceneFormat::CompressedPly},
      {(dir / "scene.SOG").string(), SceneFormat::Sog},
      {(dir / "scene.spz").string(), SceneFormat::Spz},
      {(dir / "scene.splat").string(), SceneFormat::Splat},
      {(dir / "lod-meta.json").string(), SceneFormat::HierarchicalLod},
      {(dir / "meta.json").string(), SceneFormat::Sog},
      {(dir / "lod_dir").string(), SceneFormat::HierarchicalLod},
      {(dir / "sog_dir").string(), SceneFormat::Sog},
  }};

  for (const Case& testCase : cases) {
    CHECK(DetectSceneFormat(testCase.path) == testCase.expected);
  }
  CHECK(DetectSceneFormat((dir / "scene.txt").string()) == SceneFormat::Unknown);

  std::error_code ec;
  std::filesystem::remove_all(dir, ec);
}

TEST_CASE("directory loading prioritizes scene manifests") {
  const std::filesystem::path dir = MakeTempDir("directxsplat_directory_manifest_priority");

  const std::filesystem::path lodDir = dir / "lod";
  std::filesystem::create_directories(lodDir);
  std::string decoy = BinaryPlyHeader(1u);
  AppendBinaryPlyVertex(decoy, {-10.0f, 0.0f, 0.0f}, {0.1f, 0.1f, 0.1f}, 1.0f, 128u, 128u, 128u);
  WriteText(lodDir / "a.ply", decoy);
  std::string detail = BinaryPlyHeader(1u);
  AppendBinaryPlyVertex(detail, {2.0f, 0.0f, 0.0f}, {0.1f, 0.1f, 0.1f}, 1.0f, 128u, 128u, 128u);
  WriteText(lodDir / "z.ply", detail);
  WriteText(lodDir / "lod-meta.json",
            "{\"filenames\":[\"z.ply\"],\"tree\":{\"lods\":{\"0\":{\"file\":0}}}}");

  auto loaded = LoadSceneFromFile(lodDir.string());
  REQUIRE(loaded.ok());
  REQUIRE(loaded.value.splatSets.size() == 1u);
  REQUIRE(loaded.value.splatSets.front().gaussians.size() == 1u);
  CHECK(loaded.value.splatSets.front().gaussians.front().position.x == doctest::Approx(2.0f));

  const std::filesystem::path sogDir = dir / "sog";
  WriteMinimalSogV2(sogDir, 5.0f);
  WriteText(sogDir / "a.ply", decoy);

  loaded = LoadSceneFromFile(sogDir.string());
  REQUIRE(loaded.ok());
  REQUIRE(loaded.value.splatSets.size() == 1u);
  REQUIRE(loaded.value.splatSets.front().gaussians.size() == 1u);
  CHECK(loaded.value.splatSets.front().gaussians.front().position.x == doctest::Approx(5.0f));

  std::error_code ec;
  std::filesystem::remove_all(dir, ec);
}

TEST_CASE("PLY loader accepts standard ascii point cloud and binary gaussian schemas") {
  const std::filesystem::path dir = MakeTempDir("directxsplat_ply_matrix");

  const std::filesystem::path pointCloud = dir / "pointcloud.ply";
  WriteText(pointCloud,
            "ply\n"
            "format ascii 1.0\n"
            "element vertex 2\n"
            "property float x\n"
            "property float y\n"
            "property float z\n"
            "property uchar red\n"
            "property uchar green\n"
            "property uchar blue\n"
            "end_header\n"
            "0 0 2 255 0 128\n"
            "1 0 3 0 255 64\n");
  auto loaded = LoadSceneFromFile(pointCloud.string());
  REQUIRE(loaded.ok());
  REQUIRE(loaded.value.splatSets.size() == 1u);
  const GaussianSet& pointSet = loaded.value.splatSets.front();
  REQUIRE(pointSet.gaussians.size() == 2u);
  CHECK(pointSet.bounds.valid);
  CHECK(pointSet.gaussians[0].scale.x > 0.0f);
  CHECK(pointSet.gaussians[0].sh[0] == doctest::Approx((1.0f - 0.5f) / kShC0));
  CHECK(pointSet.gaussians[1].sh[16] == doctest::Approx((1.0f - 0.5f) / kShC0));
  CHECK(IsFiniteSet(pointSet));

  const std::filesystem::path binary = dir / "binary.ply";
  std::string bytes = BinaryPlyHeader(2u);
  AppendBinaryPlyVertex(bytes, {1.0f, 2.0f, 3.0f}, {0.2f, 0.3f, 0.4f}, 0.75f, 255u, 128u, 0u);
  AppendBinaryPlyVertex(bytes, {-1.0f, -2.0f, 5.0f}, {1.2f, 1.3f, 1.4f}, 1.25f, 0u, 64u, 255u);
  WriteText(binary, bytes);
  loaded = LoadSceneFromFile(binary.string());
  REQUIRE(loaded.ok());
  REQUIRE(loaded.value.splatSets.size() == 1u);
  const GaussianSet& binarySet = loaded.value.splatSets.front();
  REQUIRE(binarySet.gaussians.size() == 2u);
  CHECK(binarySet.gaussians[0].position.x == doctest::Approx(1.0f));
  CHECK(binarySet.gaussians[0].scale.y == doctest::Approx(0.3f));
  CHECK(binarySet.gaussians[1].position.z == doctest::Approx(5.0f));
  CHECK(binarySet.gaussians[1].opacity == doctest::Approx(1.25f));
  CHECK(IsFiniteSet(binarySet));

  std::error_code ec;
  std::filesystem::remove_all(dir, ec);
}

TEST_CASE("SPLAT loader accepts valid records and rejects invalid record boundaries") {
  const std::filesystem::path dir = MakeTempDir("directxsplat_splat_matrix");
  const std::filesystem::path valid = dir / "valid.splat";
  WriteSplatRecord(valid);

  auto loaded = LoadSceneFromFile(valid.string());
  REQUIRE(loaded.ok());
  REQUIRE(loaded.value.splatSets.size() == 1u);
  const GaussianSet& set = loaded.value.splatSets.front();
  REQUIRE(set.gaussians.size() == 1u);
  CHECK(set.gaussians[0].position.x == doctest::Approx(1.25f));
  CHECK(set.gaussians[0].position.y == doctest::Approx(-2.5f));
  CHECK(set.gaussians[0].scale.z == doctest::Approx(1.25f));
  CHECK(set.bounds.valid);
  CHECK(IsFiniteSet(set));

  const std::filesystem::path invalid = dir / "invalid.splat";
  WriteText(invalid, std::string(33u, '\0'));
  loaded = LoadSceneFromFile(invalid.string());
  CHECK_FALSE(loaded.ok());

  std::error_code ec;
  std::filesystem::remove_all(dir, ec);
}

TEST_CASE("SPZ loader accepts v4 Zstandard attribute streams") {
  const std::filesystem::path dir = MakeTempDir("directxsplat_spz_v4_matrix");
  const std::filesystem::path valid = dir / "valid.spz";
  const std::string bytes = MakeSpzV4Record();
  WriteText(valid, bytes);

  auto loaded = LoadSceneFromFile(valid.string());
  REQUIRE(loaded.ok());
  REQUIRE(loaded.value.splatSets.size() == 1u);
  const GaussianSet& set = loaded.value.splatSets.front();
  REQUIRE(set.gaussians.size() == 1u);
  const Gaussian& gaussian = set.gaussians.front();
  CHECK(gaussian.position.x == doctest::Approx(1.0f));
  CHECK(gaussian.position.y == doctest::Approx(2.0f));
  CHECK(gaussian.position.z == doctest::Approx(3.0f));
  CHECK(gaussian.scale.x == doctest::Approx(1.0f));
  CHECK(gaussian.scale.y == doctest::Approx(std::exp(-1.0f)));
  CHECK(gaussian.scale.z == doctest::Approx(std::exp(1.0f)));
  CHECK(gaussian.rotation.w == doctest::Approx(1.0f));
  CHECK(gaussian.sh[2] == doctest::Approx(-127.0f / 128.0f));
  CHECK(gaussian.sh[18] == doctest::Approx(1.0f));
  CHECK(set.bounds.valid);

  std::string invalidBytes = bytes;
  invalidBytes[15] = 5;
  const std::filesystem::path invalid = dir / "invalid_stream_count.spz";
  WriteText(invalid, invalidBytes);
  loaded = LoadSceneFromFile(invalid.string());
  CHECK_FALSE(loaded.ok());

  std::error_code ec;
  std::filesystem::remove_all(dir, ec);
}

TEST_CASE("SOG loader accepts legacy v1 range-quantized assets") {
  const std::filesystem::path dir = MakeTempDir("directxsplat_sog_v1_matrix");
  WriteText(dir / "meta.json",
            "{"
            "\"means\":{\"shape\":[1,3],\"mins\":[-0.69314718056,0,-1],\"maxs\":[0,1.09861228867,1],"
            "\"files\":[\"means_l.webp\",\"means_u.webp\"]},"
            "\"scales\":{\"mins\":[-2,-1,0],\"maxs\":[0,1,2],\"files\":[\"scales.webp\"]},"
            "\"quats\":{\"files\":[\"quats.webp\"]},"
            "\"sh0\":{\"mins\":[-1,-2,-3,-4],\"maxs\":[1,2,3,4],\"files\":[\"sh0.webp\"]},"
            "\"shN\":{\"mins\":-2,\"maxs\":2,\"files\":[\"shN_centroids.webp\",\"shN_labels.webp\"]}"
            "}");

  WriteRgbaTga(dir / "means_l.webp", 1u, 1u, {0u, 255u, 0u, 255u});
  WriteRgbaTga(dir / "means_u.webp", 1u, 1u, {0u, 255u, 128u, 255u});
  WriteRgbaTga(dir / "scales.webp", 1u, 1u, {0u, 128u, 255u, 255u});
  WriteRgbaTga(dir / "quats.webp", 1u, 1u, {128u, 128u, 128u, 252u});
  WriteRgbaTga(dir / "sh0.webp", 1u, 1u, {0u, 128u, 255u, 191u});
  WriteRgbaTga(dir / "shN_labels.webp", 1u, 1u, {0u, 0u, 0u, 255u});
  std::vector<uint8_t> centroids(192u * 4u, 0u);
  centroids[0] = 0u;
  centroids[1] = 128u;
  centroids[2] = 255u;
  centroids[3] = 255u;
  WriteRgbaTga(dir / "shN_centroids.webp", 192u, 1u, centroids);

  const auto loaded = LoadSceneFromFile((dir / "meta.json").string());
  REQUIRE(loaded.ok());
  REQUIRE(loaded.value.splatSets.size() == 1u);
  REQUIRE(loaded.value.splatSets.front().gaussians.size() == 1u);
  const Gaussian& gaussian = loaded.value.splatSets.front().gaussians.front();
  CHECK(gaussian.position.x == doctest::Approx(-1.0f));
  CHECK(gaussian.position.y == doctest::Approx(2.0f));
  CHECK(gaussian.position.z == doctest::Approx(0.0f).epsilon(0.001));
  CHECK(gaussian.scale.x == doctest::Approx(std::exp(-2.0f)));
  CHECK(gaussian.scale.y == doctest::Approx(std::exp(1.0f / 255.0f)));
  CHECK(gaussian.scale.z == doctest::Approx(std::exp(2.0f)));
  CHECK(gaussian.rotation.w > 0.99f);
  CHECK(gaussian.sh[0] == doctest::Approx(-1.0f));
  CHECK(gaussian.sh[16] == doctest::Approx(2.0f / 255.0f));
  CHECK(gaussian.sh[32] == doctest::Approx(3.0f));
  CHECK(gaussian.opacity == doctest::Approx(508.0f / 255.0f));
  CHECK(gaussian.sh[1] == doctest::Approx(-2.0f));
  CHECK(gaussian.sh[17] == doctest::Approx(2.0f / 255.0f));
  CHECK(gaussian.sh[33] == doctest::Approx(2.0f));
  CHECK(loaded.value.splatSets.front().bounds.valid);
  CHECK(IsFiniteSet(loaded.value.splatSets.front()));

  std::error_code ec;
  std::filesystem::remove_all(dir, ec);
}

TEST_CASE("SOG loader preserves v2 codebook decoding") {
  const std::filesystem::path dir = MakeTempDir("directxsplat_sog_v2_matrix");
  const std::string codebook = MakeLinearCodebook();
  WriteText(dir / "meta.json",
            "{"
            "\"version\":2,\"count\":1,"
            "\"means\":{\"mins\":[-0.69314718056,0,-1],\"maxs\":[0,1.09861228867,1],"
            "\"files\":[\"means_l.webp\",\"means_u.webp\"]},"
            "\"scales\":{\"codebook\":" + codebook + ",\"files\":[\"scales.webp\"]},"
            "\"quats\":{\"files\":[\"quats.webp\"]},"
            "\"sh0\":{\"codebook\":" + codebook + ",\"files\":[\"sh0.webp\"]},"
            "\"shN\":{\"count\":1,\"bands\":1,\"codebook\":" + codebook +
                ",\"files\":[\"shN_centroids.webp\",\"shN_labels.webp\"]}"
            "}");

  WriteRgbaTga(dir / "means_l.webp", 1u, 1u, {0u, 255u, 0u, 255u});
  WriteRgbaTga(dir / "means_u.webp", 1u, 1u, {0u, 255u, 128u, 255u});
  WriteRgbaTga(dir / "scales.webp", 1u, 1u, {0u, 128u, 255u, 255u});
  WriteRgbaTga(dir / "quats.webp", 1u, 1u, {128u, 128u, 128u, 252u});
  WriteRgbaTga(dir / "sh0.webp", 1u, 1u, {0u, 128u, 255u, 191u});
  WriteRgbaTga(dir / "shN_labels.webp", 1u, 1u, {0u, 0u, 0u, 255u});
  std::vector<uint8_t> centroids(192u * 4u, 0u);
  centroids[0] = 0u;
  centroids[1] = 128u;
  centroids[2] = 255u;
  centroids[3] = 255u;
  WriteRgbaTga(dir / "shN_centroids.webp", 192u, 1u, centroids);

  const auto loaded = LoadSceneFromFile((dir / "meta.json").string());
  REQUIRE(loaded.ok());
  REQUIRE(loaded.value.splatSets.size() == 1u);
  REQUIRE(loaded.value.splatSets.front().gaussians.size() == 1u);
  const Gaussian& gaussian = loaded.value.splatSets.front().gaussians.front();
  CHECK(gaussian.position.x == doctest::Approx(-1.0f));
  CHECK(gaussian.position.y == doctest::Approx(2.0f));
  CHECK(gaussian.scale.x == doctest::Approx(1.0f));
  CHECK(gaussian.scale.y == doctest::Approx(std::exp(128.0f / 255.0f)));
  CHECK(gaussian.scale.z == doctest::Approx(std::exp(1.0f)));
  CHECK(gaussian.sh[0] == doctest::Approx(0.0f));
  CHECK(gaussian.sh[16] == doctest::Approx(128.0f / 255.0f));
  CHECK(gaussian.sh[32] == doctest::Approx(1.0f));
  CHECK(gaussian.opacity == doctest::Approx(std::log((191.0f / 255.0f) / (64.0f / 255.0f))));
  CHECK(gaussian.sh[1] == doctest::Approx(0.0f));
  CHECK(gaussian.sh[17] == doctest::Approx(128.0f / 255.0f));
  CHECK(gaussian.sh[33] == doctest::Approx(1.0f));
  CHECK(IsFiniteSet(loaded.value.splatSets.front()));

  std::error_code ec;
  std::filesystem::remove_all(dir, ec);
}

TEST_CASE("LOD loader includes the referenced environment SOG") {
  const std::filesystem::path dir = MakeTempDir("directxsplat_lod_environment_matrix");
  std::string detail = BinaryPlyHeader(1u);
  AppendBinaryPlyVertex(detail, {1.0f, 0.0f, 0.0f}, {0.1f, 0.1f, 0.1f}, 1.0f, 128u, 128u, 128u);
  WriteText(dir / "detail.ply", detail);
  WriteMinimalSogV2(dir / "env", 5.0f);
  WriteText(dir / "lod-meta.json",
            "{"
            "\"version\":1,\"count\":1,\"counts\":[1],\"lodLevels\":1,"
            "\"environment\":\"env/meta.json\",\"filenames\":[\"detail.ply\"],"
            "\"tree\":{\"bound\":{\"min\":[0,-1,-1],\"max\":[2,1,1]},"
            "\"lods\":{\"0\":{\"file\":0,\"offset\":0,\"count\":1}}}"
            "}");

  const auto loaded = LoadSceneFromFile((dir / "lod-meta.json").string());
  REQUIRE(loaded.ok());
  REQUIRE(loaded.value.splatSets.size() == 2u);
  CHECK(loaded.value.splatSets[0].name == "detail");
  CHECK(loaded.value.splatSets[0].gaussians.front().position.x == doctest::Approx(1.0f));
  CHECK(loaded.value.splatSets[1].name == "env");
  REQUIRE(loaded.value.splatSets[1].gaussians.size() == 1u);
  CHECK(loaded.value.splatSets[1].gaussians.front().position.x == doctest::Approx(5.0f));
  CHECK(loaded.value.sceneBounds.valid);
  CHECK(loaded.value.sceneBounds.max.x == doctest::Approx(2.0f));

  std::error_code ec;
  std::filesystem::remove_all(dir, ec);
}

TEST_CASE("raw PLY writer roundtrips through raw and scene loaders") {
  const std::filesystem::path dir = MakeTempDir("directxsplat_ply_writer_roundtrip");
  const std::filesystem::path path = dir / "roundtrip.ply";

  io::ply::PlyFile file{};
  io::ply::PlyElement vertex{};
  vertex.name = "vertex";
  vertex.count = 1u;
  const std::array<const char*, 14> names{{
      "x", "y", "z", "scale_0", "scale_1", "scale_2", "rot_0", "rot_1", "rot_2", "rot_3", "opacity",
      "f_dc_0", "f_dc_1", "f_dc_2",
  }};
  vertex.properties.reserve(names.size());
  vertex.scalarColumns.reserve(names.size());
  for (const char* name : names) {
    io::ply::PlyProperty prop{};
    prop.type = io::ply::PlyScalarType::Float32;
    prop.name = name;
    vertex.properties.push_back(std::move(prop));
    vertex.scalarColumns.push_back({0.0});
  }
  vertex.scalarColumns[0][0] = 1.0;
  vertex.scalarColumns[1][0] = 2.0;
  vertex.scalarColumns[2][0] = 3.0;
  vertex.scalarColumns[3][0] = -2.302585093;
  vertex.scalarColumns[4][0] = -1.609437912;
  vertex.scalarColumns[5][0] = -1.203972804;
  vertex.scalarColumns[6][0] = 1.0;
  vertex.scalarColumns[10][0] = 0.5;
  vertex.scalarColumns[11][0] = 0.1;
  vertex.scalarColumns[12][0] = 0.2;
  vertex.scalarColumns[13][0] = 0.3;
  file.elements.push_back(std::move(vertex));

  REQUIRE(io::ply::WritePlyAscii(file, path.string()).ok);
  auto raw = io::ply::ReadPlyFile(path.string());
  REQUIRE(raw.ok());
  REQUIRE(raw.value.elements.size() == 1u);
  REQUIRE(raw.value.elements.front().scalarColumns.size() == names.size());
  CHECK(raw.value.elements.front().scalarColumns[0][0] == doctest::Approx(1.0));

  auto loaded = LoadSceneFromFile(path.string());
  REQUIRE(loaded.ok());
  REQUIRE(loaded.value.splatSets.size() == 1u);
  REQUIRE(loaded.value.splatSets.front().gaussians.size() == 1u);
  CHECK(loaded.value.splatSets.front().gaussians[0].position.z == doctest::Approx(3.0f));
  CHECK(loaded.value.splatSets.front().gaussians[0].scale.x == doctest::Approx(0.1f).epsilon(0.01));

  std::error_code ec;
  std::filesystem::remove_all(dir, ec);
}

TEST_CASE("raw PLY reader rejects scalar storage above fallback budget") {
  const std::filesystem::path dir = MakeTempDir("directxsplat_raw_ply_budget");
  const std::filesystem::path path = dir / "huge_scalar_budget.ply";
  WriteText(path,
            "ply\n"
            "format ascii 1.0\n"
            "element vertex 33554433\n"
            "property float x\n"
            "end_header\n");

  auto raw = io::ply::ReadPlyFile(path.string());
  CHECK_FALSE(raw.ok());
  CHECK(raw.status.message.find("configured loader limits") != std::string::npos);

  std::error_code ec;
  std::filesystem::remove_all(dir, ec);
}

TEST_CASE("raw PLY reader rejects oversized header token lists") {
  const std::filesystem::path dir = MakeTempDir("directxsplat_raw_ply_header_tokens");
  const std::filesystem::path path = dir / "many_header_tokens.ply";
  std::string line = "unknown";
  for (int i = 0; i < 32; ++i) {
    line += " t";
  }
  WriteText(path,
            "ply\n"
            "format ascii 1.0\n" +
                line +
                "\n"
                "end_header\n");

  auto raw = io::ply::ReadPlyFile(path.string());
  CHECK_FALSE(raw.ok());
  CHECK(raw.status.message == "ply header line has too many tokens");

  const std::filesystem::path commentPath = dir / "long_comment.ply";
  std::string comment = "comment";
  for (int i = 0; i < 32; ++i) {
    comment += " word";
  }
  WriteText(commentPath,
            "ply\n"
            "format ascii 1.0\n" +
                comment +
                "\n"
                "element vertex 0\n"
                "property float x\n"
                "end_header\n");

  raw = io::ply::ReadPlyFile(commentPath.string());
  CHECK(raw.ok());

  std::error_code ec;
  std::filesystem::remove_all(dir, ec);
}

TEST_CASE("SOG metadata schema variants fail through StatusOr instead of exceptions") {
  const std::filesystem::path dir = MakeTempDir("directxsplat_sog_schema_matrix");

  const std::array<std::string, 6> cases{{
      "{}",
      "{\"version\":2,\"count\":\"1\"}",
      "{\"version\":2,\"count\":1,\"means\":[]}",
      "{\"version\":2,\"count\":1,\"means\":{\"files\":\"bad\"}}",
      "{\"version\":2,\"count\":1,\"means\":{\"files\":[\"missing.png\",\"missing.png\"],\"mins\":[0],\"maxs\":[1]}}",
      "{\"version\":2,\"count\":1,\"means\":{\"files\":[\"missing.png\",\"missing.png\"],\"mins\":[0,0,0],\"maxs\":[1,1,1]},\"quats\":null}",
  }};

  for (size_t i = 0; i < cases.size(); ++i) {
    const std::filesystem::path caseDir = dir / ("case_" + std::to_string(i));
    std::filesystem::create_directories(caseDir);
    WriteText(caseDir / "meta.json", cases[i]);
    auto loaded = LoadSceneFromFile((caseDir / "meta.json").string());
    CHECK_FALSE(loaded.ok());
    CHECK_FALSE(loaded.status.message.empty());
  }

  std::error_code ec;
  std::filesystem::remove_all(dir, ec);
}

TEST_CASE("WIC file decoder returns StatusOr errors for hostile paths") {
  const std::string hostilePath("\xff\xfe\xfd", 3);
  const auto decoded = io::DecodeImageFromFileWic(hostilePath);
  CHECK_FALSE(decoded.ok());
}

}  // namespace directxsplat
