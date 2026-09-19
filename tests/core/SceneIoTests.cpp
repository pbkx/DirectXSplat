#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <chrono>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <thread>
#include <string>
#include <vector>

#include "directxsplat/io.h"
#include "directxsplat/scene.h"

namespace directxsplat {
namespace {

std::filesystem::path AssetPath(const char* name) {
  return std::filesystem::path(DIRECTXSPLAT_TEST_ASSET_DIR) / name;
}

std::filesystem::path MakeTempDir(const char* name) {
  const std::filesystem::path dir = std::filesystem::temp_directory_path() / name;
  std::error_code ec;
  std::filesystem::remove_all(dir, ec);
  std::filesystem::create_directories(dir, ec);
  return dir;
}

void WriteFile(const std::filesystem::path& path, const std::string& data) {
  std::ofstream file(path, std::ios::binary);
  file << data;
}

std::string TinyPlyText(float x) {
  return std::string("ply\n")
       + "format ascii 1.0\n"
       + "element vertex 1\n"
       + "property float x\n"
       + "property float y\n"
       + "property float z\n"
       + "property float scale_0\n"
       + "property float scale_1\n"
       + "property float scale_2\n"
       + "property float rot_0\n"
       + "property float rot_1\n"
       + "property float rot_2\n"
       + "property float rot_3\n"
       + "property float opacity\n"
       + "property float f_dc_0\n"
       + "property float f_dc_1\n"
       + "property float f_dc_2\n"
       + "end_header\n"
       + std::to_string(x) + " 0 2 0.1 0.1 0.1 1 0 0 0 1 0 0 0\n";
}

void WriteCompressedPly(const std::filesystem::path& path, uint32_t packedRotation) {
  std::ofstream file(path, std::ios::binary);
  file << "ply\n"
          "format binary_little_endian 1.0\n"
          "element chunk 1\n"
          "property float min_x\n"
          "property float min_y\n"
          "property float min_z\n"
          "property float max_x\n"
          "property float max_y\n"
          "property float max_z\n"
          "property float min_scale_x\n"
          "property float min_scale_y\n"
          "property float min_scale_z\n"
          "property float max_scale_x\n"
          "property float max_scale_y\n"
          "property float max_scale_z\n"
          "property float min_r\n"
          "property float min_g\n"
          "property float min_b\n"
          "property float max_r\n"
          "property float max_g\n"
          "property float max_b\n"
          "element vertex 1\n"
          "property uint packed_position\n"
          "property uint packed_rotation\n"
          "property uint packed_scale\n"
          "property uint packed_color\n"
          "end_header\n";

  const float chunk[] = {
      0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 2.0f,
      std::log(0.1f), std::log(0.01f), std::log(0.001f),
      std::log(0.1f), std::log(0.01f), std::log(0.001f),
      0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f,
  };
  file.write(reinterpret_cast<const char*>(chunk), sizeof(chunk));

  const uint32_t vertex[] = {0u, packedRotation, 0u, 0x808080ffu};
  file.write(reinterpret_cast<const char*>(vertex), sizeof(vertex));
}

bool IsFiniteGaussian(const Gaussian& gaussian) {
  if (!std::isfinite(gaussian.position.x) || !std::isfinite(gaussian.position.y) || !std::isfinite(gaussian.position.z)) {
    return false;
  }
  if (!std::isfinite(gaussian.scale.x) || !std::isfinite(gaussian.scale.y) || !std::isfinite(gaussian.scale.z)) {
    return false;
  }
  if (!std::isfinite(gaussian.opacity)) {
    return false;
  }
  for (float coeff : gaussian.sh) {
    if (!std::isfinite(coeff)) {
      return false;
    }
  }
  return true;
}

}  

TEST_CASE("Scene IO loads tiny ASCII PLY") {
  const auto loaded = LoadSceneFromFile(AssetPath("tiny_ascii.ply").string());
  REQUIRE(loaded.ok());
  REQUIRE(loaded.value.splatSets.size() == 1u);
  const GaussianSet& set = loaded.value.splatSets.front();
  REQUIRE(set.gaussians.size() == 2u);
  CHECK(set.gaussians.front().position.z == doctest::Approx(2.0f));
  CHECK(set.gaussians.back().position.x == doctest::Approx(1.0f));
  for (const Gaussian& gaussian : set.gaussians) {
    CHECK(IsFiniteGaussian(gaussian));
    CHECK(gaussian.scale.x > 0.0f);
    CHECK(gaussian.scale.y > 0.0f);
    CHECK(gaussian.scale.z > 0.0f);
  }
}

TEST_CASE("Scene IO preserves compressed PLY quaternion convention") {
  const std::filesystem::path dir = MakeTempDir("directxsplat_compressed_rotation");
  const std::filesystem::path path = dir / "rotation.compressed.ply";

  // Largest component is rot_1, representing the wxyz quaternion (0, 1, 0, 0).
  constexpr uint32_t packedRotation = (1u << 30u) | (512u << 20u) | (512u << 10u) | 512u;
  WriteCompressedPly(path, packedRotation);

  const auto loaded = LoadSceneFromFile(path.string());
  REQUIRE(loaded.ok());
  REQUIRE(loaded.value.splatSets.size() == 1u);
  REQUIRE(loaded.value.splatSets.front().gaussians.size() == 1u);

  const Gaussian& gaussian = loaded.value.splatSets.front().gaussians.front();
  CHECK(gaussian.rotation.x == doctest::Approx(1.0f).epsilon(0.002f));
  CHECK(std::abs(gaussian.rotation.y) < 0.002f);
  CHECK(std::abs(gaussian.rotation.z) < 0.002f);
  CHECK(std::abs(gaussian.rotation.w) < 0.002f);
  CHECK(gaussian.scale.x == doctest::Approx(0.1f));
  CHECK(gaussian.scale.y == doctest::Approx(0.01f));
  CHECK(gaussian.scale.z == doctest::Approx(0.001f));

  std::error_code ec;
  std::filesystem::remove_all(dir, ec);
}

TEST_CASE("Scene IO rejects empty path") {
  const auto loaded = LoadSceneFromFile("");
  CHECK_FALSE(loaded.ok());
  CHECK(loaded.status.message == "scene path is empty");
}

TEST_CASE("Scene IO malformed PLY fails cleanly") {
  const auto loaded = LoadSceneFromFile(AssetPath("tiny_bad.ply").string());
  CHECK_FALSE(loaded.ok());
  CHECK_FALSE(loaded.status.message.empty());
}

TEST_CASE("Scene IO nonexistent file fails cleanly") {
  const auto loaded = LoadSceneFromFile((AssetPath("missing_scene.ply")).string());
  CHECK_FALSE(loaded.ok());
  CHECK_FALSE(loaded.status.message.empty());
}

TEST_CASE("Scene IO unsupported extension fails cleanly") {
  const std::filesystem::path tempPath = std::filesystem::temp_directory_path() / "directxsplat_unsupported_scene.txt";
  {
    std::ofstream file(tempPath, std::ios::binary);
    file << "unsupported";
  }
  const auto loaded = LoadSceneFromFile(tempPath.string());
  CHECK_FALSE(loaded.ok());
  CHECK_FALSE(loaded.status.message.empty());
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);
}

TEST_CASE("Scene IO applies source image directory load option") {
  const std::filesystem::path dir = MakeTempDir("directxsplat_scene_source_image_dir");
  const std::filesystem::path scenePath = dir / "scene.ply";
  const std::filesystem::path imageDir = dir / "images";
  std::filesystem::create_directories(imageDir);
  WriteFile(scenePath, TinyPlyText(0.0f));
  WriteFile(dir / "cameras.json",
            "[{"
            "\"name\":\"direct camera\","
            "\"position\":[1,2,3],"
            "\"rotation\":[0,0,0,1],"
            "\"fovY\":1.0471975512,"
            "\"image\":\"frame.png\""
            "}]");

  SceneLoadOptions options{};
  options.sourceImageDirectory = imageDir.string();
  const auto loaded = LoadSceneFromFile(scenePath.string(), options);

  REQUIRE(loaded.ok());
  REQUIRE(loaded.value.inputCameras.size() == 1u);
  const std::filesystem::path sourceImage(loaded.value.inputCameras.front().sourceImage);
  CHECK(sourceImage.filename() == "frame.png");
  CHECK(sourceImage.parent_path().filename() == "images");

  std::error_code ec;
  std::filesystem::remove_all(dir, ec);
}

TEST_CASE("Scene IO rejects malformed and hostile PLY inputs") {
  const std::filesystem::path dir = MakeTempDir("directxsplat_ply_regressions");

  const std::filesystem::path fractionalList = dir / "fractional_list.ply";
  WriteFile(fractionalList,
            "ply\n"
            "format ascii 1.0\n"
            "element vertex 1\n"
            "property list uchar float bad\n"
            "end_header\n"
            "1.5 0\n");
  auto loaded = LoadSceneFromFile(fractionalList.string());
  CHECK_FALSE(loaded.ok());

  const std::filesystem::path floatCountList = dir / "float_count_list.ply";
  WriteFile(floatCountList,
            "ply\n"
            "format ascii 1.0\n"
            "element vertex 1\n"
            "property list float float bad\n"
            "end_header\n"
            "1 0\n");
  loaded = LoadSceneFromFile(floatCountList.string());
  CHECK_FALSE(loaded.ok());

  const std::filesystem::path hugeCount = dir / "huge_count.ply";
  WriteFile(hugeCount,
            "ply\n"
            "format ascii 1.0\n"
            "element vertex 4294967295\n"
            "property float x\n"
            "end_header\n");
  loaded = LoadSceneFromFile(hugeCount.string());
  CHECK_FALSE(loaded.ok());

  const std::filesystem::path hugeToken = dir / "huge_token.ply";
  WriteFile(hugeToken,
            std::string("ply\n"
                        "format ascii 1.0\n"
                        "element vertex 1\n"
                        "property float x\n"
                        "end_header\n") +
                std::string(5000, '1') + "\n");
  loaded = LoadSceneFromFile(hugeToken.string());
  CHECK_FALSE(loaded.ok());

  const std::filesystem::path fastHugeHeaderLine = dir / "fast_huge_header_line.ply";
  WriteFile(fastHugeHeaderLine, std::string("ply\n") + std::string(1024 * 1024 + 1, 'x') + "\n");
  loaded = LoadSceneFromFile(fastHugeHeaderLine.string());
  CHECK_FALSE(loaded.ok());
  CHECK(loaded.status.message == "ply header line too large");

  const std::filesystem::path truncatedBinary = dir / "truncated_binary.ply";
  {
    std::ofstream file(truncatedBinary, std::ios::binary);
    file << "ply\n"
         << "format binary_little_endian 1.0\n"
         << "element vertex 1\n"
         << "property float x\n"
         << "end_header\n";
    file.put('\0');
  }
  loaded = LoadSceneFromFile(truncatedBinary.string());
  CHECK_FALSE(loaded.ok());

  const std::filesystem::path fastHugeVertex = dir / "fast_huge_vertex_binary.ply";
  WriteFile(fastHugeVertex,
            "ply\n"
            "format binary_little_endian 1.0\n"
            "element vertex 67108865\n"
            "property float x\n"
            "property float y\n"
            "property float z\n"
            "property float scale_x\n"
            "property float scale_y\n"
            "property float scale_z\n"
            "property float rot_0\n"
            "property float rot_1\n"
            "property float rot_2\n"
            "property float rot_3\n"
            "property float opacity\n"
            "property uchar red\n"
            "property uchar green\n"
            "property uchar blue\n"
            "end_header\n");
  loaded = LoadSceneFromFile(fastHugeVertex.string());
  CHECK_FALSE(loaded.ok());
  CHECK(loaded.status.message == "ply element count too large");

  const std::filesystem::path fastExpandedBudget = dir / "fast_expanded_budget_binary.ply";
  WriteFile(fastExpandedBudget,
            "ply\n"
            "format binary_little_endian 1.0\n"
            "element vertex 20000000\n"
            "property float x\n"
            "property float y\n"
            "property float z\n"
            "property float scale_x\n"
            "property float scale_y\n"
            "property float scale_z\n"
            "property float rot_0\n"
            "property float rot_1\n"
            "property float rot_2\n"
            "property float rot_3\n"
            "property float opacity\n"
            "property uchar red\n"
            "property uchar green\n"
            "property uchar blue\n"
            "end_header\n");
  loaded = LoadSceneFromFile(fastExpandedBudget.string());
  CHECK_FALSE(loaded.ok());
  CHECK(loaded.status.message == "ply expanded data too large");

  const std::filesystem::path fastHugeChunk = dir / "fast_huge_chunk_binary.ply";
  WriteFile(fastHugeChunk,
            "ply\n"
            "format binary_little_endian 1.0\n"
            "element chunk 4194305\n"
            "property float min_x\n"
            "property float min_y\n"
            "property float min_z\n"
            "property float max_x\n"
            "property float max_y\n"
            "property float max_z\n"
            "property float min_scale_x\n"
            "property float min_scale_y\n"
            "property float min_scale_z\n"
            "property float max_scale_x\n"
            "property float max_scale_y\n"
            "property float max_scale_z\n"
            "element vertex 1\n"
            "property uint packed_position\n"
            "property uint packed_rotation\n"
            "property uint packed_scale\n"
            "property uint packed_color\n"
            "end_header\n");
  loaded = LoadSceneFromFile(fastHugeChunk.string());
  CHECK_FALSE(loaded.ok());
  CHECK(loaded.status.message == "ply element count too large");

  std::error_code ec;
  std::filesystem::remove_all(dir, ec);
}

TEST_CASE("Scene IO rejects hostile SOG and LOD metadata paths") {
  const std::filesystem::path dir = MakeTempDir("directxsplat_metadata_regressions");
  WriteFile(dir / "outside.ply", TinyPlyText(3.0f));

  const std::filesystem::path lodDir = dir / "lod";
  std::filesystem::create_directories(lodDir);
  WriteFile(lodDir / "lod-meta.json",
            "{"
            "\"filenames\":[\"../outside.ply\"],"
            "\"tree\":{\"lods\":{\"0\":{\"file\":0}}}"
            "}");
  auto loaded = LoadSceneFromFile((lodDir / "lod-meta.json").string());
  CHECK_FALSE(loaded.ok());

  const std::filesystem::path invalidJsonDir = dir / "invalid_json";
  std::filesystem::create_directories(invalidJsonDir);
  WriteFile(invalidJsonDir / "lod-meta.json", "{\"filenames\":[");
  loaded = LoadSceneFromFile((invalidJsonDir / "lod-meta.json").string());
  CHECK_FALSE(loaded.ok());

  const std::filesystem::path deepDir = dir / "deep";
  std::filesystem::create_directories(deepDir);
  std::string deepJson = "{\"filenames\":[\"chunk.ply\"],\"tree\":";
  for (int i = 0; i < 300; ++i) {
    deepJson += "{\"children\":[";
  }
  deepJson += "{\"lods\":{\"0\":{\"file\":0}}}";
  for (int i = 0; i < 300; ++i) {
    deepJson += "]}";
  }
  deepJson += "}";
  WriteFile(deepDir / "lod-meta.json", deepJson);
  loaded = LoadSceneFromFile((deepDir / "lod-meta.json").string());
  CHECK_FALSE(loaded.ok());

  const std::filesystem::path sogDir = dir / "sog";
  std::filesystem::create_directories(sogDir);
  WriteFile(sogDir / "meta.json",
            "{"
            "\"count\":1,"
            "\"means\":{\"files\":[\"../outside.png\",\"inside.png\"],\"mins\":[0,0,0],\"maxs\":[1,1,1]},"
            "\"quats\":{\"files\":[\"inside.png\"]},"
            "\"scales\":{\"files\":[\"inside.png\"],\"codebook\":[0]},"
            "\"sh0\":{\"files\":[\"inside.png\"],\"codebook\":[0]}"
            "}");
  loaded = LoadSceneFromFile((sogDir / "meta.json").string());
  CHECK_FALSE(loaded.ok());

  const std::filesystem::path hugeSogDir = dir / "huge_sog";
  std::filesystem::create_directories(hugeSogDir);
  WriteFile(hugeSogDir / "meta.json", "{\"count\":10000001}");
  loaded = LoadSceneFromFile((hugeSogDir / "meta.json").string());
  CHECK_FALSE(loaded.ok());
  CHECK(loaded.status.message == "sog scene is too large");

  const std::filesystem::path badZip = dir / "bad.sog";
  WriteFile(badZip, "not a zip");
  loaded = LoadSceneFromFile(badZip.string());
  CHECK_FALSE(loaded.ok());

  std::error_code ec;
  std::filesystem::remove_all(dir, ec);
}

TEST_CASE("Scene IO rejects malformed SPZ and SPLAT inputs") {
  const std::filesystem::path dir = MakeTempDir("directxsplat_binary_loader_regressions");

  const std::filesystem::path spz = dir / "truncated.spz";
  WriteFile(spz, std::string("\x1f\x8b\x08", 3));
  auto loaded = LoadSceneFromFile(spz.string());
  CHECK_FALSE(loaded.ok());

  const std::filesystem::path hugeSpz = dir / "huge_expanded.spz";
  const unsigned char hugeSpzData[] = {0x1f, 0x8b, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0xf3,
                                       0x73, 0x0f, 0x0e, 0x60, 0x64, 0x60, 0x60, 0x68, 0x98, 0x36, 0x83,
                                       0x01, 0x04, 0x00, 0xc9, 0xf8, 0xee, 0x54, 0x10, 0x00, 0x00, 0x00};
  WriteFile(hugeSpz, std::string(reinterpret_cast<const char*>(hugeSpzData), sizeof(hugeSpzData)));
  loaded = LoadSceneFromFile(hugeSpz.string());
  CHECK_FALSE(loaded.ok());
  CHECK(loaded.status.message == "spz scene is too large");

  const std::filesystem::path splat = dir / "bad.splat";
  WriteFile(splat, std::string(31, '\0'));
  loaded = LoadSceneFromFile(splat.string());
  CHECK_FALSE(loaded.ok());

  const std::filesystem::path hugeSplat = dir / "huge_expanded.splat";
  {
    std::ofstream file(hugeSplat, std::ios::binary);
  }
  constexpr uint64_t splatExpandedBudget = 2ull * 1024ull * 1024ull * 1024ull;
  constexpr uint64_t hugeSplatBytes = ((splatExpandedBudget / sizeof(Gaussian)) + 1ull) * 32ull;
  std::error_code resizeError;
  std::filesystem::resize_file(hugeSplat, hugeSplatBytes, resizeError);
  REQUIRE_FALSE(resizeError);
  loaded = LoadSceneFromFile(hugeSplat.string());
  CHECK_FALSE(loaded.ok());
  CHECK(loaded.status.message == "splat file is too large");

  std::error_code ec;
  std::filesystem::remove_all(dir, ec);
}

TEST_CASE("DetectSceneFormat and BackgroundSceneLoader are race-tolerant") {
  const std::string invalidPath(1, '\0');
  CHECK(DetectSceneFormat(invalidPath) == SceneFormat::Unknown);

  const std::filesystem::path dir = MakeTempDir("directxsplat_background_loader_regressions");
  WriteFile(dir / "a.ply", TinyPlyText(0.0f));
  WriteFile(dir / "b.ply", TinyPlyText(1.0f));

  BackgroundSceneLoader loader;
  REQUIRE(loader.Initialize(dir.string()).ok);
  CHECK(loader.SceneCount() == 2u);
  const std::vector<std::string> paths = loader.ScenePaths();
  REQUIRE(paths.size() == 2u);

  for (int i = 0; i < 64; ++i) {
    loader.RequestLoad(0);
    loader.RequestLoad(1);
    loader.RequestLoad(9999);
  }

  bool sawLoaded = false;
  for (int attempt = 0; attempt < 100 && !sawLoaded; ++attempt) {
    size_t index = 0;
    Scene scene;
    std::string error;
    while (loader.PollLoaded(index, scene, error)) {
      CHECK(index < 2u);
      if (error.empty()) {
        CHECK_FALSE(scene.splatSets.empty());
        sawLoaded = true;
      }
    }
    if (!sawLoaded) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  CHECK(sawLoaded);

  std::atomic<bool> keepRequesting{true};
  std::vector<std::thread> threads;
  for (int t = 0; t < 4; ++t) {
    threads.emplace_back([&]() {
      while (keepRequesting.load(std::memory_order_acquire)) {
        loader.RequestLoad(0);
        loader.RequestLoad(1);
        loader.RequestLoad(9999);
      }
    });
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  loader.Shutdown();
  keepRequesting.store(false, std::memory_order_release);
  for (std::thread& thread : threads) {
    thread.join();
  }
  CHECK(loader.SceneCount() == 2u);
  REQUIRE(loader.Initialize(dir.string()).ok);
  loader.Shutdown();

  std::error_code ec;
  std::filesystem::remove_all(dir, ec);
}

}  // namespace directxsplat
