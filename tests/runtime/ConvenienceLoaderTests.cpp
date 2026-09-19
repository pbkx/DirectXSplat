#include <doctest/doctest.h>

#include <array>
#include <bit>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>

#include <directxsplat/directxsplat.h>

namespace {

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

std::filesystem::path WriteTinyPly(const std::filesystem::path& dir) {
  const std::filesystem::path path = dir / "scene.ply";
  WriteFile(path,
            "ply\n"
            "format ascii 1.0\n"
            "element vertex 1\n"
            "property float x\n"
            "property float y\n"
            "property float z\n"
            "property float scale_0\n"
            "property float scale_1\n"
            "property float scale_2\n"
            "property float rot_0\n"
            "property float rot_1\n"
            "property float rot_2\n"
            "property float rot_3\n"
            "property float opacity\n"
            "property float f_dc_0\n"
            "property float f_dc_1\n"
            "property float f_dc_2\n"
            "end_header\n"
            "0 0 2 0.1 0.1 0.1 1 0 0 0 1 0 0 0\n");
  return path;
}

std::filesystem::path WriteSplatRecord(const std::filesystem::path& dir) {
  const std::filesystem::path path = dir / "scene.splat";
  std::string bytes;
  bytes.resize(32u, '\0');
  auto putFloat = [&](size_t offset, float value) {
    const uint32_t bits = std::bit_cast<uint32_t>(value);
    bytes[offset + 0u] = static_cast<char>(bits & 0xFFu);
    bytes[offset + 1u] = static_cast<char>((bits >> 8u) & 0xFFu);
    bytes[offset + 2u] = static_cast<char>((bits >> 16u) & 0xFFu);
    bytes[offset + 3u] = static_cast<char>((bits >> 24u) & 0xFFu);
  };
  putFloat(0u, 1.0f);
  putFloat(4u, 2.0f);
  putFloat(8u, 3.0f);
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
  WriteFile(path, bytes);
  return path;
}

}  // namespace

TEST_CASE("LoadFromPly rejects empty path") {
  const auto loaded = directxsplat::LoadFromPly({});

  CHECK_FALSE(loaded.ok());
  CHECK(loaded.status.message == "scene path is empty");
}

TEST_CASE("LoadFromPly rejects .txt path") {
  const auto loaded = directxsplat::LoadFromPly("scene.txt");

  CHECK_FALSE(loaded.ok());
  CHECK(loaded.status.message.find("expected .ply") != std::string::npos);
}

TEST_CASE("LoadFromPly loads tiny PLY") {
  const std::filesystem::path dir = MakeTempDir("directxsplat_convenience_load_ply");

  const auto loaded = directxsplat::LoadFromPly(WriteTinyPly(dir));

  REQUIRE(loaded.ok());
  CHECK_FALSE(loaded.value.Empty());
  CHECK(loaded.value.Size() == 1u);

  std::error_code ec;
  std::filesystem::remove_all(dir, ec);
}

TEST_CASE("LoadFromFile routes by scene format") {
  const std::filesystem::path dir = MakeTempDir("directxsplat_convenience_load_by_format");

  const auto ply = directxsplat::LoadFromFile(WriteTinyPly(dir));
  const auto splat = directxsplat::LoadFromFile(WriteSplatRecord(dir));

  REQUIRE(ply.ok());
  REQUIRE(splat.ok());
  CHECK(ply.value.Size() == 1u);
  CHECK(splat.value.Size() == 1u);

  std::error_code ec;
  std::filesystem::remove_all(dir, ec);
}

TEST_CASE("LoadCameraSet reads camera json") {
  const std::filesystem::path dir = MakeTempDir("directxsplat_convenience_camera_json");
  const std::filesystem::path path = dir / "cameras.json";
  WriteFile(path,
            "["
            "{"
            "\"name\":\"camera 0\","
            "\"extrinsic\":[[1,2,3,4],[5,6,7,8],[9,10,11,12],[13,14,15,16]],"
            "\"intrinsic\":[[101,0,320],[0,202,240],[0,0,1]],"
            "\"width\":640,"
            "\"height\":480"
            "},"
            "{"
            "\"name\":\"camera 1\","
            "\"extrinsic\":[[17,18,19,20],[21,22,23,24],[25,26,27,28],[29,30,31,32]],"
            "\"intrinsic\":[[303,0,400],[0,404,300],[0,0,1]],"
            "\"width\":800,"
            "\"height\":600"
            "}"
            "]");

  const auto loaded = directxsplat::LoadCameraSet(path);

  REQUIRE(loaded.ok());
  REQUIRE(loaded.value.cameras.size() == 2u);
  const std::array<float, 16> expectedExtrinsic0{
      1.0f, 2.0f, 3.0f, 4.0f,
      5.0f, 6.0f, 7.0f, 8.0f,
      9.0f, 10.0f, 11.0f, 12.0f,
      13.0f, 14.0f, 15.0f, 16.0f,
  };
  const std::array<float, 9> expectedIntrinsic0{
      101.0f, 0.0f, 320.0f,
      0.0f, 202.0f, 240.0f,
      0.0f, 0.0f, 1.0f,
  };
  const std::array<float, 16> expectedExtrinsic1{
      17.0f, 18.0f, 19.0f, 20.0f,
      21.0f, 22.0f, 23.0f, 24.0f,
      25.0f, 26.0f, 27.0f, 28.0f,
      29.0f, 30.0f, 31.0f, 32.0f,
  };
  const std::array<float, 9> expectedIntrinsic1{
      303.0f, 0.0f, 400.0f,
      0.0f, 404.0f, 300.0f,
      0.0f, 0.0f, 1.0f,
  };

  CHECK(loaded.value.cameras[0].name == "camera 0");
  CHECK(loaded.value.cameras[0].width == 640);
  CHECK(loaded.value.cameras[0].height == 480);
  CHECK(loaded.value.cameras[1].width == 800);
  CHECK(loaded.value.cameras[1].height == 600);
  for (size_t index = 0; index < expectedExtrinsic0.size(); ++index) {
    CHECK(loaded.value.cameras[0].extrinsic[index] == doctest::Approx(expectedExtrinsic0[index]));
    CHECK(loaded.value.cameras[1].extrinsic[index] == doctest::Approx(expectedExtrinsic1[index]));
  }
  for (size_t index = 0; index < expectedIntrinsic0.size(); ++index) {
    CHECK(loaded.value.cameras[0].intrinsic[index] == doctest::Approx(expectedIntrinsic0[index]));
    CHECK(loaded.value.cameras[1].intrinsic[index] == doctest::Approx(expectedIntrinsic1[index]));
  }

  std::error_code ec;
  std::filesystem::remove_all(dir, ec);
}

TEST_CASE("LoadCameraSet rejects malformed matrices") {
  const std::filesystem::path dir = MakeTempDir("directxsplat_convenience_bad_camera_json");
  const std::filesystem::path path = dir / "cameras.json";
  WriteFile(path,
            "[{"
            "\"extrinsic\":[[1]],"
            "\"intrinsic\":[[1,0,0],[0,1,0],[0,0,1]],"
            "\"width\":1,"
            "\"height\":1"
            "}]");

  const auto loaded = directxsplat::LoadCameraSet(path);

  CHECK_FALSE(loaded.ok());
  CHECK(loaded.status.message.find("invalid camera matrix") != std::string::npos);

  std::error_code ec;
  std::filesystem::remove_all(dir, ec);
}

TEST_CASE("LoadCameraSet reads position rotation camera json") {
  const std::filesystem::path dir = MakeTempDir("directxsplat_convenience_position_camera_json");
  const std::filesystem::path path = dir / "cameras.json";
  WriteFile(path,
            "[{"
            "\"img_name\":\"garden camera\","
            "\"width\":800,"
            "\"height\":600,"
            "\"position\":[1,2,3],"
            "\"rotation\":[[0,-1,0],[1,0,0],[0,0,1]],"
            "\"fx\":100,"
            "\"fy\":200"
            "}]");

  const auto loaded = directxsplat::LoadCameraSet(path);

  REQUIRE(loaded.ok());
  REQUIRE(loaded.value.cameras.size() == 1u);
  const directxsplat::CameraParams& camera = loaded.value.cameras.front();
  const std::array<float, 16> expectedExtrinsic{
      0.0f, 1.0f, 0.0f, -2.0f,
      -1.0f, 0.0f, 0.0f, 1.0f,
      0.0f, 0.0f, 1.0f, -3.0f,
      0.0f, 0.0f, 0.0f, 1.0f,
  };
  const std::array<float, 9> expectedIntrinsic{
      100.0f, 0.0f, 400.0f,
      0.0f, 200.0f, 300.0f,
      0.0f, 0.0f, 1.0f,
  };

  CHECK(camera.name == "garden camera");
  CHECK(camera.width == 800);
  CHECK(camera.height == 600);
  for (size_t index = 0; index < expectedExtrinsic.size(); ++index) {
    CHECK(camera.extrinsic[index] == doctest::Approx(expectedExtrinsic[index]));
  }
  for (size_t index = 0; index < expectedIntrinsic.size(); ++index) {
    CHECK(camera.intrinsic[index] == doctest::Approx(expectedIntrinsic[index]));
  }

  std::error_code ec;
  std::filesystem::remove_all(dir, ec);
}

TEST_CASE("LoadCameraSet reads DirectXSplat camera json") {
  const std::filesystem::path dir = MakeTempDir("directxsplat_convenience_direct_camera_json");
  const std::filesystem::path path = dir / "cameras.json";
  WriteFile(path,
            "[{"
            "\"name\":\"direct camera\","
            "\"position\":[1,2,3],"
            "\"rotation\":[0,0,0,1],"
            "\"fovY\":1.0471975512"
            "}]");

  const auto loaded = directxsplat::LoadCameraSet(path);

  REQUIRE(loaded.ok());
  REQUIRE(loaded.value.cameras.size() == 1u);
  const directxsplat::CameraParams& camera = loaded.value.cameras.front();
  CHECK(camera.name == "direct camera");
  CHECK(camera.width == 1600);
  CHECK(camera.height == 900);
  CHECK(camera.extrinsic[0] == doctest::Approx(1.0f));
  CHECK(camera.extrinsic[3] == doctest::Approx(-1.0f));
  CHECK(camera.extrinsic[5] == doctest::Approx(-1.0f));
  CHECK(camera.extrinsic[7] == doctest::Approx(2.0f));
  CHECK(camera.extrinsic[10] == doctest::Approx(1.0f));
  CHECK(camera.extrinsic[11] == doctest::Approx(-3.0f));
  CHECK(camera.intrinsic[0] == doctest::Approx(camera.intrinsic[4]));
  CHECK(camera.intrinsic[2] == doctest::Approx(800.0f));
  CHECK(camera.intrinsic[5] == doctest::Approx(450.0f));
  CHECK(camera.intrinsic[8] == doctest::Approx(1.0f));

  std::error_code ec;
  std::filesystem::remove_all(dir, ec);
}

TEST_CASE("LoadCameraSet rejects invalid DirectXSplat camera render parameters") {
  const std::filesystem::path dir = MakeTempDir("directxsplat_convenience_invalid_direct_camera_json");
  const std::filesystem::path path = dir / "cameras.json";

  struct Case {
    const char* json;
    const char* expectedError;
  };
  const std::array<Case, 4> cases{{
      {"[{\"width\":0,\"height\":900}]", "camera width must be greater than zero"},
      {"[{\"width\":1600,\"height\":0}]", "camera height must be greater than zero"},
      {"[{\"width\":1048577,\"height\":900}]", "invalid camera dimensions"},
      {"[{\"position\":[1e39,0,0]}]", "invalid camera matrix"},
  }};

  for (const Case& testCase : cases) {
    WriteFile(path, testCase.json);
    const auto loaded = directxsplat::LoadCameraSet(path);
    CHECK_FALSE(loaded.ok());
    CHECK(loaded.status.message == testCase.expectedError);
  }

  std::error_code ec;
  std::filesystem::remove_all(dir, ec);
}

TEST_CASE("MakeOrbitCameraSet returns empty for empty splats") {
  const directxsplat::GaussianSplats splats;
  const directxsplat::CameraSet cameras = directxsplat::MakeOrbitCameraSet(splats, 4, 1600, 900);

  CHECK(cameras.cameras.empty());
}
