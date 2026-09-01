#include <doctest/doctest.h>

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <string>
#include <vector>

#include "platform/Image.h"
#include "metrics/ImageMetrics.h"
#include "tools/CliOptions.h"
#include "tools/ScenePathValidation.h"

namespace directxsplat {
namespace {

internal::ImageRgba8 MakeImage(uint32_t width, uint32_t height, const std::vector<uint8_t>& pixels) {
  internal::ImageRgba8 image{};
  image.width = width;
  image.height = height;
  image.pixels = pixels;
  return image;
}

}

TEST_CASE("CLI parser covers option and positional matrix") {
  auto parsed = internal::ParseCliOptions({"--help"});
  REQUIRE(parsed.ok());
  CHECK(parsed.value.showHelp);
  CHECK_FALSE(parsed.value.enableDebugLayer);
  CHECK_FALSE(parsed.value.scenePath.has_value());

  parsed = internal::ParseCliOptions({"--images-path", "images", "--scene-folder", "scenes", "--render-size", "320x240",
                                      "--debug-layer", "botanical", "garden.ply"});
  REQUIRE(parsed.ok());
  REQUIRE(parsed.value.imagePathOverride.has_value());
  REQUIRE(parsed.value.folderTraversalPath.has_value());
  REQUIRE(parsed.value.renderWidthOverride.has_value());
  REQUIRE(parsed.value.renderHeightOverride.has_value());
  REQUIRE(parsed.value.scenePath.has_value());
  CHECK(parsed.value.enableDebugLayer);
  CHECK(*parsed.value.imagePathOverride == "images");
  CHECK(*parsed.value.folderTraversalPath == "scenes");
  CHECK(*parsed.value.renderWidthOverride == 320u);
  CHECK(*parsed.value.renderHeightOverride == 240u);
  CHECK(*parsed.value.scenePath == "botanical garden.ply");

  parsed = internal::ParseCliOptions({"--render-size", "320"});
  CHECK_FALSE(parsed.ok());
  parsed = internal::ParseCliOptions({"--render-size", "0x240"});
  CHECK_FALSE(parsed.ok());
  parsed = internal::ParseCliOptions({"--force-aspect", "1.777"});
  CHECK_FALSE(parsed.ok());
  parsed = internal::ParseCliOptions({"--images-path"});
  CHECK_FALSE(parsed.ok());
}

TEST_CASE("dropped scene path validation covers supported formats") {
  auto empty = internal::ValidateDroppedScenePath({});
  CHECK_FALSE(empty.ok());
  CHECK(empty.status.message == "dropped scene path is empty");

  std::error_code ec;
  const std::filesystem::path dir = std::filesystem::temp_directory_path() / "directxsplat_dropped_scene_directory";
  std::filesystem::remove_all(dir, ec);
  std::filesystem::create_directories(dir, ec);
  REQUIRE_FALSE(ec);

  auto directory = internal::ValidateDroppedScenePath(dir);
  CHECK_FALSE(directory.ok());
  CHECK(directory.status.message == "dropped path is not a supported scene file");

  for (const std::filesystem::path path : {"scene.ply", "scene.spz", "scene.sog", "scene.splat", "lod-meta.json"}) {
    auto valid = internal::ValidateDroppedScenePath(path);
    CAPTURE(path.string());
    REQUIRE(valid.ok());
    CHECK(valid.value == path);
  }

  std::filesystem::remove_all(dir, ec);
}

TEST_CASE("image metrics match deterministic reference values") {
  const internal::ImageRgba8 a = MakeImage(2u, 1u, {
      0u, 0u, 0u, 255u,
      255u, 255u, 255u, 255u,
  });
  const internal::ImageRgba8 b = MakeImage(2u, 1u, {
      0u, 0u, 0u, 255u,
      255u, 0u, 255u, 255u,
  });

  const ImageComparison same = CompareImages(a, a);
  CHECK(same.mae == doctest::Approx(0.0));
  CHECK(same.mse == doctest::Approx(0.0));
  CHECK(same.psnr == doctest::Approx(120.0));
  CHECK(same.flipLike == doctest::Approx(0.0));

  const ImageComparison diff = CompareImages(a, b);
  CHECK(diff.mae == doctest::Approx(1.0 / 6.0));
  CHECK(diff.mse == doctest::Approx(1.0 / 6.0));
  CHECK(diff.psnr == doctest::Approx(10.0 * std::log10(6.0)));
  CHECK(diff.flipLike == doctest::Approx(1.0 / 6.0));

  const internal::ImageRgba8 diffImage = BuildDiffImage(a, b);
  REQUIRE(diffImage.width == 2u);
  REQUIRE(diffImage.height == 1u);
  REQUIRE(diffImage.pixels.size() == 8u);
  CHECK(diffImage.pixels[0] == 0u);
  CHECK(diffImage.pixels[1] == 0u);
  CHECK(diffImage.pixels[2] == 0u);
  CHECK(diffImage.pixels[3] == 255u);
  CHECK(diffImage.pixels[4] == 0u);
  CHECK(diffImage.pixels[5] == 255u);
  CHECK(diffImage.pixels[6] == 0u);
  CHECK(diffImage.pixels[7] == 255u);

  const internal::ImageRgba8 wrongSize = MakeImage(1u, 1u, {0u, 0u, 0u, 255u});
  const ImageComparison invalid = CompareImages(a, wrongSize);
  CHECK(std::isinf(invalid.mae));
  CHECK(std::isinf(invalid.mse));
  CHECK(std::isinf(invalid.flipLike));
  CHECK(invalid.psnr == doctest::Approx(0.0));
  CHECK(BuildDiffImage(a, wrongSize).Empty());

  const internal::ImageRgba8 shortPixels = MakeImage(2u, 1u, {0u, 0u, 0u, 255u});
  const ImageComparison shortInvalid = CompareImages(a, shortPixels);
  CHECK(std::isinf(shortInvalid.mae));
  CHECK(std::isinf(shortInvalid.mse));
  CHECK(std::isinf(shortInvalid.flipLike));
  CHECK(shortInvalid.psnr == doctest::Approx(0.0));
  CHECK(BuildDiffImage(a, shortPixels).Empty());

  const internal::ImageRgba8 overflowHeader =
      MakeImage(std::numeric_limits<uint32_t>::max(), std::numeric_limits<uint32_t>::max(), {255u});
  const ImageComparison overflowInvalid = CompareImages(overflowHeader, overflowHeader);
  CHECK(std::isinf(overflowInvalid.mae));
  CHECK(std::isinf(overflowInvalid.mse));
  CHECK(std::isinf(overflowInvalid.flipLike));
  CHECK(overflowInvalid.psnr == doctest::Approx(0.0));
  CHECK(BuildDiffImage(overflowHeader, overflowHeader).Empty());
}

}  // namespace directxsplat
