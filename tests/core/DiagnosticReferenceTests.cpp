#include <doctest/doctest.h>

#include <array>
#include <cstdint>
#include <vector>

#include "directxsplat/bounding.h"
#include "directxsplat/math.h"
#include "directxsplat/scene.h"
#include "renderer/diagnostics.h"

namespace directxsplat {
namespace {

Gaussian MakeDiagnosticGaussian(Vec3 position, Vec3 scale, uint32_t splatId, uint32_t instanceId) {
  Gaussian gaussian{};
  gaussian.position = position;
  gaussian.scale = scale;
  gaussian.rotation = {0.0f, 0.0f, 0.0f, 2.0f};
  gaussian.opacity = 0.25f + static_cast<float>(splatId) * 0.125f;
  gaussian.splatId = splatId;
  gaussian.instanceId = instanceId;
  for (size_t i = 0; i < gaussian.sh.size(); ++i) {
    gaussian.sh[i] = static_cast<float>(splatId * 100u + i) * 0.01f;
  }
  return gaussian;
}

GaussianSet MakeDiagnosticSet() {
  GaussianSet set{};
  set.name = "diagnostic";
  set.gaussians.push_back(MakeDiagnosticGaussian({-1.0f, 2.0f, 3.0f}, {0.5f, 0.75f, 1.0f}, 7u, 11u));
  set.gaussians.push_back(MakeDiagnosticGaussian({4.0f, -2.0f, 8.0f}, {1.5f, 2.0f, 2.5f}, 13u, 17u));
  set.bounds = ComputeAabb({set.gaussians[0].position, set.gaussians[1].position});
  return set;
}

}

TEST_CASE("diagnostic upload packers match reference ordering") {
  const GaussianSet set = MakeDiagnosticSet();

  const PackedGaussianUpload floatUpload = PackGaussianUploadBuffers(set);
  REQUIRE(floatUpload.positions.size() == 6u);
  REQUIRE(floatUpload.scales.size() == 6u);
  REQUIRE(floatUpload.rotations.size() == 8u);
  REQUIRE(floatUpload.opacity.size() == 2u);
  REQUIRE(floatUpload.sh.size() == 2u * kShOrder3CoeffCountTotal);

  CHECK(floatUpload.positions[0] == doctest::Approx(-1.0f));
  CHECK(floatUpload.positions[1] == doctest::Approx(2.0f));
  CHECK(floatUpload.positions[2] == doctest::Approx(3.0f));
  CHECK(floatUpload.positions[3] == doctest::Approx(4.0f));
  CHECK(floatUpload.scales[4] == doctest::Approx(2.0f));
  CHECK(floatUpload.rotations[3] == doctest::Approx(2.0f));
  CHECK(floatUpload.opacity[1] == doctest::Approx(1.875f));
  CHECK(floatUpload.sh[kShOrder3CoeffCountTotal + 16u] == doctest::Approx(13.16f));

  const CompactGaussianUpload compact = PackCompactGaussianUploadBuffers(set);
  REQUIRE(compact.strideBytes == 128u);
  REQUIRE(compact.words.size() == 64u);
  CHECK(compact.decodeMin[0] == doctest::Approx(-1.0f));
  CHECK(compact.decodeMin[1] == doctest::Approx(-2.0f));
  CHECK(compact.decodeMin[2] == doctest::Approx(3.0f));
  CHECK(compact.decodeExtent[0] == doctest::Approx(5.0f));
  CHECK(compact.decodeExtent[1] == doctest::Approx(4.0f));
  CHECK(compact.decodeExtent[2] == doctest::Approx(5.0f));
  CHECK(compact.words[30] == 7u);
  CHECK(compact.words[31] == 11u);
  CHECK(compact.words[62] == 13u);
  CHECK(compact.words[63] == 17u);
}

TEST_CASE("diagnostic remap bindless and chunk helpers cover matrix cases") {
  const BindlessTable emptyTable = BuildBindlessTable(0u, 99u);
  CHECK(emptyTable.descriptorIndices.empty());

  const BindlessTable table = BuildBindlessTable(4u, 20u);
  REQUIRE(table.descriptorIndices.size() == 4u);
  CHECK(table.descriptorIndices[0] == 20u);
  CHECK(table.descriptorIndices[3] == 23u);

  std::vector<uint32_t> remapped;
  RemapGlobalIndices({0u, 2u, 5u}, 100u, remapped);
  CHECK(remapped == std::vector<uint32_t>{100u, 102u, 105u});
  RemapGlobalIndices({}, 100u, remapped);
  CHECK(remapped.empty());

  struct ChunkCase {
    uint32_t itemCount;
    uint32_t chunkSize;
    std::vector<uint32_t> starts;
    std::vector<uint32_t> counts;
  };
  const std::array<ChunkCase, 5> cases{{
      {0u, 64u, {}, {}},
      {1u, 64u, {0u}, {1u}},
      {64u, 64u, {0u}, {64u}},
      {65u, 64u, {0u, 64u}, {64u, 1u}},
      {130u, 64u, {0u, 64u, 128u}, {64u, 64u, 2u}},
  }};

  for (const ChunkCase& testCase : cases) {
    const ChunkBookkeeping bookkeeping = BuildChunkBookkeeping(testCase.itemCount, testCase.chunkSize);
    CHECK(bookkeeping.chunkStarts == testCase.starts);
    CHECK(bookkeeping.chunkCounts == testCase.counts);
  }
  CHECK(BuildChunkBookkeeping(10u, 0u).chunkStarts.empty());
}

TEST_CASE("frustum helpers match reference sphere classifications") {
  FrustumPlanes planes{};
  planes[0] = {{1.0f, 0.0f, 0.0f}, 1.0f};
  planes[1] = {{-1.0f, 0.0f, 0.0f}, 1.0f};
  planes[2] = {{0.0f, 1.0f, 0.0f}, 1.0f};
  planes[3] = {{0.0f, -1.0f, 0.0f}, 1.0f};
  planes[4] = {{0.0f, 0.0f, 1.0f}, 0.0f};
  planes[5] = {{0.0f, 0.0f, -1.0f}, 5.0f};

  CHECK(SphereInFrustum(planes, {0.0f, 0.0f, 2.0f}, 0.1f));
  CHECK(SphereInFrustum(planes, {1.05f, 0.0f, 2.0f}, 0.1f));
  CHECK_FALSE(SphereInFrustum(planes, {1.25f, 0.0f, 2.0f}, 0.1f));
  CHECK_FALSE(SphereInFrustum(planes, {0.0f, 0.0f, -0.2f}, 0.1f));
  CHECK_FALSE(SphereInFrustum(planes, {0.0f, 0.0f, 5.2f}, 0.1f));

  Scene scene{};
  GaussianSet a{};
  a.gaussians.push_back(MakeDiagnosticGaussian({0.0f, 0.0f, 2.0f}, {0.05f, 0.05f, 0.05f}, 0u, 0u));
  a.gaussians.push_back(MakeDiagnosticGaussian({2.0f, 0.0f, 2.0f}, {0.05f, 0.05f, 0.05f}, 1u, 0u));
  GaussianSet b{};
  b.gaussians.push_back(MakeDiagnosticGaussian({0.9f, 0.0f, 2.0f}, {0.2f, 0.2f, 0.2f}, 2u, 0u));
  b.gaussians.push_back(MakeDiagnosticGaussian({0.0f, 0.0f, 8.0f}, {0.05f, 0.05f, 0.05f}, 3u, 0u));
  scene.splatSets.push_back(std::move(a));
  scene.splatSets.push_back(std::move(b));

  std::vector<uint32_t> visible;
  FrustumCull(scene, planes, visible);
  CHECK(visible == std::vector<uint32_t>{0u, 2u});
}

TEST_CASE("math reference invariants cover transforms and covariance") {
  const Mat4 identity = Identity4();
  const Vec4 v{1.0f, -2.0f, 3.0f, 1.0f};
  const Vec4 unchanged = Mul(identity, v);
  CHECK(unchanged.x == doctest::Approx(v.x));
  CHECK(unchanged.y == doctest::Approx(v.y));
  CHECK(unchanged.z == doctest::Approx(v.z));
  CHECK(unchanged.w == doctest::Approx(v.w));

  const Mat4 view = LookAt({0.0f, 0.0f, -5.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f});
  const Mat4 inverseView = Inverse(view);
  const Mat4 shouldBeIdentity = Mul(view, inverseView);
  CHECK(shouldBeIdentity.m[0] == doctest::Approx(1.0f));
  CHECK(shouldBeIdentity.m[5] == doctest::Approx(1.0f));
  CHECK(shouldBeIdentity.m[10] == doctest::Approx(1.0f));
  CHECK(shouldBeIdentity.m[15] == doctest::Approx(1.0f));

  const Mat3 covariance = BuildCovariance({2.0f, 3.0f, 4.0f}, {0.0f, 0.0f, 0.0f, 1.0f});
  CHECK(covariance.m[0] == doctest::Approx(4.0f));
  CHECK(covariance.m[4] == doctest::Approx(9.0f));
  CHECK(covariance.m[8] == doctest::Approx(16.0f));
  CHECK(covariance.m[1] == doctest::Approx(0.0f));
  CHECK(covariance.m[3] == doctest::Approx(0.0f));

  const Mat3 projected = ProjectCovarianceToScreen(covariance, {0.25f, -0.5f, 4.0f}, 120.0f, 100.0f);
  CHECK(projected.m[0] > 0.0f);
  CHECK(projected.m[4] > 0.0f);
  CHECK(projected.m[1] == doctest::Approx(projected.m[3]));

  const Mat3 tiltedCovariance =
      BuildCovariance({2.0f, 3.0f, 4.0f}, {0.0f, 0.38268343f, 0.0f, 0.92387953f});
  const Mat3 tiltedProjected =
      ProjectCovarianceToScreen(tiltedCovariance, {0.25f, -0.5f, 4.0f}, 120.0f, 100.0f);
  Mat3 reflectedCovariance = tiltedCovariance;
  reflectedCovariance.m[2] = -reflectedCovariance.m[2];
  reflectedCovariance.m[5] = -reflectedCovariance.m[5];
  reflectedCovariance.m[6] = -reflectedCovariance.m[6];
  reflectedCovariance.m[7] = -reflectedCovariance.m[7];
  const Mat3 reflectedProjected =
      ProjectCovarianceToScreen(reflectedCovariance, {0.25f, -0.5f, -4.0f}, 120.0f, 100.0f);
  CHECK(reflectedProjected.m[0] == doctest::Approx(tiltedProjected.m[0]));
  CHECK(reflectedProjected.m[1] == doctest::Approx(tiltedProjected.m[1]));
  CHECK(reflectedProjected.m[4] == doctest::Approx(tiltedProjected.m[4]));
}

}  // namespace directxsplat
