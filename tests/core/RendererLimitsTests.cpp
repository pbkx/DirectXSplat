#include <doctest/doctest.h>

#include "renderer/RendererLimits.h"

namespace directxsplat {
namespace {

TEST_CASE("sort capacity covers the supported scene range") {
  constexpr uint32_t previousLimit = 16u * 1024u * 1024u;

  CHECK(renderer_internal::PlanSortPairCapacity(0u, 1u) == renderer_internal::kMinSortPairCapacity);
  CHECK(renderer_internal::PlanSortPairCapacity(previousLimit + 1ull, 1u) == previousLimit + 1u);
  CHECK(renderer_internal::PlanSortPairCapacity(renderer_internal::kMaxSceneGaussians, 1u) ==
        renderer_internal::kMaxSceneGaussians);
  CHECK(renderer_internal::PlanSortPairCapacity(static_cast<uint64_t>(renderer_internal::kMaxSceneGaussians) + 1u, 1u) ==
        renderer_internal::kMaxSceneGaussians);
}

TEST_CASE("prepare dispatch covers supported chunk counts") {
  const renderer_internal::PrepareDispatchGrid empty = renderer_internal::PlanPrepareDispatch(0u);
  CHECK(empty.groupsY == 0u);
  CHECK(empty.groupsZ == 0u);

  const renderer_internal::PrepareDispatchGrid singleDimension =
      renderer_internal::PlanPrepareDispatch(renderer_internal::kMaxDispatchThreadGroupsPerDimension);
  CHECK(singleDimension.groupsY == renderer_internal::kMaxDispatchThreadGroupsPerDimension);
  CHECK(singleDimension.groupsZ == 1u);

  const renderer_internal::PrepareDispatchGrid tiled =
      renderer_internal::PlanPrepareDispatch(renderer_internal::kMaxDispatchThreadGroupsPerDimension + 1u);
  CHECK(tiled.groupsY == 32768u);
  CHECK(tiled.groupsZ == 2u);

  const renderer_internal::PrepareDispatchGrid maximum =
      renderer_internal::PlanPrepareDispatch(renderer_internal::kMaxSceneChunks);
  CHECK(maximum.groupsY <= renderer_internal::kMaxDispatchThreadGroupsPerDimension);
  CHECK(maximum.groupsZ <= renderer_internal::kMaxDispatchThreadGroupsPerDimension);
  CHECK(static_cast<uint64_t>(maximum.groupsY) * maximum.groupsZ >= renderer_internal::kMaxSceneChunks);
}

}  // namespace
}  // namespace directxsplat
