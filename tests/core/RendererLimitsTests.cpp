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

}  // namespace
}  // namespace directxsplat
