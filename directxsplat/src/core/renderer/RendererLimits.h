#pragma once

#include <cstdint>

namespace directxsplat::renderer_internal {

inline constexpr uint32_t kMinSortPairCapacity = 1024u;
inline constexpr uint32_t kMaxSceneGaussians = 64u * 1024u * 1024u;

constexpr uint32_t PlanSortPairCapacity(uint64_t sceneGaussianCount, uint32_t drawCapacity) {
  uint64_t target = sceneGaussianCount > kMinSortPairCapacity ? sceneGaussianCount : kMinSortPairCapacity;
  target = target > drawCapacity ? target : drawCapacity;
  target = target < kMaxSceneGaussians ? target : kMaxSceneGaussians;
  return static_cast<uint32_t>(target);
}

}  // namespace directxsplat::renderer_internal
