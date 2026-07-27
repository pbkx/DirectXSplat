#pragma once

#include <cstdint>

namespace directxsplat::renderer_internal {

inline constexpr uint32_t kMinSortPairCapacity = 1024u;
inline constexpr uint32_t kMaxSceneGaussians = 64u * 1024u * 1024u;
inline constexpr uint32_t kMaxSceneChunks = 1024u * 1024u;
inline constexpr uint32_t kMaxDispatchThreadGroupsPerDimension = 65535u;

struct PrepareDispatchGrid {
  uint32_t groupsY = 0;
  uint32_t groupsZ = 0;
};

constexpr uint32_t PlanSortPairCapacity(uint64_t sceneGaussianCount, uint32_t drawCapacity) {
  uint64_t target = sceneGaussianCount > kMinSortPairCapacity ? sceneGaussianCount : kMinSortPairCapacity;
  target = target > drawCapacity ? target : drawCapacity;
  target = target < kMaxSceneGaussians ? target : kMaxSceneGaussians;
  return static_cast<uint32_t>(target);
}

constexpr PrepareDispatchGrid PlanPrepareDispatch(uint32_t chunkCount) {
  if (chunkCount == 0) {
    return {};
  }
  const uint32_t groupsZ =
      chunkCount / kMaxDispatchThreadGroupsPerDimension +
      (chunkCount % kMaxDispatchThreadGroupsPerDimension != 0 ? 1u : 0u);
  const uint32_t groupsY = chunkCount / groupsZ + (chunkCount % groupsZ != 0 ? 1u : 0u);
  return {groupsY, groupsZ};
}

}  // namespace directxsplat::renderer_internal
