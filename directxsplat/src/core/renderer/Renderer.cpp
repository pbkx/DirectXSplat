#include "renderer/Renderer.h"

#include "renderer/RendererLimits.h"
#include "renderer/raster/GaussianRasterPipeline.h"

#include <Windows.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <exception>
#include <iterator>
#include <limits>
#include <map>
#include <mutex>
#include <new>
#include <numeric>
#include <shared_mutex>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include <wrl/client.h>

namespace directxsplat {

namespace {

constexpr uint64_t kFullResolutionSceneLimit = 2ull * 1024ull * 1024ull;
constexpr uint32_t kHierarchyLeafGroupTarget = 8;
constexpr float kHierarchyMidDescendRadius = 18.0f;
constexpr float kHierarchyNearDescendRadius = 48.0f;
constexpr uint64_t kMaxResidencySceneGaussians = renderer_internal::kMaxSceneGaussians;
constexpr uint64_t kMaxResidencyChunks = 1024ull * 1024ull;
constexpr uint64_t kDefaultResidencyChunkExpandedBytes = 64ull * 1024ull * 1024ull;
constexpr uint64_t kMaxResidencyChunkExpandedBytes = 2ull * 1024ull * 1024ull * 1024ull;
constexpr uint64_t kResidencyChunkExpandedStride = static_cast<uint64_t>(sizeof(Gaussian) + sizeof(Vec3)) * 3ull;
constexpr uint32_t kMaxResidencyChunkTargetSize =
    static_cast<uint32_t>(std::min<uint64_t>(kMaxResidencyChunkExpandedBytes / kResidencyChunkExpandedStride,
                                             std::numeric_limits<uint32_t>::max()));
constexpr DWORD kFenceWaitPollMs = 50;

RendererConfig SanitizeRendererConfig(RendererConfig config) {
  config.residencyBudgetGaussians = std::clamp<uint64_t>(config.residencyBudgetGaussians, 1, kMaxResidencySceneGaussians);
  config.uploadBudgetGaussians = std::clamp<uint64_t>(config.uploadBudgetGaussians, 1, kMaxResidencySceneGaussians);
  config.warmUploadBudgetGaussians = std::clamp<uint64_t>(config.warmUploadBudgetGaussians, 1, kMaxResidencySceneGaussians);
  config.maxUploadsPerFrame = std::max<uint32_t>(config.maxUploadsPerFrame, 1);
  config.chunkTargetSize = std::clamp<uint32_t>(config.chunkTargetSize, 1, kMaxResidencyChunkTargetSize);
  config.lod0ScreenRadius = std::max(config.lod0ScreenRadius, 0.001f);
  config.lod1ScreenRadius = std::clamp(config.lod1ScreenRadius, 0.001f, config.lod0ScreenRadius);
  config.cullScreenRadius = std::max(config.cullScreenRadius, 0.0f);
  return config;
}

bool Finite(float v) {
  return std::isfinite(v);
}

bool Finite(const Vec3& v) {
  return Finite(v.x) && Finite(v.y) && Finite(v.z);
}

Aabb MergeAabb(const Aabb& a, const Aabb& b) {
  if (!a.valid) {
    return b;
  }
  if (!b.valid) {
    return a;
  }
  Aabb out{};
  out.min = Min(a.min, b.min);
  out.max = Max(a.max, b.max);
  out.valid = true;
  return out;
}

Aabb BoundsFromGaussians(const std::vector<Gaussian>& gaussians) {
  std::vector<Vec3> points;
  points.reserve(gaussians.size());
  for (const Gaussian& g : gaussians) {
    if (Finite(g.position)) {
      points.push_back(g.position);
    }
  }
  return ComputeAabb(points);
}

Aabb ValidBounds(const GaussianSet& set) {
  return set.bounds.valid ? set.bounds : BoundsFromGaussians(set.gaussians);
}

float AxisValue(const Vec3& p, uint32_t axis) {
  if (axis == 0) {
    return p.x;
  }
  if (axis == 1) {
    return p.y;
  }
  return p.z;
}

uint32_t LongestAxis(const Aabb& bounds) {
  const Vec3 extent = bounds.max - bounds.min;
  if (extent.x >= extent.y && extent.x >= extent.z) {
    return 0;
  }
  if (extent.y >= extent.z) {
    return 1;
  }
  return 2;
}

Status ValidateResidencySceneInput(const Scene& scene, uint32_t chunkTargetSize) {
  if (scene.splatSets.size() > kMaxResidencyChunks) {
    return Status::Error("scene has too many splat sets");
  }
  uint64_t totalGaussians = 0;
  uint64_t totalChunks = 0;
  for (const GaussianSet& set : scene.splatSets) {
    const uint64_t count = static_cast<uint64_t>(set.gaussians.size());
    if (count > kMaxResidencySceneGaussians - totalGaussians) {
      return Status::Error("scene has too many gaussians");
    }
    totalGaussians += count;
    if (count > 0) {
      const uint64_t target = std::max<uint32_t>(chunkTargetSize, 1);
      const uint64_t setChunks = (count + target - 1u) / target;
      if (setChunks > kMaxResidencyChunks - totalChunks) {
        return Status::Error("scene has too many residency chunks");
      }
      totalChunks += setChunks;
    }
  }
  return Status::Ok();
}

Status ValidateResidencyChunkInput(const GaussianSet& chunk, const RendererConfig& config) {
  const uint64_t count = static_cast<uint64_t>(chunk.gaussians.size());
  if (count > kMaxResidencySceneGaussians) {
    return Status::Error("chunk has too many gaussians");
  }
  if (count > std::numeric_limits<uint64_t>::max() / kResidencyChunkExpandedStride) {
    return Status::Error("chunk has too many gaussians");
  }
  const uint64_t target = std::max<uint32_t>(config.chunkTargetSize, 1);
  uint64_t budget = kMaxResidencyChunkExpandedBytes;
  if (target <= std::numeric_limits<uint64_t>::max() / kResidencyChunkExpandedStride) {
    budget = std::clamp(target * kResidencyChunkExpandedStride, kDefaultResidencyChunkExpandedBytes, kMaxResidencyChunkExpandedBytes);
  }
  if (count * kResidencyChunkExpandedStride > budget) {
    return Status::Error("chunk has too many gaussians");
  }
  return Status::Ok();
}

GaussianSet BuildChunkSet(const GaussianSet& source,
                          const std::vector<size_t>& sortedIndices,
                          size_t first,
                          size_t count,
                          uint32_t chunkIndex) {
  GaussianSet out{};
  out.name = source.name.empty() ? std::string("chunk_") + std::to_string(chunkIndex)
                                 : source.name + "_" + std::to_string(chunkIndex);
  out.visible = source.visible;
  out.scalingModifier = source.scalingModifier;
  out.gaussians.reserve(count);
  const size_t end = std::min(first + count, sortedIndices.size());
  for (size_t i = first; i < end; ++i) {
    const size_t idx = sortedIndices[i];
    if (idx < source.gaussians.size()) {
      out.gaussians.push_back(source.gaussians[idx]);
    }
  }
  out.bounds = BoundsFromGaussians(out.gaussians);
  return out;
}

std::vector<GaussianSet> PartitionSet(const GaussianSet& source, uint32_t chunkTargetSize) {
  std::vector<GaussianSet> chunks;
  const size_t target = std::max<uint32_t>(chunkTargetSize, 1);
  if (source.gaussians.empty()) {
    return chunks;
  }
  if (source.gaussians.size() <= target) {
    GaussianSet single = source;
    single.bounds = ValidBounds(single);
    chunks.push_back(std::move(single));
    return chunks;
  }

  const Aabb bounds = ValidBounds(source);
  const uint32_t axis = LongestAxis(bounds);
  std::vector<size_t> indices(source.gaussians.size());
  std::iota(indices.begin(), indices.end(), size_t{0});
  std::sort(indices.begin(), indices.end(), [&](size_t a, size_t b) {
    return AxisValue(source.gaussians[a].position, axis) < AxisValue(source.gaussians[b].position, axis);
  });

  uint32_t chunkIndex = 0;
  for (size_t first = 0; first < indices.size(); first += target) {
    const size_t count = std::min(target, indices.size() - first);
    chunks.push_back(BuildChunkSet(source, indices, first, count, chunkIndex++));
  }
  return chunks;
}

GaussianSet MakeLodSet(const GaussianSet& source, uint32_t stride) {
  if (stride <= 1u || source.gaussians.size() <= stride) {
    GaussianSet out = source;
    out.bounds = ValidBounds(out);
    return out;
  }

  struct LodClusterAccum {
    Vec3 positionSum{};
    Vec3 positionSqSum{};
    Vec3 scaleSqSum{};
    Quat rotationSum{};
    Quat rotationRef{};
    std::array<float, kShOrder3CoeffCountTotal> shSum{};
    float alphaSum = 0.0f;
    float weightSum = 0.0f;
    uint32_t count = 0;
    uint32_t splatId = 0;
    uint32_t instanceId = 0;
    bool hasRotationRef = false;
  };

  auto alphaFromOpacity = [](float opacity) {
    const float clamped = Clamp(opacity, -20.0f, 20.0f);
    return 1.0f / (1.0f + std::exp(-clamped));
  };

  auto opacityFromAlpha = [](float alpha) {
    const float safe = Clamp(alpha, 1e-4f, 1.0f - 1e-4f);
    return std::log(safe / (1.0f - safe));
  };

  auto packVoxel = [](uint32_t x, uint32_t y, uint32_t z) {
    return static_cast<uint64_t>(x) | (static_cast<uint64_t>(y) << 21u) | (static_cast<uint64_t>(z) << 42u);
  };

  const Aabb bounds = ValidBounds(source);
  const Vec3 extent = Max(bounds.max - bounds.min, Vec3{1e-3f, 1e-3f, 1e-3f});
  const float targetCount = std::max<float>(1.0f, static_cast<float>(source.gaussians.size()) / static_cast<float>(stride));
  const float axisProduct = std::max(extent.x * extent.y * extent.z, 1e-9f);
  const float targetScale = std::cbrt(targetCount / axisProduct);
  const Vec3 baseDimsF{
      std::max(1.0f, extent.x * targetScale),
      std::max(1.0f, extent.y * targetScale),
      std::max(1.0f, extent.z * targetScale),
  };

  float gridScale = 1.0f;
  std::vector<LodClusterAccum> clusters;
  for (int attempt = 0; attempt < 5; ++attempt) {
    const uint32_t dimX = std::max<uint32_t>(1u, static_cast<uint32_t>(std::lround(baseDimsF.x * gridScale)));
    const uint32_t dimY = std::max<uint32_t>(1u, static_cast<uint32_t>(std::lround(baseDimsF.y * gridScale)));
    const uint32_t dimZ = std::max<uint32_t>(1u, static_cast<uint32_t>(std::lround(baseDimsF.z * gridScale)));
    std::unordered_map<uint64_t, size_t> clusterMap;
    clusterMap.reserve(static_cast<size_t>(targetCount * 1.5f) + 32u);
    clusters.clear();
    clusters.reserve(static_cast<size_t>(targetCount * 1.25f) + 32u);

    for (const Gaussian& g : source.gaussians) {
      const Vec3 rel = {
          (g.position.x - bounds.min.x) / extent.x,
          (g.position.y - bounds.min.y) / extent.y,
          (g.position.z - bounds.min.z) / extent.z,
      };
      const uint32_t ix = std::min<uint32_t>(static_cast<uint32_t>(Clamp(rel.x, 0.0f, 0.999999f) * static_cast<float>(dimX)), dimX - 1u);
      const uint32_t iy = std::min<uint32_t>(static_cast<uint32_t>(Clamp(rel.y, 0.0f, 0.999999f) * static_cast<float>(dimY)), dimY - 1u);
      const uint32_t iz = std::min<uint32_t>(static_cast<uint32_t>(Clamp(rel.z, 0.0f, 0.999999f) * static_cast<float>(dimZ)), dimZ - 1u);
      const uint64_t key = packVoxel(ix, iy, iz);

      size_t clusterIndex = 0;
      auto it = clusterMap.find(key);
      if (it == clusterMap.end()) {
        clusterIndex = clusters.size();
        clusterMap.emplace(key, clusterIndex);
        clusters.push_back({});
      } else {
        clusterIndex = it->second;
      }

      LodClusterAccum& cluster = clusters[clusterIndex];
      const Vec3 scale{
          std::max(std::abs(g.scale.x), 1e-4f),
          std::max(std::abs(g.scale.y), 1e-4f),
          std::max(std::abs(g.scale.z), 1e-4f),
      };
      const float alpha = alphaFromOpacity(g.opacity);
      const float volume = std::cbrt(std::max(scale.x * scale.y * scale.z, 1e-9f));
      const float weight = std::max(0.05f, (0.3f + alpha * 0.7f) * volume);
      const Quat q = Normalize(g.rotation);

      cluster.positionSum = cluster.positionSum + g.position * weight;
      cluster.positionSqSum = cluster.positionSqSum + Vec3{g.position.x * g.position.x, g.position.y * g.position.y, g.position.z * g.position.z} * weight;
      cluster.scaleSqSum = cluster.scaleSqSum + Vec3{scale.x * scale.x, scale.y * scale.y, scale.z * scale.z} * weight;
      if (!cluster.hasRotationRef) {
        cluster.rotationRef = q;
        cluster.hasRotationRef = true;
        cluster.splatId = g.splatId;
        cluster.instanceId = g.instanceId;
      }
      const float sign = (q.x * cluster.rotationRef.x + q.y * cluster.rotationRef.y + q.z * cluster.rotationRef.z + q.w * cluster.rotationRef.w) < 0.0f ? -1.0f : 1.0f;
      cluster.rotationSum.x += q.x * weight * sign;
      cluster.rotationSum.y += q.y * weight * sign;
      cluster.rotationSum.z += q.z * weight * sign;
      cluster.rotationSum.w += q.w * weight * sign;
      for (size_t i = 0; i < cluster.shSum.size(); ++i) {
        cluster.shSum[i] += g.sh[i] * weight;
      }
      cluster.alphaSum += alpha;
      cluster.weightSum += weight;
      cluster.count++;
    }

    if (clusters.size() <= static_cast<size_t>(targetCount * 1.15f) || attempt == 4) {
      break;
    }
    gridScale *= 0.82f;
  }

  GaussianSet out{};
  out.name = source.name;
  out.visible = source.visible;
  out.scalingModifier = source.scalingModifier;
  out.gaussians.reserve(clusters.size());
  for (const LodClusterAccum& cluster : clusters) {
    if (cluster.count == 0 || cluster.weightSum <= 0.0f) {
      continue;
    }
    Gaussian g{};
    const float invWeight = 1.0f / cluster.weightSum;
    g.position = cluster.positionSum * invWeight;
    const Vec3 meanSq = cluster.positionSqSum * invWeight;
    const Vec3 positionVar{
        std::max(meanSq.x - g.position.x * g.position.x, 0.0f),
        std::max(meanSq.y - g.position.y * g.position.y, 0.0f),
        std::max(meanSq.z - g.position.z * g.position.z, 0.0f),
    };
    const Vec3 scaleSq = cluster.scaleSqSum * invWeight + positionVar;
    g.scale = {
        std::sqrt(std::max(scaleSq.x, 1e-8f)),
        std::sqrt(std::max(scaleSq.y, 1e-8f)),
        std::sqrt(std::max(scaleSq.z, 1e-8f)),
    };
    g.rotation = Normalize(cluster.rotationSum);
    if (g.rotation.x == 0.0f && g.rotation.y == 0.0f && g.rotation.z == 0.0f && g.rotation.w == 0.0f) {
      g.rotation = cluster.hasRotationRef ? cluster.rotationRef : Quat{};
    }
    const float meanAlpha = cluster.alphaSum / static_cast<float>(std::max(cluster.count, 1u));
    g.opacity = opacityFromAlpha(meanAlpha);
    for (size_t i = 0; i < cluster.shSum.size(); ++i) {
      g.sh[i] = cluster.shSum[i] * invWeight;
    }
    g.splatId = cluster.splatId;
    g.instanceId = cluster.instanceId;
    out.gaussians.push_back(g);
  }
  if (out.gaussians.empty() && !source.gaussians.empty()) {
    out.gaussians.push_back(source.gaussians.front());
  }
  out.bounds = ValidBounds(source);
  return out;
}

float ScreenRadiusForChunk(const Aabb& bounds, const RenderInput& input) {
  if (!bounds.valid) {
    return 0.0f;
  }
  const Vec3 center = ComputeAabbCenter(bounds);
  const float radius = std::max(ComputeAabbRadius(bounds), 1e-3f);
  const Vec4 center4{center.x, center.y, center.z, 1.0f};
  const Vec4 view = Mul(input.view, center4);
  const float viewDepth = view.z * (input.settings.positiveViewSpaceZ ? 1.0f : -1.0f);
  const float nearPlane = std::max(input.nearPlane, 1e-4f);
  if (!Finite(view.z) || viewDepth + radius <= nearPlane) {
    return 0.0f;
  }
  const float focalX = std::abs(input.proj.m[0]) * static_cast<float>(std::max(input.viewportWidth, 1u)) * 0.5f;
  const float focalY = std::abs(input.proj.m[5]) * static_cast<float>(std::max(input.viewportHeight, 1u)) * 0.5f;
  const float focal = std::max(focalX, focalY);
  const float screenRadius = radius * focal / std::max(viewDepth - radius, nearPlane);
  const Vec4 clip = Mul(input.proj, view);
  if (viewDepth > nearPlane && std::abs(clip.w) > 1e-6f) {
    const float dilation = input.settings.fastCulling ? std::max(input.settings.frustumDilation, 0.0f) : 1.0f;
    const float slackX = screenRadius * 2.0f / static_cast<float>(std::max(input.viewportWidth, 1u)) + dilation;
    const float slackY = screenRadius * 2.0f / static_cast<float>(std::max(input.viewportHeight, 1u)) + dilation;
    const float ndcX = clip.x / clip.w;
    const float ndcY = clip.y / clip.w;
    if (ndcX < -1.0f - slackX || ndcX > 1.0f + slackX || ndcY < -1.0f - slackY || ndcY > 1.0f + slackY) {
      return 0.0f;
    }
  }
  return screenRadius;
}

float ScreenRadiusForChunk(const Vec3& center, float radius, const RenderInput& input) {
  if (!Finite(center)) {
    return 0.0f;
  }
  radius = std::max(radius, 1e-3f);
  const Vec4 center4{center.x, center.y, center.z, 1.0f};
  const Vec4 view = Mul(input.view, center4);
  const float viewDepth = view.z * (input.settings.positiveViewSpaceZ ? 1.0f : -1.0f);
  const float nearPlane = std::max(input.nearPlane, 1e-4f);
  if (!Finite(view.z) || viewDepth + radius <= nearPlane) {
    return 0.0f;
  }
  const float focalX = std::abs(input.proj.m[0]) * static_cast<float>(std::max(input.viewportWidth, 1u)) * 0.5f;
  const float focalY = std::abs(input.proj.m[5]) * static_cast<float>(std::max(input.viewportHeight, 1u)) * 0.5f;
  const float focal = std::max(focalX, focalY);
  const float screenRadius = radius * focal / std::max(viewDepth - radius, nearPlane);
  const Vec4 clip = Mul(input.proj, view);
  if (viewDepth > nearPlane && std::abs(clip.w) > 1e-6f) {
    const float dilation = input.settings.fastCulling ? std::max(input.settings.frustumDilation, 0.0f) : 1.0f;
    const float slackX = screenRadius * 2.0f / static_cast<float>(std::max(input.viewportWidth, 1u)) + dilation;
    const float slackY = screenRadius * 2.0f / static_cast<float>(std::max(input.viewportHeight, 1u)) + dilation;
    const float ndcX = clip.x / clip.w;
    const float ndcY = clip.y / clip.w;
    if (ndcX < -1.0f - slackX || ndcX > 1.0f + slackX || ndcY < -1.0f - slackY || ndcY > 1.0f + slackY) {
      return 0.0f;
    }
  }
  return screenRadius;
}

uint64_t HashBytes(const void* data, size_t size) {
  const uint8_t* bytes = reinterpret_cast<const uint8_t*>(data);
  uint64_t hash = 1469598103934665603ull;
  for (size_t i = 0; i < size; ++i) {
    hash ^= static_cast<uint64_t>(bytes[i]);
    hash *= 1099511628211ull;
  }
  return hash;
}

uint64_t HashCombine(uint64_t a, uint64_t b) {
  return a ^ (b + 0x9e3779b97f4a7c15ull + (a << 6u) + (a >> 2u));
}

uint64_t MakeResidencyInputSignature(const RenderInput& input) {
  uint64_t hash = 1469598103934665603ull;
  hash = HashCombine(hash, HashBytes(input.view.m.data(), sizeof(float) * input.view.m.size()));
  hash = HashCombine(hash, HashBytes(input.proj.m.data(), sizeof(float) * input.proj.m.size()));
  hash = HashCombine(hash, HashBytes(&input.cameraPosition, sizeof(input.cameraPosition)));
  hash = HashCombine(hash, HashBytes(&input.viewportWidth, sizeof(input.viewportWidth)));
  hash = HashCombine(hash, HashBytes(&input.viewportHeight, sizeof(input.viewportHeight)));
  hash = HashCombine(hash, HashBytes(&input.nearPlane, sizeof(input.nearPlane)));
  hash = HashCombine(hash, HashBytes(&input.settings.splatBudget, sizeof(input.settings.splatBudget)));
  hash = HashCombine(hash, HashBytes(&input.settings.lodHysteresis, sizeof(input.settings.lodHysteresis)));
  hash = HashCombine(hash, HashBytes(&input.settings.fastCulling, sizeof(input.settings.fastCulling)));
  hash = HashCombine(hash, HashBytes(&input.settings.frustumDilation, sizeof(input.settings.frustumDilation)));
  hash = HashCombine(hash, HashBytes(&input.settings.positiveViewSpaceZ, sizeof(input.settings.positiveViewSpaceZ)));
  return hash;
}

int ChooseLod(float screenRadius, const RendererConfig& config) {
  if (screenRadius >= config.lod0ScreenRadius) {
    return 0;
  }
  if (screenRadius >= config.lod1ScreenRadius) {
    return 1;
  }
  return 2;
}

}  

class Renderer::Impl {
 public:
  ~Impl() = default;

  struct SceneAccessState {
    uint32_t activeRenderEncoders = 0;
    uint32_t waitingMutations = 0;
    uint32_t activeMutationOps = 0;
    bool mutationActive = false;
    uint64_t mutationToken = 0;
  };

  struct ResidentChunk {
    UploadedChunkHandle handle{};
    std::array<GaussianSet, 3> lods{};
    std::array<uint32_t, 3> lodCounts{};
    Aabb bounds{};
    Vec3 center{};
    float radius = 0.0f;
    bool visible = true;
    bool resident = false;
    int residentLod = -1;
    bool residentEnabled = false;
    uint64_t lastUsedFrame = 0;
  };

  enum class ResidencyApplyKind {
    Add,
    Update,
    Remove,
    Enable,
  };

  struct ResidencyApplyStep {
    ResidencyApplyKind kind = ResidencyApplyKind::Enable;
    size_t chunkIndex = 0;
    int targetLod = -1;
    bool targetEnabled = false;
    bool previousResident = false;
    int previousLod = -1;
    bool previousEnabled = false;
    uint64_t previousLastUsedFrame = 0;
  };

  struct ResidencyNode {
    Aabb bounds{};
    Vec3 center{};
    float radius = 0.0f;
    uint32_t firstChunk = 0;
    uint32_t chunkCount = 0;
    int left = -1;
    int right = -1;
  };

  struct ResidencyCandidate {
    size_t index = 0;
    int lod = 0;
    int baselineLod = 0;
    uint32_t cost = 0;
    float score = 0.0f;
    bool selected = false;
  };

  struct ResidencyScene {
    std::vector<ResidentChunk> chunks;
    std::vector<ResidencyNode> nodes;
    int rootNode = -1;
    std::vector<ResidencyCandidate> candidates;
    std::vector<int> selectedLods;
    std::vector<int> cacheLods;
    std::vector<size_t> orderedScratch;
    std::vector<float> chunkScores;
    std::vector<float> chunkScreenRadii;
    std::vector<int> preferredLods;
    std::vector<uint8_t> visibleMask;
    uint64_t frameIndex = 0;
    uint64_t totalCpuGaussians = 0;
  };

  struct ResidencyPlanChunk {
    std::array<uint32_t, 3> lodCounts{};
    Vec3 center{};
    float radius = 0.0f;
    bool visible = true;
    bool resident = false;
    int residentLod = -1;
    uint64_t lastUsedFrame = 0;
  };

  struct ResidencyPlanSnapshot {
    std::vector<ResidencyPlanChunk> chunks;
    std::vector<ResidencyNode> nodes;
    int rootNode = -1;
    uint64_t frameIndex = 0;
    uint64_t totalCpuGaussians = 0;
    uint64_t sceneVersion = 0;
  };

  struct PreparedResidencyState {
    uint64_t sceneVersion = 0;
    uint64_t inputSignature = 0;
    bool valid = false;
    bool syncComplete = false;
    std::vector<int> selectedLods;
    std::vector<int> cacheLods;
    uint64_t budget = 0;
    FrameStats stats{};
  };

  struct ResidencyInstanceRecord {
    mutable std::mutex mutex;
    ResidencyScene residency;
    PreparedResidencyState prepared;
    uint64_t rasterSceneId = 0;
    std::atomic_uint32_t activeUsers{0};
    Microsoft::WRL::ComPtr<ID3D12Fence> inFlightFence;
    uint64_t inFlightFenceValue = 0;
    Microsoft::WRL::ComPtr<ID3D12Fence> reservedFence;
    uint64_t reservedFenceValue = 0;
  };

  struct SceneRecord {
    mutable std::mutex mutex;
    std::vector<UploadedChunkHandle> chunkHandles;
    ResidencyScene templateResidency;
    std::vector<std::shared_ptr<ResidencyInstanceRecord>> instances;
    VramFormatSettings vramFormat{};
    uint64_t version = 1;
  };

  struct RenderOp {
    Impl* owner = nullptr;
    UploadedSceneHandle scene{};
    bool active = false;
    ~RenderOp() {
      if (active && owner != nullptr) {
        owner->EndRenderAccess(scene);
      }
    }
  };

  struct MutationOp {
    Impl* owner = nullptr;
    UploadedSceneHandle scene{};
    bool active = false;
    ~MutationOp() {
      if (active && owner != nullptr) {
        owner->EndMutationOperation(scene);
      }
    }
  };

  struct ResidencyUse {
    Impl* owner = nullptr;
    std::shared_ptr<ResidencyInstanceRecord> instance;
    const RenderFrameContext* frameContext = nullptr;
    bool active = false;
    ~ResidencyUse() {
      if (active && owner != nullptr) {
        owner->ReleaseResidencyInstance(instance, frameContext);
      }
    }
  };

  struct SceneMutationGuard {
    Impl* owner = nullptr;
    SceneMutationToken token{};
    bool active = false;
    ~SceneMutationGuard() {
      if (active && owner != nullptr) {
        owner->EndSceneMutation(token);
      }
    }
    Status End() {
      if (!active || owner == nullptr) {
        active = false;
        return Status::Ok();
      }
      active = false;
      return owner->EndSceneMutation(token);
    }
    void Dismiss() {
      active = false;
    }
  };

  ResidentChunk BuildResidentChunk(UploadedChunkHandle handle, const GaussianSet& source) const {
    ResidentChunk chunk{};
    chunk.handle = handle;
    chunk.lods[0] = MakeLodSet(source, 1);
    chunk.lods[1] = MakeLodSet(source, 4);
    chunk.lods[2] = MakeLodSet(source, 16);
    for (size_t i = 0; i < chunk.lods.size(); ++i) {
      chunk.lods[i].name = chunk.lods[0].name;
      chunk.lods[i].visible = source.visible;
      chunk.lodCounts[i] = static_cast<uint32_t>(std::min<uint64_t>(chunk.lods[i].gaussians.size(), UINT32_MAX));
    }
    chunk.bounds = ValidBounds(chunk.lods[0]);
    chunk.center = chunk.bounds.valid ? ComputeAabbCenter(chunk.bounds) : Vec3{};
    chunk.radius = chunk.bounds.valid ? std::max(ComputeAabbRadius(chunk.bounds), 1e-3f) : 0.0f;
    chunk.visible = source.visible;
    return chunk;
  }

  std::vector<GaussianSet> BuildLogicalChunks(const Scene& scene) const {
    std::vector<GaussianSet> logical;
    for (const GaussianSet& set : scene.splatSets) {
      std::vector<GaussianSet> chunks = PartitionSet(set, config.chunkTargetSize);
      logical.insert(logical.end(), std::make_move_iterator(chunks.begin()), std::make_move_iterator(chunks.end()));
    }
    return logical;
  }

  float ChunkAxisCenter(const ResidentChunk& chunk, uint32_t axis) const {
    return AxisValue(ComputeAabbCenter(chunk.bounds), axis);
  }

  int BuildHierarchyNode(ResidencyScene& residency, uint32_t begin, uint32_t end) const {
    const int nodeIndex = static_cast<int>(residency.nodes.size());
    residency.nodes.push_back({});
    residency.nodes[static_cast<size_t>(nodeIndex)].firstChunk = begin;
    residency.nodes[static_cast<size_t>(nodeIndex)].chunkCount = end - begin;

    Aabb bounds{};
    for (uint32_t i = begin; i < end; ++i) {
      bounds = MergeAabb(bounds, residency.chunks[static_cast<size_t>(i)].bounds);
    }
    residency.nodes[static_cast<size_t>(nodeIndex)].bounds = bounds;
    residency.nodes[static_cast<size_t>(nodeIndex)].center = bounds.valid ? ComputeAabbCenter(bounds) : Vec3{};
    residency.nodes[static_cast<size_t>(nodeIndex)].radius = bounds.valid ? std::max(ComputeAabbRadius(bounds), 1e-3f) : 0.0f;

    if (residency.nodes[static_cast<size_t>(nodeIndex)].chunkCount <= kHierarchyLeafGroupTarget) {
      return nodeIndex;
    }

    const uint32_t axis = LongestAxis(bounds);
    const uint32_t mid = begin + residency.nodes[static_cast<size_t>(nodeIndex)].chunkCount / 2u;
    auto rangeBegin = residency.chunks.begin() + begin;
    auto rangeMid = residency.chunks.begin() + mid;
    auto rangeEnd = residency.chunks.begin() + end;
    std::nth_element(rangeBegin, rangeMid, rangeEnd, [&](const ResidentChunk& a, const ResidentChunk& b) {
      return ChunkAxisCenter(a, axis) < ChunkAxisCenter(b, axis);
    });

    residency.nodes[static_cast<size_t>(nodeIndex)].left = BuildHierarchyNode(residency, begin, mid);
    residency.nodes[static_cast<size_t>(nodeIndex)].right = BuildHierarchyNode(residency, mid, end);
    residency.nodes[static_cast<size_t>(nodeIndex)].bounds =
        MergeAabb(residency.nodes[static_cast<size_t>(residency.nodes[static_cast<size_t>(nodeIndex)].left)].bounds,
                  residency.nodes[static_cast<size_t>(residency.nodes[static_cast<size_t>(nodeIndex)].right)].bounds);
    residency.nodes[static_cast<size_t>(nodeIndex)].center =
        residency.nodes[static_cast<size_t>(nodeIndex)].bounds.valid
            ? ComputeAabbCenter(residency.nodes[static_cast<size_t>(nodeIndex)].bounds)
            : Vec3{};
    residency.nodes[static_cast<size_t>(nodeIndex)].radius =
        residency.nodes[static_cast<size_t>(nodeIndex)].bounds.valid
            ? std::max(ComputeAabbRadius(residency.nodes[static_cast<size_t>(nodeIndex)].bounds), 1e-3f)
            : 0.0f;
    return nodeIndex;
  }

  void RebuildResidencyHierarchy(ResidencyScene& residency) const {
    residency.nodes.clear();
    residency.rootNode = -1;
    if (residency.chunks.empty()) {
      residency.chunkScores.clear();
      residency.chunkScreenRadii.clear();
      residency.preferredLods.clear();
      residency.visibleMask.clear();
      return;
    }
    residency.rootNode = BuildHierarchyNode(residency, 0u, static_cast<uint32_t>(residency.chunks.size()));
    residency.chunkScores.assign(residency.chunks.size(), 0.0f);
    residency.chunkScreenRadii.assign(residency.chunks.size(), 0.0f);
    residency.preferredLods.assign(residency.chunks.size(), -1);
    residency.visibleMask.assign(residency.chunks.size(), 0u);
  }

  int ChooseChunkLod(float screenRadius, int currentLod, float hysteresis) const {
    const float h = std::clamp(hysteresis, 0.0f, 0.45f);
    switch (currentLod) {
      case 0:
        if (screenRadius >= config.lod0ScreenRadius * (1.0f - h)) {
          return 0;
        }
        if (screenRadius >= config.lod1ScreenRadius * (1.0f - h)) {
          return 1;
        }
        return 2;
      case 1:
        if (screenRadius >= config.lod0ScreenRadius * (1.0f + h)) {
          return 0;
        }
        if (screenRadius >= config.lod1ScreenRadius * (1.0f - h)) {
          return 1;
        }
        return 2;
      default:
        if (screenRadius >= config.lod0ScreenRadius * (1.0f + h)) {
          return 0;
        }
        if (screenRadius >= config.lod1ScreenRadius * (1.0f + h)) {
          return 1;
        }
        return 2;
    }
  }

  int HighestAvailableLod(const ResidentChunk& chunk) const {
    for (int lod = 2; lod >= 0; --lod) {
      if (chunk.lodCounts[static_cast<size_t>(lod)] > 0) {
        return lod;
      }
    }
    return -1;
  }

  int HighestAvailableLod(const ResidencyPlanChunk& chunk) const {
    for (int lod = 2; lod >= 0; --lod) {
      if (chunk.lodCounts[static_cast<size_t>(lod)] > 0) {
        return lod;
      }
    }
    return -1;
  }

  void AssignNodeRange(const ResidencyPlanSnapshot& snapshot,
                       const ResidencyNode& node,
                       int nodeLod,
                       const RenderInput& input,
                       std::vector<int>& selectedLods,
                       std::vector<int>& preferredLods,
                       std::vector<uint8_t>& visibleMask,
                       std::vector<float>& chunkScores,
                       std::vector<float>& chunkScreenRadii) const {
    for (uint32_t i = node.firstChunk; i < node.firstChunk + node.chunkCount; ++i) {
      const ResidencyPlanChunk& chunk = snapshot.chunks[static_cast<size_t>(i)];
      if (!chunk.visible || chunk.radius <= 0.0f || chunk.lodCounts[0] == 0) {
        continue;
      }
      const float screenRadius = ScreenRadiusForChunk(chunk.center, chunk.radius, input);
      if (screenRadius < config.cullScreenRadius) {
        continue;
      }
      const int preferred = snapshot.totalCpuGaussians <= kFullResolutionSceneLimit
                                 ? 0
                                 : ChooseChunkLod(screenRadius, chunk.residentLod < 0 ? 2 : chunk.residentLod, input.settings.lodHysteresis);
      const int availablePreferred = [&]() {
        int lod = preferred;
        while (lod < 2 && chunk.lodCounts[static_cast<size_t>(lod)] == 0) {
          ++lod;
        }
        return chunk.lodCounts[static_cast<size_t>(lod)] > 0 ? lod : HighestAvailableLod(chunk);
      }();
      const int highestAvailable = HighestAvailableLod(chunk);
      if (availablePreferred < 0 || highestAvailable < 0) {
        continue;
      }
      const int baseline = std::min(std::max(nodeLod, availablePreferred), highestAvailable);
      visibleMask[static_cast<size_t>(i)] = 1u;
      preferredLods[static_cast<size_t>(i)] = availablePreferred;
      selectedLods[static_cast<size_t>(i)] = baseline;
      chunkScreenRadii[static_cast<size_t>(i)] = screenRadius;
      chunkScores[static_cast<size_t>(i)] =
          screenRadius / std::max(1.0f, static_cast<float>(chunk.lodCounts[static_cast<size_t>(baseline)]) / 8192.0f);
    }
  }

  void GatherHierarchySelection(const ResidencyPlanSnapshot& snapshot,
                                int nodeIndex,
                                const RenderInput& input,
                                std::vector<int>& selectedLods,
                                std::vector<int>& preferredLods,
                                std::vector<uint8_t>& visibleMask,
                                std::vector<float>& chunkScores,
                                std::vector<float>& chunkScreenRadii) const {
    if (nodeIndex < 0) {
      return;
    }
    const ResidencyNode& node = snapshot.nodes[static_cast<size_t>(nodeIndex)];
    const float nodeScreenRadius = ScreenRadiusForChunk(node.center, node.radius, input);
    if (nodeScreenRadius < config.cullScreenRadius) {
      return;
    }

    const int nodeLod = snapshot.totalCpuGaussians <= kFullResolutionSceneLimit
                            ? 0
                            : ChooseLod(nodeScreenRadius, config);
    const bool isLeaf = node.left < 0 || node.right < 0;
    const bool descend =
        !isLeaf &&
        ((nodeLod == 0 && nodeScreenRadius >= kHierarchyNearDescendRadius) ||
         (nodeLod == 1 && nodeScreenRadius >= kHierarchyMidDescendRadius && node.chunkCount > kHierarchyLeafGroupTarget));

    if (descend) {
      GatherHierarchySelection(snapshot, node.left, input, selectedLods, preferredLods, visibleMask, chunkScores, chunkScreenRadii);
      GatherHierarchySelection(snapshot, node.right, input, selectedLods, preferredLods, visibleMask, chunkScores, chunkScreenRadii);
      return;
    }

    AssignNodeRange(snapshot, node, nodeLod, input, selectedLods, preferredLods, visibleMask, chunkScores, chunkScreenRadii);
  }

  void RecomputeResidencyTotals(ResidencyScene& residency) const {
    residency.totalCpuGaussians = 0;
    for (const ResidentChunk& chunk : residency.chunks) {
      residency.totalCpuGaussians += chunk.lodCounts[0];
    }
  }

  Status BuildResidencyScene(uint64_t sceneId,
                             const Scene& scene,
                             std::vector<UploadedChunkHandle>& chunks,
                             ResidencyScene& outResidency) try {
    Status inputStatus = ValidateResidencySceneInput(scene, config.chunkTargetSize);
    if (!inputStatus.ok) {
      return inputStatus;
    }
    std::vector<GaussianSet> logical = BuildLogicalChunks(scene);
    if (logical.size() > kMaxResidencyChunks) {
      return Status::Error("scene has too many residency chunks");
    }
    for (const GaussianSet& chunk : logical) {
      Status chunkStatus = ValidateResidencyChunkInput(chunk, config);
      if (!chunkStatus.ok) {
        return chunkStatus;
      }
    }
    while (chunks.size() < logical.size()) {
      chunks.push_back({nextChunkId.fetch_add(1, std::memory_order_relaxed)});
    }
    chunks.resize(logical.size());
    outResidency = {};
    outResidency.chunks.reserve(logical.size());
    for (size_t i = 0; i < logical.size(); ++i) {
      outResidency.chunks.push_back(BuildResidentChunk(chunks[i], logical[i]));
    }
    RecomputeResidencyTotals(outResidency);
    RebuildResidencyHierarchy(outResidency);
    (void)sceneId;
    return Status::Ok();
  } catch (const std::bad_alloc&) {
    return Status::Error("scene residency allocation failed");
  } catch (const std::length_error&) {
    return Status::Error("scene residency allocation failed");
  }

  ResidentChunk* FindResidencyChunk(ResidencyScene& residency, UploadedChunkHandle handle) {
    auto it = std::find_if(residency.chunks.begin(), residency.chunks.end(), [&](const ResidentChunk& chunk) {
      return chunk.handle == handle;
    });
    return it == residency.chunks.end() ? nullptr : &*it;
  }

  const ResidentChunk* FindResidencyChunk(const ResidencyScene& residency, UploadedChunkHandle handle) const {
    auto it = std::find_if(residency.chunks.begin(), residency.chunks.end(), [&](const ResidentChunk& chunk) {
      return chunk.handle == handle;
    });
    return it == residency.chunks.end() ? nullptr : &*it;
  }

  std::shared_ptr<SceneRecord> GetSceneRecord(uint64_t sceneId) const {
    std::shared_lock<std::shared_mutex> lock(scenesMutex);
    auto it = scenes.find(sceneId);
    return it == scenes.end() ? nullptr : it->second;
  }

  std::shared_ptr<ResidencyInstanceRecord> CreateResidencyInstanceFrom(const ResidencyScene& residency,
                                                                       uint64_t sceneVersion,
                                                                       uint64_t rasterSceneId) const {
    auto instance = std::make_shared<ResidencyInstanceRecord>();
    instance->rasterSceneId = rasterSceneId;
    instance->residency = residency;
    instance->residency.frameIndex = 0;
    for (ResidentChunk& chunk : instance->residency.chunks) {
      chunk.resident = false;
      chunk.residentLod = -1;
      chunk.residentEnabled = false;
      chunk.lastUsedFrame = 0;
    }
    instance->prepared.sceneVersion = sceneVersion;
    instance->prepared.valid = false;
    return instance;
  }

  std::shared_ptr<ResidencyInstanceRecord> CreateResidencyInstance(const SceneRecord& record, uint64_t rasterSceneId) const {
    return CreateResidencyInstanceFrom(record.templateResidency, record.version, rasterSceneId);
  }

  uint64_t CompletedFenceValue(const RenderFrameContext* frameContext) const {
    if (frameContext == nullptr || frameContext->fence == nullptr || submissionFence.Get() != frameContext->fence) {
      return 0;
    }
    uint64_t completed = frameContext->completedFenceValue;
    completed = std::max(completed, frameContext->fence->GetCompletedValue());
    return completed;
  }

  Status ValidateFrameContext(const RenderFrameContext* frameContext) const {
    if (frameContext == nullptr || frameContext->fence == nullptr || frameContext->submissionFenceValue == 0) {
      return Status::Error("render frame context requires a fence and submission value");
    }
    if (submissionFence == nullptr) {
      return Status::Error("render frame context fence is not registered");
    }
    if (submissionFence.Get() != frameContext->fence) {
      return Status::Error("render frame context fence changed");
    }
    const uint64_t completedFenceValue =
        std::max(frameContext->completedFenceValue, frameContext->fence->GetCompletedValue());
    if (frameContext->submissionFenceValue <= completedFenceValue) {
      return Status::Error("render frame context submission value is already completed");
    }
    return Status::Ok();
  }

  bool IsFenceComplete(ID3D12Fence* fence, uint64_t value, uint64_t completedValue) const {
    if (value == 0 || completedValue >= value) {
      return true;
    }
    return fence != nullptr && fence->GetCompletedValue() >= value;
  }

  bool IsInstanceGpuIdle(const std::shared_ptr<ResidencyInstanceRecord>& instance, uint64_t completedValue) const {
    if (instance == nullptr) {
      return true;
    }
    std::lock_guard<std::mutex> instanceLock(instance->mutex);
    if (IsFenceComplete(instance->inFlightFence.Get(), instance->inFlightFenceValue, completedValue)) {
      instance->inFlightFence.Reset();
      instance->inFlightFenceValue = 0;
      return true;
    }
    return false;
  }

  bool ReservationMatchesFrame(const ResidencyInstanceRecord& instance, const RenderFrameContext* frameContext) const {
    return frameContext != nullptr && frameContext->fence != nullptr && instance.reservedFence.Get() == frameContext->fence &&
           instance.reservedFenceValue == frameContext->submissionFenceValue;
  }

  void ClearResidencyReservation(ResidencyInstanceRecord& instance) const {
    instance.reservedFence.Reset();
    instance.reservedFenceValue = 0;
  }

  bool TryAcquireReservedResidencyInstance(const std::shared_ptr<ResidencyInstanceRecord>& instance,
                                           const RenderFrameContext* frameContext) const {
    if (instance == nullptr || frameContext == nullptr || instance->activeUsers.load(std::memory_order_acquire) != 0) {
      return false;
    }
    std::lock_guard<std::mutex> instanceLock(instance->mutex);
    if (!ReservationMatchesFrame(*instance, frameContext)) {
      return false;
    }
    ClearResidencyReservation(*instance);
    instance->activeUsers.fetch_add(1, std::memory_order_release);
    return true;
  }

  bool HasOtherResidencyReservation(const std::shared_ptr<ResidencyInstanceRecord>& instance,
                                    const RenderFrameContext* frameContext,
                                    uint64_t completedValue) const {
    if (instance == nullptr) {
      return false;
    }
    std::lock_guard<std::mutex> instanceLock(instance->mutex);
    if (instance->reservedFenceValue == 0) {
      return false;
    }
    if (IsFenceComplete(instance->reservedFence.Get(), instance->reservedFenceValue, completedValue)) {
      ClearResidencyReservation(*instance);
      return false;
    }
    return !ReservationMatchesFrame(*instance, frameContext);
  }

  void ReserveResidencyInstance(const std::shared_ptr<ResidencyInstanceRecord>& instance, const RenderFrameContext* frameContext) const {
    if (instance == nullptr || frameContext == nullptr || frameContext->fence == nullptr || frameContext->submissionFenceValue == 0) {
      return;
    }
    std::lock_guard<std::mutex> instanceLock(instance->mutex);
    instance->reservedFence = frameContext->fence;
    instance->reservedFenceValue = frameContext->submissionFenceValue;
  }

  Status CheckFenceWaitDeviceLost() {
    if (IsDeviceLost()) {
      return Status::Error("renderer device lost");
    }
    if (device != nullptr) {
      const HRESULT removed = device->GetDeviceRemovedReason();
      if (FAILED(removed)) {
        NotifyDeviceLost();
        return Status::Error("renderer device lost");
      }
    }
    return Status::Ok();
  }

  Status WaitForFence(ID3D12Fence* fence, uint64_t value) {
    if (fence == nullptr || value == 0 || fence->GetCompletedValue() >= value) {
      return Status::Ok();
    }
    Status deviceStatus = CheckFenceWaitDeviceLost();
    if (!deviceStatus.ok) {
      return deviceStatus;
    }
    HANDLE eventHandle = CreateEvent(nullptr, FALSE, FALSE, nullptr);
    if (eventHandle == nullptr) {
      return Status::Error("failed creating fence wait event");
    }
    const HRESULT hr = fence->SetEventOnCompletion(value, eventHandle);
    if (FAILED(hr)) {
      CloseHandle(eventHandle);
      deviceStatus = CheckFenceWaitDeviceLost();
      return deviceStatus.ok ? Status::Error("failed waiting for fence") : deviceStatus;
    }
    while (fence->GetCompletedValue() < value) {
      const DWORD wait = WaitForSingleObject(eventHandle, kFenceWaitPollMs);
      if (wait == WAIT_OBJECT_0) {
        break;
      }
      if (wait != WAIT_TIMEOUT) {
        CloseHandle(eventHandle);
        return Status::Error("failed waiting for fence");
      }
      deviceStatus = CheckFenceWaitDeviceLost();
      if (!deviceStatus.ok) {
        CloseHandle(eventHandle);
        return deviceStatus;
      }
    }
    CloseHandle(eventHandle);
    return Status::Ok();
  }

  Status WaitResidencyInstanceGpuIdle(const std::shared_ptr<ResidencyInstanceRecord>& instance) {
    if (instance == nullptr) {
      return Status::Ok();
    }
    Microsoft::WRL::ComPtr<ID3D12Fence> fence;
    uint64_t value = 0;
    {
      std::lock_guard<std::mutex> instanceLock(instance->mutex);
      fence = instance->inFlightFence;
      value = instance->inFlightFenceValue;
    }
    Status waited = WaitForFence(fence.Get(), value);
    if (!waited.ok) {
      return waited;
    }
    {
      std::lock_guard<std::mutex> instanceLock(instance->mutex);
      if (IsFenceComplete(instance->inFlightFence.Get(), instance->inFlightFenceValue, value)) {
        instance->inFlightFence.Reset();
        instance->inFlightFenceValue = 0;
      }
    }
    return Status::Ok();
  }

  Status WaitRecordResidencyGpuIdle(const std::shared_ptr<SceneRecord>& record) {
    if (record == nullptr) {
      return Status::Ok();
    }
    std::vector<std::shared_ptr<ResidencyInstanceRecord>> instances;
    try {
      {
        std::lock_guard<std::mutex> sceneLock(record->mutex);
        instances = record->instances;
      }
    } catch (const std::bad_alloc&) {
      return Status::Error("scene residency allocation failed");
    } catch (const std::length_error&) {
      return Status::Error("scene residency allocation failed");
    }
    for (const std::shared_ptr<ResidencyInstanceRecord>& instance : instances) {
      Status idle = WaitResidencyInstanceGpuIdle(instance);
      if (!idle.ok) {
        return idle;
      }
    }
    return Status::Ok();
  }

  Status WaitAllResidencyGpuIdle() {
    std::vector<std::shared_ptr<SceneRecord>> records;
    try {
      {
        std::shared_lock<std::shared_mutex> scenesLock(scenesMutex);
        records.reserve(scenes.size());
        for (const auto& item : scenes) {
          records.push_back(item.second);
        }
      }
    } catch (const std::bad_alloc&) {
      return Status::Error("scene residency allocation failed");
    } catch (const std::length_error&) {
      return Status::Error("scene residency allocation failed");
    }
    for (const std::shared_ptr<SceneRecord>& record : records) {
      Status idle = WaitRecordResidencyGpuIdle(record);
      if (!idle.ok) {
        return idle;
      }
    }
    return Status::Ok();
  }

  Status ResetResidencyInstances(SceneRecord& record, uint64_t publicSceneId) try {
    std::vector<uint64_t> extraRasterSceneIds;
    std::vector<std::shared_ptr<ResidencyInstanceRecord>> oldInstances = record.instances;
    for (const std::shared_ptr<ResidencyInstanceRecord>& instance : oldInstances) {
      Status idle = WaitResidencyInstanceGpuIdle(instance);
      if (!idle.ok) {
        return idle;
      }
    }
    for (const std::shared_ptr<ResidencyInstanceRecord>& instance : record.instances) {
      if (instance != nullptr && instance->rasterSceneId != publicSceneId) {
        extraRasterSceneIds.push_back(instance->rasterSceneId);
      }
    }
    const uint64_t nextVersion = record.version + 1;
    std::shared_ptr<ResidencyInstanceRecord> primary = CreateResidencyInstanceFrom(record.templateResidency, nextVersion, publicSceneId);
    std::vector<std::shared_ptr<ResidencyInstanceRecord>> newInstances;
    newInstances.push_back(primary);
    Status clear = ClearRasterScene(publicSceneId, record.vramFormat);
    if (!clear.ok) {
      return clear;
    }
    record.version = nextVersion;
    record.instances = std::move(newInstances);
    for (uint64_t rasterSceneId : extraRasterSceneIds) {
      Status destroyed = raster.DestroyScene(rasterSceneId);
      if (!destroyed.ok) {
        return destroyed;
      }
    }
    return Status::Ok();
  } catch (const std::bad_alloc&) {
    return Status::Error("scene residency allocation failed");
  } catch (const std::length_error&) {
    return Status::Error("scene residency allocation failed");
  }

  Status ReplaceResidencyScene(SceneRecord& record,
                               uint64_t publicSceneId,
                               std::vector<UploadedChunkHandle> chunkHandles,
                               ResidencyScene residency,
                               VramFormatSettings vramFormat) {
    std::vector<std::shared_ptr<ResidencyInstanceRecord>> oldInstances = record.instances;
    for (const std::shared_ptr<ResidencyInstanceRecord>& instance : oldInstances) {
      Status idle = WaitResidencyInstanceGpuIdle(instance);
      if (!idle.ok) {
        return idle;
      }
    }
    std::vector<uint64_t> extraRasterSceneIds;
    for (const std::shared_ptr<ResidencyInstanceRecord>& instance : oldInstances) {
      if (instance != nullptr && instance->rasterSceneId != publicSceneId) {
        extraRasterSceneIds.push_back(instance->rasterSceneId);
      }
    }
    const uint64_t nextVersion = record.version + 1;
    std::shared_ptr<ResidencyInstanceRecord> primary = CreateResidencyInstanceFrom(residency, nextVersion, publicSceneId);
    std::vector<std::shared_ptr<ResidencyInstanceRecord>> newInstances;
    newInstances.push_back(primary);
    vramFormat = SanitizeVramFormatSettings(vramFormat);
    Status clear = ClearRasterScene(publicSceneId, vramFormat);
    if (!clear.ok) {
      return clear;
    }
    record.chunkHandles = std::move(chunkHandles);
    record.templateResidency = std::move(residency);
    record.vramFormat = vramFormat;
    record.version = nextVersion;
    record.instances = std::move(newInstances);
    for (uint64_t rasterSceneId : extraRasterSceneIds) {
      Status destroyed = raster.DestroyScene(rasterSceneId);
      if (!destroyed.ok) {
        return destroyed;
      }
    }
    return Status::Ok();
  }

  std::shared_ptr<ResidencyInstanceRecord> GetPrimaryResidencyInstance(const SceneRecord& record) const {
    return record.instances.empty() ? nullptr : record.instances.front();
  }

  Status AcquireResidencyInstance(const std::shared_ptr<SceneRecord>& record,
                                  uint64_t publicSceneId,
                                  const RenderFrameContext* frameContext,
                                  std::shared_ptr<ResidencyInstanceRecord>& outInstance) {
    outInstance.reset();
    if (record == nullptr) {
      return Status::Error("uploaded scene handle not found");
    }
    const uint64_t completedFenceValue = CompletedFenceValue(frameContext);
    std::lock_guard<std::mutex> sceneLock(record->mutex);
    for (const std::shared_ptr<ResidencyInstanceRecord>& instance : record->instances) {
      if (TryAcquireReservedResidencyInstance(instance, frameContext)) {
        outInstance = instance;
        return Status::Ok();
      }
    }
    for (const std::shared_ptr<ResidencyInstanceRecord>& instance : record->instances) {
      if (instance != nullptr && instance->activeUsers.load(std::memory_order_acquire) == 0 &&
          !HasOtherResidencyReservation(instance, frameContext, completedFenceValue) &&
          IsInstanceGpuIdle(instance, completedFenceValue)) {
        instance->activeUsers.fetch_add(1, std::memory_order_release);
        outInstance = instance;
        return Status::Ok();
      }
    }

    const uint64_t rasterSceneId = nextRasterInstanceSceneId.fetch_add(1, std::memory_order_relaxed);
    std::shared_ptr<ResidencyInstanceRecord> instance;
    try {
      instance = CreateResidencyInstance(*record, rasterSceneId);
    } catch (const std::bad_alloc&) {
      return Status::Error("scene residency allocation failed");
    } catch (const std::length_error&) {
      return Status::Error("scene residency allocation failed");
    }
    Status clear = ClearRasterScene(rasterSceneId, record->vramFormat);
    if (!clear.ok) {
      return clear;
    }
    instance->activeUsers.store(1, std::memory_order_release);
    try {
      record->instances.push_back(instance);
    } catch (const std::bad_alloc&) {
      Status destroyed = raster.DestroyScene(rasterSceneId);
      return destroyed.ok ? Status::Error("scene residency allocation failed") : destroyed;
    } catch (const std::length_error&) {
      Status destroyed = raster.DestroyScene(rasterSceneId);
      return destroyed.ok ? Status::Error("scene residency allocation failed") : destroyed;
    }
    outInstance = instance;
    (void)publicSceneId;
    return Status::Ok();
  }

  void ReleaseResidencyInstance(const std::shared_ptr<ResidencyInstanceRecord>& instance, const RenderFrameContext* frameContext) {
    if (instance == nullptr) {
      return;
    }
    if (frameContext != nullptr && frameContext->fence != nullptr && frameContext->submissionFenceValue != 0) {
      std::lock_guard<std::mutex> instanceLock(instance->mutex);
      instance->inFlightFence = frameContext->fence;
      instance->inFlightFenceValue = frameContext->submissionFenceValue;
    }
    const uint32_t current = instance->activeUsers.load(std::memory_order_acquire);
    if (current > 0) {
      instance->activeUsers.fetch_sub(1, std::memory_order_release);
    }
  }

  Status ClearRasterScene(uint64_t sceneId, VramFormatSettings vramFormat = {}) {
    Scene empty{};
    empty.vramFormat = SanitizeVramFormatSettings(vramFormat);
    std::vector<uint64_t> emptyIds;
    return raster.CreateOrUpdateScene(sceneId, empty, emptyIds);
  }

  Status Initialize(D3D12Context& context, const RendererConfig& rendererConfig) {
    deviceLost.store(false, std::memory_order_release);
    if (context.SubmissionFence() == nullptr) {
      return Status::Error("renderer submission fence is not registered");
    }
    if (context.Device() == nullptr || context.CommandQueue() == nullptr) {
      return Status::Error("invalid D3D12 device/queue");
    }
    config = SanitizeRendererConfig(rendererConfig);
    device = context.Device();
    submissionFence = context.SubmissionFence();
    Status status = raster.Initialize(context.Device(), context.CommandQueue(), context.SubmissionFence(), context.CopyQueue(), context.UploadFence(), config.enableGpuTiming);
    if (!status.ok) {
      device.Reset();
      submissionFence.Reset();
    }
    initialized.store(status.ok, std::memory_order_release);
    return status;
  }

  Status Reset() {
    if (!initialized.load(std::memory_order_acquire)) {
      return Status::Ok();
    }
    Status accessIdle = WaitForIdle();
    if (!accessIdle.ok) {
      return accessIdle;
    }
    bool lost = IsDeviceLost();
    if (!lost) {
      Status idle = WaitAllResidencyGpuIdle();
      if (!idle.ok) {
        lost = IsDeviceLost();
        if (!lost) {
          return idle;
        }
      }
    }
    Status shutdown = Status::Ok();
    {
      shutdown = lost ? raster.ShutdownDeviceLost() : raster.Shutdown();
      if (!shutdown.ok && !lost) {
        lost = IsDeviceLost();
        if (!lost) {
          return shutdown;
        }
        shutdown = raster.ShutdownDeviceLost();
      }
    }
    ClearRendererRecords();
    ReleaseRendererConfig();
    deviceLost.store(false, std::memory_order_release);
    initialized.store(false, std::memory_order_release);
    return shutdown;
  }

  Status ResetForDestruction() {
    if (!initialized.load(std::memory_order_acquire)) {
      return Status::Ok();
    }
    WaitForActiveOperations();
    CancelOutstandingMutations();
    bool lost = IsDeviceLost();
    if (!lost) {
      Status idle = WaitAllResidencyGpuIdle();
      if (!idle.ok) {
        NotifyDeviceLost();
        lost = true;
      }
    }
    Status shutdown = lost ? raster.ShutdownDeviceLost() : raster.Shutdown();
    if (!shutdown.ok && !lost) {
      NotifyDeviceLost();
      shutdown = raster.ShutdownDeviceLost();
    }
    ClearRendererRecords();
    ReleaseRendererConfig();
    deviceLost.store(false, std::memory_order_release);
    initialized.store(false, std::memory_order_release);
    return shutdown;
  }

  void ClearRendererRecords() {
    {
      std::unique_lock<std::shared_mutex> scenesLock(scenesMutex);
      scenes.clear();
      nextSceneId = 1;
      nextChunkId = 1;
      nextRasterInstanceSceneId = 1ull << 32;
    }
    {
      std::lock_guard<std::mutex> lock(accessMutex);
      accessStates.clear();
      nextMutationToken = 1;
    }
    accessCv.notify_all();
  }

  void ReleaseRendererConfig() {
    config = {};
    device.Reset();
    submissionFence.Reset();
  }

  void CancelOutstandingMutations() {
    {
      std::lock_guard<std::mutex> lock(accessMutex);
      for (auto& [sceneId, state] : accessStates) {
        (void)sceneId;
        state.mutationActive = false;
        state.mutationToken = 0;
        state.waitingMutations = 0;
      }
    }
    accessCv.notify_all();
  }

  void NotifyDeviceLost() {
    deviceLost.store(true, std::memory_order_release);
    raster.NotifyDeviceLost();
    accessCv.notify_all();
  }

  bool IsDeviceLost() const {
    return deviceLost.load(std::memory_order_acquire) || raster.IsDeviceLost();
  }

  bool IsInitialized() const {
    return initialized.load(std::memory_order_acquire);
  }

  Status CreateUploadedScene(UploadedSceneHandle& outHandle) {
    Scene empty{};
    return CreateUploadedScene(empty, outHandle, nullptr);
  }

  Status CreateUploadedScene(const Scene& scene,
                             UploadedSceneHandle& outHandle,
                             std::vector<UploadedChunkHandle>* outChunkHandles) {
    UploadedSceneHandle handle{};
    std::vector<UploadedChunkHandle> chunks;
    std::vector<UploadedChunkHandle> outputChunks;
    std::shared_ptr<SceneRecord> record;
    bool sceneInserted = false;
    bool rasterCreated = false;
    bool accessInserted = false;
    Status rollbackStatus = Status::Ok();
    auto rollback = [&]() {
      if (accessInserted) {
        std::lock_guard<std::mutex> lock(accessMutex);
        accessStates.erase(handle.value);
      }
      if (sceneInserted) {
        std::unique_lock<std::shared_mutex> scenesLock(scenesMutex);
        scenes.erase(handle.value);
      }
      if (rasterCreated) {
        Status destroyed = raster.DestroyScene(handle.value);
        if (!destroyed.ok && rollbackStatus.ok) {
          rollbackStatus = destroyed;
        }
      }
    };
    try {
      record = std::make_shared<SceneRecord>();
      {
        std::unique_lock<std::shared_mutex> scenesLock(scenesMutex);
        handle = {nextSceneId.fetch_add(1, std::memory_order_relaxed)};
        Status buildStatus = BuildResidencyScene(handle.value, scene, chunks, record->templateResidency);
        if (!buildStatus.ok) {
          return buildStatus;
        }
        record->vramFormat = SanitizeVramFormatSettings(scene.vramFormat);
        record->chunkHandles = chunks;
        if (outChunkHandles != nullptr) {
          outputChunks = chunks;
        }
        record->version = 1;
        record->instances.push_back(CreateResidencyInstance(*record, handle.value));
        scenes.emplace(handle.value, record);
        sceneInserted = true;
      }
      Status s = ClearRasterScene(handle.value, record->vramFormat);
      if (!s.ok) {
        rollback();
        if (!rollbackStatus.ok) {
          return rollbackStatus;
        }
        return s;
      }
      rasterCreated = true;
      {
        std::lock_guard<std::mutex> lock(accessMutex);
        accessStates.emplace(handle.value, SceneAccessState{});
        accessInserted = true;
      }
      outHandle = handle;
      if (outChunkHandles != nullptr) {
        *outChunkHandles = std::move(outputChunks);
      }
      accessCv.notify_all();
      return Status::Ok();
    } catch (const std::bad_alloc&) {
      rollback();
      if (!rollbackStatus.ok) {
        return rollbackStatus;
      }
      return Status::Error("scene residency allocation failed");
    } catch (const std::length_error&) {
      rollback();
      if (!rollbackStatus.ok) {
        return rollbackStatus;
      }
      return Status::Error("scene residency allocation failed");
    }
  }

  Status UpdateUploadedScene(UploadedSceneHandle handle, const Scene& scene) {
    SceneMutationToken token{};
    Status s = BeginSceneMutation(handle, token);
    if (!s.ok) {
      return s;
    }
    s = UpdateUploadedScene(token, scene);
    Status end = EndSceneMutation(token);
    return s.ok ? end : s;
  }

  Status UpdateUploadedScene(SceneMutationToken token, const Scene& scene) try {
    MutationOp op{};
    Status s = BeginMutationOperation(token, op);
    if (!s.ok) {
      return s;
    }
    std::shared_ptr<SceneRecord> record = GetSceneRecord(token.scene.value);
    if (record == nullptr || !raster.HasScene(token.scene.value)) {
      return Status::Error("uploaded scene handle not found");
    }
    std::lock_guard<std::mutex> sceneLock(record->mutex);
    std::vector<UploadedChunkHandle> chunks = record->chunkHandles;
    ResidencyScene residency{};
    Status buildStatus = BuildResidencyScene(token.scene.value, scene, chunks, residency);
    if (!buildStatus.ok) {
      return buildStatus;
    }
    return ReplaceResidencyScene(*record, token.scene.value, std::move(chunks), std::move(residency), scene.vramFormat);
  } catch (const std::bad_alloc&) {
    return Status::Error("scene residency allocation failed");
  } catch (const std::length_error&) {
    return Status::Error("scene residency allocation failed");
  }

  Status DestroyUploadedScene(UploadedSceneHandle handle) {
    if (!handle.IsValid()) {
      return Status::Ok();
    }
    SceneMutationToken token{};
    Status s = BeginSceneMutation(handle, token);
    if (!s.ok) {
      return IsUploadedSceneValid(handle) ? s : Status::Ok();
    }
    SceneMutationGuard guard{this, token, true};
    s = DestroyUploadedScene(token);
    if (s.ok) {
      guard.Dismiss();
      return s;
    }
    guard.End();
    return s;
  }

  Status DestroyUploadedScene(SceneMutationToken token) {
    MutationOp op{};
    Status s = BeginMutationOperation(token, op);
    if (!s.ok) {
      return s;
    }
    std::shared_ptr<SceneRecord> record = GetSceneRecord(token.scene.value);
    if (record != nullptr) {
      std::lock_guard<std::mutex> sceneLock(record->mutex);
      for (const std::shared_ptr<ResidencyInstanceRecord>& instance : record->instances) {
        Status idle = WaitResidencyInstanceGpuIdle(instance);
        if (!idle.ok) {
          return idle;
        }
        if (instance != nullptr) {
          Status destroyed = raster.DestroyScene(instance->rasterSceneId);
          if (!destroyed.ok) {
            return destroyed;
          }
        }
      }
    } else {
      Status destroyed = raster.DestroyScene(token.scene.value);
      if (!destroyed.ok) {
        return destroyed;
      }
    }
    {
      std::unique_lock<std::shared_mutex> scenesLock(scenesMutex);
      scenes.erase(token.scene.value);
    }
    op.active = false;
    EndMutationOperation(token.scene);
    {
      std::lock_guard<std::mutex> lock(accessMutex);
      auto it = accessStates.find(token.scene.value);
      if (it != accessStates.end() && it->second.mutationActive && it->second.mutationToken == token.value) {
        accessStates.erase(it);
      }
    }
    accessCv.notify_all();
    return Status::Ok();
  }

  bool IsUploadedSceneValid(UploadedSceneHandle handle) const {
    if (!handle.IsValid()) {
      return false;
    }
    std::lock_guard<std::mutex> lock(accessMutex);
    return accessStates.find(handle.value) != accessStates.end();
  }

  bool IsSceneReadyToRender(UploadedSceneHandle sceneHandle) const {
    SceneAccessInfo info{};
    return GetSceneAccessInfo(sceneHandle, info).ok && info.readyToRender;
  }

  Status GetSceneAccessInfo(UploadedSceneHandle sceneHandle, SceneAccessInfo& outInfo) const {
    outInfo = {};
    if (!sceneHandle.IsValid()) {
      return Status::Error("invalid uploaded scene handle");
    }
    std::lock_guard<std::mutex> lock(accessMutex);
    auto it = accessStates.find(sceneHandle.value);
    if (it == accessStates.end()) {
      return Status::Error("uploaded scene handle not found");
    }
    const SceneAccessState& state = it->second;
    outInfo.mutationActive = state.mutationActive;
    outInfo.renderEncodingActive = state.activeRenderEncoders > 0;
    outInfo.activeRenderEncoders = state.activeRenderEncoders;
    outInfo.waitingMutations = state.waitingMutations;
    outInfo.readyToRender = !state.mutationActive && state.waitingMutations == 0;
    return Status::Ok();
  }

  Status GetUploadedSceneInfo(UploadedSceneHandle sceneHandle, UploadedSceneInfo& outInfo) const {
    outInfo = {};
    if (!sceneHandle.IsValid()) {
      return Status::Error("invalid uploaded scene handle");
    }

    SceneAccessInfo accessInfo{};
    Status accessStatus = GetSceneAccessInfo(sceneHandle, accessInfo);
    if (!accessStatus.ok) {
      return accessStatus;
    }

    std::shared_ptr<SceneRecord> record = GetSceneRecord(sceneHandle.value);
    if (record == nullptr || !raster.HasScene(sceneHandle.value)) {
      return Status::Error("uploaded scene handle not found");
    }

    std::lock_guard<std::mutex> sceneLock(record->mutex);
    const ResidencyScene& residency = record->templateResidency;
    std::shared_ptr<ResidencyInstanceRecord> primary = GetPrimaryResidencyInstance(*record);
    outInfo.handle = sceneHandle;
    outInfo.chunkCount = static_cast<uint32_t>(record->chunkHandles.size());
    outInfo.gaussianCount = residency.totalCpuGaussians;
    outInfo.readyToRender = accessInfo.readyToRender;
    for (const ResidentChunk& chunk : residency.chunks) {
      outInfo.bounds = MergeAabb(outInfo.bounds, chunk.bounds);
    }
    if (primary != nullptr) {
      std::lock_guard<std::mutex> instanceLock(primary->mutex);
      for (const ResidentChunk& chunk : primary->residency.chunks) {
        if (chunk.resident && chunk.residentLod >= 0) {
          outInfo.residentChunks++;
          outInfo.residentGaussians += chunk.lodCounts[static_cast<size_t>(chunk.residentLod)];
        }
      }
    }
    return Status::Ok();
  }

  Status GetUploadedChunkInfo(UploadedSceneHandle sceneHandle,
                              UploadedChunkHandle chunkHandle,
                              UploadedChunkInfo& outInfo) const {
    outInfo = {};
    if (!sceneHandle.IsValid() || !chunkHandle.IsValid()) {
      return Status::Error("invalid uploaded scene/chunk handle");
    }

    std::shared_ptr<SceneRecord> record = GetSceneRecord(sceneHandle.value);
    if (record == nullptr || !raster.HasScene(sceneHandle.value)) {
      return Status::Error("uploaded scene handle not found");
    }
    std::lock_guard<std::mutex> sceneLock(record->mutex);
    const ResidentChunk* templateChunk = FindResidencyChunk(record->templateResidency, chunkHandle);
    if (templateChunk == nullptr) {
      return Status::Error("uploaded chunk handle not found");
    }
    std::shared_ptr<ResidencyInstanceRecord> primary = GetPrimaryResidencyInstance(*record);

    outInfo.handle = chunkHandle;
    outInfo.gaussianCount = templateChunk->lodCounts[0];
    outInfo.bounds = templateChunk->bounds;
    outInfo.visible = templateChunk->visible;
    outInfo.scalingModifier = templateChunk->lods[0].scalingModifier;
    if (primary != nullptr) {
      std::lock_guard<std::mutex> instanceLock(primary->mutex);
      const ResidentChunk* residentChunk = FindResidencyChunk(primary->residency, chunkHandle);
      if (residentChunk != nullptr) {
        outInfo.resident = residentChunk->resident;
        outInfo.residentLod = residentChunk->residentLod;
      }
    }
    return Status::Ok();
  }

  Status GetUploadedSceneGpuResources(UploadedSceneHandle sceneHandle,
                                      const RenderFrameContext& frameContext,
                                      UploadedSceneGpuResources& outResources,
                                      bool acquireLease) {
    outResources = {};
    if (!sceneHandle.IsValid()) {
      return Status::Error("invalid uploaded scene handle");
    }

    RenderOp access{};
    Status accessStatus = BeginRenderAccess(sceneHandle, access);
    if (!accessStatus.ok) {
      return accessStatus;
    }

    std::shared_ptr<SceneRecord> record = GetSceneRecord(sceneHandle.value);
    if (record == nullptr || !raster.HasScene(sceneHandle.value)) {
      return Status::Error("uploaded scene handle not found");
    }

    Status gpuStatus = raster.GetSceneGpuResources(sceneHandle.value, &frameContext, acquireLease, outResources);
    if (!gpuStatus.ok) {
      return gpuStatus;
    }

    std::lock_guard<std::mutex> sceneLock(record->mutex);
    std::shared_ptr<ResidencyInstanceRecord> primary = GetPrimaryResidencyInstance(*record);
    if (primary != nullptr) {
      std::lock_guard<std::mutex> instanceLock(primary->mutex);
      for (UploadedChunkGpuResources& chunkResources : outResources.chunks) {
        const ResidentChunk* residencyChunk = FindResidencyChunk(primary->residency, chunkResources.handle);
        if (residencyChunk != nullptr) {
          chunkResources.visible = residencyChunk->visible;
          chunkResources.residentLod = residencyChunk->resident ? residencyChunk->residentLod : -1;
        }
      }
      if (acquireLease) {
        primary->inFlightFence = frameContext.fence;
        primary->inFlightFenceValue = frameContext.submissionFenceValue;
      }
    }
    return Status::Ok();
  }

  Status BeginSceneMutation(UploadedSceneHandle sceneHandle, SceneMutationToken& outToken) {
    outToken = {};
    if (!sceneHandle.IsValid()) {
      return Status::Error("invalid uploaded scene handle");
    }
    if (IsDeviceLost()) {
      return Status::Error("renderer device lost");
    }
    std::unique_lock<std::mutex> lock(accessMutex);
    auto it = accessStates.find(sceneHandle.value);
    if (it == accessStates.end()) {
      return Status::Error("uploaded scene handle not found");
    }
    it->second.waitingMutations++;
    accessCv.wait(lock, [&]() {
      auto current = accessStates.find(sceneHandle.value);
      return IsDeviceLost() || current == accessStates.end() ||
             (!current->second.mutationActive && current->second.activeRenderEncoders == 0 && current->second.activeMutationOps == 0);
    });
    it = accessStates.find(sceneHandle.value);
    if (it != accessStates.end() && it->second.waitingMutations > 0) {
      it->second.waitingMutations--;
    }
    if (IsDeviceLost()) {
      lock.unlock();
      accessCv.notify_all();
      return Status::Error("renderer device lost");
    }
    if (it == accessStates.end()) {
      return Status::Error("uploaded scene handle not found");
    }
    it->second.mutationActive = true;
    it->second.mutationToken = nextMutationToken.fetch_add(1, std::memory_order_relaxed);
    outToken = {it->second.mutationToken, sceneHandle};
    return Status::Ok();
  }

  Status EndSceneMutation(SceneMutationToken token) {
    if (!token.IsValid()) {
      return Status::Error("invalid scene mutation token");
    }
    std::unique_lock<std::mutex> lock(accessMutex);
    auto it = accessStates.find(token.scene.value);
    if (it == accessStates.end()) {
      return Status::Error("uploaded scene handle not found");
    }
    if (!it->second.mutationActive || it->second.mutationToken != token.value) {
      return Status::Error("scene mutation token is not active");
    }
    accessCv.wait(lock, [&]() {
      auto current = accessStates.find(token.scene.value);
      return IsDeviceLost() || current == accessStates.end() || current->second.activeMutationOps == 0;
    });
    it = accessStates.find(token.scene.value);
    if (it == accessStates.end()) {
      return Status::Ok();
    }
    if (IsDeviceLost()) {
      it->second.mutationActive = false;
      it->second.mutationToken = 0;
      lock.unlock();
      accessCv.notify_all();
      return Status::Error("renderer device lost");
    }
    it->second.mutationActive = false;
    it->second.mutationToken = 0;
    lock.unlock();
    accessCv.notify_all();
    return Status::Ok();
  }

  Status GetUploadedSceneChunks(UploadedSceneHandle sceneHandle, std::vector<UploadedChunkHandle>& outChunkHandles) const {
    outChunkHandles.clear();
    if (!sceneHandle.IsValid()) {
      return Status::Error("invalid uploaded scene handle");
    }
    std::shared_ptr<SceneRecord> record = GetSceneRecord(sceneHandle.value);
    if (record == nullptr || !raster.HasScene(sceneHandle.value)) {
      return Status::Error("uploaded scene handle not found");
    }
    std::vector<UploadedChunkHandle> chunkHandles;
    try {
      std::lock_guard<std::mutex> sceneLock(record->mutex);
      chunkHandles = record->chunkHandles;
    } catch (const std::bad_alloc&) {
      return Status::Error("scene residency allocation failed");
    } catch (const std::length_error&) {
      return Status::Error("scene residency allocation failed");
    }
    outChunkHandles.swap(chunkHandles);
    return Status::Ok();
  }

  Status AddUploadedChunk(UploadedSceneHandle sceneHandle, const GaussianSet& chunk, UploadedChunkHandle& outChunkHandle) {
    SceneMutationToken token{};
    Status s = BeginSceneMutation(sceneHandle, token);
    if (!s.ok) {
      return s;
    }
    SceneMutationGuard guard{this, token, true};
    s = AddUploadedChunk(token, chunk, outChunkHandle);
    Status end = guard.End();
    return s.ok ? end : s;
  }

  Status AddUploadedChunk(SceneMutationToken token, const GaussianSet& chunk, UploadedChunkHandle& outChunkHandle) try {
    MutationOp op{};
    Status s = BeginMutationOperation(token, op);
    if (!s.ok) {
      return s;
    }
    std::shared_ptr<SceneRecord> record = GetSceneRecord(token.scene.value);
    if (record == nullptr || !raster.HasScene(token.scene.value)) {
      return Status::Error("uploaded scene handle not found");
    }
    s = ValidateResidencyChunkInput(chunk, config);
    if (!s.ok) {
      return s;
    }
    const UploadedChunkHandle handle{nextChunkId.fetch_add(1, std::memory_order_relaxed)};
    std::lock_guard<std::mutex> sceneLock(record->mutex);
    ResidencyScene residency = record->templateResidency;
    std::vector<UploadedChunkHandle> chunkHandles = record->chunkHandles;
    residency.chunks.push_back(BuildResidentChunk(handle, chunk));
    RecomputeResidencyTotals(residency);
    RebuildResidencyHierarchy(residency);
    chunkHandles.push_back(handle);
    Status reset = ReplaceResidencyScene(*record, token.scene.value, std::move(chunkHandles), std::move(residency), record->vramFormat);
    if (!reset.ok) {
      return reset;
    }
    outChunkHandle = handle;
    return Status::Ok();
  } catch (const std::bad_alloc&) {
    return Status::Error("scene residency allocation failed");
  } catch (const std::length_error&) {
    return Status::Error("scene residency allocation failed");
  }

  Status UpdateUploadedChunk(UploadedSceneHandle sceneHandle, UploadedChunkHandle chunkHandle, const GaussianSet& chunk) {
    SceneMutationToken token{};
    Status s = BeginSceneMutation(sceneHandle, token);
    if (!s.ok) {
      return s;
    }
    SceneMutationGuard guard{this, token, true};
    s = UpdateUploadedChunk(token, chunkHandle, chunk);
    Status end = guard.End();
    return s.ok ? end : s;
  }

  Status UpdateUploadedChunk(SceneMutationToken token, UploadedChunkHandle chunkHandle, const GaussianSet& chunk) try {
    if (!chunkHandle.IsValid()) {
      return Status::Error("invalid uploaded chunk handle");
    }
    MutationOp op{};
    Status s = BeginMutationOperation(token, op);
    if (!s.ok) {
      return s;
    }
    std::shared_ptr<SceneRecord> record = GetSceneRecord(token.scene.value);
    if (record == nullptr) {
      return Status::Error("uploaded scene handle not found");
    }
    std::lock_guard<std::mutex> sceneLock(record->mutex);
    ResidentChunk* existing = FindResidencyChunk(record->templateResidency, chunkHandle);
    if (existing == nullptr) {
      return Status::Error("uploaded chunk handle not found");
    }
    s = ValidateResidencyChunkInput(chunk, config);
    if (!s.ok) {
      return s;
    }
    ResidencyScene residency = record->templateResidency;
    existing = FindResidencyChunk(residency, chunkHandle);
    if (existing == nullptr) {
      return Status::Error("uploaded chunk handle not found");
    }
    *existing = BuildResidentChunk(chunkHandle, chunk);
    RecomputeResidencyTotals(residency);
    RebuildResidencyHierarchy(residency);
    return ReplaceResidencyScene(*record, token.scene.value, record->chunkHandles, std::move(residency), record->vramFormat);
  } catch (const std::bad_alloc&) {
    return Status::Error("scene residency allocation failed");
  } catch (const std::length_error&) {
    return Status::Error("scene residency allocation failed");
  }

  Status RemoveUploadedChunk(UploadedSceneHandle sceneHandle, UploadedChunkHandle chunkHandle) {
    SceneMutationToken token{};
    Status s = BeginSceneMutation(sceneHandle, token);
    if (!s.ok) {
      return s;
    }
    SceneMutationGuard guard{this, token, true};
    s = RemoveUploadedChunk(token, chunkHandle);
    Status end = guard.End();
    return s.ok ? end : s;
  }

  Status RemoveUploadedChunk(SceneMutationToken token, UploadedChunkHandle chunkHandle) try {
    if (!chunkHandle.IsValid()) {
      return Status::Error("invalid uploaded chunk handle");
    }
    MutationOp op{};
    Status s = BeginMutationOperation(token, op);
    if (!s.ok) {
      return s;
    }
    std::shared_ptr<SceneRecord> record = GetSceneRecord(token.scene.value);
    if (record == nullptr) {
      return Status::Error("uploaded scene handle not found");
    }
    std::lock_guard<std::mutex> sceneLock(record->mutex);
    ResidencyScene residency = record->templateResidency;
    auto chunkIt =
        std::find_if(residency.chunks.begin(), residency.chunks.end(), [&](const ResidentChunk& chunk) {
      return chunk.handle == chunkHandle;
    });
    if (chunkIt == residency.chunks.end()) {
      return Status::Error("uploaded chunk handle not found");
    }
    residency.chunks.erase(chunkIt);
    RecomputeResidencyTotals(residency);
    RebuildResidencyHierarchy(residency);
    std::vector<UploadedChunkHandle> chunks = record->chunkHandles;
    chunks.erase(std::remove(chunks.begin(), chunks.end(), chunkHandle), chunks.end());
    return ReplaceResidencyScene(*record, token.scene.value, std::move(chunks), std::move(residency), record->vramFormat);
  } catch (const std::bad_alloc&) {
    return Status::Error("scene residency allocation failed");
  } catch (const std::length_error&) {
    return Status::Error("scene residency allocation failed");
  }

  Status SetUploadedChunkEnabled(UploadedSceneHandle sceneHandle, UploadedChunkHandle chunkHandle, bool enabled) {
    SceneMutationToken token{};
    Status s = BeginSceneMutation(sceneHandle, token);
    if (!s.ok) {
      return s;
    }
    SceneMutationGuard guard{this, token, true};
    s = SetUploadedChunkEnabled(token, chunkHandle, enabled);
    Status end = guard.End();
    return s.ok ? end : s;
  }

  Status SetUploadedChunkEnabled(SceneMutationToken token, UploadedChunkHandle chunkHandle, bool enabled) try {
    if (!chunkHandle.IsValid()) {
      return Status::Error("invalid uploaded chunk handle");
    }
    MutationOp op{};
    Status s = BeginMutationOperation(token, op);
    if (!s.ok) {
      return s;
    }
    std::shared_ptr<SceneRecord> record = GetSceneRecord(token.scene.value);
    if (record == nullptr) {
      return Status::Error("uploaded scene handle not found");
    }
    std::lock_guard<std::mutex> sceneLock(record->mutex);
    ResidencyScene residency = record->templateResidency;
    ResidentChunk* chunk = FindResidencyChunk(residency, chunkHandle);
    if (chunk == nullptr) {
      return Status::Error("uploaded chunk handle not found");
    }
    chunk->visible = enabled;
    for (GaussianSet& lod : chunk->lods) {
      lod.visible = enabled;
    }
    return ReplaceResidencyScene(*record, token.scene.value, record->chunkHandles, std::move(residency), record->vramFormat);
  } catch (const std::bad_alloc&) {
    return Status::Error("scene residency allocation failed");
  } catch (const std::length_error&) {
    return Status::Error("scene residency allocation failed");
  }

  Status SetUploadedChunkScalingModifier(UploadedSceneHandle sceneHandle,
                                         UploadedChunkHandle chunkHandle,
                                         float scalingModifier) {
    SceneMutationToken token{};
    Status s = BeginSceneMutation(sceneHandle, token);
    if (!s.ok) {
      return s;
    }
    SceneMutationGuard guard{this, token, true};
    s = SetUploadedChunkScalingModifier(token, chunkHandle, scalingModifier);
    Status end = guard.End();
    return s.ok ? end : s;
  }

  Status SetUploadedChunkScalingModifier(SceneMutationToken token,
                                         UploadedChunkHandle chunkHandle,
                                         float scalingModifier) try {
    if (!chunkHandle.IsValid()) {
      return Status::Error("invalid uploaded chunk handle");
    }
    MutationOp op{};
    Status s = BeginMutationOperation(token, op);
    if (!s.ok) {
      return s;
    }
    std::shared_ptr<SceneRecord> record = GetSceneRecord(token.scene.value);
    if (record == nullptr) {
      return Status::Error("uploaded scene handle not found");
    }
    std::lock_guard<std::mutex> sceneLock(record->mutex);
    ResidencyScene residency = record->templateResidency;
    ResidentChunk* chunk = FindResidencyChunk(residency, chunkHandle);
    if (chunk == nullptr) {
      return Status::Error("uploaded chunk handle not found");
    }
    const std::array<float, 3> lodBoost{1.0f, 1.0f, 1.0f};
    for (size_t i = 0; i < chunk->lods.size(); ++i) {
      chunk->lods[i].scalingModifier = scalingModifier * lodBoost[i];
    }
    return ReplaceResidencyScene(*record, token.scene.value, record->chunkHandles, std::move(residency), record->vramFormat);
  } catch (const std::bad_alloc&) {
    return Status::Error("scene residency allocation failed");
  } catch (const std::length_error&) {
    return Status::Error("scene residency allocation failed");
  }

  bool IsUploadedChunkValid(UploadedSceneHandle sceneHandle, UploadedChunkHandle chunkHandle) const {
    if (!sceneHandle.IsValid() || !chunkHandle.IsValid()) {
      return false;
    }
    std::shared_ptr<SceneRecord> record = GetSceneRecord(sceneHandle.value);
    if (record == nullptr) {
      return false;
    }
    std::lock_guard<std::mutex> sceneLock(record->mutex);
    return FindResidencyChunk(record->templateResidency, chunkHandle) != nullptr;
  }

  PreparedResidencyState BuildResidencyPlan(const ResidencyPlanSnapshot& snapshot,
                                            const RenderInput& input,
                                            uint64_t inputSignature) const {
    PreparedResidencyState prepared{};
    prepared.sceneVersion = snapshot.sceneVersion;
    prepared.inputSignature = inputSignature;
    prepared.valid = true;
    prepared.stats.gaussiansTotal += snapshot.totalCpuGaussians;

    std::vector<int> selectedLods(snapshot.chunks.size(), -1);
    std::vector<int> preferredLods(snapshot.chunks.size(), -1);
    std::vector<int> cacheLods(snapshot.chunks.size(), -1);
    std::vector<uint8_t> visibleMask(snapshot.chunks.size(), 0u);
    std::vector<float> chunkScores(snapshot.chunks.size(), 0.0f);
    std::vector<float> chunkScreenRadii(snapshot.chunks.size(), 0.0f);

    if (!snapshot.chunks.empty() && snapshot.rootNode >= 0 && !snapshot.nodes.empty()) {
      GatherHierarchySelection(snapshot, snapshot.rootNode, input, selectedLods, preferredLods, visibleMask, chunkScores,
                               chunkScreenRadii);
    }

    std::vector<ResidencyCandidate> candidates;
    candidates.reserve(snapshot.chunks.size());
    for (size_t i = 0; i < snapshot.chunks.size(); ++i) {
      if (visibleMask[i] == 0u || selectedLods[i] < 0 || preferredLods[i] < 0) {
        continue;
      }
      const ResidencyPlanChunk& chunk = snapshot.chunks[i];
      const int baselineLod = selectedLods[i];
      const uint32_t cost = chunk.lodCounts[static_cast<size_t>(baselineLod)];
      if (cost == 0) {
        continue;
      }
      float score = chunkScores[i];
      if (chunk.resident) {
        score *= 1.35f;
      }
      candidates.push_back({i, preferredLods[i], baselineLod, cost, score, false});
    }

    std::sort(candidates.begin(), candidates.end(), [](const ResidencyCandidate& a, const ResidencyCandidate& b) {
      return a.score > b.score;
    });

    const uint64_t autoBudget = std::max<uint64_t>(1, std::min<uint64_t>(
                                                          std::max<uint64_t>(config.residencyBudgetGaussians, 1),
                                                          std::max<uint64_t>(snapshot.totalCpuGaussians, 1)));
    const uint64_t budget =
        std::max<uint64_t>(1, input.settings.splatBudget > 0
                                  ? std::min<uint64_t>(input.settings.splatBudget,
                                                       std::max<uint64_t>(snapshot.totalCpuGaussians, 1))
                                  : (config.defaultSplatBudget > 0
                                         ? std::min<uint64_t>(config.defaultSplatBudget,
                                                              std::max<uint64_t>(snapshot.totalCpuGaussians, 1))
                                         : autoBudget));
    prepared.budget = budget;
    prepared.stats.splatBudget += budget;

    uint64_t selectedCost = 0;
    std::fill(selectedLods.begin(), selectedLods.end(), -1);
    for (ResidencyCandidate& candidate : candidates) {
      const ResidencyPlanChunk& chunk = snapshot.chunks[candidate.index];
      int baselineLod = candidate.baselineLod;
      while (baselineLod > 0 && chunk.lodCounts[static_cast<size_t>(baselineLod)] == 0) {
        baselineLod--;
      }
      if (chunk.lodCounts[static_cast<size_t>(baselineLod)] == 0) {
        continue;
      }
      while (baselineLod < 2 &&
             selectedCost + chunk.lodCounts[static_cast<size_t>(baselineLod)] > budget) {
        baselineLod++;
        while (baselineLod < 2 && chunk.lodCounts[static_cast<size_t>(baselineLod)] == 0) {
          baselineLod++;
        }
      }
      const uint32_t cost = chunk.lodCounts[static_cast<size_t>(baselineLod)];
      if (cost == 0 || (selectedCost + cost > budget && selectedCost > 0)) {
        continue;
      }
      selectedLods[candidate.index] = baselineLod;
      selectedCost += cost;
      candidate.selected = true;
    }

    for (ResidencyCandidate& candidate : candidates) {
      const ResidencyPlanChunk& chunk = snapshot.chunks[candidate.index];
      int currentLod = selectedLods[candidate.index];
      if (currentLod < 0) {
        continue;
      }
      const int preferredLod = candidate.lod;
      for (int lod = currentLod - 1; lod >= preferredLod; --lod) {
        const uint32_t cost = chunk.lodCounts[static_cast<size_t>(lod)];
        if (cost == 0) {
          continue;
        }
        const uint32_t currentCost = chunk.lodCounts[static_cast<size_t>(currentLod)];
        const uint32_t delta = cost > currentCost ? cost - currentCost : 0u;
        if (selectedCost + delta > budget) {
          continue;
        }
        selectedLods[candidate.index] = lod;
        selectedCost += delta;
        currentLod = lod;
      }
    }

    if (selectedCost == 0 && !candidates.empty()) {
      const ResidencyCandidate& best = candidates.front();
      const ResidencyPlanChunk& chunk = snapshot.chunks[best.index];
      const int fallbackLod = HighestAvailableLod(chunk);
      if (fallbackLod >= 0) {
        selectedLods[best.index] = fallbackLod;
      }
    }

    for (size_t i = 0; i < snapshot.chunks.size(); ++i) {
      const ResidencyPlanChunk& chunk = snapshot.chunks[i];
      if (selectedLods[i] >= 0 || !chunk.resident || chunk.residentLod < 0) {
        continue;
      }
      if ((snapshot.frameIndex - chunk.lastUsedFrame) > config.residencyCacheFrames) {
        continue;
      }
      const uint32_t cost = chunk.lodCounts[static_cast<size_t>(chunk.residentLod)];
      if (selectedCost + cost <= budget) {
        cacheLods[i] = chunk.residentLod;
        selectedCost += cost;
      }
    }

    prepared.selectedLods = std::move(selectedLods);
    prepared.cacheLods = std::move(cacheLods);
    prepared.syncComplete = false;
    return prepared;
  }

  void AccumulateResidentStats(const ResidencyScene& residency, FrameStats& stats) const {
    std::array<uint64_t, 3> lodChunkCounts{};
    uint64_t residentGaussians = 0;
    uint64_t residentChunks = 0;
    for (const ResidentChunk& chunk : residency.chunks) {
      if (!chunk.resident || chunk.residentLod < 0) {
        continue;
      }
      residentChunks++;
      residentGaussians += chunk.lodCounts[static_cast<size_t>(chunk.residentLod)];
      lodChunkCounts[static_cast<size_t>(chunk.residentLod)]++;
    }
    stats.residentGaussians += residentGaussians;
    stats.residentChunks += residentChunks;
    stats.lod0Chunks += lodChunkCounts[0];
    stats.lod1Chunks += lodChunkCounts[1];
    stats.lod2Chunks += lodChunkCounts[2];
  }

  void TouchPreparedResidency(ResidencyInstanceRecord& instance) const {
    ResidencyScene& residency = instance.residency;
    residency.frameIndex++;
    const std::vector<int>& selectedLods = instance.prepared.selectedLods;
    const size_t count = std::min(selectedLods.size(), residency.chunks.size());
    for (size_t i = 0; i < count; ++i) {
      if (selectedLods[i] >= 0 && residency.chunks[i].resident) {
        residency.chunks[i].lastUsedFrame = residency.frameIndex;
      }
    }
  }

  Status RestoreResidencyStep(uint64_t rasterSceneId, ResidencyScene& residency, const ResidencyApplyStep& step) {
    if (step.chunkIndex >= residency.chunks.size()) {
      return Status::Error("invalid residency rollback step");
    }
    ResidentChunk& chunk = residency.chunks[step.chunkIndex];
    Status status = Status::Ok();
    if (step.previousResident && step.previousLod >= 0) {
      if (step.kind == ResidencyApplyKind::Add) {
        status = raster.RemoveChunk(rasterSceneId, chunk.handle.value);
        if (!status.ok) {
          return status;
        }
        status = raster.AddChunk(rasterSceneId, chunk.handle.value, chunk.lods[static_cast<size_t>(step.previousLod)]);
      } else if (step.kind == ResidencyApplyKind::Remove) {
        status = raster.AddChunk(rasterSceneId, chunk.handle.value, chunk.lods[static_cast<size_t>(step.previousLod)]);
      } else if (step.kind == ResidencyApplyKind::Update) {
        status = raster.UpdateChunk(rasterSceneId, chunk.handle.value, chunk.lods[static_cast<size_t>(step.previousLod)]);
      }
      if (!status.ok) {
        return status;
      }
      status = raster.SetChunkEnabled(rasterSceneId, chunk.handle.value, step.previousEnabled);
      if (!status.ok) {
        return status;
      }
    } else if (step.kind != ResidencyApplyKind::Remove) {
      status = raster.RemoveChunk(rasterSceneId, chunk.handle.value);
      if (!status.ok) {
        return status;
      }
    }
    chunk.resident = step.previousResident;
    chunk.residentLod = step.previousLod;
    chunk.residentEnabled = step.previousEnabled;
    chunk.lastUsedFrame = step.previousLastUsedFrame;
    return Status::Ok();
  }

  Status ApplyResidencyStep(uint64_t rasterSceneId, ResidencyScene& residency, const ResidencyApplyStep& step) {
    if (step.chunkIndex >= residency.chunks.size() || step.targetLod < -1 || step.targetLod > 2) {
      return Status::Error("invalid residency apply step");
    }
    ResidentChunk& chunk = residency.chunks[step.chunkIndex];
    Status status = Status::Ok();
    switch (step.kind) {
      case ResidencyApplyKind::Add:
        if (step.targetLod < 0) {
          return Status::Error("invalid residency apply step");
        }
        status = raster.AddChunk(rasterSceneId, chunk.handle.value, chunk.lods[static_cast<size_t>(step.targetLod)]);
        if (!status.ok) {
          return status;
        }
        status = raster.SetChunkEnabled(rasterSceneId, chunk.handle.value, step.targetEnabled);
        if (!status.ok) {
          Status restored = RestoreResidencyStep(rasterSceneId, residency, step);
          return restored.ok ? status : restored;
        }
        chunk.resident = true;
        chunk.residentLod = step.targetLod;
        chunk.residentEnabled = step.targetEnabled;
        return Status::Ok();
      case ResidencyApplyKind::Update:
        if (step.targetLod < 0) {
          return Status::Error("invalid residency apply step");
        }
        status = raster.UpdateChunk(rasterSceneId, chunk.handle.value, chunk.lods[static_cast<size_t>(step.targetLod)]);
        if (!status.ok) {
          return status;
        }
        status = raster.SetChunkEnabled(rasterSceneId, chunk.handle.value, step.targetEnabled);
        if (!status.ok) {
          Status restored = RestoreResidencyStep(rasterSceneId, residency, step);
          return restored.ok ? status : restored;
        }
        chunk.resident = true;
        chunk.residentLod = step.targetLod;
        chunk.residentEnabled = step.targetEnabled;
        return Status::Ok();
      case ResidencyApplyKind::Remove:
        status = raster.RemoveChunk(rasterSceneId, chunk.handle.value);
        if (!status.ok) {
          return status;
        }
        chunk.resident = false;
        chunk.residentLod = -1;
        chunk.residentEnabled = false;
        return Status::Ok();
      case ResidencyApplyKind::Enable:
        status = raster.SetChunkEnabled(rasterSceneId, chunk.handle.value, step.targetEnabled);
        if (!status.ok) {
          return status;
        }
        chunk.residentEnabled = step.targetEnabled;
        return Status::Ok();
    }
    return Status::Error("invalid residency apply step");
  }

  Status RollbackResidencySteps(uint64_t rasterSceneId,
                                ResidencyScene& residency,
                                const std::vector<ResidencyApplyStep>& steps,
                                size_t appliedCount,
                                uint64_t previousFrameIndex) {
    Status firstFailure = Status::Ok();
    for (size_t i = appliedCount; i > 0; --i) {
      Status restored = RestoreResidencyStep(rasterSceneId, residency, steps[i - 1]);
      if (!restored.ok && firstFailure.ok) {
        firstFailure = restored;
      }
    }
    residency.frameIndex = previousFrameIndex;
    return firstFailure;
  }

  Status ApplyPreparedResidency(uint64_t rasterSceneId, ResidencyInstanceRecord& instance, FrameStats& stats) {
    ResidencyScene& residency = instance.residency;
    PreparedResidencyState& prepared = instance.prepared;
    const uint64_t previousFrameIndex = residency.frameIndex;
    const uint64_t nextFrameIndex = previousFrameIndex + 1;

    const std::vector<int>& selectedLods = prepared.selectedLods;
    const std::vector<int>& cacheLods = prepared.cacheLods;
    const uint64_t uploadBudget =
        nextFrameIndex <= 3 ? config.uploadBudgetGaussians : config.warmUploadBudgetGaussians;

    uint64_t uploads = 0;
    uint64_t evictions = 0;
    uint64_t uploadCost = 0;
    bool syncComplete = true;
    std::vector<ResidencyApplyStep> steps;
    steps.reserve(residency.chunks.size());

    for (size_t i = 0; i < residency.chunks.size(); ++i) {
      ResidentChunk& chunk = residency.chunks[i];
      const int desiredLod = i < selectedLods.size() ? selectedLods[i] : -1;
      const int cacheLod = i < cacheLods.size() ? cacheLods[i] : -1;
      int targetLod = desiredLod >= 0 ? desiredLod : cacheLod;
      const bool targetEnabled = desiredLod >= 0;
      bool finalResident = chunk.resident;
      int finalLod = chunk.residentLod;
      const auto makeStep = [&](ResidencyApplyKind kind, int lod, bool enabled) {
        ResidencyApplyStep step{};
        step.kind = kind;
        step.chunkIndex = i;
        step.targetLod = lod;
        step.targetEnabled = enabled;
        step.previousResident = chunk.resident;
        step.previousLod = chunk.residentLod;
        step.previousEnabled = chunk.residentEnabled;
        step.previousLastUsedFrame = chunk.lastUsedFrame;
        return step;
      };
      if (targetLod < 0) {
        if (chunk.resident) {
          steps.push_back(makeStep(ResidencyApplyKind::Remove, -1, false));
          finalResident = false;
          finalLod = -1;
          evictions++;
        }
        continue;
      }

      if (!chunk.resident) {
        const uint32_t cost = chunk.lodCounts[static_cast<size_t>(targetLod)];
        if (uploads >= config.maxUploadsPerFrame || (uploadCost > 0 && uploadCost + cost > uploadBudget)) {
          syncComplete = false;
          continue;
        }
        steps.push_back(makeStep(ResidencyApplyKind::Add, targetLod, targetEnabled));
        finalResident = true;
        finalLod = targetLod;
        uploads++;
        uploadCost += cost;
      } else if (chunk.residentLod != targetLod) {
        const uint32_t cost = chunk.lodCounts[static_cast<size_t>(targetLod)];
        if (uploads >= config.maxUploadsPerFrame || (uploadCost > 0 && uploadCost + cost > uploadBudget)) {
          syncComplete = false;
          targetLod = chunk.residentLod;
        } else {
          steps.push_back(makeStep(ResidencyApplyKind::Update, targetLod, targetEnabled));
          finalResident = true;
          finalLod = targetLod;
          uploads++;
          uploadCost += cost;
        }
      }

      if (chunk.resident && chunk.residentLod == targetLod && chunk.residentEnabled != targetEnabled) {
        steps.push_back(makeStep(ResidencyApplyKind::Enable, targetLod, targetEnabled));
      }
      if (!finalResident || finalLod != targetLod) {
        syncComplete = false;
      }
    }

    residency.frameIndex = nextFrameIndex;
    size_t appliedCount = 0;
    for (const ResidencyApplyStep& step : steps) {
      Status applied = ApplyResidencyStep(rasterSceneId, residency, step);
      if (!applied.ok) {
        Status rolledBack = RollbackResidencySteps(rasterSceneId, residency, steps, appliedCount, previousFrameIndex);
        return rolledBack.ok ? applied : rolledBack;
      }
      ++appliedCount;
    }
    const size_t selectedCount = std::min(selectedLods.size(), residency.chunks.size());
    for (size_t i = 0; i < selectedCount; ++i) {
      ResidentChunk& chunk = residency.chunks[i];
      if (selectedLods[i] >= 0 && chunk.resident) {
        chunk.lastUsedFrame = residency.frameIndex;
      }
    }
    prepared.syncComplete = syncComplete;
    stats.streamedUploads += uploads;
    stats.streamedEvictions += evictions;
    AccumulateResidentStats(residency, stats);
    return Status::Ok();
  }

  Status PrepareSceneInternal(const std::shared_ptr<SceneRecord>& record,
                              const std::shared_ptr<ResidencyInstanceRecord>& instance,
                              uint64_t sceneVersion,
                              const RenderInput& input,
                              FrameStats& stats) {
    stats = {};
    if (record == nullptr || instance == nullptr) {
      return Status::Error("uploaded scene handle not found");
    }

    const uint64_t inputSignature = MakeResidencyInputSignature(input);
    bool needsBuild = false;
    ResidencyPlanSnapshot snapshot{};

    {
      std::lock_guard<std::mutex> instanceLock(instance->mutex);
      if (instance->prepared.valid && instance->prepared.sceneVersion == sceneVersion &&
          instance->prepared.inputSignature == inputSignature) {
        if (instance->prepared.syncComplete) {
          TouchPreparedResidency(*instance);
        }
      } else {
        snapshot.chunks.reserve(instance->residency.chunks.size());
        snapshot.nodes = instance->residency.nodes;
        snapshot.rootNode = instance->residency.rootNode;
        snapshot.frameIndex = instance->residency.frameIndex;
        snapshot.totalCpuGaussians = instance->residency.totalCpuGaussians;
        snapshot.sceneVersion = sceneVersion;
        for (const ResidentChunk& chunk : instance->residency.chunks) {
          ResidencyPlanChunk planChunk{};
          planChunk.lodCounts = chunk.lodCounts;
          planChunk.center = chunk.center;
          planChunk.radius = chunk.radius;
          planChunk.visible = chunk.visible;
          planChunk.resident = chunk.resident;
          planChunk.residentLod = chunk.residentLod;
          planChunk.lastUsedFrame = chunk.lastUsedFrame;
          snapshot.chunks.push_back(planChunk);
        }
        needsBuild = true;
      }
    }

    if (needsBuild) {
      PreparedResidencyState built = BuildResidencyPlan(snapshot, input, inputSignature);
      std::lock_guard<std::mutex> instanceLock(instance->mutex);
      if (!instance->prepared.valid || instance->prepared.sceneVersion != sceneVersion ||
          instance->prepared.inputSignature != inputSignature) {
        instance->prepared = std::move(built);
      } else if (instance->prepared.syncComplete) {
        TouchPreparedResidency(*instance);
      }
    }

    {
      std::lock_guard<std::mutex> instanceLock(instance->mutex);
      stats = instance->prepared.stats;
      if (instance->prepared.syncComplete) {
        AccumulateResidentStats(instance->residency, stats);
        return Status::Ok();
      }
    }

    std::lock_guard<std::mutex> instanceLock(instance->mutex);
    stats = instance->prepared.stats;
    if (!instance->prepared.syncComplete) {
      Status apply = ApplyPreparedResidency(instance->rasterSceneId, *instance, stats);
      if (!apply.ok) {
        return apply;
      }
    } else {
      AccumulateResidentStats(instance->residency, stats);
    }
    return Status::Ok();
  }

  Status PrepareSceneForRender(UploadedSceneHandle sceneHandle, const RenderInput& input, const RenderFrameContext* frameContext, RenderPreparationResult& result) try {
    Status frameStatus = ValidateFrameContext(frameContext);
    if (!frameStatus.ok) {
      return frameStatus;
    }
    RenderOp op{};
    Status access = BeginRenderAccess(sceneHandle, op);
    if (!access.ok) {
      return access;
    }
    std::shared_ptr<SceneRecord> record = GetSceneRecord(sceneHandle.value);
    if (record == nullptr) {
      return Status::Error("uploaded scene handle not found");
    }
    std::shared_ptr<ResidencyInstanceRecord> instance;
    Status acquired = AcquireResidencyInstance(record, sceneHandle.value, frameContext, instance);
    if (!acquired.ok) {
      return acquired;
    }
    ResidencyUse residencyUse{this, instance, nullptr, true};
    result = {};
    Status prepared = PrepareSceneInternal(record, instance, record->version, input, result.stats);
    if (prepared.ok) {
      ReserveResidencyInstance(instance, frameContext);
    }
    return prepared;
  } catch (const std::bad_alloc&) {
    return Status::Error("prepare allocation failed");
  } catch (const std::length_error&) {
    return Status::Error("prepare allocation failed");
  } catch (const std::exception&) {
    return Status::Error("prepare failed");
  } catch (...) {
    return Status::Error("prepare failed");
  }

  Status Render(ID3D12GraphicsCommandList* commandList,
                const RenderTargetBinding& target,
                UploadedSceneHandle sceneHandle,
                const RenderInput& input,
                const AdvancedRenderOptions& options,
                const RenderFrameContext* frameContext,
                RenderResult& outResult) {
    outResult = {};
    const auto cpuStart = std::chrono::steady_clock::now();
    Status frameStatus = ValidateFrameContext(frameContext);
    if (!frameStatus.ok) {
      return frameStatus;
    }
    RenderOp op{};
    Status s = BeginRenderAccess(sceneHandle, op);
    if (!s.ok) {
      return s;
    }
    if (!sceneHandle.IsValid()) {
      return Status::Error("invalid uploaded scene handle");
    }
    RenderInput routed = input;
    routed.settings.renderType = SanitizeRenderType(input.settings.renderType);
    routed.settings.shadingDegree = SanitizeShadingDegree(input.settings.shadingDegree);
    routed.settings.antialiasing = input.settings.antialiasing;
    std::shared_ptr<SceneRecord> record = GetSceneRecord(sceneHandle.value);
    if (record == nullptr) {
      return Status::Error("uploaded scene handle not found");
    }
    std::shared_ptr<ResidencyInstanceRecord> instance;
    try {
      Status instanceStatus = AcquireResidencyInstance(record, sceneHandle.value, frameContext, instance);
      if (!instanceStatus.ok) {
        return instanceStatus;
      }
      const uint64_t inputSignature = MakeResidencyInputSignature(routed);
      {
        std::lock_guard<std::mutex> instanceLock(instance->mutex);
        if (!instance->prepared.valid || instance->prepared.sceneVersion != record->version ||
            instance->prepared.inputSignature != inputSignature) {
          ReleaseResidencyInstance(instance, nullptr);
          return Status::Error("scene is not prepared for this render input");
        }
        outResult.stats = instance->prepared.stats;
        AccumulateResidentStats(instance->residency, outResult.stats);
      }
      const uint64_t totalBeforeRaster = outResult.stats.gaussiansTotal;
      Status rendered =
          raster.Render(commandList, target, instance->rasterSceneId, sceneHandle, routed, outResult.stats, &options, &outResult, frameContext);
      ReleaseResidencyInstance(instance, outResult.submission.submissionRequired ? frameContext : nullptr);
      outResult.stats.gaussiansTotal = totalBeforeRaster;
      outResult.stats.cpuMs =
          std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - cpuStart).count();
      if (rendered.ok) {
        outResult.stats.cpuMs = std::max(outResult.stats.cpuMs, 0.0f);
      }
      return rendered;
    } catch (const std::bad_alloc&) {
      ReleaseResidencyInstance(instance, nullptr);
      return Status::Error("render allocation failed");
    } catch (const std::length_error&) {
      ReleaseResidencyInstance(instance, nullptr);
      return Status::Error("render allocation failed");
    } catch (const std::exception&) {
      ReleaseResidencyInstance(instance, nullptr);
      return Status::Error("render failed");
    } catch (...) {
      ReleaseResidencyInstance(instance, nullptr);
      return Status::Error("render failed");
    }
  }

  Status BeginRenderAccess(UploadedSceneHandle sceneHandle, RenderOp& outOp) {
    outOp = {};
    if (!sceneHandle.IsValid()) {
      return Status::Error("invalid uploaded scene handle");
    }
    if (IsDeviceLost()) {
      return Status::Error("renderer device lost");
    }
    std::unique_lock<std::mutex> lock(accessMutex);
    accessCv.wait(lock, [&]() {
      auto current = accessStates.find(sceneHandle.value);
      return IsDeviceLost() || current == accessStates.end() ||
             (!current->second.mutationActive && current->second.waitingMutations == 0 &&
              current->second.activeMutationOps == 0);
    });
    if (IsDeviceLost()) {
      return Status::Error("renderer device lost");
    }
    auto it = accessStates.find(sceneHandle.value);
    if (it == accessStates.end()) {
      return Status::Error("uploaded scene handle not found");
    }
    SceneAccessState& state = it->second;
    state.activeRenderEncoders++;
    outOp.owner = this;
    outOp.scene = sceneHandle;
    outOp.active = true;
    return Status::Ok();
  }

  void EndRenderAccess(UploadedSceneHandle sceneHandle) {
    std::lock_guard<std::mutex> lock(accessMutex);
    auto it = accessStates.find(sceneHandle.value);
    if (it != accessStates.end() && it->second.activeRenderEncoders > 0) {
      it->second.activeRenderEncoders--;
    }
    accessCv.notify_all();
  }

  Status BeginMutationOperation(SceneMutationToken token, MutationOp& outOp) {
    outOp = {};
    if (!token.IsValid()) {
      return Status::Error("invalid scene mutation token");
    }
    std::lock_guard<std::mutex> lock(accessMutex);
    auto it = accessStates.find(token.scene.value);
    if (it == accessStates.end()) {
      return Status::Error("uploaded scene handle not found");
    }
    if (IsDeviceLost()) {
      return Status::Error("renderer device lost");
    }
    if (!it->second.mutationActive || it->second.mutationToken != token.value) {
      return Status::Error("scene mutation token is not active");
    }
    it->second.activeMutationOps++;
    outOp.owner = this;
    outOp.scene = token.scene;
    outOp.active = true;
    return Status::Ok();
  }

  void EndMutationOperation(UploadedSceneHandle sceneHandle) {
    std::lock_guard<std::mutex> lock(accessMutex);
    auto it = accessStates.find(sceneHandle.value);
    if (it != accessStates.end() && it->second.activeMutationOps > 0) {
      it->second.activeMutationOps--;
    }
    accessCv.notify_all();
  }

  Status WaitForIdle() {
    std::unique_lock<std::mutex> lock(accessMutex);
    accessCv.wait(lock, [&]() {
      for (const auto& [sceneId, state] : accessStates) {
        (void)sceneId;
        if (state.activeRenderEncoders > 0 || state.activeMutationOps > 0) {
          return false;
        }
      }
      return true;
    });
    for (const auto& [sceneId, state] : accessStates) {
      (void)sceneId;
      if (state.mutationActive) {
        return Status::Error("scene mutation token is still active");
      }
    }
    return Status::Ok();
  }

  void WaitForActiveOperations() {
    std::unique_lock<std::mutex> lock(accessMutex);
    accessCv.wait(lock, [&]() {
      for (const auto& [sceneId, state] : accessStates) {
        (void)sceneId;
        if (state.activeRenderEncoders > 0 || state.activeMutationOps > 0) {
          return false;
        }
      }
      return true;
    });
  }

  GaussianRasterPipeline raster;
  mutable std::shared_mutex scenesMutex;
  mutable std::mutex accessMutex;
  std::condition_variable accessCv;
  std::unordered_map<uint64_t, std::shared_ptr<SceneRecord>> scenes;
  std::map<uint64_t, SceneAccessState> accessStates;
  std::atomic_uint64_t nextSceneId{1};
  std::atomic_uint64_t nextChunkId{1};
  std::atomic_uint64_t nextRasterInstanceSceneId{1ull << 32};
  std::atomic_uint64_t nextMutationToken{1};
  std::atomic_bool deviceLost{false};
  std::atomic_bool initialized{false};
  RendererConfig config{};
  Microsoft::WRL::ComPtr<ID3D12Device> device;
  Microsoft::WRL::ComPtr<ID3D12Fence> submissionFence;
};

Renderer::Renderer() : impl_(std::make_unique<Impl>()) {}

Renderer::~Renderer() {
  if (impl_ != nullptr) {
    (void)impl_->ResetForDestruction();
  }
}

Status Renderer::Initialize(D3D12Context& context) {
  return Initialize(context, RendererConfig{});
}

Status Renderer::Initialize(D3D12Context& context, const RendererConfig& config) try {
  if (impl_ != nullptr && impl_->IsInitialized()) {
    return Status::Error("renderer is already initialized");
  }
  impl_ = std::make_unique<Impl>();
  return impl_->Initialize(context, config);
} catch (const std::bad_alloc&) {
  return Status::Error("renderer allocation failed");
} catch (const std::length_error&) {
  return Status::Error("renderer allocation failed");
} catch (const std::exception&) {
  return Status::Error("renderer initialization failed");
} catch (...) {
  return Status::Error("renderer initialization failed");
}

Status Renderer::Reset() {
  if (impl_ != nullptr) {
    return impl_->Reset();
  }
  return Status::Ok();
}

Status Renderer::Shutdown() {
  return Reset();
}

void Renderer::NotifyDeviceLost() {
  if (impl_ != nullptr) {
    impl_->NotifyDeviceLost();
  }
}

bool Renderer::IsDeviceLost() const {
  return impl_ != nullptr && impl_->IsDeviceLost();
}

Status Renderer::CreateUploadedScene(UploadedSceneHandle& outHandle) {
  if (impl_ == nullptr || !impl_->IsInitialized()) {
    return Status::Error("renderer is not initialized");
  }
  return impl_->CreateUploadedScene(outHandle);
}

Status Renderer::CreateUploadedScene(const Scene& scene,
                                     UploadedSceneHandle& outHandle,
                                     std::vector<UploadedChunkHandle>* outChunkHandles) {
  if (impl_ == nullptr || !impl_->IsInitialized()) {
    return Status::Error("renderer is not initialized");
  }
  return impl_->CreateUploadedScene(scene, outHandle, outChunkHandles);
}

Status Renderer::UpdateUploadedScene(UploadedSceneHandle handle, const Scene& scene) {
  if (impl_ == nullptr || !impl_->IsInitialized()) {
    return Status::Error("renderer is not initialized");
  }
  return impl_->UpdateUploadedScene(handle, scene);
}

Status Renderer::UpdateUploadedScene(SceneMutationToken token, const Scene& scene) {
  if (impl_ == nullptr || !impl_->IsInitialized()) {
    return Status::Error("renderer is not initialized");
  }
  return impl_->UpdateUploadedScene(token, scene);
}

Status Renderer::DestroyUploadedScene(UploadedSceneHandle handle) {
  if (impl_ == nullptr || !impl_->IsInitialized()) {
    return Status::Error("renderer is not initialized");
  }
  return impl_->DestroyUploadedScene(handle);
}

Status Renderer::DestroyUploadedScene(SceneMutationToken token) {
  if (impl_ == nullptr || !impl_->IsInitialized()) {
    return Status::Error("renderer is not initialized");
  }
  return impl_->DestroyUploadedScene(token);
}

bool Renderer::IsUploadedSceneValid(UploadedSceneHandle handle) const {
  if (impl_ == nullptr || !impl_->IsInitialized()) {
    return false;
  }
  return impl_->IsUploadedSceneValid(handle);
}

bool Renderer::IsSceneReadyToRender(UploadedSceneHandle sceneHandle) const {
  if (impl_ == nullptr || !impl_->IsInitialized()) {
    return false;
  }
  return impl_->IsSceneReadyToRender(sceneHandle);
}

Status Renderer::GetSceneAccessInfo(UploadedSceneHandle sceneHandle, SceneAccessInfo& outInfo) const {
  if (impl_ == nullptr || !impl_->IsInitialized()) {
    return Status::Error("renderer is not initialized");
  }
  return impl_->GetSceneAccessInfo(sceneHandle, outInfo);
}

Status Renderer::GetUploadedSceneInfo(UploadedSceneHandle sceneHandle, UploadedSceneInfo& outInfo) const {
  if (impl_ == nullptr || !impl_->IsInitialized()) {
    return Status::Error("renderer is not initialized");
  }
  return impl_->GetUploadedSceneInfo(sceneHandle, outInfo);
}

Status Renderer::GetUploadedChunkInfo(UploadedSceneHandle sceneHandle,
                                      UploadedChunkHandle chunkHandle,
                                      UploadedChunkInfo& outInfo) const {
  if (impl_ == nullptr || !impl_->IsInitialized()) {
    return Status::Error("renderer is not initialized");
  }
  return impl_->GetUploadedChunkInfo(sceneHandle, chunkHandle, outInfo);
}

Status Renderer::GetUploadedSceneGpuResources(UploadedSceneHandle sceneHandle,
                                              const RenderFrameContext& frameContext,
                                              UploadedSceneGpuResources& outResources) const {
  if (impl_ == nullptr || !impl_->IsInitialized()) {
    return Status::Error("renderer is not initialized");
  }
  return impl_->GetUploadedSceneGpuResources(sceneHandle, frameContext, outResources, false);
}

Status Renderer::AcquireUploadedSceneGpuResources(UploadedSceneHandle sceneHandle,
                                                  const RenderFrameContext& frameContext,
                                                  UploadedSceneGpuResources& outResources) {
  if (impl_ == nullptr || !impl_->IsInitialized()) {
    return Status::Error("renderer is not initialized");
  }
  return impl_->GetUploadedSceneGpuResources(sceneHandle, frameContext, outResources, true);
}

Status Renderer::PrepareSceneForRender(UploadedSceneHandle sceneHandle,
                                      const RenderInput& input,
                                      const RenderFrameContext& frameContext,
                                      RenderPreparationResult* outResult) {
  if (impl_ == nullptr || !impl_->IsInitialized()) {
    return Status::Error("renderer is not initialized");
  }
  RenderPreparationResult result{};
  Status prepared = impl_->PrepareSceneForRender(sceneHandle, input, &frameContext, result);
  if (prepared.ok && outResult != nullptr) {
    *outResult = result;
  }
  return prepared;
}

Status Renderer::BeginSceneMutation(UploadedSceneHandle sceneHandle, SceneMutationToken& outToken) {
  if (impl_ == nullptr || !impl_->IsInitialized()) {
    return Status::Error("renderer is not initialized");
  }
  return impl_->BeginSceneMutation(sceneHandle, outToken);
}

Status Renderer::EndSceneMutation(SceneMutationToken token) {
  if (impl_ == nullptr || !impl_->IsInitialized()) {
    return Status::Error("renderer is not initialized");
  }
  return impl_->EndSceneMutation(token);
}

Status Renderer::GetUploadedSceneChunks(UploadedSceneHandle sceneHandle, std::vector<UploadedChunkHandle>& outChunkHandles) const {
  if (impl_ == nullptr || !impl_->IsInitialized()) {
    return Status::Error("renderer is not initialized");
  }
  return impl_->GetUploadedSceneChunks(sceneHandle, outChunkHandles);
}

Status Renderer::AddUploadedChunk(UploadedSceneHandle sceneHandle,
                                  const GaussianSet& chunk,
                                  UploadedChunkHandle& outChunkHandle) {
  if (impl_ == nullptr || !impl_->IsInitialized()) {
    return Status::Error("renderer is not initialized");
  }
  return impl_->AddUploadedChunk(sceneHandle, chunk, outChunkHandle);
}

Status Renderer::AddUploadedChunk(SceneMutationToken token, const GaussianSet& chunk, UploadedChunkHandle& outChunkHandle) {
  if (impl_ == nullptr || !impl_->IsInitialized()) {
    return Status::Error("renderer is not initialized");
  }
  return impl_->AddUploadedChunk(token, chunk, outChunkHandle);
}

Status Renderer::UpdateUploadedChunk(UploadedSceneHandle sceneHandle,
                                     UploadedChunkHandle chunkHandle,
                                     const GaussianSet& chunk) {
  if (impl_ == nullptr || !impl_->IsInitialized()) {
    return Status::Error("renderer is not initialized");
  }
  return impl_->UpdateUploadedChunk(sceneHandle, chunkHandle, chunk);
}

Status Renderer::UpdateUploadedChunk(SceneMutationToken token, UploadedChunkHandle chunkHandle, const GaussianSet& chunk) {
  if (impl_ == nullptr || !impl_->IsInitialized()) {
    return Status::Error("renderer is not initialized");
  }
  return impl_->UpdateUploadedChunk(token, chunkHandle, chunk);
}

Status Renderer::RemoveUploadedChunk(UploadedSceneHandle sceneHandle, UploadedChunkHandle chunkHandle) {
  if (impl_ == nullptr || !impl_->IsInitialized()) {
    return Status::Error("renderer is not initialized");
  }
  return impl_->RemoveUploadedChunk(sceneHandle, chunkHandle);
}

Status Renderer::RemoveUploadedChunk(SceneMutationToken token, UploadedChunkHandle chunkHandle) {
  if (impl_ == nullptr || !impl_->IsInitialized()) {
    return Status::Error("renderer is not initialized");
  }
  return impl_->RemoveUploadedChunk(token, chunkHandle);
}

Status Renderer::SetUploadedChunkEnabled(UploadedSceneHandle sceneHandle, UploadedChunkHandle chunkHandle, bool enabled) {
  if (impl_ == nullptr || !impl_->IsInitialized()) {
    return Status::Error("renderer is not initialized");
  }
  return impl_->SetUploadedChunkEnabled(sceneHandle, chunkHandle, enabled);
}

Status Renderer::SetUploadedChunkEnabled(SceneMutationToken token, UploadedChunkHandle chunkHandle, bool enabled) {
  if (impl_ == nullptr || !impl_->IsInitialized()) {
    return Status::Error("renderer is not initialized");
  }
  return impl_->SetUploadedChunkEnabled(token, chunkHandle, enabled);
}

Status Renderer::SetUploadedChunkScalingModifier(UploadedSceneHandle sceneHandle,
                                                 UploadedChunkHandle chunkHandle,
                                                 float scalingModifier) {
  if (impl_ == nullptr || !impl_->IsInitialized()) {
    return Status::Error("renderer is not initialized");
  }
  return impl_->SetUploadedChunkScalingModifier(sceneHandle, chunkHandle, scalingModifier);
}

Status Renderer::SetUploadedChunkScalingModifier(SceneMutationToken token,
                                                 UploadedChunkHandle chunkHandle,
                                                 float scalingModifier) {
  if (impl_ == nullptr || !impl_->IsInitialized()) {
    return Status::Error("renderer is not initialized");
  }
  return impl_->SetUploadedChunkScalingModifier(token, chunkHandle, scalingModifier);
}

bool Renderer::IsUploadedChunkValid(UploadedSceneHandle sceneHandle, UploadedChunkHandle chunkHandle) const {
  if (impl_ == nullptr || !impl_->IsInitialized()) {
    return false;
  }
  return impl_->IsUploadedChunkValid(sceneHandle, chunkHandle);
}

Status Renderer::Render(ID3D12GraphicsCommandList* commandList,
                        const RenderTargetBinding& target,
                        UploadedSceneHandle sceneHandle,
                        const RenderInput& input,
                        const RenderFrameContext& frameContext,
                        RenderResult& outResult) {
  if (impl_ == nullptr || !impl_->IsInitialized()) {
    return Status::Error("renderer is not initialized");
  }
  return impl_->Render(commandList, target, sceneHandle, input, AdvancedRenderOptions{}, &frameContext, outResult);
}

Status Renderer::Render(ID3D12GraphicsCommandList* commandList,
                        const RenderTargetBinding& target,
                        UploadedSceneHandle sceneHandle,
                        const RenderInput& input,
                        const AdvancedRenderOptions& options,
                        const RenderFrameContext& frameContext,
                        RenderResult& outResult) {
  if (impl_ == nullptr || !impl_->IsInitialized()) {
    return Status::Error("renderer is not initialized");
  }
  return impl_->Render(commandList, target, sceneHandle, input, options, &frameContext, outResult);
}

}
