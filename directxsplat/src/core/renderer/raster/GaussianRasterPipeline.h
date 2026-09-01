#pragma once

#include <d3d12.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <unordered_map>
#include <vector>

#include <wrl/client.h>

#include "directxsplat/extensions.h"
#include "directxsplat/gpu_resources.h"
#include "directxsplat/renderer.h"
#include "directxsplat/scene.h"
#include "directxsplat/settings.h"
#include "directxsplat/status.h"

namespace directxsplat {

class OneSweep;

class GaussianRasterPipeline {
 public:
  GaussianRasterPipeline();
  ~GaussianRasterPipeline();
  Status Initialize(ID3D12Device* device,
                    ID3D12CommandQueue* queue,
                    ID3D12Fence* directFence,
                    ID3D12CommandQueue* copyQueue,
                    ID3D12Fence* uploadFence,
                    bool enableGpuTiming);
  Status Shutdown();
  Status ShutdownDeviceLost();
  void NotifyDeviceLost();
  bool IsDeviceLost() const;
  Status CreateOrUpdateScene(uint64_t sceneId, const Scene& scene, const std::vector<uint64_t>& chunkIds);
  Status DestroyScene(uint64_t sceneId);
  bool HasScene(uint64_t sceneId) const;
  Status AddChunk(uint64_t sceneId, uint64_t chunkId, const GaussianSet& chunk);
  Status UpdateChunk(uint64_t sceneId, uint64_t chunkId, const GaussianSet& chunk);
  Status RemoveChunk(uint64_t sceneId, uint64_t chunkId);
  Status SetChunkEnabled(uint64_t sceneId, uint64_t chunkId, bool enabled);
  Status SetChunkScalingModifier(uint64_t sceneId, uint64_t chunkId, float scalingModifier);
  bool HasChunk(uint64_t sceneId, uint64_t chunkId) const;
  Status Render(ID3D12GraphicsCommandList* commandList,
                const RenderTargetBinding& target,
                uint64_t sceneId,
                UploadedSceneHandle publicSceneHandle,
                const RenderInput& input,
                FrameStats& stats,
                const AdvancedRenderOptions* options,
                RenderResult* outResult,
                const RenderFrameContext* frameContext);
  Status GetSceneGpuResources(uint64_t sceneId,
                              const RenderFrameContext* frameContext,
                              bool acquireLease,
                              UploadedSceneGpuResources& out);
  Status GetChunkGpuResources(uint64_t sceneId, uint64_t chunkId, UploadedChunkGpuResources& out) const;

 private:
  struct SetParamsGpu {
    float scalingModifier = 1.0f;
    uint32_t visible = 1;
    float decodeMin[3]{};
    float decodeExtent[3]{};
    float pad[2]{};
  };

  struct ChunkPrepGpu {
    SetParamsGpu params;
    uint32_t gaussianOffset = 0;
    uint32_t gaussianCount = 0;
    uint32_t pad[2]{};
  };
  static_assert(sizeof(ChunkPrepGpu) == 56);

  struct SortMetaGpu {
    uint32_t pairCount = 0;
    uint32_t visibleCount = 0;
    uint32_t visibleBlocks = 0;
    uint32_t sortPassCount = 0;
    uint32_t sortCount = 1;
    uint32_t oneSweepPartitions = 1;
    uint32_t oneSweepGlobalHistPartitions = 1;
    uint32_t packDispatchCount = 1;
  };

  struct UploadedChunkRuntime {
    uint64_t chunkId = 0;
    uint32_t gaussianCount = 0;
    uint32_t gaussianOffset = 0;
    uint32_t gaussianCapacity = 0;
    uint32_t packedStrideBytes = 0;
    SetParamsGpu params;
    std::vector<uint8_t> packedGaussians;
    bool atlasUploadPending = false;
  };

  struct AtlasFreeRange {
    uint32_t offset = 0;
    uint32_t count = 0;
  };

  struct PrepConstants {
    float view[16]{};
    float proj[16]{};
    float cameraPos[3]{};
    float globalScale = 1.0f;
    float focalX = 1.0f;
    float focalY = 1.0f;
    float ndcX = 1.0f;
    float ndcY = 1.0f;
    float maxAxisPixels = 1.0f;
    float nearPlane = 0.01f;
    uint32_t sceneCount = 0;
    uint32_t paddedCount = 0;
    uint32_t setCount = 0;
    uint32_t fastCulling = 1;
    uint32_t renderType = 0;
    uint32_t antialiasingMode = 0;
    uint32_t shadingDegree = 3;
    uint32_t positiveViewSpaceZ = 1;
    float antialiasingStrength = 1.0f;
    uint32_t gammaCorrection = 0;
    uint32_t drawCapacity = 0;
    uint32_t pairCapacity = 0;
    uint32_t viewportWidth = 1;
    uint32_t viewportHeight = 1;
    float backgroundColor[3]{};
    float farPlane = 0.0f;
    float frustumDilation = 0.05f;
    uint32_t sceneGaussianStride = 0;
    uint32_t rgbaFormat = 1;
    uint32_t shFormat = 1;
    uint32_t rgbaOffset = 24;
    uint32_t shOffset = 32;
    uint32_t idOffset = 124;
    uint32_t chunkDispatchStride = 0;
  };
  static_assert(sizeof(PrepConstants) == 272);

  using RasterConstants = PrepConstants;

  struct RenderScratch {
    Microsoft::WRL::ComPtr<ID3D12QueryHeap> timestampQueryHeap;
    Microsoft::WRL::ComPtr<ID3D12Resource> timestampReadbackBuffer;
    Microsoft::WRL::ComPtr<ID3D12Resource> sortKeysBuffer;
    Microsoft::WRL::ComPtr<ID3D12Resource> sortKeysTempBuffer;
    Microsoft::WRL::ComPtr<ID3D12Resource> sortValuesBuffer;
    Microsoft::WRL::ComPtr<ID3D12Resource> sortValuesTempBuffer;
    Microsoft::WRL::ComPtr<ID3D12Resource> visibleCounterBuffer;
    Microsoft::WRL::ComPtr<ID3D12Resource> sortMetaBuffer;
    Microsoft::WRL::ComPtr<ID3D12Resource> oneSweepPassHistogramBuffer;
    Microsoft::WRL::ComPtr<ID3D12Resource> oneSweepGlobalHistogramBuffer;
    Microsoft::WRL::ComPtr<ID3D12Resource> oneSweepIndexBuffer;
    Microsoft::WRL::ComPtr<ID3D12Resource> oneSweepDispatchArgsBuffer;
    Microsoft::WRL::ComPtr<ID3D12Resource> sortMetaReadback;
    Microsoft::WRL::ComPtr<ID3D12Resource> projectionActiveThreadsReadback;
    Microsoft::WRL::ComPtr<ID3D12Resource> drawArgsBuffer;
    Microsoft::WRL::ComPtr<ID3D12Resource> prepConstantsUpload;
    Microsoft::WRL::ComPtr<ID3D12Resource> rasterConstantsUpload;

    D3D12_RESOURCE_STATES sortKeysState = D3D12_RESOURCE_STATE_COMMON;
    D3D12_RESOURCE_STATES sortKeysTempState = D3D12_RESOURCE_STATE_COMMON;
    D3D12_RESOURCE_STATES sortValuesState = D3D12_RESOURCE_STATE_COMMON;
    D3D12_RESOURCE_STATES sortValuesTempState = D3D12_RESOURCE_STATE_COMMON;
    D3D12_RESOURCE_STATES visibleCounterState = D3D12_RESOURCE_STATE_COMMON;
    D3D12_RESOURCE_STATES sortMetaState = D3D12_RESOURCE_STATE_COMMON;
    D3D12_RESOURCE_STATES oneSweepPassHistogramState = D3D12_RESOURCE_STATE_COMMON;
    D3D12_RESOURCE_STATES oneSweepGlobalHistogramState = D3D12_RESOURCE_STATE_COMMON;
    D3D12_RESOURCE_STATES oneSweepIndexState = D3D12_RESOURCE_STATE_COMMON;
    D3D12_RESOURCE_STATES oneSweepDispatchArgsState = D3D12_RESOURCE_STATE_COMMON;
    D3D12_RESOURCE_STATES drawArgsState = D3D12_RESOURCE_STATE_COMMON;

    uint32_t lastPairCount = 0;
    uint32_t lastVisibleCount = 0;
    uint32_t lastVisibleBlocks = 0;
    uint32_t lastSortPassCount = 0;
    std::array<uint32_t, 50> lastSplatAlphaBins{};
    std::array<uint32_t, 64> lastProjectionActiveThreadBins{};
    uint32_t sortStatsFrame = 0;
    uint32_t sortMetaCopyFrame = 0;
    Microsoft::WRL::ComPtr<ID3D12Fence> sortMetaCopyFence;
    uint64_t sortMetaCopyFenceValue = 0;
    bool sortMetaCopyPending = false;
    uint32_t gpuTimingFrame = 0;
    uint32_t timestampCopyFrame = 0;
    Microsoft::WRL::ComPtr<ID3D12Fence> timestampCopyFence;
    uint64_t timestampCopyFenceValue = 0;
    bool timestampCopyPending = false;
    bool lastSortedInPrimary = true;
    uint32_t drawCapacity = 1;
    uint32_t sortPairCapacity = 1;
    uint32_t oneSweepPartitionCount = 1;
    uint32_t prepConstantStrideBytes = 256;
    size_t prepConstantsCapacityBytes = 0;
    void* prepConstantsMapped = nullptr;
    size_t rasterConstantsCapacityBytes = 0;
    void* rasterConstantsMapped = nullptr;
    float lastGpuPrepareMs = 0.0f;
    float lastGpuSortMs = 0.0f;
    float lastGpuRasterMs = 0.0f;
    float lastGpuDepthMs = 0.0f;
    float lastGpuMs = 0.0f;
    std::shared_ptr<RenderScratch> inFlightSelf;
    Microsoft::WRL::ComPtr<ID3D12Fence> inFlightFence;
    uint64_t inFlightFenceValue = 0;
    RenderScratch* inFlightNext = nullptr;
    uint64_t lastUseCount = 0;
    uint64_t underutilizedSinceUse = 0;
  };

  struct UploadedSceneRuntime {
    std::shared_ptr<std::mutex> mutex = std::make_shared<std::mutex>();
    std::vector<UploadedChunkRuntime> chunks;
    Microsoft::WRL::ComPtr<ID3D12Resource> sceneAtlasBuffer;
    Microsoft::WRL::ComPtr<ID3D12Resource> sceneIndexToChunkBuffer;
    Microsoft::WRL::ComPtr<ID3D12Resource> batchedChunkParamsUpload;
    void* batchedChunkParamsMapped = nullptr;
    size_t batchedChunkParamsCapacityBytes = 0;
    uint32_t sceneGaussianCount = 0;
    uint32_t sceneAtlasCapacity = 0;
    uint32_t sceneAtlasTail = 0;
    VramFormatSettings vramFormat{};
    uint32_t sceneGaussianStride = 0;
    uint32_t rgbaOffset = 24;
    uint32_t shOffset = 32;
    uint32_t idOffset = 124;
    uint32_t sceneIndexToChunkCapacity = 0;
    uint32_t batchedChunkCount = 0;
    uint32_t maxPrepareGroups = 1;
    uint32_t drawCapacity = 1;
    uint32_t sortPairCapacity = 1;
    uint64_t pendingUploadFenceValue = 0;
    uint64_t directQueueUploadWaitValue = 0;
    bool sceneIndexToChunkUploadPending = true;
    std::vector<AtlasFreeRange> atlasFreeRanges;
    std::vector<std::shared_ptr<RenderScratch>> availableScratch;
    RenderScratch* inFlightScratchHead = nullptr;
    std::vector<std::shared_ptr<RenderScratch>> retainedScratch;
    std::weak_ptr<RenderScratch> publishedScratch;
    uint64_t scratchUseCount = 0;
  };

  struct RetiredResource {
    Microsoft::WRL::ComPtr<ID3D12Resource> resource;
    Microsoft::WRL::ComPtr<ID3D12Fence> fence;
    uint64_t fenceValue = 0;
    Microsoft::WRL::ComPtr<ID3D12Fence> uploadFence;
    uint64_t uploadFenceValue = 0;
  };

  struct UploadContext {
    Microsoft::WRL::ComPtr<ID3D12CommandAllocator> allocator;
    Microsoft::WRL::ComPtr<ID3D12GraphicsCommandList> commandList;
    Microsoft::WRL::ComPtr<ID3D12Resource> uploadBuffer;
    Microsoft::WRL::ComPtr<ID3D12Resource> submittedDestination;
    void* mapped = nullptr;
    size_t capacityBytes = 0;
    uint64_t fenceValue = 0;
  };

  Status CreatePipelines();
  Status EnsureColorRasterPso(DXGI_FORMAT colorFormat);
  Status EnsureDepthRasterPso(DXGI_FORMAT depthFormat);
  Status BuildChunkRuntime(uint64_t chunkId, const GaussianSet& set, VramFormatSettings format, uint32_t strideBytes, uint32_t rgbaOffset, uint32_t shOffset, uint32_t idOffset, UploadedChunkRuntime& out);
  Status AllocateAtlasRange(UploadedSceneRuntime& runtime, uint32_t count, uint32_t& outOffset);
  Status FreeAtlasRange(UploadedSceneRuntime& runtime, uint32_t offset, uint32_t count);
  Status EnsureSceneAtlasCapacity(UploadedSceneRuntime& runtime, uint32_t requiredCapacity);
  Status UploadChunkAtlasRange(UploadedSceneRuntime& sceneRuntime,
                               const UploadedChunkRuntime& chunkRuntime,
                               uint64_t* outFenceValue = nullptr);
  Status UpdateSceneCapacity(UploadedSceneRuntime& runtime);
  Status RefreshScenePrepResources(UploadedSceneRuntime& runtime, bool rebuildSceneData);
  Status EnsureRenderScratchBuffers(const UploadedSceneRuntime& runtime,
                                    uint32_t requiredPairCapacity,
                                    RenderScratch& scratch);
  void ReleaseChunkRuntime(UploadedChunkRuntime& runtime);
  Status ReleaseRenderScratchResources(RenderScratch& scratch);
  Status ReleaseSceneRuntime(UploadedSceneRuntime& runtime);
  void ReleaseUploadContextDeviceLost(UploadContext& context);
  void ReleaseRenderScratchResourcesDeviceLost(RenderScratch& scratch);
  void ReleaseSceneRuntimeDeviceLost(UploadedSceneRuntime& runtime);
  Status AcquireRenderScratch(UploadedSceneRuntime& runtime,
                              uint32_t requiredPairCapacity,
                              const RenderFrameContext* frameContext,
                              std::shared_ptr<RenderScratch>& outScratch);
  void ReleaseRenderScratch(UploadedSceneRuntime& runtime,
                            std::shared_ptr<RenderScratch> scratch,
                            const RenderFrameContext* frameContext,
                            bool trackSubmission);
  Status ReserveRetiredResourceSlots(size_t additionalCount);
  Status RetireResource(Microsoft::WRL::ComPtr<ID3D12Resource>& resource);
  Status ReleaseUploadResource(Microsoft::WRL::ComPtr<ID3D12Resource>& resource, void*& mappedPtr, size_t& capacityBytes);
  void CollectRetiredResources(uint64_t completedFenceValue);
  Status ValidateRenderFrameContext(const RenderFrameContext* frameContext, bool reserveSubmission);
  void UpdateDirectQueueFenceProgress(const RenderFrameContext* frameContext);
  void RecordDirectQueueSubmission(const RenderFrameContext* frameContext);
  uint64_t CurrentCompletedDirectFenceValue() const;
  uint64_t CurrentSubmittedDirectFenceValue() const;
  void CollectRuntimeScratch(UploadedSceneRuntime& runtime);
  void TrimAvailableRuntimeScratch(UploadedSceneRuntime& runtime, uint64_t currentUseCount);
  Status EnsureUploadBuffer(size_t requiredBytes,
                            Microsoft::WRL::ComPtr<ID3D12Resource>& resource,
                            size_t& capacityBytes,
                            void*& mappedPtr);
  Status CreateDefaultBuffer(size_t bytes,
                             D3D12_RESOURCE_FLAGS flags,
                             D3D12_RESOURCE_STATES initialState,
                             Microsoft::WRL::ComPtr<ID3D12Resource>& out);
  Status CreateReadbackBuffer(size_t bytes, Microsoft::WRL::ComPtr<ID3D12Resource>& out);
  Status UploadBufferQueued(ID3D12Resource* dst,
                            uint64_t dstOffsetBytes,
                            const void* srcData,
                            size_t bytes,
                            uint64_t* outFenceValue = nullptr);
  Status CreateUploadContext(UploadContext& out);
  Status AcquireUploadContext(size_t requiredBytes, UploadContext*& out);
  Status EnsureUploadCommandObjects();
  Status WaitUploadQueue();
  Status ShutdownInternal(bool deviceLostCleanup);
  void Transition(ID3D12GraphicsCommandList* cmd, ID3D12Resource* resource,
                  D3D12_RESOURCE_STATES before, D3D12_RESOURCE_STATES after);
  uint32_t NextPowerOfTwo(uint32_t v) const;
  GpuBufferView MakeBufferView(ID3D12Resource* resource,
                               D3D12_RESOURCE_STATES state,
                               uint64_t sizeBytes,
                               uint32_t strideBytes,
                               GpuViewLifetime lifetime,
                               GpuResourceAccess access,
                               bool callerMayTransition,
                               bool callerMayWrite) const;
  GpuBufferView MakeBufferView(ID3D12Resource* resource,
                               D3D12_RESOURCE_STATES state,
                               uint64_t byteOffset,
                               uint64_t sizeBytes,
                               uint32_t strideBytes,
                               GpuViewLifetime lifetime,
                               GpuResourceAccess access,
                               bool callerMayTransition,
                               bool callerMayWrite) const;
  GpuTextureView MakeTextureView(ID3D12Resource* resource,
                                 DXGI_FORMAT format,
                                 uint32_t width,
                                 uint32_t height,
                                 D3D12_CPU_DESCRIPTOR_HANDLE rtv,
                                 D3D12_CPU_DESCRIPTOR_HANDLE dsv,
                                 D3D12_GPU_DESCRIPTOR_HANDLE srv,
                                 D3D12_RESOURCE_STATES state,
                                 GpuResourceAccess access,
                                 GpuViewLifetime lifetime,
                                 bool callerMayTransition,
                                 bool callerMayWrite) const;
  void PopulateFrameResources(const RenderTargetBinding& target,
                              const UploadedSceneRuntime& runtime,
                              const RenderScratch& scratch,
                              D3D12_RESOURCE_STATES colorState,
                              D3D12_RESOURCE_STATES depthState,
                              D3D12_RESOURCE_STATES motionState,
                              GpuFrameResources& out) const;
  Status InvokeHook(const std::function<void(const RenderHookContext&)>& hook,
                    RenderHookStage stage,
                    ID3D12GraphicsCommandList* commandList,
                    UploadedSceneHandle sceneHandle,
                    const RenderInput& input,
                    const RenderTargetBinding& target,
                    const GpuFrameResources& resources,
                    FrameStats& stats) const;

  Microsoft::WRL::ComPtr<ID3D12Device> device_;
  Microsoft::WRL::ComPtr<ID3D12CommandQueue> queue_;
  Microsoft::WRL::ComPtr<ID3D12CommandQueue> copyQueue_;
  mutable std::shared_mutex uploadedScenesMutex_;
  std::unordered_map<uint64_t, std::shared_ptr<UploadedSceneRuntime>> uploadedScenes_;

  Microsoft::WRL::ComPtr<ID3D12RootSignature> prepRootSignature_;
  Microsoft::WRL::ComPtr<ID3D12RootSignature> fillSortTailRootSignature_;
  Microsoft::WRL::ComPtr<ID3D12RootSignature> sortMetaRootSignature_;
  Microsoft::WRL::ComPtr<ID3D12RootSignature> buildSortDispatchArgsRootSignature_;
  Microsoft::WRL::ComPtr<ID3D12RootSignature> finalizeRootSignature_;
  Microsoft::WRL::ComPtr<ID3D12RootSignature> rasterRootSignature_;

  Microsoft::WRL::ComPtr<ID3D12PipelineState> prepPso_;
  Microsoft::WRL::ComPtr<ID3D12PipelineState> fillSortTailPso_;
  Microsoft::WRL::ComPtr<ID3D12PipelineState> sortMetaPso_;
  Microsoft::WRL::ComPtr<ID3D12PipelineState> buildSortDispatchArgsPso_;
  Microsoft::WRL::ComPtr<ID3D12PipelineState> resetPso_;
  Microsoft::WRL::ComPtr<ID3D12PipelineState> finalizePso_;
  mutable std::mutex colorRasterMutex_;
  std::unordered_map<int, Microsoft::WRL::ComPtr<ID3D12PipelineState>> colorRasterPsos_;
  mutable std::mutex depthRasterMutex_;
  std::unordered_map<int, Microsoft::WRL::ComPtr<ID3D12PipelineState>> depthRasterPsos_;

  Microsoft::WRL::ComPtr<ID3D12CommandSignature> drawCommandSignature_;
  std::unique_ptr<directxsplat::OneSweep> oneSweep_;

  mutable std::recursive_mutex uploadMutex_;
  Microsoft::WRL::ComPtr<ID3D12Fence> uploadFence_;
  HANDLE uploadFenceEvent_ = nullptr;
  uint64_t uploadFenceValue_ = 0;
  uint64_t directQueueCompletedFenceValue_ = 0;
  uint64_t directQueueReservedFenceValue_ = 0;
  uint64_t directQueueSubmittedFenceValue_ = 0;
  Microsoft::WRL::ComPtr<ID3D12Fence> directQueueFence_;
  double gpuTimestampMsPerTick_ = 0.0;
  std::vector<RetiredResource> retiredResources_;
  std::vector<RetiredResource> untrackedRetiredResources_;
  std::vector<UploadContext> uploadContexts_;
  bool uploadQueueFailed_ = false;
  bool deviceLost_ = false;
  bool gpuTimingEnabled_ = true;
};

}  // namespace directxsplat
