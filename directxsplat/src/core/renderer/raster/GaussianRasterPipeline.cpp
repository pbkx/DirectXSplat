#include "renderer/raster/GaussianRasterPipeline.h"
#include "EmbeddedShaders.h"

#include <Windows.h>
#include <d3dcompiler.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <exception>
#include <limits>
#include <new>
#include <stdexcept>

#include <DirectXPackedVector.h>

#include "renderer/sort/OneSweep.h"

namespace directxsplat {

namespace {

using Microsoft::WRL::ComPtr;
using DirectX::PackedVector::XMConvertFloatToHalf;

constexpr uint32_t kSortPairMultiplier = 1u;
constexpr uint32_t kMinSortPairCapacity = 1024;
constexpr uint32_t kMaxSortPairCapacity = 16u * 1024u * 1024u;
constexpr uint32_t kOneSweepRadix = 256u;
constexpr uint32_t kOneSweepPassCount = 4u;
constexpr uint32_t kOneSweepIndirectCommandStride = sizeof(uint32_t) * 7u;
constexpr uint32_t kOneSweepInitCommandIndex = 0u;
constexpr uint32_t kOneSweepGlobalHistogramCommandIndex = 1u;
constexpr uint32_t kOneSweepScanCommandIndex = 2u;
constexpr uint32_t kOneSweepDigitCommandIndex = 3u;
constexpr uint32_t kSortIndirectCommandCount = kOneSweepDigitCommandIndex + kOneSweepPassCount;
constexpr uint32_t kSortStatsReadbackPeriod = 1;
constexpr float kPackedMipFilterFraction = 0.18f;
constexpr uint32_t kMaxSceneIndexToChunkEntries = 64u * 1024u * 1024u;
constexpr uint32_t kSplatAlphaHistogramBins = 50u;
constexpr uint32_t kSplatAlphaHistogramOffset = 8u;
constexpr uint32_t kSplatAlphaHistogramBytes = kSplatAlphaHistogramBins * sizeof(uint32_t);
constexpr uint32_t kProjectionActiveThreadBins = 64u;
constexpr uint32_t kProjectionActiveThreadHistogramOffset = kSplatAlphaHistogramOffset + kSplatAlphaHistogramBytes;
constexpr uint32_t kProjectionActiveThreadHistogramBytes = kProjectionActiveThreadBins * sizeof(uint32_t);
constexpr uint32_t kStatsHistogramBytes = kSplatAlphaHistogramBytes + kProjectionActiveThreadHistogramBytes;
constexpr uint32_t kVisibleCounterBytes = kProjectionActiveThreadHistogramOffset + kProjectionActiveThreadHistogramBytes;
constexpr size_t kRenderScratchRetiredResourceSlots = 16;
constexpr DWORD kFenceWaitPollMs = 50;

constexpr size_t AlignUp(size_t value, size_t alignment) {
  return (value + alignment - 1u) & ~(alignment - 1u);
}

constexpr int ColorPsoKey(DXGI_FORMAT colorFormat) {
  return static_cast<int>(colorFormat);
}

enum TimestampQueryIndex : uint32_t {
  kTimestampFrameBegin = 0,
  kTimestampPrepareBegin,
  kTimestampPrepareEnd,
  kTimestampSortBegin,
  kTimestampSortEnd,
  kTimestampRasterBegin,
  kTimestampRasterEnd,
  kTimestampDepthBegin,
  kTimestampDepthEnd,
  kTimestampFrameEnd,
  kTimestampQueryCount,
};

bool IsFinite(float v) {
  return std::isfinite(v);
}

bool IsFinite(const Vec3& v) {
  return IsFinite(v.x) && IsFinite(v.y) && IsFinite(v.z);
}

bool IsFinite(const Quat& q) {
  return IsFinite(q.x) && IsFinite(q.y) && IsFinite(q.z) && IsFinite(q.w);
}

Aabb BoundsFromGaussians(const std::vector<Gaussian>& gaussians) {
  std::vector<Vec3> points;
  points.reserve(gaussians.size());
  for (const Gaussian& g : gaussians) {
    if (IsFinite(g.position)) {
      points.push_back(g.position);
    }
  }
  return ComputeAabb(points);
}

float DecodeScaleValue(float raw) {
  if (!IsFinite(raw)) {
    return 1e-4f;
  }
  if (raw <= 0.0f) {
    return std::max(std::exp(std::clamp(raw, -14.0f, 8.0f)), 1e-4f);
  }
  return std::max(raw, 1e-4f);
}

uint16_t FloatToHalfBits(float v) {
  return XMConvertFloatToHalf(v);
}

uint32_t PackHalf2(float a, float b) {
  return static_cast<uint32_t>(FloatToHalfBits(a)) | (static_cast<uint32_t>(FloatToHalfBits(b)) << 16u);
}

uint16_t PackUnorm16(float v) {
  const float clamped = std::clamp(v, 0.0f, 1.0f);
  return static_cast<uint16_t>(std::lround(clamped * 65535.0f));
}

uint16_t PackSnorm16(float v) {
  const float clamped = std::clamp(v, -1.0f, 1.0f);
  const int scaled = static_cast<int>(std::lround(clamped * 32767.0f));
  return static_cast<uint16_t>(scaled & 0xFFFF);
}

uint32_t PackSnorm16Pair(float a, float b) {
  return static_cast<uint32_t>(PackSnorm16(a)) | (static_cast<uint32_t>(PackSnorm16(b)) << 16u);
}

constexpr uint32_t kPackedCommonBytes = 24u;
constexpr uint32_t kPackedShCoeffCount = 45u;
constexpr float kPackedShUint8Range = 4.0f;

constexpr uint32_t Align4(uint32_t v) {
  return (v + 3u) & ~3u;
}

uint32_t AttributeBytes(VramAttributeFormat format) {
  switch (SanitizeVramAttributeFormat(format)) {
    case VramAttributeFormat::Float32:
      return 4u;
    case VramAttributeFormat::Float16:
      return 2u;
    case VramAttributeFormat::Uint8:
      return 1u;
    default:
      return 2u;
  }
}

struct PackedLayout {
  uint32_t stride = 0;
  uint32_t rgbaOffset = kPackedCommonBytes;
  uint32_t shOffset = 0;
  uint32_t idOffset = 0;
};

PackedLayout ComputePackedLayout(VramFormatSettings settings) {
  settings = SanitizeVramFormatSettings(settings);
  PackedLayout layout{};
  layout.rgbaOffset = kPackedCommonBytes;
  const uint32_t rgbaBytes = 4u * AttributeBytes(settings.rgbaFormat);
  layout.shOffset = Align4(layout.rgbaOffset + rgbaBytes);
  const uint32_t shBytes = kPackedShCoeffCount * AttributeBytes(settings.shFormat);
  layout.idOffset = Align4(layout.shOffset + shBytes);
  layout.stride = Align4(layout.idOffset + 2u * sizeof(uint32_t));
  return layout;
}

uint32_t MaxShaderAddressableGaussians(uint32_t strideBytes) {
  if (strideBytes == 0) {
    return 0;
  }
  return std::numeric_limits<uint32_t>::max() / strideBytes;
}

uint8_t PackUnorm8(float v) {
  return static_cast<uint8_t>(std::lround(std::clamp(v, 0.0f, 1.0f) * 255.0f));
}

uint8_t PackShUint8(float v) {
  const float normalized = (std::clamp(v, -kPackedShUint8Range, kPackedShUint8Range) / kPackedShUint8Range) * 0.5f + 0.5f;
  return PackUnorm8(normalized);
}

void StoreU32(std::vector<uint8_t>& bytes, size_t byteOffset, uint32_t value) {
  std::memcpy(bytes.data() + byteOffset, &value, sizeof(value));
}

void StoreF32(std::vector<uint8_t>& bytes, size_t byteOffset, float value) {
  std::memcpy(bytes.data() + byteOffset, &value, sizeof(value));
}

void StoreF16(std::vector<uint8_t>& bytes, size_t byteOffset, float value) {
  const uint16_t h = FloatToHalfBits(value);
  std::memcpy(bytes.data() + byteOffset, &h, sizeof(h));
}

void StoreAttributeFloat(std::vector<uint8_t>& bytes, size_t byteOffset, VramAttributeFormat format, float value, bool shCoefficient) {
  switch (SanitizeVramAttributeFormat(format)) {
    case VramAttributeFormat::Float32:
      StoreF32(bytes, byteOffset, value);
      break;
    case VramAttributeFormat::Float16:
      StoreF16(bytes, byteOffset, value);
      break;
    case VramAttributeFormat::Uint8:
      bytes[byteOffset] = shCoefficient ? PackShUint8(value) : PackUnorm8(value);
      break;
  }
}

uint32_t UploadedSceneBufferFormatCode(VramFormatSettings settings) {
  settings = SanitizeVramFormatSettings(settings);
  return (static_cast<uint32_t>(settings.rgbaFormat) << 8u) | static_cast<uint32_t>(settings.shFormat);
}

Status CompileShader(const char* sourceName,
                     const char* source,
                     size_t sourceSize,
                     const char* entry,
                     const char* profile,
                     ComPtr<ID3DBlob>& blob) {
  ComPtr<ID3DBlob> errorBlob;
  UINT flags = D3DCOMPILE_ENABLE_STRICTNESS;
#if defined(NDEBUG)
  flags |= D3DCOMPILE_OPTIMIZATION_LEVEL3;
#endif
  const HRESULT hr = D3DCompile(source, sourceSize, sourceName, nullptr, nullptr, entry, profile, flags, 0,
                                blob.GetAddressOf(), errorBlob.GetAddressOf());
  if (FAILED(hr)) {
    std::string message = std::string("shader compile failed: ") + entry + " [" + sourceName + "]";
    if (errorBlob != nullptr && errorBlob->GetBufferPointer() != nullptr && errorBlob->GetBufferSize() > 0) {
      message += " : ";
      message.append(static_cast<const char*>(errorBlob->GetBufferPointer()), errorBlob->GetBufferSize());
    }
    return Status::Error(std::move(message));
  }
  return Status::Ok();
}

}

GaussianRasterPipeline::GaussianRasterPipeline() = default;

GaussianRasterPipeline::~GaussianRasterPipeline() = default;

Status GaussianRasterPipeline::Initialize(ID3D12Device* device,
                                            ID3D12CommandQueue* queue,
                                            ID3D12Fence* directFence,
                                            ID3D12CommandQueue* externalCopyQueue,
                                            ID3D12Fence* externalUploadFence,
                                            bool enableGpuTiming) try {
  if (device == nullptr || queue == nullptr || directFence == nullptr) {
    return Status::Error("invalid D3D12 device/queue");
  }
  if ((externalCopyQueue == nullptr) != (externalUploadFence == nullptr)) {
    return Status::Error("copy queue and upload fence must both be provided");
  }
  {
    std::shared_lock<std::shared_mutex> scenesLock(uploadedScenesMutex_);
    if (!uploadedScenes_.empty()) {
      return Status::Error("raster pipeline is already initialized");
    }
  }
  {
    std::lock_guard<std::recursive_mutex> uploadLock(uploadMutex_);
    if (device_.Get() != nullptr || queue_.Get() != nullptr || copyQueue_ != nullptr || uploadFence_ != nullptr ||
        uploadFenceEvent_ != nullptr || !uploadContexts_.empty() || !retiredResources_.empty() ||
        !untrackedRetiredResources_.empty()) {
      return Status::Error("raster pipeline is already initialized");
    }
  }
  device_ = device;
  queue_ = queue;
  directQueueFence_ = directFence;
  copyQueue_ = externalCopyQueue;
  uploadFence_ = externalUploadFence;
  gpuTimingEnabled_ = enableGpuTiming;
  deviceLost_ = false;
  uploadQueueFailed_ = false;
  uploadFenceValue_ = 0;
  uint64_t timestampFrequency = 0;
  if (FAILED(queue_->GetTimestampFrequency(&timestampFrequency)) || timestampFrequency == 0) {
    ShutdownInternal(true);
    return Status::Error("failed querying timestamp frequency");
  }
  gpuTimestampMsPerTick_ = 1000.0 / static_cast<double>(timestampFrequency);

  Status s = EnsureUploadCommandObjects();
  if (!s.ok) {
    ShutdownInternal(true);
    return s;
  }
  uploadFenceValue_ = uploadFence_ != nullptr ? uploadFence_->GetCompletedValue() : 0;
  s = CreatePipelines();
  if (!s.ok) {
    ShutdownInternal(true);
    return s;
  }
  return Status::Ok();
} catch (const std::bad_alloc&) {
  ShutdownInternal(true);
  return Status::Error("raster pipeline allocation failed");
} catch (const std::length_error&) {
  ShutdownInternal(true);
  return Status::Error("raster pipeline allocation failed");
} catch (const std::exception&) {
  ShutdownInternal(true);
  return Status::Error("raster pipeline initialization failed");
} catch (...) {
  ShutdownInternal(true);
  return Status::Error("raster pipeline initialization failed");
}

Status GaussianRasterPipeline::Shutdown() {
  if (IsDeviceLost()) {
    return ShutdownDeviceLost();
  }
  Status idle = WaitUploadQueue();
  if (!idle.ok) {
    return idle;
  }
  return ShutdownInternal(false);
}

Status GaussianRasterPipeline::ShutdownDeviceLost() {
  Status cleanup = ShutdownInternal(true);
  if (!cleanup.ok) {
    return cleanup;
  }
  return Status::Error("renderer device lost");
}

void GaussianRasterPipeline::NotifyDeviceLost() {
  std::lock_guard<std::recursive_mutex> uploadLock(uploadMutex_);
  deviceLost_ = true;
  uploadQueueFailed_ = true;
}

bool GaussianRasterPipeline::IsDeviceLost() const {
  std::lock_guard<std::recursive_mutex> uploadLock(uploadMutex_);
  return deviceLost_ || uploadQueueFailed_;
}

Status GaussianRasterPipeline::ShutdownInternal(bool deviceLostCleanup) {
  Status shutdownStatus = Status::Ok();

  std::unique_lock<std::shared_mutex> scenesLock(uploadedScenesMutex_);
  for (auto& [id, runtime] : uploadedScenes_) {
    (void)id;
    if (runtime) {
      if (deviceLostCleanup) {
        ReleaseSceneRuntimeDeviceLost(*runtime);
      } else {
        Status released = ReleaseSceneRuntime(*runtime);
        if (!released.ok) {
          return released;
        }
      }
    }
  }
  uploadedScenes_.clear();
  scenesLock.unlock();
  if (!deviceLostCleanup) {
    Status idle = WaitUploadQueue();
    if (!idle.ok && shutdownStatus.ok) {
      shutdownStatus = idle;
    }
  }
  {
    std::lock_guard<std::recursive_mutex> uploadLock(uploadMutex_);
    retiredResources_.clear();
    untrackedRetiredResources_.clear();
    for (UploadContext& context : uploadContexts_) {
      ReleaseUploadContextDeviceLost(context);
    }
    uploadContexts_.clear();
  }

  if (uploadFenceEvent_ != nullptr) {
    CloseHandle(uploadFenceEvent_);
    uploadFenceEvent_ = nullptr;
  }

  uploadFence_.Reset();
  copyQueue_.Reset();
  uploadFenceValue_ = 0;
  if (!deviceLostCleanup) {
    uploadQueueFailed_ = false;
    deviceLost_ = false;
  }
  directQueueCompletedFenceValue_ = 0;
  directQueueReservedFenceValue_ = 0;
  directQueueSubmittedFenceValue_ = 0;
  directQueueFence_.Reset();
  gpuTimingEnabled_ = true;

  drawCommandSignature_.Reset();
  prepRootSignature_.Reset();
  fillSortTailRootSignature_.Reset();
  sortMetaRootSignature_.Reset();
  buildSortDispatchArgsRootSignature_.Reset();
  finalizeRootSignature_.Reset();
  rasterRootSignature_.Reset();
  prepPso_.Reset();
  fillSortTailPso_.Reset();
  sortMetaPso_.Reset();
  buildSortDispatchArgsPso_.Reset();
  resetPso_.Reset();
  finalizePso_.Reset();
  if (oneSweep_ != nullptr) {
    oneSweep_->Shutdown();
    oneSweep_.reset();
  }
  {
    std::lock_guard<std::mutex> colorLock(colorRasterMutex_);
    colorRasterPsos_.clear();
  }
  {
    std::lock_guard<std::mutex> depthLock(depthRasterMutex_);
    depthRasterPsos_.clear();
  }

  device_.Reset();
  queue_.Reset();
  gpuTimestampMsPerTick_ = 0.0;
  return shutdownStatus;
}

Status GaussianRasterPipeline::EnsureUploadCommandObjects() {
  if (deviceLost_ || uploadQueueFailed_) {
    return Status::Error("upload queue is lost");
  }
  if (copyQueue_ != nullptr && uploadFence_ != nullptr && uploadFenceEvent_ != nullptr) {
    return Status::Ok();
  }

  HRESULT hr = S_OK;
  if (copyQueue_ == nullptr) {
    D3D12_COMMAND_QUEUE_DESC queueDesc{};
    queueDesc.Type = D3D12_COMMAND_LIST_TYPE_COPY;
    hr = device_->CreateCommandQueue(&queueDesc, IID_PPV_ARGS(copyQueue_.GetAddressOf()));
    if (FAILED(hr)) {
      return Status::Error("failed creating copy queue");
    }
  }

  if (uploadFence_ == nullptr) {
    hr = device_->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(uploadFence_.GetAddressOf()));
    if (FAILED(hr)) {
      return Status::Error("failed creating upload fence");
    }
  }

  uploadFenceEvent_ = CreateEvent(nullptr, FALSE, FALSE, nullptr);
  if (uploadFenceEvent_ == nullptr) {
    return Status::Error("failed creating upload fence event");
  }

  return Status::Ok();
}

Status GaussianRasterPipeline::WaitUploadQueue() {
  std::lock_guard<std::recursive_mutex> uploadLock(uploadMutex_);
  auto failLost = [&]() {
    deviceLost_ = true;
    uploadQueueFailed_ = true;
    return Status::Error("upload queue is lost");
  };
  auto checkDevice = [&]() -> Status {
    if (deviceLost_ || uploadQueueFailed_) {
      return Status::Error("upload queue is lost");
    }
    if (device_ != nullptr) {
      const HRESULT removed = device_->GetDeviceRemovedReason();
      if (FAILED(removed)) {
        return failLost();
      }
    }
    return Status::Ok();
  };
  if (deviceLost_ || uploadQueueFailed_) {
    return Status::Error("upload queue is lost");
  }
  if (queue_.Get() == nullptr || uploadFence_ == nullptr || uploadFenceEvent_ == nullptr) {
    return Status::Ok();
  }
  if (uploadFence_->GetCompletedValue() < uploadFenceValue_) {
    Status deviceStatus = checkDevice();
    if (!deviceStatus.ok) {
      return deviceStatus;
    }
    const HRESULT hr = uploadFence_->SetEventOnCompletion(uploadFenceValue_, uploadFenceEvent_);
    if (FAILED(hr)) {
      deviceStatus = checkDevice();
      return deviceStatus.ok ? Status::Error("failed waiting for upload fence") : deviceStatus;
    }
    while (uploadFence_->GetCompletedValue() < uploadFenceValue_) {
      const DWORD wait = WaitForSingleObject(uploadFenceEvent_, kFenceWaitPollMs);
      if (wait == WAIT_OBJECT_0) {
        break;
      }
      if (wait != WAIT_TIMEOUT) {
        deviceLost_ = true;
        uploadQueueFailed_ = true;
        return Status::Error("failed waiting for upload fence");
      }
      deviceStatus = checkDevice();
      if (!deviceStatus.ok) {
        return deviceStatus;
      }
    }
  }
  CollectRetiredResources(directQueueCompletedFenceValue_);
  return Status::Ok();
}

void GaussianRasterPipeline::CollectRetiredResources(uint64_t completedFenceValue) {
  std::lock_guard<std::recursive_mutex> uploadLock(uploadMutex_);
  directQueueCompletedFenceValue_ = std::max(directQueueCompletedFenceValue_, completedFenceValue);
  if (directQueueFence_ != nullptr) {
    directQueueCompletedFenceValue_ = std::max(directQueueCompletedFenceValue_, directQueueFence_->GetCompletedValue());
  }
  uint64_t completedUploadFenceValue = 0;
  if (uploadFence_ != nullptr) {
    completedUploadFenceValue = uploadFence_->GetCompletedValue();
  }
  const uint64_t completedDirectFenceValue = directQueueCompletedFenceValue_;
  for (UploadContext& context : uploadContexts_) {
    if (context.fenceValue != std::numeric_limits<uint64_t>::max() &&
        context.fenceValue <= completedUploadFenceValue) {
      context.submittedDestination.Reset();
    }
  }
  retiredResources_.erase(std::remove_if(retiredResources_.begin(), retiredResources_.end(),
                                         [completedDirectFenceValue, completedUploadFenceValue](
                                             const RetiredResource& item) {
                                           const bool directComplete =
                                               item.fenceValue == 0 ||
                                               completedDirectFenceValue >= item.fenceValue ||
                                               (item.fence != nullptr && item.fence->GetCompletedValue() >= item.fenceValue);
                                           const bool uploadComplete =
                                               item.uploadFenceValue == 0 ||
                                               completedUploadFenceValue >= item.uploadFenceValue ||
                                               (item.uploadFence != nullptr &&
                                                item.uploadFence->GetCompletedValue() >= item.uploadFenceValue);
                                           return directComplete && uploadComplete;
                                         }),
                          retiredResources_.end());
}

Status GaussianRasterPipeline::ReserveRetiredResourceSlots(size_t additionalCount) {
  std::lock_guard<std::recursive_mutex> uploadLock(uploadMutex_);
  if (deviceLost_ || uploadQueueFailed_) {
    return Status::Ok();
  }
  if (additionalCount > std::numeric_limits<size_t>::max() - retiredResources_.size() ||
      additionalCount > std::numeric_limits<size_t>::max() - untrackedRetiredResources_.size()) {
    return Status::Error("resource retirement allocation failed");
  }
  try {
    retiredResources_.reserve(retiredResources_.size() + additionalCount);
    untrackedRetiredResources_.reserve(untrackedRetiredResources_.size() + additionalCount);
  } catch (const std::bad_alloc&) {
    return Status::Error("resource retirement allocation failed");
  } catch (const std::length_error&) {
    return Status::Error("resource retirement allocation failed");
  }
  return Status::Ok();
}

Status GaussianRasterPipeline::RetireResource(ComPtr<ID3D12Resource>& resource) {
  std::lock_guard<std::recursive_mutex> uploadLock(uploadMutex_);
  if (resource == nullptr) {
    return Status::Ok();
  }
  RetiredResource retired{};
  retired.resource = resource;
  retired.fence = directQueueFence_;
  retired.fenceValue = directQueueSubmittedFenceValue_;
  retired.uploadFence = uploadFence_;
  retired.uploadFenceValue = uploadFenceValue_;
  try {
    if (deviceLost_ || uploadQueueFailed_) {
      untrackedRetiredResources_.push_back(std::move(retired));
    } else if ((retired.fence != nullptr && retired.fenceValue != 0) ||
               (retired.uploadFence != nullptr && retired.uploadFenceValue != 0)) {
      retiredResources_.push_back(std::move(retired));
    } else {
      untrackedRetiredResources_.push_back(std::move(retired));
    }
  } catch (const std::bad_alloc&) {
    return Status::Error("resource retirement allocation failed");
  } catch (const std::length_error&) {
    return Status::Error("resource retirement allocation failed");
  }
  resource.Reset();
  return Status::Ok();
}

Status GaussianRasterPipeline::ReleaseUploadResource(ComPtr<ID3D12Resource>& resource, void*& mappedPtr, size_t& capacityBytes) {
  if (resource != nullptr && mappedPtr != nullptr) {
    resource->Unmap(0, nullptr);
  }
  mappedPtr = nullptr;
  capacityBytes = 0;
  return RetireResource(resource);
}

Status GaussianRasterPipeline::ValidateRenderFrameContext(const RenderFrameContext* frameContext,
                                                          bool reserveSubmission) {
  if (frameContext == nullptr || frameContext->fence == nullptr || frameContext->submissionFenceValue == 0) {
    return Status::Error("render frame context requires a fence and submission value");
  }
  std::lock_guard<std::recursive_mutex> uploadLock(uploadMutex_);
  if (deviceLost_ || uploadQueueFailed_) {
    return Status::Error("renderer device lost");
  }
  const uint64_t completedFenceValue =
      std::max(frameContext->completedFenceValue, frameContext->fence->GetCompletedValue());
  if (frameContext->submissionFenceValue <= completedFenceValue) {
    return Status::Error("render frame context submission value is already completed");
  }
  if (directQueueFence_ == nullptr) {
    return Status::Error("render frame context fence is not registered");
  }
  if (directQueueFence_.Get() != frameContext->fence) {
    return Status::Error("render frame context fence changed");
  }
  if (reserveSubmission) {
    if (frameContext->submissionFenceValue <= directQueueReservedFenceValue_) {
      return Status::Error("render frame context submission value was already reserved");
    }
    directQueueReservedFenceValue_ = frameContext->submissionFenceValue;
  }
  return Status::Ok();
}

void GaussianRasterPipeline::UpdateDirectQueueFenceProgress(const RenderFrameContext* frameContext) {
  if (frameContext == nullptr) {
    return;
  }
  uint64_t completedFenceValue = frameContext->completedFenceValue;
  if (frameContext->fence != nullptr) {
    completedFenceValue = std::max(completedFenceValue, frameContext->fence->GetCompletedValue());
  }
  {
    std::lock_guard<std::recursive_mutex> uploadLock(uploadMutex_);
    directQueueCompletedFenceValue_ = std::max(directQueueCompletedFenceValue_, completedFenceValue);
  }
  if (completedFenceValue != 0) {
    CollectRetiredResources(completedFenceValue);
  }
}

void GaussianRasterPipeline::RecordDirectQueueSubmission(const RenderFrameContext* frameContext) {
  if (frameContext == nullptr || frameContext->fence == nullptr || frameContext->submissionFenceValue == 0) {
    return;
  }
  std::lock_guard<std::recursive_mutex> uploadLock(uploadMutex_);
  if (directQueueFence_.Get() == frameContext->fence) {
    directQueueSubmittedFenceValue_ = std::max(directQueueSubmittedFenceValue_, frameContext->submissionFenceValue);
  }
}

uint64_t GaussianRasterPipeline::CurrentCompletedDirectFenceValue() const {
  std::lock_guard<std::recursive_mutex> uploadLock(uploadMutex_);
  return directQueueCompletedFenceValue_;
}

uint64_t GaussianRasterPipeline::CurrentSubmittedDirectFenceValue() const {
  std::lock_guard<std::recursive_mutex> uploadLock(uploadMutex_);
  return directQueueSubmittedFenceValue_;
}

void GaussianRasterPipeline::CollectRuntimeScratch(UploadedSceneRuntime& runtime) {
  std::lock_guard<std::mutex> runtimeLock(*runtime.mutex);
  RenderScratch** current = &runtime.inFlightScratchHead;
  while (*current != nullptr) {
    RenderScratch* scratch = *current;
    if (scratch->inFlightFence != nullptr && scratch->inFlightFenceValue != 0 &&
        scratch->inFlightFence->GetCompletedValue() >= scratch->inFlightFenceValue) {
      std::shared_ptr<RenderScratch> keepAlive = scratch->inFlightSelf;
      try {
        runtime.availableScratch.push_back(keepAlive);
      } catch (...) {
        current = &scratch->inFlightNext;
        continue;
      }
      *current = scratch->inFlightNext;
      scratch->inFlightSelf.reset();
      scratch->inFlightFence.Reset();
      scratch->inFlightFenceValue = 0;
      scratch->inFlightNext = nullptr;
    } else {
      current = &scratch->inFlightNext;
    }
  }
}

Status GaussianRasterPipeline::CreateDefaultBuffer(size_t bytes,
                                                   D3D12_RESOURCE_FLAGS flags,
                                                   D3D12_RESOURCE_STATES initialState,
                                                   ComPtr<ID3D12Resource>& out) {
  out.Reset();

  D3D12_HEAP_PROPERTIES heap{};
  heap.Type = D3D12_HEAP_TYPE_DEFAULT;
  heap.CreationNodeMask = 1;
  heap.VisibleNodeMask = 1;

  D3D12_RESOURCE_DESC desc{};
  desc.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
  desc.Width = std::max<size_t>(bytes, 4u);
  desc.Height = 1;
  desc.DepthOrArraySize = 1;
  desc.MipLevels = 1;
  desc.SampleDesc.Count = 1;
  desc.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
  desc.Flags = flags;

  const HRESULT hr = device_->CreateCommittedResource(&heap, D3D12_HEAP_FLAG_NONE, &desc, initialState,
                                                      nullptr, IID_PPV_ARGS(out.GetAddressOf()));
  if (FAILED(hr)) {
    return Status::Error("failed creating default buffer");
  }
  return Status::Ok();
}

Status GaussianRasterPipeline::CreateReadbackBuffer(size_t bytes, ComPtr<ID3D12Resource>& out) {
  out.Reset();

  D3D12_HEAP_PROPERTIES heap{};
  heap.Type = D3D12_HEAP_TYPE_READBACK;
  heap.CreationNodeMask = 1;
  heap.VisibleNodeMask = 1;

  D3D12_RESOURCE_DESC desc{};
  desc.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
  desc.Width = std::max<size_t>(bytes, 4u);
  desc.Height = 1;
  desc.DepthOrArraySize = 1;
  desc.MipLevels = 1;
  desc.SampleDesc.Count = 1;
  desc.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;

  const HRESULT hr = device_->CreateCommittedResource(&heap, D3D12_HEAP_FLAG_NONE, &desc, D3D12_RESOURCE_STATE_COPY_DEST,
                                                      nullptr, IID_PPV_ARGS(out.GetAddressOf()));
  if (FAILED(hr)) {
    return Status::Error("failed creating readback buffer");
  }
  return Status::Ok();
}

Status GaussianRasterPipeline::UploadBufferQueued(ID3D12Resource* dst,
                                                  uint64_t dstOffsetBytes,
                                                  const void* srcData,
                                                  size_t bytes,
                                                  uint64_t* outFenceValue) {
  std::lock_guard<std::recursive_mutex> uploadLock(uploadMutex_);
  if (dst == nullptr || srcData == nullptr || bytes == 0) {
    return Status::Error("invalid upload arguments");
  }
  Status ready = EnsureUploadCommandObjects();
  if (!ready.ok) {
    return ready;
  }

  UploadContext* context = nullptr;
  Status acquired = AcquireUploadContext(bytes, context);
  if (!acquired.ok) {
    return acquired;
  }
  if (context == nullptr || context->allocator == nullptr || context->commandList == nullptr || context->mapped == nullptr ||
      context->uploadBuffer == nullptr) {
    return Status::Error("failed acquiring upload context");
  }

  HRESULT hr = context->allocator->Reset();
  if (FAILED(hr)) {
    return Status::Error("failed resetting upload allocator");
  }
  hr = context->commandList->Reset(context->allocator.Get(), nullptr);
  if (FAILED(hr)) {
    return Status::Error("failed resetting upload command list");
  }

  std::memcpy(context->mapped, srcData, bytes);
  D3D12_RESOURCE_BARRIER barrier{};
  barrier.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
  barrier.Transition.pResource = dst;
  barrier.Transition.StateBefore = D3D12_RESOURCE_STATE_COMMON;
  barrier.Transition.StateAfter = D3D12_RESOURCE_STATE_COPY_DEST;
  barrier.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
  context->commandList->ResourceBarrier(1, &barrier);
  context->commandList->CopyBufferRegion(dst, dstOffsetBytes, context->uploadBuffer.Get(), 0, bytes);
  barrier.Transition.StateBefore = D3D12_RESOURCE_STATE_COPY_DEST;
  barrier.Transition.StateAfter = D3D12_RESOURCE_STATE_COMMON;
  context->commandList->ResourceBarrier(1, &barrier);

  hr = context->commandList->Close();
  if (FAILED(hr)) {
    return Status::Error("failed closing upload command list");
  }

  if (uploadFenceValue_ == std::numeric_limits<uint64_t>::max()) {
    return Status::Error("upload fence value overflow");
  }
  const uint64_t nextFenceValue = uploadFenceValue_ + 1;
  ID3D12CommandList* lists[] = {context->commandList.Get()};
  context->submittedDestination = dst;
  copyQueue_->ExecuteCommandLists(1, lists);
  if (FAILED(copyQueue_->Signal(uploadFence_.Get(), nextFenceValue))) {
    deviceLost_ = true;
    uploadQueueFailed_ = true;
    context->fenceValue = std::numeric_limits<uint64_t>::max();
    return Status::Error("failed signaling upload fence");
  }
  uploadFenceValue_ = nextFenceValue;
  context->fenceValue = nextFenceValue;
  if (outFenceValue != nullptr) {
    *outFenceValue = context->fenceValue;
  }
  CollectRetiredResources(CurrentCompletedDirectFenceValue());
  return Status::Ok();
}

Status GaussianRasterPipeline::CreateUploadContext(UploadContext& out) {
  out = {};
  HRESULT hr = device_->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_COPY, IID_PPV_ARGS(out.allocator.GetAddressOf()));
  if (FAILED(hr)) {
    return Status::Error("failed creating upload allocator");
  }

  hr = device_->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_COPY, out.allocator.Get(), nullptr,
                                  IID_PPV_ARGS(out.commandList.GetAddressOf()));
  if (FAILED(hr)) {
    return Status::Error("failed creating upload command list");
  }

  hr = out.commandList->Close();
  if (FAILED(hr)) {
    return Status::Error("failed closing upload command list");
  }
  return Status::Ok();
}

Status GaussianRasterPipeline::AcquireUploadContext(size_t requiredBytes, UploadContext*& out) {
  out = nullptr;
  const uint64_t completed = uploadFence_ != nullptr ? uploadFence_->GetCompletedValue() : 0;
  for (UploadContext& context : uploadContexts_) {
    if (context.fenceValue > completed) {
      continue;
    }
    context.submittedDestination.Reset();
    Status resized = EnsureUploadBuffer(requiredBytes, context.uploadBuffer, context.capacityBytes, context.mapped);
    if (!resized.ok) {
      return resized;
    }
    out = &context;
    return Status::Ok();
  }

  try {
    uploadContexts_.push_back({});
  } catch (const std::bad_alloc&) {
    return Status::Error("upload context allocation failed");
  } catch (const std::length_error&) {
    return Status::Error("upload context allocation failed");
  }
  UploadContext& context = uploadContexts_.back();
  Status created = CreateUploadContext(context);
  if (!created.ok) {
    uploadContexts_.pop_back();
    return created;
  }
  Status resized = EnsureUploadBuffer(requiredBytes, context.uploadBuffer, context.capacityBytes, context.mapped);
  if (!resized.ok) {
    if (context.uploadBuffer != nullptr && context.mapped != nullptr) {
      context.uploadBuffer->Unmap(0, nullptr);
    }
    uploadContexts_.pop_back();
    return resized;
  }
  out = &context;
  return Status::Ok();
}

Status GaussianRasterPipeline::EnsureUploadBuffer(size_t requiredBytes,
                                                  ComPtr<ID3D12Resource>& resource,
                                                  size_t& capacityBytes,
                                                  void*& mappedPtr) {
  if (requiredBytes == 0) {
    return Status::Ok();
  }
  if (resource != nullptr && mappedPtr != nullptr && requiredBytes <= capacityBytes) {
    return Status::Ok();
  }

  if (resource != nullptr && mappedPtr != nullptr) {
    resource->Unmap(0, nullptr);
  }
  resource.Reset();
  mappedPtr = nullptr;

  if (requiredBytes > std::numeric_limits<size_t>::max() - 255ull) {
    return Status::Error("upload buffer is too large");
  }
  const size_t rounded = std::max<size_t>((requiredBytes + 255ull) & ~255ull, 1024u);

  D3D12_HEAP_PROPERTIES heap{};
  heap.Type = D3D12_HEAP_TYPE_UPLOAD;
  heap.CreationNodeMask = 1;
  heap.VisibleNodeMask = 1;

  D3D12_RESOURCE_DESC desc{};
  desc.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
  desc.Width = rounded;
  desc.Height = 1;
  desc.DepthOrArraySize = 1;
  desc.MipLevels = 1;
  desc.SampleDesc.Count = 1;
  desc.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;

  const HRESULT hr = device_->CreateCommittedResource(&heap, D3D12_HEAP_FLAG_NONE, &desc,
                                                      D3D12_RESOURCE_STATE_GENERIC_READ, nullptr,
                                                      IID_PPV_ARGS(resource.GetAddressOf()));
  if (FAILED(hr)) {
    return Status::Error("failed creating upload buffer");
  }
  if (FAILED(resource->Map(0, nullptr, &mappedPtr))) {
    return Status::Error("failed mapping upload buffer");
  }
  capacityBytes = rounded;
  return Status::Ok();
}

uint32_t GaussianRasterPipeline::NextPowerOfTwo(uint32_t v) const {
  if (v <= 1u) {
    return 1u;
  }
  --v;
  v |= v >> 1;
  v |= v >> 2;
  v |= v >> 4;
  v |= v >> 8;
  v |= v >> 16;
  return v + 1u;
}

void GaussianRasterPipeline::ReleaseChunkRuntime(UploadedChunkRuntime& runtime) {
  runtime.packedGaussians.clear();
  runtime.gaussianOffset = 0;
  runtime.gaussianCapacity = 0;
  runtime.gaussianCount = 0;
  runtime.atlasUploadPending = false;
  runtime.chunkId = 0;
}

void GaussianRasterPipeline::ReleaseUploadContextDeviceLost(UploadContext& context) {
  if (context.uploadBuffer != nullptr && context.mapped != nullptr) {
    context.uploadBuffer->Unmap(0, nullptr);
  }
  context.mapped = nullptr;
  context.capacityBytes = 0;
  context.submittedDestination.Reset();
  context.uploadBuffer.Reset();
  context.commandList.Reset();
  context.allocator.Reset();
  context.fenceValue = 0;
}

void GaussianRasterPipeline::ReleaseRenderScratchResourcesDeviceLost(RenderScratch& scratch) {
  if (scratch.prepConstantsUpload != nullptr && scratch.prepConstantsMapped != nullptr) {
    scratch.prepConstantsUpload->Unmap(0, nullptr);
  }
  if (scratch.rasterConstantsUpload != nullptr && scratch.rasterConstantsMapped != nullptr) {
    scratch.rasterConstantsUpload->Unmap(0, nullptr);
  }
  scratch = RenderScratch{};
}

void GaussianRasterPipeline::ReleaseSceneRuntimeDeviceLost(UploadedSceneRuntime& runtime) {
  if (runtime.batchedChunkParamsUpload != nullptr && runtime.batchedChunkParamsMapped != nullptr) {
    runtime.batchedChunkParamsUpload->Unmap(0, nullptr);
  }
  runtime.batchedChunkParamsMapped = nullptr;
  runtime.batchedChunkParamsCapacityBytes = 0;
  runtime.batchedChunkParamsUpload.Reset();
  runtime.sceneAtlasBuffer.Reset();
  runtime.sceneIndexToChunkBuffer.Reset();
  runtime.sceneAtlasCapacity = 0;
  runtime.sceneAtlasTail = 0;
  runtime.sceneGaussianStride = 0;
  runtime.rgbaOffset = 24;
  runtime.shOffset = 32;
  runtime.idOffset = 124;
  runtime.sceneIndexToChunkCapacity = 0;
  runtime.batchedChunkCount = 0;
  runtime.maxPrepareGroups = 1;
  runtime.pendingUploadFenceValue = 0;
  runtime.directQueueUploadWaitValue = 0;
  runtime.sceneIndexToChunkUploadPending = true;
  runtime.atlasFreeRanges.clear();
  for (UploadedChunkRuntime& chunk : runtime.chunks) {
    ReleaseChunkRuntime(chunk);
  }
  for (const std::shared_ptr<RenderScratch>& scratch : runtime.availableScratch) {
    if (scratch != nullptr) {
      ReleaseRenderScratchResourcesDeviceLost(*scratch);
    }
  }
  RenderScratch* scratchNode = runtime.inFlightScratchHead;
  while (scratchNode != nullptr) {
    RenderScratch* next = scratchNode->inFlightNext;
    std::shared_ptr<RenderScratch> keepAlive = scratchNode->inFlightSelf;
    if (keepAlive != nullptr) {
      ReleaseRenderScratchResourcesDeviceLost(*keepAlive);
    } else {
      ReleaseRenderScratchResourcesDeviceLost(*scratchNode);
    }
    scratchNode = next;
  }
  for (const std::shared_ptr<RenderScratch>& scratch : runtime.retainedScratch) {
    if (scratch != nullptr) {
      ReleaseRenderScratchResourcesDeviceLost(*scratch);
    }
  }
  std::shared_ptr<RenderScratch> publishedScratch = runtime.publishedScratch.lock();
  if (publishedScratch != nullptr) {
    ReleaseRenderScratchResourcesDeviceLost(*publishedScratch);
  }
  runtime.availableScratch.clear();
  runtime.inFlightScratchHead = nullptr;
  runtime.retainedScratch.clear();
  runtime.publishedScratch.reset();
  runtime.chunks.clear();
}

Status GaussianRasterPipeline::ReleaseRenderScratchResources(RenderScratch& scratch) {
  Status s = ReserveRetiredResourceSlots(kRenderScratchRetiredResourceSlots);
  if (!s.ok) return s;
  s = RetireResource(scratch.timestampReadbackBuffer);
  if (!s.ok) return s;
  scratch.timestampQueryHeap.Reset();
  s = RetireResource(scratch.sortKeysBuffer);
  if (!s.ok) return s;
  s = RetireResource(scratch.sortKeysTempBuffer);
  if (!s.ok) return s;
  s = RetireResource(scratch.sortValuesBuffer);
  if (!s.ok) return s;
  s = RetireResource(scratch.sortValuesTempBuffer);
  if (!s.ok) return s;
  s = RetireResource(scratch.visibleCounterBuffer);
  if (!s.ok) return s;
  s = RetireResource(scratch.sortMetaBuffer);
  if (!s.ok) return s;
  s = RetireResource(scratch.oneSweepPassHistogramBuffer);
  if (!s.ok) return s;
  s = RetireResource(scratch.oneSweepGlobalHistogramBuffer);
  if (!s.ok) return s;
  s = RetireResource(scratch.oneSweepIndexBuffer);
  if (!s.ok) return s;
  s = RetireResource(scratch.oneSweepDispatchArgsBuffer);
  if (!s.ok) return s;
  s = RetireResource(scratch.sortMetaReadback);
  if (!s.ok) return s;
  s = RetireResource(scratch.projectionActiveThreadsReadback);
  if (!s.ok) return s;
  s = RetireResource(scratch.drawArgsBuffer);
  if (!s.ok) return s;
  s = ReleaseUploadResource(scratch.prepConstantsUpload, scratch.prepConstantsMapped, scratch.prepConstantsCapacityBytes);
  if (!s.ok) return s;
  s = ReleaseUploadResource(scratch.rasterConstantsUpload, scratch.rasterConstantsMapped, scratch.rasterConstantsCapacityBytes);
  if (!s.ok) return s;

  scratch.sortKeysState = D3D12_RESOURCE_STATE_COMMON;
  scratch.sortKeysTempState = D3D12_RESOURCE_STATE_COMMON;
  scratch.sortValuesState = D3D12_RESOURCE_STATE_COMMON;
  scratch.sortValuesTempState = D3D12_RESOURCE_STATE_COMMON;
  scratch.visibleCounterState = D3D12_RESOURCE_STATE_COMMON;
  scratch.sortMetaState = D3D12_RESOURCE_STATE_COMMON;
  scratch.oneSweepPassHistogramState = D3D12_RESOURCE_STATE_COMMON;
  scratch.oneSweepGlobalHistogramState = D3D12_RESOURCE_STATE_COMMON;
  scratch.oneSweepIndexState = D3D12_RESOURCE_STATE_COMMON;
  scratch.oneSweepDispatchArgsState = D3D12_RESOURCE_STATE_COMMON;
  scratch.drawArgsState = D3D12_RESOURCE_STATE_COMMON;
  scratch.lastPairCount = 0;
  scratch.lastVisibleCount = 0;
  scratch.lastVisibleBlocks = 0;
  scratch.lastSortPassCount = 0;
  scratch.lastSplatAlphaBins = {};
  scratch.lastProjectionActiveThreadBins = {};
  scratch.sortStatsFrame = 0;
  scratch.sortMetaCopyFrame = 0;
  scratch.sortMetaCopyFence.Reset();
  scratch.sortMetaCopyFenceValue = 0;
  scratch.sortMetaCopyPending = false;
  scratch.gpuTimingFrame = 0;
  scratch.timestampCopyFrame = 0;
  scratch.timestampCopyFence.Reset();
  scratch.timestampCopyFenceValue = 0;
  scratch.timestampCopyPending = false;
  scratch.lastSortedInPrimary = true;
  scratch.drawCapacity = 1;
  scratch.sortPairCapacity = 1;
  scratch.oneSweepPartitionCount = 1;
  scratch.lastGpuPrepareMs = 0.0f;
  scratch.lastGpuSortMs = 0.0f;
  scratch.lastGpuRasterMs = 0.0f;
  scratch.lastGpuDepthMs = 0.0f;
  scratch.lastGpuMs = 0.0f;
  scratch.inFlightSelf.reset();
  scratch.inFlightFence.Reset();
  scratch.inFlightFenceValue = 0;
  scratch.inFlightNext = nullptr;
  return Status::Ok();
}

Status GaussianRasterPipeline::ReleaseSceneRuntime(UploadedSceneRuntime& runtime) {
  size_t scratchCount = runtime.availableScratch.size();
  if (scratchCount > std::numeric_limits<size_t>::max() - runtime.retainedScratch.size()) {
    return Status::Error("resource retirement allocation failed");
  }
  scratchCount += runtime.retainedScratch.size();
  RenderScratch* scratchNode = runtime.inFlightScratchHead;
  while (scratchNode != nullptr) {
    if (scratchCount == std::numeric_limits<size_t>::max()) {
      return Status::Error("resource retirement allocation failed");
    }
    scratchCount++;
    scratchNode = scratchNode->inFlightNext;
  }
  if (!runtime.publishedScratch.expired()) {
    if (scratchCount == std::numeric_limits<size_t>::max()) {
      return Status::Error("resource retirement allocation failed");
    }
    scratchCount++;
  }
  size_t resourceSlots = 0;
  if (runtime.batchedChunkParamsUpload != nullptr) {
    resourceSlots++;
  }
  if (runtime.sceneAtlasBuffer != nullptr) {
    resourceSlots++;
  }
  if (runtime.sceneIndexToChunkBuffer != nullptr) {
    resourceSlots++;
  }
  if (scratchCount > (std::numeric_limits<size_t>::max() - resourceSlots) / kRenderScratchRetiredResourceSlots) {
    return Status::Error("resource retirement allocation failed");
  }
  Status s = ReserveRetiredResourceSlots(resourceSlots + scratchCount * kRenderScratchRetiredResourceSlots);
  if (!s.ok) return s;
  s = ReleaseUploadResource(runtime.batchedChunkParamsUpload, runtime.batchedChunkParamsMapped, runtime.batchedChunkParamsCapacityBytes);
  if (!s.ok) return s;
  s = RetireResource(runtime.sceneAtlasBuffer);
  if (!s.ok) return s;
  s = RetireResource(runtime.sceneIndexToChunkBuffer);
  if (!s.ok) return s;
  runtime.sceneAtlasCapacity = 0;
  runtime.sceneAtlasTail = 0;
  runtime.sceneGaussianStride = 0;
  runtime.rgbaOffset = 24;
  runtime.shOffset = 32;
  runtime.idOffset = 124;
  runtime.sceneIndexToChunkCapacity = 0;
  runtime.batchedChunkCount = 0;
  runtime.maxPrepareGroups = 1;
  runtime.pendingUploadFenceValue = 0;
  runtime.directQueueUploadWaitValue = 0;
  runtime.sceneIndexToChunkUploadPending = true;
  runtime.atlasFreeRanges.clear();
  for (UploadedChunkRuntime& chunk : runtime.chunks) {
    ReleaseChunkRuntime(chunk);
  }
  auto releaseScratch = [&](const std::shared_ptr<RenderScratch>& scratch) -> Status {
    std::shared_ptr<RenderScratch> keepAlive = scratch;
    if (keepAlive == nullptr) {
      return Status::Ok();
    }
    return ReleaseRenderScratchResources(*keepAlive);
  };
  for (const std::shared_ptr<RenderScratch>& scratch : runtime.availableScratch) {
    s = releaseScratch(scratch);
    if (!s.ok) return s;
  }
  scratchNode = runtime.inFlightScratchHead;
  while (scratchNode != nullptr) {
    RenderScratch* next = scratchNode->inFlightNext;
    std::shared_ptr<RenderScratch> keepAlive = scratchNode->inFlightSelf;
    s = releaseScratch(keepAlive);
    if (!s.ok) return s;
    scratchNode->inFlightFence.Reset();
    scratchNode->inFlightFenceValue = 0;
    scratchNode->inFlightNext = nullptr;
    scratchNode->inFlightSelf.reset();
    scratchNode = next;
  }
  for (const std::shared_ptr<RenderScratch>& scratch : runtime.retainedScratch) {
    s = releaseScratch(scratch);
    if (!s.ok) return s;
  }
  s = releaseScratch(runtime.publishedScratch.lock());
  if (!s.ok) return s;
  runtime.availableScratch.clear();
  runtime.inFlightScratchHead = nullptr;
  runtime.retainedScratch.clear();
  runtime.publishedScratch.reset();
  runtime.chunks.clear();
  return Status::Ok();
}

Status GaussianRasterPipeline::BuildChunkRuntime(uint64_t chunkId,
                                                       const GaussianSet& set,
                                                       VramFormatSettings format,
                                                       uint32_t strideBytes,
                                                       uint32_t rgbaOffset,
                                                       uint32_t shOffset,
                                                       uint32_t idOffset,
                                                       UploadedChunkRuntime& out) {
  if (chunkId == 0) {
    return Status::Error("invalid chunk id");
  }
  out = {};
  out.chunkId = chunkId;
  out.packedStrideBytes = strideBytes;
  format = SanitizeVramFormatSettings(format);
  if (strideBytes == 0) {
    return Status::Error("invalid scene Gaussian stride");
  }
  if (set.gaussians.size() > std::numeric_limits<uint32_t>::max()) {
    return Status::Error("gaussian set is too large");
  }
  if (set.gaussians.size() > std::numeric_limits<size_t>::max() / strideBytes) {
    return Status::Error("gaussian set is too large");
  }

  const Aabb bounds = set.bounds.valid ? set.bounds : BoundsFromGaussians(set.gaussians);
  const Vec3 decodeMin = bounds.valid ? bounds.min : Vec3{};
  const Vec3 decodeExtent = bounds.valid
                                ? Max(bounds.max - bounds.min, Vec3{1e-6f, 1e-6f, 1e-6f})
                                : Vec3{1.0f, 1.0f, 1.0f};
  out.params.scalingModifier = set.scalingModifier;
  out.params.visible = set.visible ? 1u : 0u;
  out.params.decodeMin[0] = decodeMin.x;
  out.params.decodeMin[1] = decodeMin.y;
  out.params.decodeMin[2] = decodeMin.z;
  out.params.decodeExtent[0] = decodeExtent.x;
  out.params.decodeExtent[1] = decodeExtent.y;
  out.params.decodeExtent[2] = decodeExtent.z;

  std::vector<uint8_t> gaussians;
  try {
    gaussians.reserve(static_cast<size_t>(set.gaussians.size()) * strideBytes);
  } catch (const std::bad_alloc&) {
    return Status::Error("gaussian set allocation failed");
  } catch (const std::length_error&) {
    return Status::Error("gaussian set is too large");
  }
  const uint32_t fallbackInstanceId = static_cast<uint32_t>((chunkId & 0xFFFFFFFFu) == 0 ? 1u : (chunkId & 0xFFFFFFFFu));
  const uint32_t rgbaElementBytes = AttributeBytes(format.rgbaFormat);
  const uint32_t shElementBytes = AttributeBytes(format.shFormat);

  for (const Gaussian& g : set.gaussians) {
    if (!IsFinite(g.position) || !IsFinite(g.scale) || !IsFinite(g.rotation)) {
      continue;
    }

    const size_t base = gaussians.size();
    try {
      gaussians.resize(base + strideBytes, 0u);
    } catch (const std::bad_alloc&) {
      return Status::Error("gaussian set allocation failed");
    } catch (const std::length_error&) {
      return Status::Error("gaussian set is too large");
    }

    const Vec3 position01{
        decodeExtent.x > 1e-8f ? (g.position.x - decodeMin.x) / decodeExtent.x : 0.0f,
        decodeExtent.y > 1e-8f ? (g.position.y - decodeMin.y) / decodeExtent.y : 0.0f,
        decodeExtent.z > 1e-8f ? (g.position.z - decodeMin.z) / decodeExtent.z : 0.0f,
    };
    const float sx = DecodeScaleValue(g.scale.x);
    const float sy = DecodeScaleValue(g.scale.y);
    const float sz = DecodeScaleValue(g.scale.z);
    const float filterScale = std::max(std::min({sx, sy, sz}) * kPackedMipFilterFraction, 1e-5f);
    const Quat q = Normalize(g.rotation);

    StoreU32(gaussians, base + 0u, static_cast<uint32_t>(PackUnorm16(position01.x)) |
                                      (static_cast<uint32_t>(PackUnorm16(position01.y)) << 16u));
    StoreU32(gaussians, base + 4u, static_cast<uint32_t>(PackUnorm16(position01.z)));
    StoreU32(gaussians, base + 8u, PackHalf2(sx, sy));
    StoreU32(gaussians, base + 12u, PackHalf2(sz, filterScale));
    StoreU32(gaussians, base + 16u, PackSnorm16Pair(q.x, q.y));
    StoreU32(gaussians, base + 20u, PackSnorm16Pair(q.z, q.w));

    const float opacity = 1.0f / (1.0f + std::exp(-std::clamp(g.opacity, -20.0f, 20.0f)));

    StoreAttributeFloat(gaussians, base + rgbaOffset + 0u * rgbaElementBytes, format.rgbaFormat, g.sh[0], true);
    StoreAttributeFloat(gaussians, base + rgbaOffset + 1u * rgbaElementBytes, format.rgbaFormat, g.sh[16], true);
    StoreAttributeFloat(gaussians, base + rgbaOffset + 2u * rgbaElementBytes, format.rgbaFormat, g.sh[32], true);
    StoreAttributeFloat(gaussians, base + rgbaOffset + 3u * rgbaElementBytes, format.rgbaFormat, opacity, false);

    uint32_t dstCoeff = 0;
    for (uint32_t channel = 0; channel < 3u; ++channel) {
      const uint32_t srcBase = channel * kShOrder3CoeffCountPerChannel;
      for (uint32_t coeff = 1u; coeff < kShOrder3CoeffCountPerChannel; ++coeff) {
        StoreAttributeFloat(gaussians,
                            base + shOffset + static_cast<size_t>(dstCoeff) * shElementBytes,
                            format.shFormat,
                            g.sh[srcBase + coeff],
                            true);
        ++dstCoeff;
      }
    }

    StoreU32(gaussians, base + idOffset + 0u, g.splatId);
    StoreU32(gaussians, base + idOffset + 4u, g.instanceId == 0 ? fallbackInstanceId : g.instanceId);
  }

  out.gaussianCount = static_cast<uint32_t>(gaussians.size() / strideBytes);
  out.packedGaussians = std::move(gaussians);
  out.gaussianCapacity = out.gaussianCount;
  out.atlasUploadPending = out.gaussianCount > 0;
  return Status::Ok();
}

Status GaussianRasterPipeline::AllocateAtlasRange(UploadedSceneRuntime& runtime, uint32_t count, uint32_t& outOffset) {
  outOffset = 0;
  if (count == 0) {
    return Status::Ok();
  }
  for (size_t i = 0; i < runtime.atlasFreeRanges.size(); ++i) {
    AtlasFreeRange& range = runtime.atlasFreeRanges[i];
    if (range.count < count) {
      continue;
    }
    if (range.offset > std::numeric_limits<uint32_t>::max() - count) {
      return Status::Error("scene atlas range overflow");
    }
    outOffset = range.offset;
    range.offset += count;
    range.count -= count;
    if (range.count == 0) {
      runtime.atlasFreeRanges.erase(runtime.atlasFreeRanges.begin() + static_cast<std::ptrdiff_t>(i));
    }
    return Status::Ok();
  }
  if (runtime.sceneAtlasTail > std::numeric_limits<uint32_t>::max() - count) {
    return Status::Error("scene atlas capacity overflow");
  }
  outOffset = runtime.sceneAtlasTail;
  runtime.sceneAtlasTail += count;
  return Status::Ok();
}

Status GaussianRasterPipeline::FreeAtlasRange(UploadedSceneRuntime& runtime, uint32_t offset, uint32_t count) try {
  if (count == 0) {
    return Status::Ok();
  }
  if (offset > std::numeric_limits<uint32_t>::max() - count) {
    return Status::Error("scene atlas range overflow");
  }
  AtlasFreeRange freeRange{};
  freeRange.offset = offset;
  freeRange.count = count;
  std::vector<AtlasFreeRange> ranges = runtime.atlasFreeRanges;
  auto insertIt = std::lower_bound(
      ranges.begin(), ranges.end(), freeRange,
      [](const AtlasFreeRange& a, const AtlasFreeRange& b) { return a.offset < b.offset; });
  ranges.insert(insertIt, freeRange);
  std::vector<AtlasFreeRange> merged;
  merged.reserve(ranges.size());
  for (const AtlasFreeRange& range : ranges) {
    if (merged.empty()) {
      merged.push_back(range);
      continue;
    }
    AtlasFreeRange& back = merged.back();
    const uint64_t backEnd = static_cast<uint64_t>(back.offset) + back.count;
    const uint64_t rangeEnd = static_cast<uint64_t>(range.offset) + range.count;
    if (backEnd > std::numeric_limits<uint32_t>::max() || rangeEnd > std::numeric_limits<uint32_t>::max()) {
      return Status::Error("scene atlas range overflow");
    }
    if (range.offset <= backEnd) {
      back.count = static_cast<uint32_t>(std::max(backEnd, rangeEnd) - back.offset);
    } else {
      merged.push_back(range);
    }
  }
  uint32_t sceneAtlasTail = runtime.sceneAtlasTail;
  while (!merged.empty()) {
    AtlasFreeRange& back = merged.back();
    const uint64_t backEnd = static_cast<uint64_t>(back.offset) + back.count;
    if (backEnd != sceneAtlasTail) {
      break;
    }
    sceneAtlasTail = back.offset;
    merged.pop_back();
  }
  runtime.sceneAtlasTail = sceneAtlasTail;
  runtime.atlasFreeRanges = std::move(merged);
  return Status::Ok();
} catch (const std::bad_alloc&) {
  return Status::Error("scene atlas allocation failed");
} catch (const std::length_error&) {
  return Status::Error("scene atlas allocation failed");
}

Status GaussianRasterPipeline::EnsureSceneAtlasCapacity(UploadedSceneRuntime& runtime, uint32_t requiredCapacity) {
  if (requiredCapacity == 0) {
    Status retired = RetireResource(runtime.sceneAtlasBuffer);
    if (!retired.ok) {
      return retired;
    }
    runtime.sceneAtlasCapacity = 0;
    runtime.sceneAtlasTail = 0;
    runtime.pendingUploadFenceValue = 0;
    runtime.directQueueUploadWaitValue = 0;
    runtime.atlasFreeRanges.clear();
    for (UploadedChunkRuntime& chunk : runtime.chunks) {
      chunk.gaussianOffset = 0;
      chunk.gaussianCapacity = chunk.gaussianCount;
      chunk.atlasUploadPending = false;
    }
    return Status::Ok();
  }
  if (runtime.sceneAtlasBuffer != nullptr && runtime.sceneAtlasCapacity >= requiredCapacity) {
    return Status::Ok();
  }

  const uint32_t maxAddressableCapacity = MaxShaderAddressableGaussians(runtime.sceneGaussianStride);
  if (requiredCapacity > maxAddressableCapacity) {
    return Status::Error("scene atlas is too large");
  }

  uint32_t newCapacity = std::max<uint32_t>(requiredCapacity, 1024u);
  if (runtime.sceneAtlasCapacity > 0) {
    const uint32_t doubled = runtime.sceneAtlasCapacity > std::numeric_limits<uint32_t>::max() / 2u
                                 ? std::numeric_limits<uint32_t>::max()
                                 : runtime.sceneAtlasCapacity * 2u;
    newCapacity = std::max<uint32_t>(newCapacity, doubled);
  }
  newCapacity = std::min<uint32_t>(newCapacity, maxAddressableCapacity);
  if (runtime.sceneGaussianStride == 0 ||
      newCapacity > std::numeric_limits<size_t>::max() / runtime.sceneGaussianStride) {
    return Status::Error("scene atlas is too large");
  }

  Microsoft::WRL::ComPtr<ID3D12Resource> replacement;
  Status s = CreateDefaultBuffer(static_cast<size_t>(newCapacity) * runtime.sceneGaussianStride,
                                 D3D12_RESOURCE_FLAG_NONE,
                                 D3D12_RESOURCE_STATE_COMMON,
                                 replacement);
  if (!s.ok) {
    return s;
  }

  uint64_t maxFenceValue = 0;
  {
    std::lock_guard<std::recursive_mutex> uploadLock(uploadMutex_);
    s = ReserveRetiredResourceSlots(1);
    if (!s.ok) {
      return s;
    }
    for (UploadedChunkRuntime& chunk : runtime.chunks) {
      if (chunk.gaussianCount == 0 || chunk.packedGaussians.empty()) {
        chunk.atlasUploadPending = false;
        continue;
      }
      uint64_t uploadFenceValue = 0;
      s = UploadBufferQueued(replacement.Get(),
                             static_cast<uint64_t>(chunk.gaussianOffset) * runtime.sceneGaussianStride,
                             chunk.packedGaussians.data(),
                             static_cast<size_t>(chunk.gaussianCount) * runtime.sceneGaussianStride,
                             &uploadFenceValue);
      if (!s.ok) {
        return s;
      }
      maxFenceValue = std::max(maxFenceValue, uploadFenceValue);
      chunk.atlasUploadPending = false;
    }

    s = RetireResource(runtime.sceneAtlasBuffer);
    if (!s.ok) {
      return s;
    }
  }
  runtime.sceneAtlasBuffer = std::move(replacement);
  runtime.sceneAtlasCapacity = newCapacity;
  runtime.pendingUploadFenceValue = std::max(runtime.pendingUploadFenceValue, maxFenceValue);
  return Status::Ok();
}

Status GaussianRasterPipeline::UploadChunkAtlasRange(UploadedSceneRuntime& sceneRuntime,
                                                     const UploadedChunkRuntime& chunkRuntime,
                                                     uint64_t* outFenceValue) {
  if (outFenceValue != nullptr) {
    *outFenceValue = 0;
  }
  if (chunkRuntime.gaussianCount == 0 || chunkRuntime.packedGaussians.empty()) {
    return Status::Ok();
  }
  if (sceneRuntime.sceneAtlasBuffer == nullptr) {
    return Status::Error("scene atlas buffer is missing");
  }
  return UploadBufferQueued(sceneRuntime.sceneAtlasBuffer.Get(),
                            static_cast<uint64_t>(chunkRuntime.gaussianOffset) * sceneRuntime.sceneGaussianStride,
                            chunkRuntime.packedGaussians.data(),
                            static_cast<size_t>(chunkRuntime.gaussianCount) * sceneRuntime.sceneGaussianStride,
                            outFenceValue);
}

Status GaussianRasterPipeline::UpdateSceneCapacity(UploadedSceneRuntime& runtime) {
  uint64_t total = 0;
  uint32_t maxPrepareGroups = 1;
  for (const UploadedChunkRuntime& chunk : runtime.chunks) {
    total += chunk.gaussianCount;
    maxPrepareGroups = std::max<uint32_t>(maxPrepareGroups, (chunk.gaussianCount + 255u) / 256u);
  }
  runtime.sceneGaussianCount = static_cast<uint32_t>(std::min<uint64_t>(total, UINT32_MAX));
  runtime.batchedChunkCount = static_cast<uint32_t>(runtime.chunks.size());
  runtime.maxPrepareGroups = maxPrepareGroups;
  runtime.drawCapacity = std::max<uint32_t>(runtime.sceneGaussianCount, 1u);
  const uint64_t pairMultiplier = kSortPairMultiplier;
  const uint64_t pairTarget = std::max<uint64_t>(
      std::max<uint64_t>(runtime.sceneGaussianCount, 1u) * pairMultiplier,
      static_cast<uint64_t>(kMinSortPairCapacity));
  runtime.sortPairCapacity =
      static_cast<uint32_t>(std::min<uint64_t>(std::max<uint64_t>(pairTarget, runtime.drawCapacity), kMaxSortPairCapacity));
  if (runtime.sceneGaussianCount == 0) {
    return Status::Ok();
  }
  return Status::Ok();
}

Status GaussianRasterPipeline::RefreshScenePrepResources(UploadedSceneRuntime& runtime, bool rebuildSceneData) try {
  const uint32_t chunkCount = static_cast<uint32_t>(runtime.chunks.size());
  const size_t chunkParamsBytes = static_cast<size_t>(std::max<uint32_t>(chunkCount, 1u)) * sizeof(ChunkPrepGpu);
  Status s = EnsureUploadBuffer(chunkParamsBytes, runtime.batchedChunkParamsUpload, runtime.batchedChunkParamsCapacityBytes,
                                runtime.batchedChunkParamsMapped);
  if (!s.ok) {
    return s;
  }

  std::vector<ChunkPrepGpu> chunkPrep;
  chunkPrep.reserve(runtime.chunks.size());
  for (const UploadedChunkRuntime& chunk : runtime.chunks) {
    ChunkPrepGpu prep{};
    prep.params = chunk.params;
    prep.gaussianOffset = chunk.gaussianOffset;
    prep.gaussianCount = chunk.gaussianCount;
    chunkPrep.push_back(prep);
  }
  if (runtime.batchedChunkParamsMapped != nullptr && !chunkPrep.empty()) {
    std::memcpy(runtime.batchedChunkParamsMapped, chunkPrep.data(), chunkPrep.size() * sizeof(ChunkPrepGpu));
  }

  if (runtime.sceneIndexToChunkUploadPending) {
    const uint32_t indexCount = std::max<uint32_t>(runtime.sceneAtlasTail, 1u);
    if (indexCount > kMaxSceneIndexToChunkEntries) {
      return Status::Error("scene index buffer is too large");
    }
    if (runtime.sceneIndexToChunkBuffer == nullptr || runtime.sceneIndexToChunkCapacity < indexCount) {
      uint32_t newCapacity = std::max<uint32_t>(indexCount, 1024u);
      if (runtime.sceneIndexToChunkCapacity > 0) {
        const uint32_t doubled = runtime.sceneIndexToChunkCapacity > std::numeric_limits<uint32_t>::max() / 2u
                                     ? std::numeric_limits<uint32_t>::max()
                                     : runtime.sceneIndexToChunkCapacity * 2u;
        newCapacity = std::max<uint32_t>(newCapacity, doubled);
      }
      newCapacity = std::min<uint32_t>(newCapacity, kMaxSceneIndexToChunkEntries);
      if (newCapacity < indexCount) {
        return Status::Error("scene index buffer is too large");
      }
      if (newCapacity > std::numeric_limits<size_t>::max() / sizeof(uint32_t)) {
        return Status::Error("scene index buffer is too large");
      }
      Microsoft::WRL::ComPtr<ID3D12Resource> replacement;
      s = CreateDefaultBuffer(static_cast<size_t>(newCapacity) * sizeof(uint32_t),
                              D3D12_RESOURCE_FLAG_NONE,
                              D3D12_RESOURCE_STATE_COMMON,
                              replacement);
      if (!s.ok) {
        return s;
      }
      s = RetireResource(runtime.sceneIndexToChunkBuffer);
      if (!s.ok) {
        return s;
      }
      runtime.sceneIndexToChunkBuffer = std::move(replacement);
      runtime.sceneIndexToChunkCapacity = newCapacity;
    }

    std::vector<uint32_t> sceneIndexToChunk(indexCount, 0u);
    for (uint32_t ci = 0; ci < static_cast<uint32_t>(runtime.chunks.size()); ++ci) {
      const UploadedChunkRuntime& chunk = runtime.chunks[ci];
      for (uint32_t i = 0; i < chunk.gaussianCount; ++i) {
        const uint64_t sceneIndex64 = static_cast<uint64_t>(chunk.gaussianOffset) + i;
        if (sceneIndex64 > std::numeric_limits<uint32_t>::max()) {
          return Status::Error("scene atlas range overflow");
        }
        const uint32_t sceneIndex = static_cast<uint32_t>(sceneIndex64);
        if (sceneIndex < sceneIndexToChunk.size()) {
          sceneIndexToChunk[sceneIndex] = ci;
        }
      }
    }

    uint64_t indexUploadFenceValue = 0;
    s = UploadBufferQueued(runtime.sceneIndexToChunkBuffer.Get(),
                           0,
                           sceneIndexToChunk.data(),
                           sceneIndexToChunk.size() * sizeof(uint32_t),
                           &indexUploadFenceValue);
    if (!s.ok) {
      return s;
    }
    runtime.pendingUploadFenceValue = std::max(runtime.pendingUploadFenceValue, indexUploadFenceValue);
    runtime.sceneIndexToChunkUploadPending = false;
  }

  if (!rebuildSceneData) {
    return Status::Ok();
  }

  s = EnsureSceneAtlasCapacity(runtime, runtime.sceneAtlasTail);
  if (!s.ok) {
    return s;
  }

  uint64_t maxFenceValue = 0;
  for (UploadedChunkRuntime& chunk : runtime.chunks) {
    if (!chunk.atlasUploadPending) {
      continue;
    }
    uint64_t uploadFenceValue = 0;
    s = UploadChunkAtlasRange(runtime, chunk, &uploadFenceValue);
    if (!s.ok) {
      return s;
    }
    maxFenceValue = std::max(maxFenceValue, uploadFenceValue);
    chunk.atlasUploadPending = false;
  }
  runtime.pendingUploadFenceValue = std::max(runtime.pendingUploadFenceValue, maxFenceValue);
  return Status::Ok();
} catch (const std::bad_alloc&) {
  return Status::Error("scene prep allocation failed");
} catch (const std::length_error&) {
  return Status::Error("scene prep allocation failed");
}

Status GaussianRasterPipeline::CreateOrUpdateScene(uint64_t sceneId, const Scene& scene, const std::vector<uint64_t>& chunkIds) {
  if (sceneId == 0) {
    return Status::Error("invalid scene id");
  }
  if (chunkIds.size() != scene.splatSets.size()) {
    return Status::Error("scene chunk handle count mismatch");
  }

  UploadedSceneRuntime runtime{};
  auto releaseRuntimeAfterFailure = [&](UploadedSceneRuntime& target) -> Status {
    Status released = ReleaseSceneRuntime(target);
    if (released.ok) {
      return Status::Ok();
    }
    Status idle = WaitUploadQueue();
    if (idle.ok) {
      released = ReleaseSceneRuntime(target);
      if (released.ok) {
        return Status::Ok();
      }
    }
    NotifyDeviceLost();
    (void)ReleaseSceneRuntime(target);
    return released;
  };
  runtime.vramFormat = SanitizeVramFormatSettings(scene.vramFormat);
  const PackedLayout layout = ComputePackedLayout(runtime.vramFormat);
  runtime.sceneGaussianStride = layout.stride;
  runtime.rgbaOffset = layout.rgbaOffset;
  runtime.shOffset = layout.shOffset;
  runtime.idOffset = layout.idOffset;
  runtime.chunks.reserve(scene.splatSets.size());
  for (size_t i = 0; i < scene.splatSets.size(); ++i) {
    UploadedChunkRuntime chunk{};
    Status s = BuildChunkRuntime(chunkIds[i], scene.splatSets[i], runtime.vramFormat, runtime.sceneGaussianStride,
                                 runtime.rgbaOffset, runtime.shOffset, runtime.idOffset, chunk);
    if (!s.ok) {
      Status released = releaseRuntimeAfterFailure(runtime);
      if (!released.ok) return released;
      return s;
    }
    chunk.gaussianOffset = runtime.sceneAtlasTail;
    chunk.gaussianCapacity = chunk.gaussianCount;
    if (runtime.sceneAtlasTail > std::numeric_limits<uint32_t>::max() - chunk.gaussianCapacity) {
      Status released = releaseRuntimeAfterFailure(runtime);
      if (!released.ok) return released;
      return Status::Error("scene atlas capacity overflow");
    }
    runtime.sceneAtlasTail += chunk.gaussianCapacity;
    runtime.chunks.push_back(std::move(chunk));
  }

  Status s = UpdateSceneCapacity(runtime);
  if (!s.ok) {
    Status released = releaseRuntimeAfterFailure(runtime);
    if (!released.ok) return released;
    return s;
  }
  s = RefreshScenePrepResources(runtime, true);
  if (!s.ok) {
    Status released = releaseRuntimeAfterFailure(runtime);
    if (!released.ok) return released;
    return s;
  }
  std::shared_ptr<UploadedSceneRuntime> replacement;
  try {
    replacement = std::make_shared<UploadedSceneRuntime>(std::move(runtime));
  } catch (const std::bad_alloc&) {
    Status released = releaseRuntimeAfterFailure(runtime);
    if (!released.ok) return released;
    return Status::Error("scene runtime allocation failed");
  } catch (const std::length_error&) {
    Status released = releaseRuntimeAfterFailure(runtime);
    if (!released.ok) return released;
    return Status::Error("scene runtime allocation failed");
  }
  std::unique_lock<std::shared_mutex> lock(uploadedScenesMutex_);
  const auto existing = uploadedScenes_.find(sceneId);
  if (existing != uploadedScenes_.end()) {
    if (existing->second) {
      std::lock_guard<std::mutex> runtimeLock(*existing->second->mutex);
      Status released = ReleaseSceneRuntime(*existing->second);
      if (!released.ok) {
        Status replacementReleased = releaseRuntimeAfterFailure(*replacement);
        if (!replacementReleased.ok) return replacementReleased;
        return released;
      }
    }
    existing->second = std::move(replacement);
  } else {
    try {
      uploadedScenes_.emplace(sceneId, std::move(replacement));
    } catch (const std::bad_alloc&) {
      if (replacement) {
        Status released = releaseRuntimeAfterFailure(*replacement);
        if (!released.ok) return released;
      }
      return Status::Error("scene runtime allocation failed");
    } catch (const std::length_error&) {
      if (replacement) {
        Status released = releaseRuntimeAfterFailure(*replacement);
        if (!released.ok) return released;
      }
      return Status::Error("scene runtime allocation failed");
    }
  }
  return Status::Ok();
}

Status GaussianRasterPipeline::DestroyScene(uint64_t sceneId) {
  std::unique_lock<std::shared_mutex> lock(uploadedScenesMutex_);
  auto it = uploadedScenes_.find(sceneId);
  if (it == uploadedScenes_.end()) {
    return Status::Ok();
  }
  std::shared_ptr<UploadedSceneRuntime> runtime = it->second;
  lock.unlock();
  if (runtime) {
    std::lock_guard<std::mutex> runtimeLock(*runtime->mutex);
    Status released = ReleaseSceneRuntime(*runtime);
    if (!released.ok) {
      return released;
    }
  }
  lock.lock();
  uploadedScenes_.erase(sceneId);
  lock.unlock();
  // Renderer waits for this scene's direct queue fence before destruction.
  // Collect now so an idle application does not retain completed resources.
  CollectRetiredResources(CurrentCompletedDirectFenceValue());
  return Status::Ok();
}

bool GaussianRasterPipeline::HasScene(uint64_t sceneId) const {
  if (sceneId == 0) {
    return false;
  }
  std::shared_lock<std::shared_mutex> lock(uploadedScenesMutex_);
  return uploadedScenes_.find(sceneId) != uploadedScenes_.end();
}

bool GaussianRasterPipeline::HasChunk(uint64_t sceneId, uint64_t chunkId) const {
  if (chunkId == 0) {
    return false;
  }
  std::shared_ptr<UploadedSceneRuntime> runtime;
  {
    std::shared_lock<std::shared_mutex> lock(uploadedScenesMutex_);
    const auto sceneIt = uploadedScenes_.find(sceneId);
    if (sceneIt == uploadedScenes_.end()) {
      return false;
    }
    runtime = sceneIt->second;
  }
  if (runtime == nullptr) {
    return false;
  }
  std::lock_guard<std::mutex> runtimeLock(*runtime->mutex);
  const auto& chunks = runtime->chunks;
  return std::find_if(chunks.begin(), chunks.end(), [chunkId](const UploadedChunkRuntime& chunk) {
           return chunk.chunkId == chunkId;
         }) != chunks.end();
}

Status GaussianRasterPipeline::AddChunk(uint64_t sceneId, uint64_t chunkId, const GaussianSet& chunkSet) try {
  std::shared_ptr<UploadedSceneRuntime> runtime;
  {
    std::shared_lock<std::shared_mutex> lock(uploadedScenesMutex_);
    auto it = uploadedScenes_.find(sceneId);
    if (it == uploadedScenes_.end()) {
      return Status::Error("uploaded scene handle not found");
    }
    runtime = it->second;
  }
  if (runtime == nullptr) {
    return Status::Error("uploaded scene handle not found");
  }
  std::lock_guard<std::mutex> runtimeLock(*runtime->mutex);
  if (std::find_if(runtime->chunks.begin(), runtime->chunks.end(), [chunkId](const UploadedChunkRuntime& chunk) {
        return chunk.chunkId == chunkId;
      }) != runtime->chunks.end()) {
    return Status::Error("uploaded chunk handle already exists");
  }
  runtime->chunks.reserve(runtime->chunks.size() + 1u);
  UploadedChunkRuntime chunk{};
  Status s = BuildChunkRuntime(chunkId, chunkSet, runtime->vramFormat, runtime->sceneGaussianStride,
                             runtime->rgbaOffset, runtime->shOffset, runtime->idOffset, chunk);
  if (!s.ok) {
    return s;
  }
  const uint32_t previousTail = runtime->sceneAtlasTail;
  const auto previousFreeRanges = runtime->atlasFreeRanges;
  if (chunk.gaussianCount > 0) {
    uint32_t atlasOffset = 0;
    s = AllocateAtlasRange(*runtime, chunk.gaussianCount, atlasOffset);
    if (!s.ok) {
      runtime->sceneAtlasTail = previousTail;
      runtime->atlasFreeRanges = previousFreeRanges;
      return s;
    }
    chunk.gaussianOffset = atlasOffset;
    chunk.gaussianCapacity = chunk.gaussianCount;
  }
  s = EnsureSceneAtlasCapacity(*runtime, runtime->sceneAtlasTail);
  if (!s.ok) {
    runtime->sceneAtlasTail = previousTail;
    runtime->atlasFreeRanges = previousFreeRanges;
    return s;
  }
  uint64_t uploadFenceValue = 0;
  s = UploadChunkAtlasRange(*runtime, chunk, &uploadFenceValue);
  if (!s.ok) {
    runtime->sceneAtlasTail = previousTail;
    runtime->atlasFreeRanges = previousFreeRanges;
    return s;
  }
  runtime->pendingUploadFenceValue = std::max(runtime->pendingUploadFenceValue, uploadFenceValue);
  chunk.atlasUploadPending = false;
  runtime->chunks.push_back(std::move(chunk));
  runtime->sceneIndexToChunkUploadPending = true;
  s = UpdateSceneCapacity(*runtime);
  if (!s.ok) {
    ReleaseChunkRuntime(runtime->chunks.back());
    runtime->chunks.pop_back();
    runtime->sceneAtlasTail = previousTail;
    runtime->atlasFreeRanges = previousFreeRanges;
    UpdateSceneCapacity(*runtime);
    return s;
  }
  s = RefreshScenePrepResources(*runtime, false);
  if (!s.ok) {
    ReleaseChunkRuntime(runtime->chunks.back());
    runtime->chunks.pop_back();
    runtime->sceneAtlasTail = previousTail;
    runtime->atlasFreeRanges = previousFreeRanges;
    UpdateSceneCapacity(*runtime);
    RefreshScenePrepResources(*runtime, false);
    return s;
  }
  return Status::Ok();
} catch (const std::bad_alloc&) {
  return Status::Error("scene chunk allocation failed");
} catch (const std::length_error&) {
  return Status::Error("scene chunk allocation failed");
}

Status GaussianRasterPipeline::UpdateChunk(uint64_t sceneId, uint64_t chunkId, const GaussianSet& chunkSet) try {
  std::shared_ptr<UploadedSceneRuntime> runtime;
  {
    std::shared_lock<std::shared_mutex> lock(uploadedScenesMutex_);
    auto it = uploadedScenes_.find(sceneId);
    if (it == uploadedScenes_.end()) {
      return Status::Error("uploaded scene handle not found");
    }
    runtime = it->second;
  }
  if (runtime == nullptr) {
    return Status::Error("uploaded scene handle not found");
  }
  std::lock_guard<std::mutex> runtimeLock(*runtime->mutex);
  const auto chunkIt = std::find_if(runtime->chunks.begin(), runtime->chunks.end(), [chunkId](const UploadedChunkRuntime& chunk) {
    return chunk.chunkId == chunkId;
  });
  if (chunkIt == runtime->chunks.end()) {
    return Status::Error("uploaded chunk handle not found");
  }
  UploadedChunkRuntime replacement{};
  Status s = BuildChunkRuntime(chunkId, chunkSet, runtime->vramFormat, runtime->sceneGaussianStride,
                             runtime->rgbaOffset, runtime->shOffset, runtime->idOffset, replacement);
  if (!s.ok) {
    return s;
  }
  const auto previousFreeRanges = runtime->atlasFreeRanges;
  const uint32_t previousTail = runtime->sceneAtlasTail;
  const bool previousIndexUploadPending = runtime->sceneIndexToChunkUploadPending;
  const UploadedChunkRuntime oldSnapshot = *chunkIt;

  if (replacement.gaussianCount == 0) {
    if (oldSnapshot.gaussianCapacity > 0) {
      s = FreeAtlasRange(*runtime, oldSnapshot.gaussianOffset, oldSnapshot.gaussianCapacity);
      if (!s.ok) {
        return s;
      }
    }
    replacement.gaussianOffset = 0;
    replacement.gaussianCapacity = 0;
    replacement.atlasUploadPending = false;
  } else if (replacement.gaussianCount <= oldSnapshot.gaussianCapacity && oldSnapshot.gaussianCapacity > 0) {
    replacement.gaussianOffset = oldSnapshot.gaussianOffset;
    replacement.gaussianCapacity = oldSnapshot.gaussianCapacity;
    replacement.atlasUploadPending = true;
  } else {
    if (oldSnapshot.gaussianCapacity > 0) {
      s = FreeAtlasRange(*runtime, oldSnapshot.gaussianOffset, oldSnapshot.gaussianCapacity);
      if (!s.ok) {
        return s;
      }
    }
    uint32_t atlasOffset = 0;
    s = AllocateAtlasRange(*runtime, replacement.gaussianCount, atlasOffset);
    if (!s.ok) {
      runtime->atlasFreeRanges = previousFreeRanges;
      runtime->sceneAtlasTail = previousTail;
      return s;
    }
    replacement.gaussianOffset = atlasOffset;
    replacement.gaussianCapacity = replacement.gaussianCount;
    replacement.atlasUploadPending = true;
  }

  s = EnsureSceneAtlasCapacity(*runtime, runtime->sceneAtlasTail);
  if (!s.ok) {
    runtime->atlasFreeRanges = previousFreeRanges;
    runtime->sceneAtlasTail = previousTail;
    return s;
  }
  uint64_t uploadFenceValue = 0;
  s = UploadChunkAtlasRange(*runtime, replacement, &uploadFenceValue);
  if (!s.ok) {
    runtime->atlasFreeRanges = previousFreeRanges;
    runtime->sceneAtlasTail = previousTail;
    return s;
  }
  runtime->pendingUploadFenceValue = std::max(runtime->pendingUploadFenceValue, uploadFenceValue);

  UploadedChunkRuntime old = std::move(*chunkIt);
  replacement.atlasUploadPending = false;
  *chunkIt = std::move(replacement);
  runtime->sceneIndexToChunkUploadPending = true;
  auto rollbackUploadedReplacement = [&](Status failure) {
    ReleaseChunkRuntime(*chunkIt);
    *chunkIt = std::move(old);
    runtime->atlasFreeRanges = previousFreeRanges;
    runtime->sceneAtlasTail = previousTail;
    runtime->sceneIndexToChunkUploadPending = previousIndexUploadPending;
    UpdateSceneCapacity(*runtime);
    uint64_t rollbackFenceValue = 0;
    Status rollback = UploadChunkAtlasRange(*runtime, *chunkIt, &rollbackFenceValue);
    if (!rollback.ok) {
      return rollback;
    }
    runtime->pendingUploadFenceValue = std::max(runtime->pendingUploadFenceValue, rollbackFenceValue);
    runtime->sceneIndexToChunkUploadPending = true;
    Status refresh = RefreshScenePrepResources(*runtime, false);
    if (!refresh.ok) {
      return refresh;
    }
    return failure;
  };
  s = UpdateSceneCapacity(*runtime);
  if (!s.ok) {
    return rollbackUploadedReplacement(s);
  }
  s = RefreshScenePrepResources(*runtime, false);
  if (!s.ok) {
    return rollbackUploadedReplacement(s);
  }
  ReleaseChunkRuntime(old);
  return Status::Ok();
} catch (const std::bad_alloc&) {
  return Status::Error("scene chunk allocation failed");
} catch (const std::length_error&) {
  return Status::Error("scene chunk allocation failed");
}

Status GaussianRasterPipeline::RemoveChunk(uint64_t sceneId, uint64_t chunkId) {
  std::shared_ptr<UploadedSceneRuntime> runtime;
  {
    std::shared_lock<std::shared_mutex> lock(uploadedScenesMutex_);
    auto it = uploadedScenes_.find(sceneId);
    if (it == uploadedScenes_.end()) {
      return Status::Error("uploaded scene handle not found");
    }
    runtime = it->second;
  }
  if (runtime == nullptr) {
    return Status::Error("uploaded scene handle not found");
  }
  std::lock_guard<std::mutex> runtimeLock(*runtime->mutex);
  const auto chunkIt = std::find_if(runtime->chunks.begin(), runtime->chunks.end(), [chunkId](const UploadedChunkRuntime& chunk) {
    return chunk.chunkId == chunkId;
  });
  if (chunkIt == runtime->chunks.end()) {
    return Status::Error("uploaded chunk handle not found");
  }
  const size_t chunkIndex = static_cast<size_t>(std::distance(runtime->chunks.begin(), chunkIt));
  const uint32_t previousTail = runtime->sceneAtlasTail;
  const auto previousFreeRanges = runtime->atlasFreeRanges;
  const bool previousIndexUploadPending = runtime->sceneIndexToChunkUploadPending;
  UploadedChunkRuntime removed = std::move(*chunkIt);
  if (removed.gaussianCapacity > 0) {
    Status freeStatus = FreeAtlasRange(*runtime, removed.gaussianOffset, removed.gaussianCapacity);
    if (!freeStatus.ok) {
      *chunkIt = std::move(removed);
      return freeStatus;
    }
  }
  runtime->chunks.erase(chunkIt);
  runtime->sceneIndexToChunkUploadPending = true;
  Status s = UpdateSceneCapacity(*runtime);
  if (!s.ok) {
    runtime->chunks.insert(runtime->chunks.begin() + static_cast<std::ptrdiff_t>(chunkIndex), std::move(removed));
    runtime->sceneAtlasTail = previousTail;
    runtime->atlasFreeRanges = previousFreeRanges;
    runtime->sceneIndexToChunkUploadPending = previousIndexUploadPending;
    UpdateSceneCapacity(*runtime);
    return s;
  }
  s = RefreshScenePrepResources(*runtime, true);
  if (!s.ok) {
    runtime->chunks.insert(runtime->chunks.begin() + static_cast<std::ptrdiff_t>(chunkIndex), std::move(removed));
    runtime->sceneAtlasTail = previousTail;
    runtime->atlasFreeRanges = previousFreeRanges;
    UpdateSceneCapacity(*runtime);
    runtime->sceneIndexToChunkUploadPending = true;
    Status rollback = RefreshScenePrepResources(*runtime, false);
    if (!rollback.ok) {
      return rollback;
    }
    return s;
  }
  ReleaseChunkRuntime(removed);
  return Status::Ok();
}

Status GaussianRasterPipeline::SetChunkEnabled(uint64_t sceneId, uint64_t chunkId, bool enabled) {
  std::shared_ptr<UploadedSceneRuntime> runtime;
  {
    std::shared_lock<std::shared_mutex> lock(uploadedScenesMutex_);
    auto it = uploadedScenes_.find(sceneId);
    if (it == uploadedScenes_.end()) {
      return Status::Error("uploaded scene handle not found");
    }
    runtime = it->second;
  }
  if (runtime == nullptr) {
    return Status::Error("uploaded scene handle not found");
  }
  std::lock_guard<std::mutex> runtimeLock(*runtime->mutex);
  const auto chunkIt = std::find_if(runtime->chunks.begin(), runtime->chunks.end(), [chunkId](const UploadedChunkRuntime& chunk) {
    return chunk.chunkId == chunkId;
  });
  if (chunkIt == runtime->chunks.end()) {
    return Status::Error("uploaded chunk handle not found");
  }
  const uint32_t previousVisible = chunkIt->params.visible;
  chunkIt->params.visible = enabled ? 1u : 0u;
  Status refresh = RefreshScenePrepResources(*runtime, false);
  if (!refresh.ok) {
    chunkIt->params.visible = previousVisible;
    RefreshScenePrepResources(*runtime, false);
    return refresh;
  }
  return Status::Ok();
}

Status GaussianRasterPipeline::SetChunkScalingModifier(uint64_t sceneId, uint64_t chunkId, float scalingModifier) {
  std::shared_ptr<UploadedSceneRuntime> runtime;
  {
    std::shared_lock<std::shared_mutex> lock(uploadedScenesMutex_);
    auto it = uploadedScenes_.find(sceneId);
    if (it == uploadedScenes_.end()) {
      return Status::Error("uploaded scene handle not found");
    }
    runtime = it->second;
  }
  if (runtime == nullptr) {
    return Status::Error("uploaded scene handle not found");
  }
  std::lock_guard<std::mutex> runtimeLock(*runtime->mutex);
  const auto chunkIt = std::find_if(runtime->chunks.begin(), runtime->chunks.end(), [chunkId](const UploadedChunkRuntime& chunk) {
    return chunk.chunkId == chunkId;
  });
  if (chunkIt == runtime->chunks.end()) {
    return Status::Error("uploaded chunk handle not found");
  }
  const float previousScalingModifier = chunkIt->params.scalingModifier;
  chunkIt->params.scalingModifier = std::max(0.0f, scalingModifier);
  Status refresh = RefreshScenePrepResources(*runtime, false);
  if (!refresh.ok) {
    chunkIt->params.scalingModifier = previousScalingModifier;
    RefreshScenePrepResources(*runtime, false);
    return refresh;
  }
  return Status::Ok();
}

Status GaussianRasterPipeline::EnsureRenderScratchBuffers(const UploadedSceneRuntime& runtime,
                                                          uint32_t requiredPairCapacity,
                                                          RenderScratch& scratch) {
  const uint32_t requiredDrawCapacity = std::max<uint32_t>(runtime.drawCapacity, 1u);
  requiredPairCapacity = std::max<uint32_t>(requiredPairCapacity, 1u);
  const uint32_t requiredPartitions = std::max<uint32_t>(oneSweep_ != nullptr ? oneSweep_->MaxPartitionsForElementCount(requiredPairCapacity) : 1u, 1u);
  const uint32_t prepStrideBytes = static_cast<uint32_t>(AlignUp(sizeof(PrepConstants), 256u));
  const size_t prepBytes = prepStrideBytes;

  if (scratch.sortMetaReadback == nullptr) {
    Status s = CreateReadbackBuffer(sizeof(SortMetaGpu), scratch.sortMetaReadback);
    if (!s.ok) {
      return s;
    }
  }
  if (scratch.projectionActiveThreadsReadback == nullptr) {
    Status s = CreateReadbackBuffer(kStatsHistogramBytes, scratch.projectionActiveThreadsReadback);
    if (!s.ok) {
      return s;
    }
  }
  if (gpuTimingEnabled_ && scratch.timestampQueryHeap == nullptr) {
    D3D12_QUERY_HEAP_DESC desc{};
    desc.Count = kTimestampQueryCount;
    desc.Type = D3D12_QUERY_HEAP_TYPE_TIMESTAMP;
    HRESULT hr = device_->CreateQueryHeap(&desc, IID_PPV_ARGS(scratch.timestampQueryHeap.GetAddressOf()));
    if (FAILED(hr)) {
      return Status::Error("failed creating timestamp query heap");
    }
  }
  if (gpuTimingEnabled_ && scratch.timestampReadbackBuffer == nullptr) {
    Status s = CreateReadbackBuffer(static_cast<size_t>(kTimestampQueryCount) * sizeof(uint64_t), scratch.timestampReadbackBuffer);
    if (!s.ok) {
      return s;
    }
  }

  Status s = EnsureUploadBuffer(prepBytes, scratch.prepConstantsUpload, scratch.prepConstantsCapacityBytes, scratch.prepConstantsMapped);
  if (!s.ok) {
    return s;
  }
  s = EnsureUploadBuffer(sizeof(RasterConstants), scratch.rasterConstantsUpload, scratch.rasterConstantsCapacityBytes,
                         scratch.rasterConstantsMapped);
  if (!s.ok) {
    return s;
  }

  const bool missingScratch = scratch.sortKeysBuffer == nullptr ||
                              scratch.sortKeysTempBuffer == nullptr ||
                              scratch.sortValuesBuffer == nullptr ||
                              scratch.sortValuesTempBuffer == nullptr ||
                              scratch.visibleCounterBuffer == nullptr ||
                              scratch.sortMetaBuffer == nullptr ||
                              scratch.oneSweepPassHistogramBuffer == nullptr ||
                              scratch.oneSweepGlobalHistogramBuffer == nullptr ||
                              scratch.oneSweepIndexBuffer == nullptr ||
                              scratch.oneSweepDispatchArgsBuffer == nullptr ||
                              scratch.drawArgsBuffer == nullptr;
  if (!missingScratch && scratch.drawCapacity >= requiredDrawCapacity &&
      scratch.sortPairCapacity >= requiredPairCapacity &&
      scratch.oneSweepPartitionCount >= requiredPartitions) {
    scratch.prepConstantStrideBytes = prepStrideBytes;
    return Status::Ok();
  }

  uint64_t newDraw = requiredDrawCapacity;
  if (scratch.drawCapacity > 0) {
    newDraw = std::max<uint64_t>(newDraw, static_cast<uint64_t>(scratch.drawCapacity) * 2u);
  }
  newDraw = std::min<uint64_t>(newDraw, UINT32_MAX);
  uint64_t newPairs = requiredPairCapacity;
  if (scratch.sortPairCapacity > 0) {
    newPairs = std::max<uint64_t>(newPairs, static_cast<uint64_t>(scratch.sortPairCapacity) * 2u);
  }
  newPairs = std::min<uint64_t>(newPairs, kMaxSortPairCapacity);
  const uint32_t newDrawCapacity = static_cast<uint32_t>(std::max<uint64_t>(newDraw, requiredDrawCapacity));
  const uint32_t newPairCapacity = static_cast<uint32_t>(std::max<uint64_t>(newPairs, requiredPairCapacity));
  const uint32_t newPartitionCount = std::max<uint32_t>(oneSweep_ != nullptr ? oneSweep_->MaxPartitionsForElementCount(newPairCapacity) : 1u, 1u);
  const size_t keyBytes = static_cast<size_t>(newPairCapacity) * sizeof(uint32_t);
  const size_t valueBytes = static_cast<size_t>(newPairCapacity) * sizeof(uint32_t);
  const size_t passHistogramBytes = static_cast<size_t>(kOneSweepRadix) * kOneSweepPassCount * newPartitionCount * sizeof(uint32_t);
  const size_t globalHistogramBytes = static_cast<size_t>(kOneSweepRadix) * kOneSweepPassCount * sizeof(uint32_t);
  const size_t oneSweepIndexBytes = static_cast<size_t>(kOneSweepPassCount) * sizeof(uint32_t);
  const size_t oneSweepDispatchArgsBytes = static_cast<size_t>(kSortIndirectCommandCount) * kOneSweepIndirectCommandStride;

  ComPtr<ID3D12Resource> sortKeysBuffer;
  ComPtr<ID3D12Resource> sortKeysTempBuffer;
  ComPtr<ID3D12Resource> sortValuesBuffer;
  ComPtr<ID3D12Resource> sortValuesTempBuffer;
  ComPtr<ID3D12Resource> visibleCounterBuffer;
  ComPtr<ID3D12Resource> sortMetaBuffer;
  ComPtr<ID3D12Resource> oneSweepPassHistogramBuffer;
  ComPtr<ID3D12Resource> oneSweepGlobalHistogramBuffer;
  ComPtr<ID3D12Resource> oneSweepIndexBuffer;
  ComPtr<ID3D12Resource> oneSweepDispatchArgsBuffer;
  ComPtr<ID3D12Resource> drawArgsBuffer;

  s = CreateDefaultBuffer(keyBytes, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS,
                          D3D12_RESOURCE_STATE_UNORDERED_ACCESS, sortKeysBuffer);
  if (!s.ok) return s;
  s = CreateDefaultBuffer(keyBytes, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS,
                          D3D12_RESOURCE_STATE_UNORDERED_ACCESS, sortKeysTempBuffer);
  if (!s.ok) return s;
  s = CreateDefaultBuffer(valueBytes, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS,
                          D3D12_RESOURCE_STATE_UNORDERED_ACCESS, sortValuesBuffer);
  if (!s.ok) return s;
  s = CreateDefaultBuffer(valueBytes, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS,
                          D3D12_RESOURCE_STATE_UNORDERED_ACCESS, sortValuesTempBuffer);
  if (!s.ok) return s;
  s = CreateDefaultBuffer(kVisibleCounterBytes, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS,
                          D3D12_RESOURCE_STATE_UNORDERED_ACCESS, visibleCounterBuffer);
  if (!s.ok) return s;
  s = CreateDefaultBuffer(sizeof(SortMetaGpu), D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS,
                          D3D12_RESOURCE_STATE_UNORDERED_ACCESS, sortMetaBuffer);
  if (!s.ok) return s;
  s = CreateDefaultBuffer(passHistogramBytes, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS,
                          D3D12_RESOURCE_STATE_UNORDERED_ACCESS, oneSweepPassHistogramBuffer);
  if (!s.ok) return s;
  s = CreateDefaultBuffer(globalHistogramBytes, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS,
                          D3D12_RESOURCE_STATE_UNORDERED_ACCESS, oneSweepGlobalHistogramBuffer);
  if (!s.ok) return s;
  s = CreateDefaultBuffer(oneSweepIndexBytes, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS,
                          D3D12_RESOURCE_STATE_UNORDERED_ACCESS, oneSweepIndexBuffer);
  if (!s.ok) return s;
  s = CreateDefaultBuffer(oneSweepDispatchArgsBytes, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS,
                          D3D12_RESOURCE_STATE_UNORDERED_ACCESS, oneSweepDispatchArgsBuffer);
  if (!s.ok) return s;
  s = CreateDefaultBuffer(16u, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS,
                          D3D12_RESOURCE_STATE_UNORDERED_ACCESS, drawArgsBuffer);
  if (!s.ok) return s;

  s = ReserveRetiredResourceSlots(11);
  if (!s.ok) return s;
  s = RetireResource(scratch.sortKeysBuffer);
  if (!s.ok) return s;
  s = RetireResource(scratch.sortKeysTempBuffer);
  if (!s.ok) return s;
  s = RetireResource(scratch.sortValuesBuffer);
  if (!s.ok) return s;
  s = RetireResource(scratch.sortValuesTempBuffer);
  if (!s.ok) return s;
  s = RetireResource(scratch.visibleCounterBuffer);
  if (!s.ok) return s;
  s = RetireResource(scratch.sortMetaBuffer);
  if (!s.ok) return s;
  s = RetireResource(scratch.oneSweepPassHistogramBuffer);
  if (!s.ok) return s;
  s = RetireResource(scratch.oneSweepGlobalHistogramBuffer);
  if (!s.ok) return s;
  s = RetireResource(scratch.oneSweepIndexBuffer);
  if (!s.ok) return s;
  s = RetireResource(scratch.oneSweepDispatchArgsBuffer);
  if (!s.ok) return s;
  s = RetireResource(scratch.drawArgsBuffer);
  if (!s.ok) return s;

  scratch.sortKeysBuffer = std::move(sortKeysBuffer);
  scratch.sortKeysTempBuffer = std::move(sortKeysTempBuffer);
  scratch.sortValuesBuffer = std::move(sortValuesBuffer);
  scratch.sortValuesTempBuffer = std::move(sortValuesTempBuffer);
  scratch.visibleCounterBuffer = std::move(visibleCounterBuffer);
  scratch.sortMetaBuffer = std::move(sortMetaBuffer);
  scratch.oneSweepPassHistogramBuffer = std::move(oneSweepPassHistogramBuffer);
  scratch.oneSweepGlobalHistogramBuffer = std::move(oneSweepGlobalHistogramBuffer);
  scratch.oneSweepIndexBuffer = std::move(oneSweepIndexBuffer);
  scratch.oneSweepDispatchArgsBuffer = std::move(oneSweepDispatchArgsBuffer);
  scratch.drawArgsBuffer = std::move(drawArgsBuffer);

  scratch.sortKeysState = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
  scratch.sortKeysTempState = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
  scratch.sortValuesState = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
  scratch.sortValuesTempState = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
  scratch.visibleCounterState = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
  scratch.sortMetaState = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
  scratch.oneSweepPassHistogramState = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
  scratch.oneSweepGlobalHistogramState = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
  scratch.oneSweepIndexState = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
  scratch.oneSweepDispatchArgsState = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
  scratch.drawArgsState = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
  scratch.drawCapacity = newDrawCapacity;
  scratch.sortPairCapacity = newPairCapacity;
  scratch.oneSweepPartitionCount = newPartitionCount;
  scratch.prepConstantStrideBytes = prepStrideBytes;
  return Status::Ok();
}

Status GaussianRasterPipeline::AcquireRenderScratch(UploadedSceneRuntime& runtime,
                                                    uint32_t requiredPairCapacity,
                                                    const RenderFrameContext* frameContext,
                                                    std::shared_ptr<RenderScratch>& outScratch) {
  outScratch.reset();
  UpdateDirectQueueFenceProgress(frameContext);
  CollectRuntimeScratch(runtime);
  std::shared_ptr<RenderScratch> scratch;
  {
    std::lock_guard<std::mutex> runtimeLock(*runtime.mutex);
    if (!runtime.availableScratch.empty()) {
      scratch = runtime.availableScratch.back();
      runtime.availableScratch.pop_back();
    }
  }
  if (scratch == nullptr) {
    scratch = std::make_shared<RenderScratch>();
  }
  Status ready = EnsureRenderScratchBuffers(runtime, requiredPairCapacity, *scratch);
  if (!ready.ok) {
    Status released = ReleaseRenderScratchResources(*scratch);
    if (!released.ok) {
      return released;
    }
    return ready;
  }
  outScratch = std::move(scratch);
  return Status::Ok();
}

void GaussianRasterPipeline::ReleaseRenderScratch(UploadedSceneRuntime& runtime,
                                                  std::shared_ptr<RenderScratch> scratch,
                                                  const RenderFrameContext* frameContext,
                                                  bool trackSubmission) {
  if (scratch == nullptr) {
    return;
  }
  UpdateDirectQueueFenceProgress(frameContext);
  std::lock_guard<std::mutex> runtimeLock(*runtime.mutex);
  runtime.publishedScratch = scratch;
  if (trackSubmission && frameContext != nullptr && frameContext->fence != nullptr && frameContext->submissionFenceValue != 0) {
    scratch->inFlightSelf = scratch;
    scratch->inFlightFence = frameContext->fence;
    scratch->inFlightFenceValue = frameContext->submissionFenceValue;
    scratch->inFlightNext = runtime.inFlightScratchHead;
    runtime.inFlightScratchHead = scratch.get();
  }
}

GpuBufferView GaussianRasterPipeline::MakeBufferView(ID3D12Resource* resource,
                                                     D3D12_RESOURCE_STATES state,
                                                     uint64_t sizeBytes,
                                                     uint32_t strideBytes,
                                                     GpuViewLifetime lifetime,
                                                     GpuResourceAccess access,
                                                     bool callerMayTransition,
                                                     bool callerMayWrite) const {
  return MakeBufferView(resource, state, 0, sizeBytes, strideBytes, lifetime, access, callerMayTransition, callerMayWrite);
}

GpuBufferView GaussianRasterPipeline::MakeBufferView(ID3D12Resource* resource,
                                                     D3D12_RESOURCE_STATES state,
                                                     uint64_t byteOffset,
                                                     uint64_t sizeBytes,
                                                     uint32_t strideBytes,
                                                     GpuViewLifetime lifetime,
                                                     GpuResourceAccess access,
                                                     bool callerMayTransition,
                                                     bool callerMayWrite) const {
  GpuBufferView view{};
  view.resource = resource;
  view.gpuVirtualAddress = resource != nullptr ? resource->GetGPUVirtualAddress() + byteOffset : 0;
  view.sizeBytes = sizeBytes;
  view.strideBytes = strideBytes;
  view.state = state;
  view.access = access;
  view.lifetime = lifetime;
  view.callerMayTransition = callerMayTransition;
  view.callerMayWrite = callerMayWrite;
  return view;
}

GpuTextureView GaussianRasterPipeline::MakeTextureView(ID3D12Resource* resource,
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
                                                       bool callerMayWrite) const {
  GpuTextureView view{};
  view.resource = resource;
  if (resource != nullptr) {
    const D3D12_RESOURCE_DESC desc = resource->GetDesc();
    if (desc.Dimension != D3D12_RESOURCE_DIMENSION_BUFFER) {
      view.width = static_cast<uint32_t>(std::max<UINT64>(desc.Width, 0));
      view.height = desc.Height;
      view.mipLevels = std::max<uint16_t>(desc.MipLevels, 1);
      if (format == DXGI_FORMAT_UNKNOWN) {
        format = desc.Format;
      }
    }
  }
  if (view.width == 0) {
    view.width = width;
  }
  if (view.height == 0) {
    view.height = height;
  }
  view.format = format;
  view.rtv = rtv;
  view.dsv = dsv;
  view.srv = srv;
  view.state = state;
  view.access = access;
  view.lifetime = lifetime;
  view.callerMayTransition = callerMayTransition;
  view.callerMayWrite = callerMayWrite;
  return view;
}

void GaussianRasterPipeline::PopulateFrameResources(const RenderTargetBinding& target,
                                                    const UploadedSceneRuntime& runtime,
                                                    const RenderScratch& scratch,
                                                    D3D12_RESOURCE_STATES colorState,
                                                    D3D12_RESOURCE_STATES depthState,
                                                    D3D12_RESOURCE_STATES motionState,
                                                    GpuFrameResources& out) const {
  out = {};
  out.colorTarget = MakeTextureView(target.colorTarget,
                                    target.colorFormat,
                                    static_cast<uint32_t>(std::max<LONG>(target.viewport.Width > 0.0f ? static_cast<LONG>(target.viewport.Width) : 0, 0)),
                                    static_cast<uint32_t>(std::max<LONG>(target.viewport.Height > 0.0f ? static_cast<LONG>(target.viewport.Height) : 0, 0)),
                                    target.colorRtv,
                                    {},
                                    target.colorSrv,
                                    colorState,
                                    GpuResourceAccess::ReadWrite,
                                    GpuViewLifetime::CurrentRenderCall,
                                    false,
                                    false);
  out.depthTarget = MakeTextureView(target.depthTarget,
                                    target.depthFormat,
                                    out.colorTarget.width,
                                    out.colorTarget.height,
                                    {},
                                    target.depthDsv,
                                    target.depthSrv,
                                    depthState,
                                    GpuResourceAccess::ReadWrite,
                                    GpuViewLifetime::CurrentRenderCall,
                                    false,
                                    false);
  out.motionVectorsTarget = MakeTextureView(target.motionVectorsTarget,
                                            target.motionVectorsFormat,
                                            out.colorTarget.width,
                                            out.colorTarget.height,
                                            target.motionVectorsRtv,
                                            {},
                                            target.motionVectorsSrv,
                                            motionState,
                                            GpuResourceAccess::ReadWrite,
                                            GpuViewLifetime::CurrentRenderCall,
                                            false,
                                            false);
  out.colorValid = target.colorTarget != nullptr;
  out.depthValid = target.depthTarget != nullptr;
  out.motionVectorsValid = false;
  out.vramFormat = runtime.vramFormat;
  out.packedStrideBytes = runtime.sceneGaussianStride;
  out.sceneGaussians = MakeBufferView(runtime.sceneAtlasBuffer.Get(),
                                     D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE,
                                     static_cast<uint64_t>(runtime.sceneAtlasCapacity) * runtime.sceneGaussianStride,
                                     runtime.sceneGaussianStride,
                                     GpuViewLifetime::UploadedSceneLifetime,
                                     GpuResourceAccess::ReadOnly,
                                     false,
                                     false);
  out.sceneIndexToChunk = MakeBufferView(runtime.sceneIndexToChunkBuffer.Get(),
                                         D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE,
                                         static_cast<uint64_t>(runtime.sceneIndexToChunkCapacity) * sizeof(uint32_t),
                                         sizeof(uint32_t),
                                         GpuViewLifetime::UploadedSceneLifetime,
                                         GpuResourceAccess::ReadOnly,
                                         false,
                                         false);
  const bool sortedInPrimary = scratch.lastSortedInPrimary;
  out.sortedSceneIndices = MakeBufferView(sortedInPrimary ? scratch.sortValuesBuffer.Get() : scratch.sortValuesTempBuffer.Get(),
                                          sortedInPrimary ? scratch.sortValuesState : scratch.sortValuesTempState,
                                          static_cast<uint64_t>(scratch.sortPairCapacity) * sizeof(uint32_t),
                                          sizeof(uint32_t),
                                          GpuViewLifetime::CurrentRenderCall,
                                          GpuResourceAccess::ReadOnly,
                                          false,
                                          false);
  out.secondarySortedSceneIndices = MakeBufferView(sortedInPrimary ? scratch.sortValuesTempBuffer.Get() : scratch.sortValuesBuffer.Get(),
                                                   sortedInPrimary ? scratch.sortValuesTempState : scratch.sortValuesState,
                                                   static_cast<uint64_t>(scratch.sortPairCapacity) * sizeof(uint32_t),
                                                   sizeof(uint32_t),
                                                   GpuViewLifetime::CurrentRenderCall,
                                                   GpuResourceAccess::ReadWrite,
                                                   false,
                                                   false);
  out.visibleCounter = MakeBufferView(scratch.visibleCounterBuffer.Get(),
                                      scratch.visibleCounterState,
                                      kVisibleCounterBytes,
                                      4u,
                                      GpuViewLifetime::CurrentRenderCall,
                                      GpuResourceAccess::ReadWrite,
                                      false,
                                      false);
  out.projectionActiveThreads = MakeBufferView(scratch.visibleCounterBuffer.Get(),
                                               scratch.visibleCounterState,
                                               kProjectionActiveThreadHistogramOffset,
                                               kProjectionActiveThreadHistogramBytes,
                                               sizeof(uint32_t),
                                               GpuViewLifetime::CurrentRenderCall,
                                               GpuResourceAccess::ReadWrite,
                                               false,
                                               false);
  out.drawArgs = MakeBufferView(scratch.drawArgsBuffer.Get(),
                                scratch.drawArgsState,
                                16u,
                                sizeof(uint32_t),
                                GpuViewLifetime::CurrentRenderCall,
                                GpuResourceAccess::ReadWrite,
                                false,
                                false);
  out.sortMeta = MakeBufferView(scratch.sortMetaBuffer.Get(),
                                scratch.sortMetaState,
                                sizeof(SortMetaGpu),
                                sizeof(uint32_t),
                                GpuViewLifetime::CurrentRenderCall,
                                GpuResourceAccess::ReadWrite,
                                false,
                                false);
  out.visibleSplatCount = scratch.lastVisibleCount;
  out.emittedSortEntryCount = scratch.lastPairCount;
  out.drawInstanceCount = scratch.lastVisibleCount;
  out.sortBackend = SortBackend::OneSweep;
  out.depthOutput = DepthOutputKind::None;
}

Status GaussianRasterPipeline::InvokeHook(const std::function<void(const RenderHookContext&)>& hook,
                                          RenderHookStage stage,
                                          ID3D12GraphicsCommandList* commandList,
                                          UploadedSceneHandle sceneHandle,
                                          const RenderInput& input,
                                          const RenderTargetBinding& target,
                                          const GpuFrameResources& resources,
                                          FrameStats& stats) const {
  if (!hook) {
    return Status::Ok();
  }
  RenderHookContext context{};
  context.stage = stage;
  context.commandList = commandList;
  context.scene = sceneHandle;
  context.input = &input;
  context.targets = &target;
  context.resources = &resources;
  context.stats = &stats;
  context.frameIndex = input.frameIndex;
  try {
    hook(context);
  } catch (const std::exception&) {
    return Status::Error("render hook failed");
  } catch (...) {
    return Status::Error("render hook failed");
  }
  return Status::Ok();
}

Status GaussianRasterPipeline::EnsureColorRasterPso(DXGI_FORMAT colorFormat) {
  std::lock_guard<std::mutex> colorLock(colorRasterMutex_);
  const int cacheKey = ColorPsoKey(colorFormat);
  auto existing = colorRasterPsos_.find(cacheKey);
  if (existing != colorRasterPsos_.end() && existing->second != nullptr) {
    return Status::Ok();
  }

  ComPtr<ID3DBlob> vsBlob;
  ComPtr<ID3DBlob> psBlob;
  Status s = CompileShader(embedded::kGaussianRasterShaderName, embedded::kGaussianRasterShaderSource,
                           embedded::kGaussianRasterShaderSize, "VSMainBeauty", "vs_5_1", vsBlob);
  if (!s.ok) {
    return s;
  }
  s = CompileShader(embedded::kGaussianRasterShaderName, embedded::kGaussianRasterShaderSource,
                    embedded::kGaussianRasterShaderSize, "PSMainBeauty", "ps_5_1", psBlob);
  if (!s.ok) {
    return s;
  }

  D3D12_BLEND_DESC blend{};
  D3D12_RENDER_TARGET_BLEND_DESC rtBlend{};
  rtBlend.BlendEnable = TRUE;
  rtBlend.SrcBlend = D3D12_BLEND_SRC_ALPHA;
  rtBlend.DestBlend = D3D12_BLEND_INV_SRC_ALPHA;
  rtBlend.BlendOp = D3D12_BLEND_OP_ADD;
  rtBlend.SrcBlendAlpha = D3D12_BLEND_ONE;
  rtBlend.DestBlendAlpha = D3D12_BLEND_INV_SRC_ALPHA;
  rtBlend.BlendOpAlpha = D3D12_BLEND_OP_ADD;
  rtBlend.RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL;
  blend.RenderTarget[0] = rtBlend;

  D3D12_RASTERIZER_DESC raster{};
  raster.FillMode = D3D12_FILL_MODE_SOLID;
  raster.CullMode = D3D12_CULL_MODE_NONE;
  raster.DepthClipEnable = TRUE;

  D3D12_GRAPHICS_PIPELINE_STATE_DESC desc{};
  desc.pRootSignature = rasterRootSignature_.Get();
  desc.VS = {vsBlob->GetBufferPointer(), vsBlob->GetBufferSize()};
  desc.PS = {psBlob->GetBufferPointer(), psBlob->GetBufferSize()};
  desc.BlendState = blend;
  desc.SampleMask = UINT_MAX;
  desc.RasterizerState = raster;
  desc.DepthStencilState.DepthEnable = FALSE;
  desc.DepthStencilState.StencilEnable = FALSE;
  desc.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
  desc.NumRenderTargets = 1;
  desc.RTVFormats[0] = colorFormat;
  desc.SampleDesc.Count = 1;

  ComPtr<ID3D12PipelineState> pso;
  if (FAILED(device_->CreateGraphicsPipelineState(&desc, IID_PPV_ARGS(pso.GetAddressOf())))) {
    return Status::Error("failed creating color raster pso");
  }
  colorRasterPsos_[cacheKey] = std::move(pso);
  return Status::Ok();
}

Status GaussianRasterPipeline::EnsureDepthRasterPso(DXGI_FORMAT depthFormat) {
  std::lock_guard<std::mutex> depthLock(depthRasterMutex_);
  auto existing = depthRasterPsos_.find(static_cast<int>(depthFormat));
  if (existing != depthRasterPsos_.end() && existing->second != nullptr) {
    return Status::Ok();
  }

  ComPtr<ID3DBlob> vsBlob;
  ComPtr<ID3DBlob> psBlob;
  Status s = CompileShader(embedded::kGaussianRasterShaderName, embedded::kGaussianRasterShaderSource,
                           embedded::kGaussianRasterShaderSize, "VSMainDepth", "vs_5_1", vsBlob);
  if (!s.ok) {
    return s;
  }
  s = CompileShader(embedded::kGaussianRasterShaderName, embedded::kGaussianRasterShaderSource,
                    embedded::kGaussianRasterShaderSize, "PSDepth", "ps_5_1", psBlob);
  if (!s.ok) {
    return s;
  }

  D3D12_RASTERIZER_DESC raster{};
  raster.FillMode = D3D12_FILL_MODE_SOLID;
  raster.CullMode = D3D12_CULL_MODE_NONE;
  raster.DepthClipEnable = TRUE;

  D3D12_DEPTH_STENCIL_DESC depth{};
  depth.DepthEnable = TRUE;
  depth.DepthWriteMask = D3D12_DEPTH_WRITE_MASK_ALL;
  depth.DepthFunc = D3D12_COMPARISON_FUNC_LESS_EQUAL;
  depth.StencilEnable = FALSE;

  D3D12_GRAPHICS_PIPELINE_STATE_DESC desc{};
  desc.pRootSignature = rasterRootSignature_.Get();
  desc.VS = {vsBlob->GetBufferPointer(), vsBlob->GetBufferSize()};
  desc.PS = {psBlob->GetBufferPointer(), psBlob->GetBufferSize()};
  desc.SampleMask = UINT_MAX;
  desc.RasterizerState = raster;
  desc.DepthStencilState = depth;
  desc.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
  desc.NumRenderTargets = 0;
  desc.DSVFormat = depthFormat;
  desc.SampleDesc.Count = 1;

  ComPtr<ID3D12PipelineState> pso;
  if (FAILED(device_->CreateGraphicsPipelineState(&desc, IID_PPV_ARGS(pso.GetAddressOf())))) {
    return Status::Error("failed creating depth raster pso");
  }
  depthRasterPsos_[static_cast<int>(depthFormat)] = std::move(pso);
  return Status::Ok();
}

Status GaussianRasterPipeline::GetChunkGpuResources(uint64_t sceneId,
                                                    uint64_t chunkId,
                                                    UploadedChunkGpuResources& out) const {
  out = {};
  std::shared_ptr<UploadedSceneRuntime> runtime;
  {
    std::shared_lock<std::shared_mutex> lock(uploadedScenesMutex_);
    const auto sceneIt = uploadedScenes_.find(sceneId);
    if (sceneIt == uploadedScenes_.end()) {
      return Status::Error("uploaded scene handle not found");
    }
    runtime = sceneIt->second;
  }
  if (runtime == nullptr) {
    return Status::Error("uploaded scene handle not found");
  }
  std::lock_guard<std::mutex> runtimeLock(*runtime->mutex);
  const auto chunkIt = std::find_if(runtime->chunks.begin(), runtime->chunks.end(),
                                    [chunkId](const UploadedChunkRuntime& chunk) { return chunk.chunkId == chunkId; });
  if (chunkIt == runtime->chunks.end()) {
    return Status::Error("uploaded chunk handle not found");
  }
  out.handle = UploadedChunkHandle{chunkId};
  out.gaussianCount = chunkIt->gaussianCount;
  out.packedStrideBytes = runtime->sceneGaussianStride;
  out.format = UploadedSceneBufferFormat::CompactPackedVariable;
  out.vramFormat = runtime->vramFormat;
  out.decodeMin[0] = chunkIt->params.decodeMin[0];
  out.decodeMin[1] = chunkIt->params.decodeMin[1];
  out.decodeMin[2] = chunkIt->params.decodeMin[2];
  out.decodeExtent[0] = chunkIt->params.decodeExtent[0];
  out.decodeExtent[1] = chunkIt->params.decodeExtent[1];
  out.decodeExtent[2] = chunkIt->params.decodeExtent[2];
  out.visible = chunkIt->params.visible != 0u;
  out.gaussianData = MakeBufferView(runtime->sceneAtlasBuffer.Get(),
                                    D3D12_RESOURCE_STATE_COMMON,
                                    static_cast<uint64_t>(chunkIt->gaussianOffset) * runtime->sceneGaussianStride,
                                    static_cast<uint64_t>(chunkIt->gaussianCount) * runtime->sceneGaussianStride,
                                    runtime->sceneGaussianStride,
                                    GpuViewLifetime::UploadedSceneLifetime,
                                    GpuResourceAccess::ReadOnly,
                                    false,
                                    false);
  return Status::Ok();
}

Status GaussianRasterPipeline::GetSceneGpuResources(uint64_t sceneId,
                                                    const RenderFrameContext* frameContext,
                                                    bool acquireLease,
                                                    UploadedSceneGpuResources& out) try {
  out = {};
  Status frameStatus = ValidateRenderFrameContext(frameContext, false);
  if (!frameStatus.ok) {
    return frameStatus;
  }
  std::shared_ptr<UploadedSceneRuntime> runtime;
  {
    std::shared_lock<std::shared_mutex> lock(uploadedScenesMutex_);
    const auto sceneIt = uploadedScenes_.find(sceneId);
    if (sceneIt == uploadedScenes_.end()) {
      return Status::Error("uploaded scene handle not found");
    }
    runtime = sceneIt->second;
  }
  if (runtime == nullptr) {
    return Status::Error("uploaded scene handle not found");
  }
  if (acquireLease) {
    frameStatus = ValidateRenderFrameContext(frameContext, true);
    if (!frameStatus.ok) {
      return frameStatus;
    }
  }
  std::lock_guard<std::mutex> runtimeLock(*runtime->mutex);
  out.scene = UploadedSceneHandle{sceneId};
  out.vramFormat = runtime->vramFormat;
  out.packedStrideBytes = runtime->sceneGaussianStride;
  uint64_t pendingUploadFenceValue = runtime->pendingUploadFenceValue;
  uint64_t directQueueUploadWaitValue = runtime->directQueueUploadWaitValue;
  out.sceneGaussians = MakeBufferView(runtime->sceneAtlasBuffer.Get(),
                                      D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE,
                                      static_cast<uint64_t>(runtime->sceneAtlasCapacity) * runtime->sceneGaussianStride,
                                      runtime->sceneGaussianStride,
                                      GpuViewLifetime::UploadedSceneLifetime,
                                      GpuResourceAccess::ReadOnly,
                                      false,
                                      false);
  out.sceneIndexToChunk = MakeBufferView(runtime->sceneIndexToChunkBuffer.Get(),
                                         D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE,
                                         static_cast<uint64_t>(runtime->sceneIndexToChunkCapacity) * sizeof(uint32_t),
                                         sizeof(uint32_t),
                                         GpuViewLifetime::UploadedSceneLifetime,
                                         GpuResourceAccess::ReadOnly,
                                         false,
                                         false);
  out.chunks.reserve(runtime->chunks.size());
  for (const UploadedChunkRuntime& chunk : runtime->chunks) {
    UploadedChunkGpuResources chunkResources{};
    chunkResources.handle = UploadedChunkHandle{chunk.chunkId};
    chunkResources.gaussianCount = chunk.gaussianCount;
    chunkResources.packedStrideBytes = runtime->sceneGaussianStride;
    chunkResources.format = UploadedSceneBufferFormat::CompactPackedVariable;
    chunkResources.vramFormat = runtime->vramFormat;
    chunkResources.decodeMin[0] = chunk.params.decodeMin[0];
    chunkResources.decodeMin[1] = chunk.params.decodeMin[1];
    chunkResources.decodeMin[2] = chunk.params.decodeMin[2];
    chunkResources.decodeExtent[0] = chunk.params.decodeExtent[0];
    chunkResources.decodeExtent[1] = chunk.params.decodeExtent[1];
    chunkResources.decodeExtent[2] = chunk.params.decodeExtent[2];
    chunkResources.visible = chunk.params.visible != 0u;
    chunkResources.gaussianData = MakeBufferView(runtime->sceneAtlasBuffer.Get(),
                                                 D3D12_RESOURCE_STATE_COMMON,
                                                 static_cast<uint64_t>(chunk.gaussianOffset) * runtime->sceneGaussianStride,
                                                 static_cast<uint64_t>(chunk.gaussianCount) * runtime->sceneGaussianStride,
                                                 runtime->sceneGaussianStride,
                                                 GpuViewLifetime::UploadedSceneLifetime,
                                                 GpuResourceAccess::ReadOnly,
                                                 false,
                                                 false);
    out.chunks.push_back(std::move(chunkResources));
  }
  if (pendingUploadFenceValue > directQueueUploadWaitValue && uploadFence_ != nullptr) {
    std::lock_guard<std::recursive_mutex> uploadLock(uploadMutex_);
    if (uploadFence_->GetCompletedValue() < pendingUploadFenceValue) {
      out.submission.uploadSyncPoint.fence = uploadFence_.Get();
      out.submission.uploadSyncPoint.value = pendingUploadFenceValue;
    } else {
      runtime->directQueueUploadWaitValue = std::max(runtime->directQueueUploadWaitValue, pendingUploadFenceValue);
    }
  }
  if (acquireLease) {
    RecordDirectQueueSubmission(frameContext);
    out.leaseFence = frameContext->fence;
    out.leaseFenceValue = frameContext->submissionFenceValue;
    out.submission.submissionRequired = true;
  }
  return Status::Ok();
} catch (const std::bad_alloc&) {
  out = {};
  return Status::Error("gpu resource snapshot allocation failed");
} catch (const std::length_error&) {
  out = {};
  return Status::Error("gpu resource snapshot allocation failed");
}
void GaussianRasterPipeline::Transition(ID3D12GraphicsCommandList* cmd, ID3D12Resource* resource,
                                        D3D12_RESOURCE_STATES before, D3D12_RESOURCE_STATES after) {
  if (resource == nullptr || before == after) {
    return;
  }
  D3D12_RESOURCE_BARRIER barrier{};
  barrier.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
  barrier.Transition.pResource = resource;
  barrier.Transition.StateBefore = before;
  barrier.Transition.StateAfter = after;
  barrier.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
  cmd->ResourceBarrier(1, &barrier);
}

Status GaussianRasterPipeline::CreatePipelines() {
  ComPtr<ID3DBlob> prepCs;
  ComPtr<ID3DBlob> fillSortTailCs;
  ComPtr<ID3DBlob> sortMetaCs;
  ComPtr<ID3DBlob> buildSortDispatchArgsCs;
  ComPtr<ID3DBlob> resetCs;
  ComPtr<ID3DBlob> finalizeCs;
  Status s = CompileShader(embedded::kGaussianComputeShaderName, embedded::kGaussianComputeShaderSource,
                           embedded::kGaussianComputeShaderSize, "CSPrepare", "cs_5_1", prepCs);
  if (!s.ok) return s;
  s = CompileShader(embedded::kGaussianComputeShaderName, embedded::kGaussianComputeShaderSource,
                    embedded::kGaussianComputeShaderSize, "CSFillSortTail", "cs_5_1", fillSortTailCs);
  if (!s.ok) return s;
  s = CompileShader(embedded::kGaussianComputeShaderName, embedded::kGaussianComputeShaderSource,
                    embedded::kGaussianComputeShaderSize, "CSBuildSortMeta", "cs_5_1", sortMetaCs);
  if (!s.ok) return s;
  s = CompileShader(embedded::kGaussianComputeShaderName, embedded::kGaussianComputeShaderSource,
                    embedded::kGaussianComputeShaderSize, "CSBuildOneSweepDispatchArgs", "cs_5_1", buildSortDispatchArgsCs);
  if (!s.ok) return s;
  s = CompileShader(embedded::kGaussianComputeShaderName, embedded::kGaussianComputeShaderSource,
                    embedded::kGaussianComputeShaderSize, "CSReset", "cs_5_1", resetCs);
  if (!s.ok) return s;
  s = CompileShader(embedded::kGaussianComputeShaderName, embedded::kGaussianComputeShaderSource,
                    embedded::kGaussianComputeShaderSize, "CSFinalizeDrawArgs", "cs_5_1", finalizeCs);
  if (!s.ok) return s;

  auto createRootSignature = [&](D3D12_ROOT_PARAMETER* params, UINT paramCount,
                                 D3D12_ROOT_SIGNATURE_FLAGS flags,
                                 Microsoft::WRL::ComPtr<ID3D12RootSignature>& out,
                                 const char* label) -> Status {
    D3D12_ROOT_SIGNATURE_DESC desc{};
    desc.NumParameters = paramCount;
    desc.pParameters = params;
    desc.Flags = flags;
    ComPtr<ID3DBlob> blob;
    ComPtr<ID3DBlob> err;
    if (FAILED(D3D12SerializeRootSignature(&desc, D3D_ROOT_SIGNATURE_VERSION_1, blob.GetAddressOf(), err.GetAddressOf()))) {
      return Status::Error(std::string("failed serializing ") + label + " root signature");
    }
    if (FAILED(device_->CreateRootSignature(0, blob->GetBufferPointer(), blob->GetBufferSize(),
                                            IID_PPV_ARGS(out.GetAddressOf())))) {
      return Status::Error(std::string("failed creating ") + label + " root signature");
    }
    return Status::Ok();
  };

  {
    D3D12_ROOT_PARAMETER params[6]{};
    params[0].ParameterType = D3D12_ROOT_PARAMETER_TYPE_CBV;
    params[0].Descriptor.ShaderRegister = 0;
    params[0].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;
    params[1].ParameterType = D3D12_ROOT_PARAMETER_TYPE_SRV;
    params[1].Descriptor.ShaderRegister = 0;
    params[1].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;
    params[2].ParameterType = D3D12_ROOT_PARAMETER_TYPE_SRV;
    params[2].Descriptor.ShaderRegister = 1;
    params[2].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;
    params[3].ParameterType = D3D12_ROOT_PARAMETER_TYPE_UAV;
    params[3].Descriptor.ShaderRegister = 0;
    params[3].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;
    params[4].ParameterType = D3D12_ROOT_PARAMETER_TYPE_UAV;
    params[4].Descriptor.ShaderRegister = 1;
    params[4].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;
    params[5].ParameterType = D3D12_ROOT_PARAMETER_TYPE_UAV;
    params[5].Descriptor.ShaderRegister = 2;
    params[5].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;
    s = createRootSignature(params, static_cast<UINT>(std::size(params)), D3D12_ROOT_SIGNATURE_FLAG_NONE,
                            prepRootSignature_, "prep");
    if (!s.ok) return s;
  }

  {
    D3D12_ROOT_PARAMETER params[4]{};
    params[0].ParameterType = D3D12_ROOT_PARAMETER_TYPE_32BIT_CONSTANTS;
    params[0].Constants.ShaderRegister = 0;
    params[0].Constants.RegisterSpace = 0;
    params[0].Constants.Num32BitValues = 4;
    params[0].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;
    params[1].ParameterType = D3D12_ROOT_PARAMETER_TYPE_UAV;
    params[1].Descriptor.ShaderRegister = 0;
    params[1].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;
    params[2].ParameterType = D3D12_ROOT_PARAMETER_TYPE_UAV;
    params[2].Descriptor.ShaderRegister = 1;
    params[2].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;
    params[3].ParameterType = D3D12_ROOT_PARAMETER_TYPE_UAV;
    params[3].Descriptor.ShaderRegister = 2;
    params[3].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;
    s = createRootSignature(params, static_cast<UINT>(std::size(params)), D3D12_ROOT_SIGNATURE_FLAG_NONE,
                            fillSortTailRootSignature_, "fill sort tail");
    if (!s.ok) return s;
  }

  {
    D3D12_ROOT_PARAMETER params[5]{};
    params[0].ParameterType = D3D12_ROOT_PARAMETER_TYPE_32BIT_CONSTANTS;
    params[0].Constants.ShaderRegister = 0;
    params[0].Constants.RegisterSpace = 0;
    params[0].Constants.Num32BitValues = 8;
    params[0].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;
    params[1].ParameterType = D3D12_ROOT_PARAMETER_TYPE_UAV;
    params[1].Descriptor.ShaderRegister = 0;
    params[1].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;
    params[2].ParameterType = D3D12_ROOT_PARAMETER_TYPE_UAV;
    params[2].Descriptor.ShaderRegister = 1;
    params[2].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;
    params[3].ParameterType = D3D12_ROOT_PARAMETER_TYPE_UAV;
    params[3].Descriptor.ShaderRegister = 2;
    params[3].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;
    params[4].ParameterType = D3D12_ROOT_PARAMETER_TYPE_UAV;
    params[4].Descriptor.ShaderRegister = 3;
    params[4].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;
    s = createRootSignature(params, static_cast<UINT>(std::size(params)), D3D12_ROOT_SIGNATURE_FLAG_NONE,
                            sortMetaRootSignature_, "sort meta");
    if (!s.ok) return s;
  }

  {
    D3D12_ROOT_PARAMETER params[2]{};
    params[0].ParameterType = D3D12_ROOT_PARAMETER_TYPE_UAV;
    params[0].Descriptor.ShaderRegister = 0;
    params[0].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;
    params[1].ParameterType = D3D12_ROOT_PARAMETER_TYPE_UAV;
    params[1].Descriptor.ShaderRegister = 1;
    params[1].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;
    s = createRootSignature(params, static_cast<UINT>(std::size(params)), D3D12_ROOT_SIGNATURE_FLAG_NONE,
                            buildSortDispatchArgsRootSignature_, "build sort dispatch args");
    if (!s.ok) return s;
  }

  {
    D3D12_ROOT_PARAMETER params[3]{};
    params[0].ParameterType = D3D12_ROOT_PARAMETER_TYPE_UAV;
    params[0].Descriptor.ShaderRegister = 0;
    params[0].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;
    params[1].ParameterType = D3D12_ROOT_PARAMETER_TYPE_UAV;
    params[1].Descriptor.ShaderRegister = 1;
    params[1].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;
    params[2].ParameterType = D3D12_ROOT_PARAMETER_TYPE_UAV;
    params[2].Descriptor.ShaderRegister = 2;
    params[2].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;
    s = createRootSignature(params, static_cast<UINT>(std::size(params)), D3D12_ROOT_SIGNATURE_FLAG_NONE,
                            finalizeRootSignature_, "finalize");
    if (!s.ok) return s;
  }

  {
    D3D12_ROOT_PARAMETER params[5]{};
    params[0].ParameterType = D3D12_ROOT_PARAMETER_TYPE_CBV;
    params[0].Descriptor.ShaderRegister = 0;
    params[0].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;
    params[1].ParameterType = D3D12_ROOT_PARAMETER_TYPE_SRV;
    params[1].Descriptor.ShaderRegister = 0;
    params[1].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;
    params[2].ParameterType = D3D12_ROOT_PARAMETER_TYPE_SRV;
    params[2].Descriptor.ShaderRegister = 1;
    params[2].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;
    params[3].ParameterType = D3D12_ROOT_PARAMETER_TYPE_SRV;
    params[3].Descriptor.ShaderRegister = 2;
    params[3].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;
    params[4].ParameterType = D3D12_ROOT_PARAMETER_TYPE_SRV;
    params[4].Descriptor.ShaderRegister = 3;
    params[4].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;
    s = createRootSignature(params, static_cast<UINT>(std::size(params)),
                            D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT,
                            rasterRootSignature_, "raster");
    if (!s.ok) return s;
  }

  auto createComputePso = [&](ID3D12RootSignature* rootSignature, ID3DBlob* shaderBlob,
                              Microsoft::WRL::ComPtr<ID3D12PipelineState>& out,
                              const char* label) -> Status {
    D3D12_COMPUTE_PIPELINE_STATE_DESC desc{};
    desc.pRootSignature = rootSignature;
    desc.CS = {shaderBlob->GetBufferPointer(), shaderBlob->GetBufferSize()};
    if (FAILED(device_->CreateComputePipelineState(&desc, IID_PPV_ARGS(out.GetAddressOf())))) {
      return Status::Error(std::string("failed creating ") + label + " pso");
    }
    return Status::Ok();
  };

  s = createComputePso(prepRootSignature_.Get(), prepCs.Get(), prepPso_, "prepare");
  if (!s.ok) return s;
  s = createComputePso(fillSortTailRootSignature_.Get(), fillSortTailCs.Get(), fillSortTailPso_, "fill sort tail");
  if (!s.ok) return s;
  s = createComputePso(sortMetaRootSignature_.Get(), sortMetaCs.Get(), sortMetaPso_, "sort meta");
  if (!s.ok) return s;
  s = createComputePso(buildSortDispatchArgsRootSignature_.Get(), buildSortDispatchArgsCs.Get(), buildSortDispatchArgsPso_,
                       "build sort dispatch args");
  if (!s.ok) return s;
  s = createComputePso(finalizeRootSignature_.Get(), resetCs.Get(), resetPso_, "reset");
  if (!s.ok) return s;
  s = createComputePso(finalizeRootSignature_.Get(), finalizeCs.Get(), finalizePso_, "finalize");
  if (!s.ok) return s;

  oneSweep_ = std::make_unique<directxsplat::OneSweep>();
  s = oneSweep_->Initialize(device_.Get());
  if (!s.ok) return s;

  s = EnsureColorRasterPso(DXGI_FORMAT_R8G8B8A8_UNORM);
  if (!s.ok) return s;

  {
    D3D12_INDIRECT_ARGUMENT_DESC arg{};
    arg.Type = D3D12_INDIRECT_ARGUMENT_TYPE_DRAW;
    D3D12_COMMAND_SIGNATURE_DESC desc{};
    desc.ByteStride = sizeof(D3D12_DRAW_ARGUMENTS);
    desc.NumArgumentDescs = 1;
    desc.pArgumentDescs = &arg;
    if (FAILED(device_->CreateCommandSignature(&desc, nullptr, IID_PPV_ARGS(drawCommandSignature_.GetAddressOf())))) {
      return Status::Error("failed creating draw command signature");
    }
  }


  return Status::Ok();
}

Status GaussianRasterPipeline::Render(ID3D12GraphicsCommandList* commandList,
                                      const RenderTargetBinding& target,
                                      uint64_t sceneId,
                                      UploadedSceneHandle publicSceneHandle,
                                      const RenderInput& input,
                                      FrameStats& stats,
                                      const AdvancedRenderOptions* options,
                                      RenderResult* outResult,
                                      const RenderFrameContext* frameContext) {
  if (outResult != nullptr) {
    *outResult = {};
  }
  RenderSubmissionInfo submission{};
  if (commandList == nullptr) {
    return Status::Error("invalid command list");
  }
  if (target.colorRtv.ptr == 0) {
    return Status::Error("invalid render target binding");
  }
  Status frameStatus = ValidateRenderFrameContext(frameContext, false);
  if (!frameStatus.ok) {
    return frameStatus;
  }

  std::shared_ptr<UploadedSceneRuntime> runtimePtr;
  {
    std::shared_lock<std::shared_mutex> scenesLock(uploadedScenesMutex_);
    auto it = uploadedScenes_.find(sceneId);
    if (it == uploadedScenes_.end()) {
      return Status::Error("uploaded scene handle not found");
    }
    runtimePtr = it->second;
  }
  if (runtimePtr == nullptr) {
    return Status::Error("uploaded scene handle not found");
  }
  frameStatus = ValidateRenderFrameContext(frameContext, true);
  if (!frameStatus.ok) {
    return frameStatus;
  }
  UploadedSceneRuntime& runtime = *runtimePtr;
  const uint64_t plannedPairTarget = std::max<uint64_t>(std::max<uint64_t>(runtime.sceneGaussianCount, 1u) * kSortPairMultiplier,
                                                        static_cast<uint64_t>(kMinSortPairCapacity));
  const uint32_t requiredPairCapacity = static_cast<uint32_t>(
      std::min<uint64_t>(std::max<uint64_t>(plannedPairTarget, runtime.drawCapacity), kMaxSortPairCapacity));
  std::shared_ptr<RenderScratch> scratch;
  Status scratchStatus = AcquireRenderScratch(runtime, requiredPairCapacity, frameContext, scratch);
  if (!scratchStatus.ok) {
    return scratchStatus;
  }
  if (scratch == nullptr) {
    return Status::Error("failed acquiring render scratch");
  }
  scratch->lastSortedInPrimary = true;
  bool commandsRecorded = false;

  stats.gpuPrepareMs = scratch->lastGpuPrepareMs;
  stats.gpuSortMs = scratch->lastGpuSortMs;
  stats.gpuRasterMs = scratch->lastGpuRasterMs;
  stats.gpuDepthMs = scratch->lastGpuDepthMs;
  stats.gpuMs = scratch->lastGpuMs;

  D3D12_RESOURCE_STATES currentColorState =
      target.transitionMode == ResourceTransitionMode::LibraryManaged ? target.colorStateBefore : D3D12_RESOURCE_STATE_RENDER_TARGET;
  D3D12_RESOURCE_STATES currentDepthState =
      target.transitionMode == ResourceTransitionMode::LibraryManaged ? target.depthStateBefore : D3D12_RESOURCE_STATE_DEPTH_WRITE;
  D3D12_RESOURCE_STATES currentMotionState = target.transitionMode == ResourceTransitionMode::LibraryManaged
                                                 ? target.motionVectorsStateBefore
                                                 : D3D12_RESOURCE_STATE_RENDER_TARGET;

  GpuFrameResources frameResources{};
  auto refreshFrameResources = [&]() {
    PopulateFrameResources(target, runtime, *scratch, currentColorState, currentDepthState, currentMotionState, frameResources);
  };
  refreshFrameResources();
  const bool writeDepth =
      input.settings.outputDepth && target.depthTarget != nullptr && target.depthDsv.ptr != 0 && target.depthFormat != DXGI_FORMAT_UNKNOWN;

  auto invokeStage = [&](const std::function<void(const RenderHookContext&)>& hook, RenderHookStage stage) {
    return InvokeHook(hook, stage, commandList, publicSceneHandle, input, target, frameResources, stats);
  };
  auto writeTimestamp = [&](uint32_t index) {
    if (scratch->timestampQueryHeap != nullptr) {
      commandList->EndQuery(scratch->timestampQueryHeap.Get(), D3D12_QUERY_TYPE_TIMESTAMP, index);
    }
  };
  bool managedTargetsTouched = false;
  auto transitionManagedTarget = [&](ID3D12Resource* resource,
                                     D3D12_RESOURCE_STATES& currentState,
                                     D3D12_RESOURCE_STATES desiredState) {
    if (target.transitionMode != ResourceTransitionMode::LibraryManaged || resource == nullptr) {
      return;
    }
    Transition(commandList, resource, currentState, desiredState);
    currentState = desiredState;
    managedTargetsTouched = true;
  };
  auto finalizeManagedTarget = [&](ID3D12Resource* resource,
                                   D3D12_RESOURCE_STATES& currentState,
                                   D3D12_RESOURCE_STATES finalState) {
    if (target.transitionMode != ResourceTransitionMode::LibraryManaged || resource == nullptr) {
      return;
    }
    Transition(commandList, resource, currentState, finalState);
    currentState = finalState;
  };
  auto finish = [&](Status status) {
    const bool trackSubmission = status.ok || commandsRecorded;
    if (!status.ok && commandsRecorded && managedTargetsTouched) {
      finalizeManagedTarget(target.colorTarget, currentColorState, target.colorStateAfter);
      finalizeManagedTarget(target.depthTarget, currentDepthState, target.depthStateAfter);
      finalizeManagedTarget(target.motionVectorsTarget, currentMotionState, target.motionVectorsStateAfter);
    }
    submission.submissionRequired = trackSubmission;
    if (outResult != nullptr) {
      outResult->submission = submission;
    }
    if (trackSubmission) {
      RecordDirectQueueSubmission(frameContext);
    }
    ReleaseRenderScratch(runtime, std::move(scratch), frameContext, trackSubmission);
    return status;
  };

  if (runtime.sceneGaussianCount > 0) {
    try {
      Status colorStatus = EnsureColorRasterPso(target.colorFormat);
      if (!colorStatus.ok) {
        return finish(colorStatus);
      }
      if (writeDepth) {
        Status depthStatus = EnsureDepthRasterPso(target.depthFormat);
        if (!depthStatus.ok) {
          return finish(depthStatus);
        }
      }
    } catch (const std::bad_alloc&) {
      return finish(Status::Error("failed creating raster pso"));
    } catch (const std::length_error&) {
      return finish(Status::Error("failed creating raster pso"));
    } catch (const std::exception&) {
      return finish(Status::Error("failed creating raster pso"));
    } catch (...) {
      return finish(Status::Error("failed creating raster pso"));
    }
  }

  uint64_t pendingUploadFenceValue = 0;
  uint64_t directQueueUploadWaitValue = 0;
  {
    std::lock_guard<std::mutex> runtimeLock(*runtime.mutex);
    pendingUploadFenceValue = runtime.pendingUploadFenceValue;
    directQueueUploadWaitValue = runtime.directQueueUploadWaitValue;
  }
  if (pendingUploadFenceValue > directQueueUploadWaitValue && uploadFence_ != nullptr) {
    std::lock_guard<std::recursive_mutex> uploadLock(uploadMutex_);
    if (uploadFence_->GetCompletedValue() < pendingUploadFenceValue) {
      submission.uploadSyncPoint.fence = uploadFence_.Get();
      submission.uploadSyncPoint.value = pendingUploadFenceValue;
    } else {
      std::lock_guard<std::mutex> runtimeLock(*runtime.mutex);
      runtime.directQueueUploadWaitValue = std::max(runtime.directQueueUploadWaitValue, pendingUploadFenceValue);
    }
  }

  commandsRecorded = true;
  if (options != nullptr && options->hooks != nullptr) {
    Status hookStatus = invokeStage(options->hooks->beforePrepare, RenderHookStage::BeforePrepare);
    if (!hookStatus.ok) {
      return finish(hookStatus);
    }
  }
  writeTimestamp(kTimestampFrameBegin);
  writeTimestamp(kTimestampPrepareBegin);

  if (runtime.sceneGaussianCount == 0) {
    scratch->lastGpuPrepareMs = 0.0f;
    scratch->lastGpuSortMs = 0.0f;
    scratch->lastGpuRasterMs = 0.0f;
    scratch->lastGpuDepthMs = 0.0f;
    scratch->lastGpuMs = 0.0f;
    stats.gpuPrepareMs = 0.0f;
    stats.gpuSortMs = 0.0f;
    stats.gpuRasterMs = 0.0f;
    stats.gpuDepthMs = 0.0f;
    stats.gpuMs = 0.0f;

    transitionManagedTarget(target.colorTarget, currentColorState, D3D12_RESOURCE_STATE_RENDER_TARGET);
    if (target.clearColor) {
      commandList->OMSetRenderTargets(1, &target.colorRtv, FALSE, nullptr);
      commandList->ClearRenderTargetView(target.colorRtv, target.clearColorValue, 0, nullptr);
    }
    if (input.settings.outputDepth && target.depthTarget != nullptr && target.depthDsv.ptr != 0) {
      transitionManagedTarget(target.depthTarget, currentDepthState, D3D12_RESOURCE_STATE_DEPTH_WRITE);
      if (target.clearDepth) {
        commandList->ClearDepthStencilView(target.depthDsv, D3D12_CLEAR_FLAG_DEPTH, target.clearDepthValue, target.clearStencilValue, 0,
                                           nullptr);
      }
      frameResources.depthOutput = DepthOutputKind::ApproximateSplatDepth;
    }
    if (target.motionVectorsTarget != nullptr && target.motionVectorsRtv.ptr != 0 && target.clearMotionVectors) {
      transitionManagedTarget(target.motionVectorsTarget, currentMotionState, D3D12_RESOURCE_STATE_RENDER_TARGET);
      commandList->ClearRenderTargetView(target.motionVectorsRtv, target.clearMotionVectorsValue, 0, nullptr);
    }

    finalizeManagedTarget(target.colorTarget, currentColorState, target.colorStateAfter);
    finalizeManagedTarget(target.depthTarget, currentDepthState, target.depthStateAfter);
    finalizeManagedTarget(target.motionVectorsTarget, currentMotionState, target.motionVectorsStateAfter);
    refreshFrameResources();
    if (input.settings.outputDepth && target.depthTarget != nullptr && target.depthDsv.ptr != 0) {
      frameResources.depthOutput = DepthOutputKind::ApproximateSplatDepth;
    }
    if (options != nullptr && options->hooks != nullptr) {
      Status hookStatus = invokeStage(options->hooks->afterPrepare, RenderHookStage::AfterPrepare);
      if (!hookStatus.ok) {
        return finish(hookStatus);
      }
      hookStatus = invokeStage(options->hooks->afterSort, RenderHookStage::AfterSort);
      if (!hookStatus.ok) {
        return finish(hookStatus);
      }
      hookStatus = invokeStage(options->hooks->beforeRaster, RenderHookStage::BeforeRaster);
      if (!hookStatus.ok) {
        return finish(hookStatus);
      }
      hookStatus = invokeStage(options->hooks->afterRaster, RenderHookStage::AfterRaster);
      if (!hookStatus.ok) {
        return finish(hookStatus);
      }
      hookStatus = invokeStage(options->hooks->beforePostProcess, RenderHookStage::BeforePostProcess);
      if (!hookStatus.ok) {
        return finish(hookStatus);
      }
      hookStatus = invokeStage(options->hooks->afterPostProcess, RenderHookStage::AfterPostProcess);
      if (!hookStatus.ok) {
        return finish(hookStatus);
      }
    }
    if (outResult != nullptr) {
      outResult->stats = stats;
      outResult->resources = frameResources;
      outResult->upscalerInput.color = frameResources.colorTarget;
      outResult->upscalerInput.depth = frameResources.depthTarget;
      outResult->upscalerInput.motionVectors = frameResources.motionVectorsTarget;
      outResult->upscalerInput.renderWidth = std::max<uint32_t>(input.viewportWidth, 1u);
      outResult->upscalerInput.renderHeight = std::max<uint32_t>(input.viewportHeight, 1u);
      outResult->upscalerInput.outputWidth = std::max<uint32_t>(frameResources.colorTarget.width, outResult->upscalerInput.renderWidth);
      outResult->upscalerInput.outputHeight = std::max<uint32_t>(frameResources.colorTarget.height, outResult->upscalerInput.renderHeight);
      outResult->upscalerInput.view = input.view;
      outResult->upscalerInput.proj = input.proj;
      outResult->upscalerInput.jitter = input.jitter;
      outResult->upscalerInput.cameraCut = input.cameraCut;
      outResult->hasUpscalerInput = frameResources.colorTarget.resource != nullptr;
    }
    return finish(Status::Ok());
  }

  ++scratch->sortStatsFrame;
  ++scratch->gpuTimingFrame;
  const bool sortMetaReady = scratch->sortMetaCopyFence != nullptr && scratch->sortMetaCopyFenceValue != 0 &&
                             scratch->sortMetaCopyFence->GetCompletedValue() >= scratch->sortMetaCopyFenceValue;
  if (scratch->sortMetaReadback != nullptr && scratch->sortMetaCopyPending && sortMetaReady) {
    void* mapped = nullptr;
    if (SUCCEEDED(scratch->sortMetaReadback->Map(0, nullptr, &mapped)) && mapped != nullptr) {
      const SortMetaGpu* meta = reinterpret_cast<const SortMetaGpu*>(mapped);
      scratch->lastPairCount = meta->pairCount;
      scratch->lastVisibleCount = meta->visibleCount;
      scratch->lastVisibleBlocks = meta->visibleBlocks;
      scratch->lastSortPassCount = meta->sortPassCount;
      scratch->sortMetaReadback->Unmap(0, nullptr);
    }
    mapped = nullptr;
    if (scratch->projectionActiveThreadsReadback != nullptr &&
        SUCCEEDED(scratch->projectionActiveThreadsReadback->Map(0, nullptr, &mapped)) && mapped != nullptr) {
      const uint32_t* bins = reinterpret_cast<const uint32_t*>(mapped);
      std::copy_n(bins, scratch->lastSplatAlphaBins.size(),
                  scratch->lastSplatAlphaBins.begin());
      std::copy_n(bins + scratch->lastSplatAlphaBins.size(), scratch->lastProjectionActiveThreadBins.size(),
                  scratch->lastProjectionActiveThreadBins.begin());
      scratch->projectionActiveThreadsReadback->Unmap(0, nullptr);
    }
    scratch->sortMetaCopyFence.Reset();
    scratch->sortMetaCopyFenceValue = 0;
    scratch->sortMetaCopyPending = false;
  }
  const bool timestampReady = scratch->timestampCopyFence != nullptr && scratch->timestampCopyFenceValue != 0 &&
                              scratch->timestampCopyFence->GetCompletedValue() >= scratch->timestampCopyFenceValue;
  if (scratch->timestampReadbackBuffer != nullptr && scratch->timestampCopyPending && timestampReady) {
    void* mapped = nullptr;
    if (SUCCEEDED(scratch->timestampReadbackBuffer->Map(0, nullptr, &mapped)) && mapped != nullptr) {
      const uint64_t* timestamps = reinterpret_cast<const uint64_t*>(mapped);
      auto durationMs = [&](uint32_t beginIndex, uint32_t endIndex) {
        const uint64_t beginValue = timestamps[beginIndex];
        const uint64_t endValue = timestamps[endIndex];
        if (endValue <= beginValue || gpuTimestampMsPerTick_ <= 0.0) {
          return 0.0f;
        }
        return static_cast<float>(static_cast<double>(endValue - beginValue) * gpuTimestampMsPerTick_);
      };
      scratch->lastGpuPrepareMs = durationMs(kTimestampPrepareBegin, kTimestampPrepareEnd);
      scratch->lastGpuSortMs = durationMs(kTimestampSortBegin, kTimestampSortEnd);
      scratch->lastGpuRasterMs = durationMs(kTimestampRasterBegin, kTimestampRasterEnd);
      scratch->lastGpuDepthMs = durationMs(kTimestampDepthBegin, kTimestampDepthEnd);
      scratch->lastGpuMs = durationMs(kTimestampFrameBegin, kTimestampFrameEnd);
      stats.gpuPrepareMs = scratch->lastGpuPrepareMs;
      stats.gpuSortMs = scratch->lastGpuSortMs;
      stats.gpuRasterMs = scratch->lastGpuRasterMs;
      stats.gpuDepthMs = scratch->lastGpuDepthMs;
      stats.gpuMs = scratch->lastGpuMs;
      scratch->timestampReadbackBuffer->Unmap(0, nullptr);
    }
    scratch->timestampCopyFence.Reset();
    scratch->timestampCopyFenceValue = 0;
    scratch->timestampCopyPending = false;
  }

  PrepConstants prepBase{};
  std::memcpy(prepBase.view, input.view.m.data(), sizeof(prepBase.view));
  std::memcpy(prepBase.proj, input.proj.m.data(), sizeof(prepBase.proj));
  prepBase.cameraPos[0] = input.cameraPosition.x;
  prepBase.cameraPos[1] = input.cameraPosition.y;
  prepBase.cameraPos[2] = input.cameraPosition.z;
  prepBase.globalScale = std::max(0.01f, input.settings.gaussianScalingModifier);
  prepBase.focalX = std::abs(input.proj.m[0]) * static_cast<float>(input.viewportWidth) * 0.5f;
  prepBase.focalY = std::abs(input.proj.m[5]) * static_cast<float>(input.viewportHeight) * 0.5f;
  prepBase.ndcX = 2.0f / std::max(1.0f, static_cast<float>(input.viewportWidth));
  prepBase.ndcY = 2.0f / std::max(1.0f, static_cast<float>(input.viewportHeight));
  prepBase.maxAxisPixels = std::max(1.0f, input.settings.maxAxisPixels);
  prepBase.nearPlane = std::max(0.0001f, input.nearPlane);
  prepBase.fastCulling = input.settings.fastCulling ? 1u : 0u;
  const RenderType renderType = SanitizeRenderType(input.settings.renderType);
  prepBase.renderType = static_cast<uint32_t>(renderType);
  prepBase.antialiasingMode = input.settings.antialiasing ? 1u : 0u;
  const ShadingDegree shadingDegree =
      renderType == RenderType::Color ? SanitizeShadingDegree(input.settings.shadingDegree) : ShadingDegree::Dc;
  prepBase.shadingDegree = static_cast<uint32_t>(shadingDegree);
  prepBase.positiveViewSpaceZ = input.settings.positiveViewSpaceZ ? 1u : 0u;
  prepBase.antialiasingStrength = std::max(0.0f, input.settings.antialiasingStrength);
  prepBase.gammaCorrection = renderType == RenderType::Color && input.settings.gammaCorrection ? 1u : 0u;
  prepBase.drawCapacity = runtime.drawCapacity;
  prepBase.pairCapacity = scratch->sortPairCapacity;
  prepBase.viewportWidth = std::max<uint32_t>(input.viewportWidth, 1u);
  prepBase.viewportHeight = std::max<uint32_t>(input.viewportHeight, 1u);
  prepBase.backgroundColor[0] = input.settings.backgroundColor.x;
  prepBase.backgroundColor[1] = input.settings.backgroundColor.y;
  prepBase.backgroundColor[2] = input.settings.backgroundColor.z;
  prepBase.farPlane = std::max(input.farPlane, input.nearPlane + 0.001f);
  prepBase.frustumDilation = std::clamp(input.settings.frustumDilation, 0.0f, 1.0f);
  prepBase.sceneGaussianStride = runtime.sceneGaussianStride;
  prepBase.rgbaFormat = static_cast<uint32_t>(runtime.vramFormat.rgbaFormat);
  prepBase.shFormat = static_cast<uint32_t>(runtime.vramFormat.shFormat);
  prepBase.rgbaOffset = runtime.rgbaOffset;
  prepBase.shOffset = runtime.shOffset;
  prepBase.idOffset = runtime.idOffset;
  prepBase.sceneCount = runtime.sceneAtlasTail;
  prepBase.paddedCount = runtime.maxPrepareGroups;
  prepBase.setCount = runtime.batchedChunkCount;

  if (scratch->prepConstantsMapped == nullptr || scratch->prepConstantsUpload == nullptr || runtime.sceneAtlasBuffer == nullptr ||
      runtime.batchedChunkParamsUpload == nullptr || runtime.sceneIndexToChunkBuffer == nullptr) {
    return finish(Status::Error("uploaded scene prep resources are incomplete"));
  }

  Transition(commandList, scratch->sortKeysBuffer.Get(), scratch->sortKeysState, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
  scratch->sortKeysState = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
  Transition(commandList, scratch->sortKeysTempBuffer.Get(), scratch->sortKeysTempState, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
  scratch->sortKeysTempState = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
  Transition(commandList, scratch->sortValuesBuffer.Get(), scratch->sortValuesState, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
  scratch->sortValuesState = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
  Transition(commandList, scratch->sortValuesTempBuffer.Get(), scratch->sortValuesTempState, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
  scratch->sortValuesTempState = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
  Transition(commandList, scratch->visibleCounterBuffer.Get(), scratch->visibleCounterState, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
  scratch->visibleCounterState = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
  Transition(commandList, scratch->sortMetaBuffer.Get(), scratch->sortMetaState, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
  scratch->sortMetaState = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
  Transition(commandList, scratch->oneSweepPassHistogramBuffer.Get(), scratch->oneSweepPassHistogramState,
             D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
  scratch->oneSweepPassHistogramState = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
  Transition(commandList, scratch->oneSweepGlobalHistogramBuffer.Get(), scratch->oneSweepGlobalHistogramState,
             D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
  scratch->oneSweepGlobalHistogramState = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
  Transition(commandList, scratch->oneSweepIndexBuffer.Get(), scratch->oneSweepIndexState, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
  scratch->oneSweepIndexState = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
  Transition(commandList, scratch->oneSweepDispatchArgsBuffer.Get(), scratch->oneSweepDispatchArgsState, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
  scratch->oneSweepDispatchArgsState = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
  Transition(commandList, scratch->drawArgsBuffer.Get(), scratch->drawArgsState, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
  scratch->drawArgsState = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
  refreshFrameResources();

  commandList->SetComputeRootSignature(finalizeRootSignature_.Get());
  commandList->SetPipelineState(resetPso_.Get());
  commandList->SetComputeRootUnorderedAccessView(0, scratch->visibleCounterBuffer->GetGPUVirtualAddress());
  commandList->SetComputeRootUnorderedAccessView(1, scratch->drawArgsBuffer->GetGPUVirtualAddress());
  commandList->Dispatch(1, 1, 1);

  D3D12_RESOURCE_BARRIER barrier{};
  barrier.Type = D3D12_RESOURCE_BARRIER_TYPE_UAV;
  barrier.UAV.pResource = scratch->visibleCounterBuffer.Get();
  commandList->ResourceBarrier(1, &barrier);

  commandList->SetComputeRootSignature(prepRootSignature_.Get());
  commandList->SetPipelineState(prepPso_.Get());
  commandList->SetComputeRootUnorderedAccessView(3, scratch->sortKeysBuffer->GetGPUVirtualAddress());
  commandList->SetComputeRootUnorderedAccessView(4, scratch->sortValuesBuffer->GetGPUVirtualAddress());
  commandList->SetComputeRootUnorderedAccessView(5, scratch->visibleCounterBuffer->GetGPUVirtualAddress());
  std::memcpy(scratch->prepConstantsMapped, &prepBase, sizeof(prepBase));
  commandList->SetComputeRootConstantBufferView(0, scratch->prepConstantsUpload->GetGPUVirtualAddress());
  commandList->SetComputeRootShaderResourceView(1, runtime.sceneAtlasBuffer->GetGPUVirtualAddress());
  commandList->SetComputeRootShaderResourceView(2, runtime.batchedChunkParamsUpload->GetGPUVirtualAddress());
  if (runtime.batchedChunkCount > 0 && runtime.maxPrepareGroups > 0) {
    commandList->Dispatch(runtime.maxPrepareGroups, runtime.batchedChunkCount, 1);
  }

  barrier.UAV.pResource = scratch->visibleCounterBuffer.Get();
  commandList->ResourceBarrier(1, &barrier);
  barrier.UAV.pResource = scratch->sortKeysBuffer.Get();
  commandList->ResourceBarrier(1, &barrier);
  barrier.UAV.pResource = scratch->sortValuesBuffer.Get();
  commandList->ResourceBarrier(1, &barrier);

  struct SortMetaConstantsCpu {
    uint32_t sortCapacity;
    uint32_t sortGroupSize;
    uint32_t sortPassCount;
    uint32_t oneSweepPartitionSize;
    uint32_t oneSweepGlobalHistPartitionSize;
    uint32_t indirectCommandStride;
    uint32_t packGroupSize;
    uint32_t sortPad;
  } sortMetaConstants{scratch->sortPairCapacity, 512u, kOneSweepPassCount, oneSweep_->PartitionSize(),
                      32768u, kOneSweepIndirectCommandStride, 256u, 0u};
  commandList->SetComputeRootSignature(sortMetaRootSignature_.Get());
  commandList->SetPipelineState(sortMetaPso_.Get());
  commandList->SetComputeRoot32BitConstants(0, 8, &sortMetaConstants, 0);
  commandList->SetComputeRootUnorderedAccessView(1, scratch->visibleCounterBuffer->GetGPUVirtualAddress());
  commandList->SetComputeRootUnorderedAccessView(2, scratch->sortMetaBuffer->GetGPUVirtualAddress());
  commandList->SetComputeRootUnorderedAccessView(3, scratch->sortKeysBuffer->GetGPUVirtualAddress());
  commandList->SetComputeRootUnorderedAccessView(4, scratch->sortValuesBuffer->GetGPUVirtualAddress());
  commandList->Dispatch(1, 1, 1);
  barrier.UAV.pResource = scratch->sortMetaBuffer.Get();
  commandList->ResourceBarrier(1, &barrier);

  commandList->SetComputeRootSignature(buildSortDispatchArgsRootSignature_.Get());
  commandList->SetPipelineState(buildSortDispatchArgsPso_.Get());
  commandList->SetComputeRootUnorderedAccessView(0, scratch->sortMetaBuffer->GetGPUVirtualAddress());
  commandList->SetComputeRootUnorderedAccessView(1, scratch->oneSweepDispatchArgsBuffer->GetGPUVirtualAddress());
  commandList->Dispatch(1, 1, 1);
  barrier.UAV.pResource = scratch->oneSweepDispatchArgsBuffer.Get();
  commandList->ResourceBarrier(1, &barrier);
  Transition(commandList, scratch->oneSweepDispatchArgsBuffer.Get(), scratch->oneSweepDispatchArgsState,
             D3D12_RESOURCE_STATE_INDIRECT_ARGUMENT);
  scratch->oneSweepDispatchArgsState = D3D12_RESOURCE_STATE_INDIRECT_ARGUMENT;

  writeTimestamp(kTimestampPrepareEnd);
  refreshFrameResources();
  if (options != nullptr && options->hooks != nullptr) {
    Status hookStatus = invokeStage(options->hooks->afterPrepare, RenderHookStage::AfterPrepare);
    if (!hookStatus.ok) {
      return finish(hookStatus);
    }
  }

  bool sortedInPrimary = true;
  uint32_t sortPassCount = 0;
  SortBackend activeSortBackend = SortBackend::OneSweep;
  uint32_t emittedPairs = scratch->lastPairCount;
  bool drawArgsFinalized = false;
  bool sortMetaValid = true;
  writeTimestamp(kTimestampSortBegin);

  if (options != nullptr && options->sortBackend != nullptr) {
    SortDispatchContext sortContext{};
    sortContext.commandList = commandList;
    sortContext.scene = publicSceneHandle;
    sortContext.input = &input;
    sortContext.primaryKeys = MakeBufferView(scratch->sortKeysBuffer.Get(),
                                             scratch->sortKeysState,
                                             static_cast<uint64_t>(scratch->sortPairCapacity) * sizeof(uint32_t),
                                             sizeof(uint32_t),
                                             GpuViewLifetime::CurrentRenderCall,
                                             GpuResourceAccess::ReadWrite,
                                             true,
                                             false);
    sortContext.secondaryKeys = MakeBufferView(scratch->sortKeysTempBuffer.Get(),
                                               scratch->sortKeysTempState,
                                               static_cast<uint64_t>(scratch->sortPairCapacity) * sizeof(uint32_t),
                                               sizeof(uint32_t),
                                               GpuViewLifetime::CurrentRenderCall,
                                               GpuResourceAccess::ReadWrite,
                                               true,
                                               false);
    sortContext.primaryValues = MakeBufferView(scratch->sortValuesBuffer.Get(),
                                               scratch->sortValuesState,
                                               static_cast<uint64_t>(scratch->sortPairCapacity) * sizeof(uint32_t),
                                               sizeof(uint32_t),
                                               GpuViewLifetime::CurrentRenderCall,
                                               GpuResourceAccess::ReadWrite,
                                               true,
                                               false);
    sortContext.secondaryValues = MakeBufferView(scratch->sortValuesTempBuffer.Get(),
                                                 scratch->sortValuesTempState,
                                                 static_cast<uint64_t>(scratch->sortPairCapacity) * sizeof(uint32_t),
                                                 sizeof(uint32_t),
                                                 GpuViewLifetime::CurrentRenderCall,
                                                 GpuResourceAccess::ReadWrite,
                                                 true,
                                                 false);
    sortContext.visibleCounter = MakeBufferView(scratch->visibleCounterBuffer.Get(),
                                                scratch->visibleCounterState,
                                                16u,
                                                4u,
                                                GpuViewLifetime::CurrentRenderCall,
                                                GpuResourceAccess::ReadWrite,
                                                true,
                                                false);
    sortContext.sortMeta = MakeBufferView(scratch->sortMetaBuffer.Get(),
                                          scratch->sortMetaState,
                                          sizeof(SortMetaGpu),
                                          sizeof(uint32_t),
                                          GpuViewLifetime::CurrentRenderCall,
                                          GpuResourceAccess::ReadWrite,
                                          true,
                                          false);
    sortContext.drawArgs = MakeBufferView(scratch->drawArgsBuffer.Get(),
                                          scratch->drawArgsState,
                                          16u,
                                          sizeof(uint32_t),
                                          GpuViewLifetime::CurrentRenderCall,
                                          GpuResourceAccess::ReadWrite,
                                          true,
                                          false);
    sortContext.sortEntryCapacity = scratch->sortPairCapacity;
    sortContext.maxVisibleBlocks = scratch->oneSweepPartitionCount;
    sortContext.radixBits = 8u;

    SortDispatchResult customSort{};
    Status sortStatus = Status::Ok();
    try {
      sortStatus = options->sortBackend->Sort(sortContext, customSort);
    } catch (const std::exception&) {
      sortStatus = Status::Error("custom sort backend failed");
    } catch (...) {
      sortStatus = Status::Error("custom sort backend failed");
    }
    if (!sortStatus.ok) {
      return finish(sortStatus);
    }
    if (customSort.outputFormat != SortOutputFormat::SortedValuesU32) {
      return finish(Status::Error("custom sort backend must output sorted uint32 scene indices"));
    }
    sortedInPrimary = customSort.sortedBuffer == SortBufferSelection::Primary;
    sortPassCount = customSort.passCount;
    activeSortBackend = customSort.backend;
    if (customSort.emittedSortEntryCount > 0) {
      emittedPairs = customSort.emittedSortEntryCount;
    }
    scratch->sortKeysState = customSort.primaryKeysState;
    scratch->sortKeysTempState = customSort.secondaryKeysState;
    scratch->sortValuesState = customSort.primaryValuesState;
    scratch->sortValuesTempState = customSort.secondaryValuesState;
    scratch->visibleCounterState = customSort.visibleCounterState;
    scratch->sortMetaState = customSort.sortMetaState;
    scratch->drawArgsState = customSort.drawArgsState;
    drawArgsFinalized = customSort.drawArgsFinalized;
    sortMetaValid = customSort.sortMetaValid;
  } else {
    OneSweepResult sortResult{};
    Status sortStatus = oneSweep_->DispatchIndirect({.commandList = commandList,
                                                     .keyPrimaryResource = scratch->sortKeysBuffer.Get(),
                                                     .keyPrimaryUav = scratch->sortKeysBuffer->GetGPUVirtualAddress(),
                                                     .keyTempResource = scratch->sortKeysTempBuffer.Get(),
                                                     .keyTempUav = scratch->sortKeysTempBuffer->GetGPUVirtualAddress(),
                                                     .valuePrimaryResource = scratch->sortValuesBuffer.Get(),
                                                     .valuePrimaryUav = scratch->sortValuesBuffer->GetGPUVirtualAddress(),
                                                     .valueTempResource = scratch->sortValuesTempBuffer.Get(),
                                                     .valueTempUav = scratch->sortValuesTempBuffer->GetGPUVirtualAddress(),
                                                     .passHistogramResource = scratch->oneSweepPassHistogramBuffer.Get(),
                                                     .passHistogramUav = scratch->oneSweepPassHistogramBuffer->GetGPUVirtualAddress(),
                                                     .globalHistogramResource = scratch->oneSweepGlobalHistogramBuffer.Get(),
                                                     .globalHistogramUav = scratch->oneSweepGlobalHistogramBuffer->GetGPUVirtualAddress(),
                                                     .indexResource = scratch->oneSweepIndexBuffer.Get(),
                                                     .indexUav = scratch->oneSweepIndexBuffer->GetGPUVirtualAddress(),
                                                     .argumentBufferResource = scratch->oneSweepDispatchArgsBuffer.Get(),
                                                     .initArgsOffset = static_cast<uint64_t>(kOneSweepInitCommandIndex) * kOneSweepIndirectCommandStride,
                                                     .histogramArgsOffset = static_cast<uint64_t>(kOneSweepGlobalHistogramCommandIndex) *
                                                                            kOneSweepIndirectCommandStride,
                                                     .scanArgsOffset = static_cast<uint64_t>(kOneSweepScanCommandIndex) * kOneSweepIndirectCommandStride,
                                                     .digitArgsOffset = static_cast<uint64_t>(kOneSweepDigitCommandIndex) * kOneSweepIndirectCommandStride,
                                                     .digitPassCount = kOneSweepPassCount},
                                                    sortResult);
    if (!sortStatus.ok) {
      return finish(sortStatus);
    }

    sortedInPrimary = sortResult.sortedInPrimary;
    sortPassCount = sortResult.passCount;
    activeSortBackend = SortBackend::OneSweep;
  }

  scratch->lastSortedInPrimary = sortedInPrimary;
  writeTimestamp(kTimestampSortEnd);
  frameResources.sortBackend = activeSortBackend;
  frameResources.emittedSortEntryCount = emittedPairs;
  refreshFrameResources();
  if (options != nullptr && options->hooks != nullptr) {
    Status hookStatus = invokeStage(options->hooks->afterSort, RenderHookStage::AfterSort);
    if (!hookStatus.ok) {
      return finish(hookStatus);
    }
  }

  ID3D12Resource* sortedValuesResource = sortedInPrimary ? scratch->sortValuesBuffer.Get() : scratch->sortValuesTempBuffer.Get();
  D3D12_GPU_VIRTUAL_ADDRESS sortedValuesAddress =
      sortedInPrimary ? scratch->sortValuesBuffer->GetGPUVirtualAddress() : scratch->sortValuesTempBuffer->GetGPUVirtualAddress();
  D3D12_RESOURCE_STATES& sortedValuesState = sortedInPrimary ? scratch->sortValuesState : scratch->sortValuesTempState;
  Microsoft::WRL::ComPtr<ID3D12PipelineState> colorRasterPso;
  {
    std::lock_guard<std::mutex> colorLock(colorRasterMutex_);
    auto colorIt = colorRasterPsos_.find(ColorPsoKey(target.colorFormat));
    if (colorIt == colorRasterPsos_.end() || colorIt->second == nullptr) {
      return finish(Status::Error("failed creating color raster pso"));
    }
    colorRasterPso = colorIt->second;
  }
  Microsoft::WRL::ComPtr<ID3D12PipelineState> depthRasterPso;
  if (writeDepth) {
    std::lock_guard<std::mutex> depthLock(depthRasterMutex_);
    auto depthIt = depthRasterPsos_.find(static_cast<int>(target.depthFormat));
    if (depthIt == depthRasterPsos_.end() || depthIt->second == nullptr) {
      return finish(Status::Error("failed creating depth raster pso"));
    }
    depthRasterPso = depthIt->second;
  }

  if (!drawArgsFinalized) {
    Transition(commandList, scratch->visibleCounterBuffer.Get(), scratch->visibleCounterState, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
    scratch->visibleCounterState = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
    Transition(commandList, scratch->drawArgsBuffer.Get(), scratch->drawArgsState, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
    scratch->drawArgsState = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
    Transition(commandList, scratch->sortMetaBuffer.Get(), scratch->sortMetaState, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
    scratch->sortMetaState = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
    barrier.UAV.pResource = scratch->visibleCounterBuffer.Get();
    commandList->ResourceBarrier(1, &barrier);
    commandList->SetComputeRootSignature(finalizeRootSignature_.Get());
    commandList->SetPipelineState(finalizePso_.Get());
    commandList->SetComputeRootUnorderedAccessView(0, scratch->visibleCounterBuffer->GetGPUVirtualAddress());
    commandList->SetComputeRootUnorderedAccessView(1, scratch->drawArgsBuffer->GetGPUVirtualAddress());
    commandList->SetComputeRootUnorderedAccessView(2, scratch->sortMetaBuffer->GetGPUVirtualAddress());
    commandList->Dispatch(1, 1, 1);
    barrier.UAV.pResource = scratch->drawArgsBuffer.Get();
    commandList->ResourceBarrier(1, &barrier);
    sortMetaValid = true;
  }

  if (sortMetaValid && scratch->sortMetaReadback != nullptr && scratch->projectionActiveThreadsReadback != nullptr &&
      !scratch->sortMetaCopyPending &&
      scratch->sortStatsFrame % kSortStatsReadbackPeriod == 0) {
    Transition(commandList, scratch->sortMetaBuffer.Get(), scratch->sortMetaState, D3D12_RESOURCE_STATE_COPY_SOURCE);
    scratch->sortMetaState = D3D12_RESOURCE_STATE_COPY_SOURCE;
    commandList->CopyBufferRegion(scratch->sortMetaReadback.Get(), 0, scratch->sortMetaBuffer.Get(), 0, sizeof(SortMetaGpu));
    Transition(commandList, scratch->visibleCounterBuffer.Get(), scratch->visibleCounterState, D3D12_RESOURCE_STATE_COPY_SOURCE);
    scratch->visibleCounterState = D3D12_RESOURCE_STATE_COPY_SOURCE;
    commandList->CopyBufferRegion(scratch->projectionActiveThreadsReadback.Get(),
                                  0,
                                  scratch->visibleCounterBuffer.Get(),
                                  kSplatAlphaHistogramOffset,
                                  kStatsHistogramBytes);
    scratch->sortMetaCopyFrame = scratch->sortStatsFrame;
    scratch->sortMetaCopyFence = frameContext->fence;
    scratch->sortMetaCopyFenceValue = frameContext->submissionFenceValue;
    scratch->sortMetaCopyPending = true;
  }

  Transition(commandList, sortedValuesResource, sortedValuesState, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
  sortedValuesState = D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE;
  Transition(commandList, scratch->drawArgsBuffer.Get(), scratch->drawArgsState, D3D12_RESOURCE_STATE_INDIRECT_ARGUMENT);
  scratch->drawArgsState = D3D12_RESOURCE_STATE_INDIRECT_ARGUMENT;
  refreshFrameResources();

  transitionManagedTarget(target.colorTarget, currentColorState, D3D12_RESOURCE_STATE_RENDER_TARGET);
  if (target.motionVectorsTarget != nullptr && target.motionVectorsRtv.ptr != 0 && target.clearMotionVectors) {
    transitionManagedTarget(target.motionVectorsTarget, currentMotionState, D3D12_RESOURCE_STATE_RENDER_TARGET);
    commandList->ClearRenderTargetView(target.motionVectorsRtv, target.clearMotionVectorsValue, 0, nullptr);
  }
  commandList->RSSetViewports(1, &target.viewport);
  commandList->RSSetScissorRects(1, &target.scissor);
  commandList->OMSetRenderTargets(1, &target.colorRtv, FALSE, nullptr);
  if (target.clearColor) {
    commandList->ClearRenderTargetView(target.colorRtv, target.clearColorValue, 0, nullptr);
  }
  refreshFrameResources();
  if (options != nullptr && options->hooks != nullptr) {
    Status hookStatus = invokeStage(options->hooks->beforeRaster, RenderHookStage::BeforeRaster);
    if (!hookStatus.ok) {
      return finish(hookStatus);
    }
  }
  writeTimestamp(kTimestampRasterBegin);

  std::memcpy(scratch->rasterConstantsMapped, &prepBase, sizeof(prepBase));

  commandList->SetPipelineState(colorRasterPso.Get());
  commandList->SetGraphicsRootSignature(rasterRootSignature_.Get());
  commandList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLESTRIP);
  commandList->SetGraphicsRootConstantBufferView(0, scratch->rasterConstantsUpload->GetGPUVirtualAddress());
  commandList->SetGraphicsRootShaderResourceView(1, runtime.sceneAtlasBuffer->GetGPUVirtualAddress());
  commandList->SetGraphicsRootShaderResourceView(2, sortedValuesAddress);
  commandList->SetGraphicsRootShaderResourceView(3, runtime.batchedChunkParamsUpload->GetGPUVirtualAddress());
  commandList->SetGraphicsRootShaderResourceView(4, runtime.sceneIndexToChunkBuffer->GetGPUVirtualAddress());
  commandList->ExecuteIndirect(drawCommandSignature_.Get(), 1, scratch->drawArgsBuffer.Get(), 0, nullptr, 0);
  writeTimestamp(kTimestampRasterEnd);

  if (writeDepth) {
    transitionManagedTarget(target.depthTarget, currentDepthState, D3D12_RESOURCE_STATE_DEPTH_WRITE);
    if (target.clearDepth) {
      commandList->ClearDepthStencilView(target.depthDsv, D3D12_CLEAR_FLAG_DEPTH, target.clearDepthValue, target.clearStencilValue, 0,
                                         nullptr);
    }
    writeTimestamp(kTimestampDepthBegin);
    commandList->OMSetRenderTargets(0, nullptr, FALSE, &target.depthDsv);
    commandList->SetPipelineState(depthRasterPso.Get());
    commandList->SetGraphicsRootSignature(rasterRootSignature_.Get());
    commandList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLESTRIP);
    commandList->SetGraphicsRootConstantBufferView(0, scratch->rasterConstantsUpload->GetGPUVirtualAddress());
    commandList->SetGraphicsRootShaderResourceView(1, runtime.sceneAtlasBuffer->GetGPUVirtualAddress());
    commandList->SetGraphicsRootShaderResourceView(2, sortedValuesAddress);
    commandList->SetGraphicsRootShaderResourceView(3, runtime.batchedChunkParamsUpload->GetGPUVirtualAddress());
    commandList->SetGraphicsRootShaderResourceView(4, runtime.sceneIndexToChunkBuffer->GetGPUVirtualAddress());
    commandList->ExecuteIndirect(drawCommandSignature_.Get(), 1, scratch->drawArgsBuffer.Get(), 0, nullptr, 0);
    writeTimestamp(kTimestampDepthEnd);
    frameResources.depthOutput = DepthOutputKind::ApproximateSplatDepth;
  } else {
    writeTimestamp(kTimestampDepthBegin);
    writeTimestamp(kTimestampDepthEnd);
  }

  finalizeManagedTarget(target.colorTarget, currentColorState, target.colorStateAfter);
  finalizeManagedTarget(target.depthTarget, currentDepthState, target.depthStateAfter);
  finalizeManagedTarget(target.motionVectorsTarget, currentMotionState, target.motionVectorsStateAfter);
  writeTimestamp(kTimestampFrameEnd);
  if (scratch->timestampReadbackBuffer != nullptr && !scratch->timestampCopyPending) {
    commandList->ResolveQueryData(scratch->timestampQueryHeap.Get(), D3D12_QUERY_TYPE_TIMESTAMP, 0, kTimestampQueryCount,
                                  scratch->timestampReadbackBuffer.Get(), 0);
    scratch->timestampCopyFrame = scratch->gpuTimingFrame;
    scratch->timestampCopyFence = frameContext->fence;
    scratch->timestampCopyFenceValue = frameContext->submissionFenceValue;
    scratch->timestampCopyPending = true;
  }
  refreshFrameResources();
  frameResources.sortBackend = activeSortBackend;
  frameResources.visibleSplatCount = scratch->lastVisibleCount;
  frameResources.emittedSortEntryCount = emittedPairs;
  frameResources.drawInstanceCount = emittedPairs;
  if (input.settings.outputDepth && target.depthTarget != nullptr && target.depthDsv.ptr != 0) {
    frameResources.depthOutput = DepthOutputKind::ApproximateSplatDepth;
  }

  if (options != nullptr && options->hooks != nullptr) {
    Status hookStatus = invokeStage(options->hooks->afterRaster, RenderHookStage::AfterRaster);
    if (!hookStatus.ok) {
      return finish(hookStatus);
    }
    hookStatus = invokeStage(options->hooks->beforePostProcess, RenderHookStage::BeforePostProcess);
    if (!hookStatus.ok) {
      return finish(hookStatus);
    }
    hookStatus = invokeStage(options->hooks->afterPostProcess, RenderHookStage::AfterPostProcess);
    if (!hookStatus.ok) {
      return finish(hookStatus);
    }
  }

  stats.gaussiansTotal += runtime.sceneGaussianCount;
  stats.gaussiansVisible += std::min<uint32_t>(scratch->lastVisibleCount, runtime.sceneGaussianCount);
  stats.sortPasses += sortPassCount;
  stats.sortBackend = activeSortBackend;
  stats.splatAlpha.minValue = 0.0f;
  stats.splatAlpha.maxValue = 1.0f;
  for (size_t i = 0; i < scratch->lastSplatAlphaBins.size(); ++i) {
    stats.splatAlpha.bins[i] = static_cast<float>(scratch->lastSplatAlphaBins[i]);
  }
  stats.projectionActiveThreads.minValue = 0.0f;
  stats.projectionActiveThreads.maxValue = 64.0f;
  for (size_t i = 0; i < scratch->lastProjectionActiveThreadBins.size(); ++i) {
    stats.projectionActiveThreads.bins[i] = static_cast<float>(scratch->lastProjectionActiveThreadBins[i]);
  }

  if (outResult != nullptr) {
    outResult->stats = stats;
    outResult->resources = frameResources;
    outResult->upscalerInput.color = frameResources.colorTarget;
    outResult->upscalerInput.depth = frameResources.depthTarget;
    outResult->upscalerInput.motionVectors = frameResources.motionVectorsTarget;
    outResult->upscalerInput.renderWidth = std::max<uint32_t>(input.viewportWidth, 1u);
    outResult->upscalerInput.renderHeight = std::max<uint32_t>(input.viewportHeight, 1u);
    outResult->upscalerInput.outputWidth = std::max<uint32_t>(frameResources.colorTarget.width, outResult->upscalerInput.renderWidth);
    outResult->upscalerInput.outputHeight = std::max<uint32_t>(frameResources.colorTarget.height, outResult->upscalerInput.renderHeight);
    outResult->upscalerInput.view = input.view;
    outResult->upscalerInput.proj = input.proj;
    outResult->upscalerInput.jitter = input.jitter;
    outResult->upscalerInput.cameraCut = input.cameraCut;
    outResult->hasUpscalerInput = frameResources.colorTarget.resource != nullptr;
  }
  return finish(Status::Ok());
}


}  // namespace directxsplat
