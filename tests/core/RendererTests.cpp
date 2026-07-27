#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <Windows.h>
#include <d3d12.h>
#include <d3d12sdklayers.h>
#include <dxgi1_6.h>
#include <wrl/client.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <future>
#include <stdexcept>
#include <sstream>
#include <vector>

#include "directxsplat/context.h"
#include "directxsplat/extensions.h"
#include "directxsplat/gpu_resources.h"
#include "directxsplat/math.h"
#include "directxsplat/render_hooks.h"
#include "directxsplat/renderer.h"
#include "directxsplat/scene.h"
#include "directxsplat/settings.h"
#include "directxsplat/types.h"
#include "directxsplat/vram_format.h"
#include "renderer/raster/GaussianRasterPipeline.h"

namespace directxsplat {
namespace {

using Microsoft::WRL::ComPtr;

constexpr float kShC0 = 0.28209479177387814f;
constexpr float kShC1 = 0.4886025119029199f;

struct OffscreenFrame {
  ComPtr<ID3D12Resource> colorTexture;
  ComPtr<ID3D12Resource> colorReadback;
  ComPtr<ID3D12DescriptorHeap> rtvHeap;
  D3D12_CPU_DESCRIPTOR_HANDLE rtv{};
  D3D12_PLACED_SUBRESOURCE_FOOTPRINT footprint{};
  uint32_t width = 0;
  uint32_t height = 0;
  RenderTargetBinding binding{};
};

class RenderHarness {
 public:
  ~RenderHarness() {
    renderer_.Shutdown();
    context_.Shutdown();
    if (fenceEvent_ != nullptr) {
      CloseHandle(fenceEvent_);
      fenceEvent_ = nullptr;
    }
  }

  Status Initialize() {
    if (ComPtr<ID3D12Debug> debug; SUCCEEDED(D3D12GetDebugInterface(IID_PPV_ARGS(debug.GetAddressOf())))) {
      debug->EnableDebugLayer();
    }
    if (ComPtr<ID3D12DeviceRemovedExtendedDataSettings> dredSettings;
        SUCCEEDED(D3D12GetDebugInterface(IID_PPV_ARGS(dredSettings.GetAddressOf())))) {
      dredSettings->SetAutoBreadcrumbsEnablement(D3D12_DRED_ENABLEMENT_FORCED_ON);
      dredSettings->SetPageFaultEnablement(D3D12_DRED_ENABLEMENT_FORCED_ON);
    }

    HRESULT hr = CreateDXGIFactory2(DXGI_CREATE_FACTORY_DEBUG, IID_PPV_ARGS(factory_.GetAddressOf()));
    if (FAILED(hr)) {
      hr = CreateDXGIFactory2(0, IID_PPV_ARGS(factory_.GetAddressOf()));
    }
    if (FAILED(hr)) {
      return Status::Error("failed creating DXGI factory");
    }

    const bool forceWarp = GetEnvironmentVariableW(L"DXSPLAT_TEST_FORCE_WARP", nullptr, 0) != 0;
    if (!forceWarp) {
      for (UINT adapterIndex = 0; ; ++adapterIndex) {
        ComPtr<IDXGIAdapter1> candidate;
        hr = factory_->EnumAdapterByGpuPreference(adapterIndex,
                                                  DXGI_GPU_PREFERENCE_HIGH_PERFORMANCE,
                                                  IID_PPV_ARGS(candidate.GetAddressOf()));
        if (hr == DXGI_ERROR_NOT_FOUND) {
          break;
        }
        if (FAILED(hr)) {
          break;
        }
        DXGI_ADAPTER_DESC1 desc{};
        candidate->GetDesc1(&desc);
        if ((desc.Flags & DXGI_ADAPTER_FLAG_SOFTWARE) != 0) {
          continue;
        }
        if (SUCCEEDED(D3D12CreateDevice(candidate.Get(), D3D_FEATURE_LEVEL_11_0, IID_PPV_ARGS(device_.ReleaseAndGetAddressOf())))) {
          adapter_ = candidate;
          break;
        }
      }
    }

    if (device_ == nullptr) {
      hr = factory_->EnumWarpAdapter(IID_PPV_ARGS(adapter_.ReleaseAndGetAddressOf()));
      if (FAILED(hr)) {
        return Status::Error("failed acquiring WARP adapter");
      }
      hr = D3D12CreateDevice(adapter_.Get(), D3D_FEATURE_LEVEL_11_0, IID_PPV_ARGS(device_.ReleaseAndGetAddressOf()));
      if (FAILED(hr)) {
        return Status::Error("failed creating D3D12 device");
      }
    }

    D3D12_COMMAND_QUEUE_DESC queueDesc{};
    queueDesc.Type = D3D12_COMMAND_LIST_TYPE_DIRECT;
    hr = device_->CreateCommandQueue(&queueDesc, IID_PPV_ARGS(queue_.GetAddressOf()));
    if (FAILED(hr)) {
      return Status::Error("failed creating direct queue");
    }

    hr = device_->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(allocator_.GetAddressOf()));
    if (FAILED(hr)) {
      return Status::Error("failed creating command allocator");
    }

    hr = device_->CreateCommandList(0,
                                    D3D12_COMMAND_LIST_TYPE_DIRECT,
                                    allocator_.Get(),
                                    nullptr,
                                    IID_PPV_ARGS(commandList_.GetAddressOf()));
    if (FAILED(hr)) {
      return Status::Error("failed creating command list");
    }
    commandList_->SetName(L"DirectXSplatCoreTestsCommandList");
    commandList_->Close();

    hr = device_->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(fence_.GetAddressOf()));
    if (FAILED(hr)) {
      return Status::Error("failed creating fence");
    }

    fenceEvent_ = CreateEventW(nullptr, FALSE, FALSE, nullptr);
    if (fenceEvent_ == nullptr) {
      return Status::Error("failed creating fence event");
    }

    Status status = context_.Initialize(device_.Get(), queue_.Get(), fence_.Get());
    if (!status.ok) {
      return status;
    }
    return renderer_.Initialize(context_);
  }

  Status ResetCommandList() {
    HRESULT hr = allocator_->Reset();
    if (FAILED(hr)) {
      return Status::Error("failed resetting command allocator");
    }
    hr = commandList_->Reset(allocator_.Get(), nullptr);
    if (FAILED(hr)) {
      return Status::Error("failed resetting command list");
    }
    return Status::Ok();
  }

  Status QueueUploadSync(UploadSyncPoint sync) {
    if (!sync.IsValid()) {
      return Status::Ok();
    }
    HRESULT hr = queue_->Wait(sync.fence, sync.value);
    if (FAILED(hr)) {
      return Status::Error("failed waiting for upload sync point");
    }
    return Status::Ok();
  }

  Status ExecuteAndWait(UploadSyncPoint sync = {}) {
    Status syncStatus = QueueUploadSync(sync);
    if (!syncStatus.ok) {
      return syncStatus;
    }
    HRESULT hr = commandList_->Close();
    if (FAILED(hr)) {
      return Status::Error("failed closing command list");
    }
    ID3D12CommandList* lists[] = {commandList_.Get()};
    queue_->ExecuteCommandLists(1, lists);
    const uint64_t targetFence = ++fenceValue_;
    hr = queue_->Signal(fence_.Get(), targetFence);
    if (FAILED(hr)) {
      return Status::Error("failed signaling fence");
    }
    if (fence_->GetCompletedValue() < targetFence) {
      hr = fence_->SetEventOnCompletion(targetFence, fenceEvent_);
      if (FAILED(hr)) {
        return Status::Error("failed waiting for fence");
      }
      WaitForSingleObject(fenceEvent_, INFINITE);
    }
    return Status::Ok();
  }

  Status CreateCommandList(ComPtr<ID3D12CommandAllocator>& allocator, ComPtr<ID3D12GraphicsCommandList>& commandList) {
    HRESULT hr = device_->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(allocator.ReleaseAndGetAddressOf()));
    if (FAILED(hr)) {
      return Status::Error("failed creating command allocator");
    }
    hr = device_->CreateCommandList(0,
                                    D3D12_COMMAND_LIST_TYPE_DIRECT,
                                    allocator.Get(),
                                    nullptr,
                                    IID_PPV_ARGS(commandList.ReleaseAndGetAddressOf()));
    if (FAILED(hr)) {
      return Status::Error("failed creating command list");
    }
    return Status::Ok();
  }

  Status ExecuteCommandList(ID3D12GraphicsCommandList* commandList, UploadSyncPoint sync, uint64_t signalValue) {
    Status syncStatus = QueueUploadSync(sync);
    if (!syncStatus.ok) {
      return syncStatus;
    }
    HRESULT hr = commandList->Close();
    if (FAILED(hr)) {
      return Status::Error("failed closing command list");
    }
    ID3D12CommandList* lists[] = {commandList};
    queue_->ExecuteCommandLists(1, lists);
    hr = queue_->Signal(fence_.Get(), signalValue);
    if (FAILED(hr)) {
      return Status::Error("failed signaling fence");
    }
    fenceValue_ = std::max(fenceValue_, signalValue);
    return Status::Ok();
  }

  Status SignalFenceOnly() {
    const uint64_t targetFence = ++fenceValue_;
    HRESULT hr = queue_->Signal(fence_.Get(), targetFence);
    if (FAILED(hr)) {
      return Status::Error("failed signaling fence");
    }
    if (fence_->GetCompletedValue() < targetFence) {
      hr = fence_->SetEventOnCompletion(targetFence, fenceEvent_);
      if (FAILED(hr)) {
        return Status::Error("failed waiting for fence");
      }
      WaitForSingleObject(fenceEvent_, INFINITE);
    }
    return Status::Ok();
  }

  Status WaitForSubmittedWork() {
    if (fence_->GetCompletedValue() >= fenceValue_) {
      return Status::Ok();
    }
    HRESULT hr = fence_->SetEventOnCompletion(fenceValue_, fenceEvent_);
    if (FAILED(hr)) {
      return Status::Error("failed waiting for fence");
    }
    WaitForSingleObject(fenceEvent_, INFINITE);
    return Status::Ok();
  }

  RenderFrameContext FrameContext() const {
    RenderFrameContext context{};
    context.fence = fence_.Get();
    context.completedFenceValue = fence_ != nullptr ? fence_->GetCompletedValue() : 0;
    context.submissionFenceValue = fenceValue_ + 1;
    context.frameIndex = fenceValue_;
    return context;
  }

  Status CreateOffscreenFrame(uint32_t width, uint32_t height, OffscreenFrame& out) {
    out = {};
    out.width = width;
    out.height = height;

    D3D12_HEAP_PROPERTIES heapProps{};
    heapProps.Type = D3D12_HEAP_TYPE_DEFAULT;
    heapProps.CreationNodeMask = 1;
    heapProps.VisibleNodeMask = 1;

    D3D12_RESOURCE_DESC desc{};
    desc.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
    desc.Width = width;
    desc.Height = height;
    desc.DepthOrArraySize = 1;
    desc.MipLevels = 1;
    desc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    desc.SampleDesc.Count = 1;
    desc.Layout = D3D12_TEXTURE_LAYOUT_UNKNOWN;
    desc.Flags = D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET;

    D3D12_CLEAR_VALUE clearValue{};
    clearValue.Format = desc.Format;
    clearValue.Color[0] = 0.0f;
    clearValue.Color[1] = 0.0f;
    clearValue.Color[2] = 0.0f;
    clearValue.Color[3] = 1.0f;

    HRESULT hr = device_->CreateCommittedResource(&heapProps,
                                                  D3D12_HEAP_FLAG_NONE,
                                                  &desc,
                                                  D3D12_RESOURCE_STATE_COMMON,
                                                  &clearValue,
                                                  IID_PPV_ARGS(out.colorTexture.GetAddressOf()));
    if (FAILED(hr)) {
      return Status::Error("failed creating offscreen color texture");
    }

    D3D12_DESCRIPTOR_HEAP_DESC rtvHeapDesc{};
    rtvHeapDesc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_RTV;
    rtvHeapDesc.NumDescriptors = 1;
    hr = device_->CreateDescriptorHeap(&rtvHeapDesc, IID_PPV_ARGS(out.rtvHeap.GetAddressOf()));
    if (FAILED(hr)) {
      return Status::Error("failed creating RTV heap");
    }
    out.rtv = out.rtvHeap->GetCPUDescriptorHandleForHeapStart();
    device_->CreateRenderTargetView(out.colorTexture.Get(), nullptr, out.rtv);

    D3D12_PLACED_SUBRESOURCE_FOOTPRINT footprint{};
    UINT rows = 0;
    UINT64 rowBytes = 0;
    UINT64 totalBytes = 0;
    device_->GetCopyableFootprints(&desc, 0, 1, 0, &footprint, &rows, &rowBytes, &totalBytes);
    out.footprint = footprint;

    D3D12_HEAP_PROPERTIES readbackHeapProps{};
    readbackHeapProps.Type = D3D12_HEAP_TYPE_READBACK;
    readbackHeapProps.CreationNodeMask = 1;
    readbackHeapProps.VisibleNodeMask = 1;

    D3D12_RESOURCE_DESC readbackDesc{};
    readbackDesc.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
    readbackDesc.Width = totalBytes;
    readbackDesc.Height = 1;
    readbackDesc.DepthOrArraySize = 1;
    readbackDesc.MipLevels = 1;
    readbackDesc.SampleDesc.Count = 1;
    readbackDesc.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;

    hr = device_->CreateCommittedResource(&readbackHeapProps,
                                          D3D12_HEAP_FLAG_NONE,
                                          &readbackDesc,
                                          D3D12_RESOURCE_STATE_COPY_DEST,
                                          nullptr,
                                          IID_PPV_ARGS(out.colorReadback.GetAddressOf()));
    if (FAILED(hr)) {
      return Status::Error("failed creating color readback buffer");
    }

    out.binding.colorTarget = out.colorTexture.Get();
    out.binding.colorRtv = out.rtv;
    out.binding.colorFormat = DXGI_FORMAT_R8G8B8A8_UNORM;
    out.binding.colorStateBefore = D3D12_RESOURCE_STATE_COMMON;
    out.binding.colorStateAfter = D3D12_RESOURCE_STATE_COPY_SOURCE;
    out.binding.transitionMode = ResourceTransitionMode::LibraryManaged;
    out.binding.clearColor = true;
    out.binding.clearColorValue[0] = 0.0f;
    out.binding.clearColorValue[1] = 0.0f;
    out.binding.clearColorValue[2] = 0.0f;
    out.binding.clearColorValue[3] = 1.0f;
    out.binding.viewport.TopLeftX = 0.0f;
    out.binding.viewport.TopLeftY = 0.0f;
    out.binding.viewport.Width = static_cast<float>(width);
    out.binding.viewport.Height = static_cast<float>(height);
    out.binding.viewport.MinDepth = 0.0f;
    out.binding.viewport.MaxDepth = 1.0f;
    out.binding.scissor.left = 0;
    out.binding.scissor.top = 0;
    out.binding.scissor.right = static_cast<LONG>(width);
    out.binding.scissor.bottom = static_cast<LONG>(height);
    return Status::Ok();
  }

  void QueueColorReadback(ID3D12GraphicsCommandList* commandList, const OffscreenFrame& frame) {
    D3D12_TEXTURE_COPY_LOCATION src{};
    src.pResource = frame.colorTexture.Get();
    src.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
    src.SubresourceIndex = 0;

    D3D12_TEXTURE_COPY_LOCATION dst{};
    dst.pResource = frame.colorReadback.Get();
    dst.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
    dst.PlacedFootprint = frame.footprint;

    commandList->CopyTextureRegion(&dst, 0, 0, 0, &src, nullptr);
  }

  void QueueColorReadback(const OffscreenFrame& frame) {
    QueueColorReadback(commandList_.Get(), frame);
  }

  std::vector<uint8_t> ReadbackColor(const OffscreenFrame& frame) const {
    std::vector<uint8_t> pixels(static_cast<size_t>(frame.width) * frame.height * 4u, 0u);
    void* mapped = nullptr;
    const HRESULT hr = frame.colorReadback->Map(0, nullptr, &mapped);
    if (FAILED(hr) || mapped == nullptr) {
      return {};
    }
    const uint8_t* src = reinterpret_cast<const uint8_t*>(mapped);
    for (uint32_t y = 0; y < frame.height; ++y) {
      std::memcpy(pixels.data() + static_cast<size_t>(y) * frame.width * 4u,
                  src + static_cast<size_t>(y) * frame.footprint.Footprint.RowPitch,
                  static_cast<size_t>(frame.width) * 4u);
    }
    frame.colorReadback->Unmap(0, nullptr);
    return pixels;
  }

  HRESULT DeviceRemovedReason() const {
    return device_ != nullptr ? device_->GetDeviceRemovedReason() : E_FAIL;
  }

  std::string DebugMessages() const {
    if (device_ == nullptr) {
      return {};
    }
    std::string out;
    if (ComPtr<ID3D12InfoQueue> infoQueue; SUCCEEDED(device_.As(&infoQueue)) && infoQueue != nullptr) {
      const UINT64 count = infoQueue->GetNumStoredMessagesAllowedByRetrievalFilter();
      const UINT64 begin = count > 12 ? count - 12 : 0;
      for (UINT64 i = begin; i < count; ++i) {
        SIZE_T bytes = 0;
        if (FAILED(infoQueue->GetMessage(i, nullptr, &bytes)) || bytes == 0) {
          continue;
        }
        std::string storage(bytes, '\0');
        auto* message = reinterpret_cast<D3D12_MESSAGE*>(storage.data());
        if (FAILED(infoQueue->GetMessage(i, message, &bytes)) || message->pDescription == nullptr) {
          continue;
        }
        out += "\n";
        out += message->pDescription;
      }
    }
    if (ComPtr<ID3D12DeviceRemovedExtendedData1> dred; SUCCEEDED(device_.As(&dred)) && dred != nullptr) {
      D3D12_DRED_AUTO_BREADCRUMBS_OUTPUT1 breadcrumbs{};
      if (SUCCEEDED(dred->GetAutoBreadcrumbsOutput1(&breadcrumbs)) && breadcrumbs.pHeadAutoBreadcrumbNode != nullptr) {
        out += "\nDRED breadcrumbs:";
        for (const D3D12_AUTO_BREADCRUMB_NODE1* node = breadcrumbs.pHeadAutoBreadcrumbNode; node != nullptr;
             node = node->pNext) {
          out += "\n";
          if (node->pCommandListDebugNameA != nullptr) {
            out += node->pCommandListDebugNameA;
          } else if (node->pCommandQueueDebugNameA != nullptr) {
            out += node->pCommandQueueDebugNameA;
          } else {
            out += "<unnamed>";
          }
          out += " last=";
          out += std::to_string(node->BreadcrumbCount);
          out += "/";
          out += std::to_string(node->pLastBreadcrumbValue != nullptr ? *node->pLastBreadcrumbValue : 0u);
        }
      }
    }
    return out;
  }

  Renderer& renderer() { return renderer_; }
  ID3D12Device* device() const { return device_.Get(); }
  ID3D12CommandQueue* queue() const { return queue_.Get(); }
  ID3D12Fence* fence() const { return fence_.Get(); }
  ID3D12GraphicsCommandList* commandList() const { return commandList_.Get(); }

 private:
  ComPtr<IDXGIFactory6> factory_;
  ComPtr<IDXGIAdapter1> adapter_;
  ComPtr<ID3D12Device> device_;
  ComPtr<ID3D12CommandQueue> queue_;
  ComPtr<ID3D12CommandAllocator> allocator_;
  ComPtr<ID3D12GraphicsCommandList> commandList_;
  ComPtr<ID3D12Fence> fence_;
  HANDLE fenceEvent_ = nullptr;
  uint64_t fenceValue_ = 0;
  D3D12Context context_;
  Renderer renderer_;
};

Gaussian MakeGaussian(const Vec3& position, float r, float g, float b) {
  Gaussian gaussian{};
  gaussian.position = position;
  gaussian.scale = {0.18f, 0.18f, 0.18f};
  gaussian.rotation = {0.0f, 0.0f, 0.0f, 1.0f};
  gaussian.opacity = 1.0f;
  gaussian.sh[0] = (r - 0.5f) / kShC0;
  gaussian.sh[16] = (g - 0.5f) / kShC0;
  gaussian.sh[32] = (b - 0.5f) / kShC0;
  return gaussian;
}

Scene MakeTinyScene() {
  Scene scene{};
  GaussianSet set{};
  set.name = "tiny";
  set.gaussians.push_back(MakeGaussian({-0.2f, 0.0f, 2.5f}, 1.0f, 0.2f, 0.2f));
  set.gaussians.push_back(MakeGaussian({0.2f, 0.0f, 2.8f}, 0.2f, 1.0f, 0.2f));
  scene.splatSets.push_back(std::move(set));
  return scene;
}

Scene MakeDirectionalShScene() {
  Scene scene{};
  GaussianSet set{};
  set.name = "directional-sh";
  const float directional = (0.5f - 1.25f) / kShC1;
  auto addGaussian = [&](const Vec3& position) {
    Gaussian gaussian = MakeGaussian(position, 1.25f, 1.25f, 1.25f);
    gaussian.scale = {0.45f, 0.45f, 0.45f};
    gaussian.opacity = 8.0f;
    gaussian.sh[2] = directional;
    gaussian.sh[18] = directional;
    gaussian.sh[34] = directional;
    set.gaussians.push_back(gaussian);
  };
  addGaussian({-0.2f, 0.0f, 2.5f});
  addGaussian({0.2f, 0.0f, 2.8f});
  scene.splatSets.push_back(std::move(set));
  return scene;
}

Scene MakeAnisotropicScene() {
  Scene scene{};
  GaussianSet set{};
  set.name = "anisotropic";
  auto addGaussian = [&](const Vec3& position, float angle) {
    Gaussian gaussian = MakeGaussian(position, 0.8f, 0.5f, 0.2f);
    gaussian.scale = {0.08f, 0.35f, 0.65f};
    gaussian.rotation = {0.0f, std::sin(angle * 0.5f), 0.0f, std::cos(angle * 0.5f)};
    set.gaussians.push_back(gaussian);
  };
  addGaussian({-0.25f, 0.1f, 2.5f}, 0.75f);
  addGaussian({0.35f, -0.15f, 3.0f}, -0.6f);
  scene.splatSets.push_back(std::move(set));
  return scene;
}

Scene MakeSceneWithColor(float x, float r, float g, float b) {
  Scene scene{};
  GaussianSet set{};
  set.name = "color";
  Gaussian a = MakeGaussian({x, 0.0f, 2.6f}, r, g, b);
  a.scale = {0.45f, 0.45f, 0.45f};
  Gaussian b0 = MakeGaussian({x + 0.18f, 0.08f, 2.9f}, b, r, g);
  b0.scale = {0.35f, 0.35f, 0.35f};
  set.gaussians.push_back(a);
  set.gaussians.push_back(b0);
  scene.splatSets.push_back(std::move(set));
  return scene;
}

RenderInput MakeRenderInput(uint32_t width, uint32_t height) {
  RenderInput input{};
  input.view = LookAt({0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {0.0f, 1.0f, 0.0f});
  input.proj = Perspective(1.0f, static_cast<float>(width) / static_cast<float>(height), 0.01f, 100.0f);
  input.cameraPosition = {0.0f, 0.0f, 0.0f};
  input.viewportWidth = width;
  input.viewportHeight = height;
  input.settings.fastCulling = true;
  input.settings.antialiasing = false;
  input.settings.gaussianScalingModifier = 1.25f;
  return input;
}

size_t CountNonZeroPixels(const std::vector<uint8_t>& pixels) {
  size_t nonZero = 0;
  for (size_t i = 0; i + 3 < pixels.size(); i += 4) {
    if (pixels[i] != 0 || pixels[i + 1] != 0 || pixels[i + 2] != 0) {
      ++nonZero;
    }
  }
  return nonZero;
}

bool ImagesDiffer(const std::vector<uint8_t>& a, const std::vector<uint8_t>& b) {
  if (a.size() != b.size()) {
    return true;
  }
  for (size_t i = 0; i < a.size(); ++i) {
    if (a[i] != b[i]) {
      return true;
    }
  }
  return false;
}

}  

TEST_CASE("Renderer default handles and settings are safe") {
  UploadedSceneHandle sceneHandle{};
  UploadedChunkHandle chunkHandle{};
  SceneMutationToken mutationToken{};
  SceneAccessInfo accessInfo{};
  RenderSettings settings{};

  CHECK_FALSE(sceneHandle.IsValid());
  CHECK_FALSE(chunkHandle.IsValid());
  CHECK_FALSE(mutationToken.IsValid());
  CHECK_FALSE(accessInfo.readyToRender);
  CHECK(settings.fastCulling);
  CHECK(settings.antialiasing);
}

TEST_CASE("Renderer public APIs fail cleanly before initialization") {
  Renderer renderer;
  UploadedSceneHandle sceneHandle{};
  UploadedChunkHandle chunkHandle{};
  SceneMutationToken token{};
  Scene scene = MakeTinyScene();
  GaussianSet chunk = scene.splatSets.front();
  UploadedSceneInfo sceneInfo{};
  UploadedChunkInfo chunkInfo{};
  SceneAccessInfo accessInfo{};
  UploadedSceneGpuResources gpuResources{};
  std::vector<UploadedChunkHandle> chunks;
  RenderPreparationResult preparation{};
  RenderResult result{};
  RenderTargetBinding target{};
  RenderInput input = MakeRenderInput(16, 16);

  CHECK_FALSE(renderer.CreateUploadedScene(sceneHandle).ok);
  CHECK_FALSE(renderer.CreateUploadedScene(scene, sceneHandle).ok);
  CHECK_FALSE(renderer.UpdateUploadedScene(sceneHandle, scene).ok);
  CHECK_FALSE(renderer.UpdateUploadedScene(token, scene).ok);
  CHECK_FALSE(renderer.DestroyUploadedScene(sceneHandle).ok);
  CHECK_FALSE(renderer.GetSceneAccessInfo(sceneHandle, accessInfo).ok);
  CHECK_FALSE(renderer.GetUploadedSceneInfo(sceneHandle, sceneInfo).ok);
  CHECK_FALSE(renderer.GetUploadedChunkInfo(sceneHandle, chunkHandle, chunkInfo).ok);
  CHECK_FALSE(renderer.GetUploadedSceneGpuResources(sceneHandle, RenderFrameContext{}, gpuResources).ok);
  CHECK_FALSE(renderer.AcquireUploadedSceneGpuResources(sceneHandle, RenderFrameContext{}, gpuResources).ok);
  CHECK_FALSE(renderer.PrepareSceneForRender(sceneHandle, input, RenderFrameContext{}, &preparation).ok);
  CHECK_FALSE(renderer.BeginSceneMutation(sceneHandle, token).ok);
  CHECK_FALSE(renderer.EndSceneMutation(token).ok);
  CHECK_FALSE(renderer.GetUploadedSceneChunks(sceneHandle, chunks).ok);
  CHECK_FALSE(renderer.AddUploadedChunk(sceneHandle, chunk, chunkHandle).ok);
  CHECK_FALSE(renderer.AddUploadedChunk(token, chunk, chunkHandle).ok);
  CHECK_FALSE(renderer.UpdateUploadedChunk(sceneHandle, chunkHandle, chunk).ok);
  CHECK_FALSE(renderer.UpdateUploadedChunk(token, chunkHandle, chunk).ok);
  CHECK_FALSE(renderer.RemoveUploadedChunk(sceneHandle, chunkHandle).ok);
  CHECK_FALSE(renderer.RemoveUploadedChunk(token, chunkHandle).ok);
  CHECK_FALSE(renderer.SetUploadedChunkEnabled(sceneHandle, chunkHandle, true).ok);
  CHECK_FALSE(renderer.SetUploadedChunkEnabled(token, chunkHandle, true).ok);
  CHECK_FALSE(renderer.SetUploadedChunkScalingModifier(sceneHandle, chunkHandle, 1.0f).ok);
  CHECK_FALSE(renderer.SetUploadedChunkScalingModifier(token, chunkHandle, 1.0f).ok);
  CHECK_FALSE(renderer.Render(nullptr, target, sceneHandle, input, RenderFrameContext{}, result).ok);
  CHECK_FALSE(renderer.Render(nullptr, target, sceneHandle, input, AdvancedRenderOptions{}, RenderFrameContext{}, result).ok);
  CHECK_FALSE(renderer.IsUploadedSceneValid(sceneHandle));
  CHECK_FALSE(renderer.IsUploadedChunkValid(sceneHandle, chunkHandle));
}

TEST_CASE("Renderer handles empty scenes and chunk operations") {
  RenderHarness harness;
  const Status init = harness.Initialize();
  REQUIRE_MESSAGE(init.ok, init.message);

  UploadedSceneHandle sceneHandle{};
  REQUIRE(harness.renderer().CreateUploadedScene(sceneHandle).ok);
  REQUIRE(sceneHandle.IsValid());

  UploadedSceneInfo sceneInfo{};
  REQUIRE(harness.renderer().GetUploadedSceneInfo(sceneHandle, sceneInfo).ok);
  CHECK(sceneInfo.chunkCount == 0u);

  RenderPreparationResult preparation{};
  RenderResult renderResult{};
  const RenderInput input = MakeRenderInput(64, 64);
  const RenderFrameContext frameContext = harness.FrameContext();
  CHECK(harness.renderer().PrepareSceneForRender(sceneHandle, input, frameContext, &preparation).ok);

  OffscreenFrame frame{};
  REQUIRE(harness.CreateOffscreenFrame(64, 64, frame).ok);
  REQUIRE(harness.ResetCommandList().ok);
  REQUIRE(harness.renderer().Render(harness.commandList(), frame.binding, sceneHandle, input, frameContext, renderResult).ok);
  harness.QueueColorReadback(frame);
  REQUIRE(harness.ExecuteAndWait(renderResult.submission.uploadSyncPoint).ok);
  CHECK_MESSAGE(harness.DeviceRemovedReason() == S_OK, harness.DebugMessages());

  GaussianSet chunk = MakeTinyScene().splatSets.front();
  UploadedChunkHandle chunkHandle{};
  REQUIRE(harness.renderer().AddUploadedChunk(sceneHandle, chunk, chunkHandle).ok);
  CHECK(chunkHandle.IsValid());
  CHECK(harness.renderer().IsUploadedChunkValid(sceneHandle, chunkHandle));

  UploadedChunkInfo chunkInfo{};
  REQUIRE(harness.renderer().GetUploadedChunkInfo(sceneHandle, chunkHandle, chunkInfo).ok);
  CHECK(chunkInfo.visible);
  CHECK(chunkInfo.scalingModifier == doctest::Approx(1.0f));

  std::vector<UploadedChunkHandle> chunkHandles;
  REQUIRE(harness.renderer().GetUploadedSceneChunks(sceneHandle, chunkHandles).ok);
  CHECK(chunkHandles.size() == 1u);

  REQUIRE(harness.renderer().SetUploadedChunkScalingModifier(sceneHandle, chunkHandle, 0.75f).ok);
  REQUIRE(harness.renderer().GetUploadedChunkInfo(sceneHandle, chunkHandle, chunkInfo).ok);
  CHECK(chunkInfo.scalingModifier == doctest::Approx(0.75f));
  REQUIRE(harness.renderer().SetUploadedChunkEnabled(sceneHandle, chunkHandle, false).ok);
  REQUIRE(harness.renderer().GetUploadedChunkInfo(sceneHandle, chunkHandle, chunkInfo).ok);
  CHECK_FALSE(chunkInfo.visible);
  REQUIRE(harness.renderer().SetUploadedChunkEnabled(sceneHandle, chunkHandle, true).ok);
  REQUIRE(harness.renderer().GetUploadedChunkInfo(sceneHandle, chunkHandle, chunkInfo).ok);
  CHECK(chunkInfo.visible);
  CHECK_FALSE(harness.renderer().UpdateUploadedChunk(sceneHandle, UploadedChunkHandle{chunkHandle.value + 1000u}, chunk).ok);
  REQUIRE(harness.renderer().GetUploadedChunkInfo(sceneHandle, chunkHandle, chunkInfo).ok);
  CHECK(chunkInfo.visible);
  CHECK(chunkInfo.scalingModifier == doctest::Approx(0.75f));

  constexpr size_t oversizedChunkCount =
      static_cast<size_t>((64ull * 1024ull * 1024ull) / ((static_cast<uint64_t>(sizeof(Gaussian)) + sizeof(Vec3)) * 3ull)) + 1u;
  GaussianSet oversizedChunk = chunk;
  oversizedChunk.gaussians.assign(oversizedChunkCount, chunk.gaussians.front());
  UploadedChunkHandle oversizedHandle{};
  Status oversizedStatus = harness.renderer().AddUploadedChunk(sceneHandle, oversizedChunk, oversizedHandle);
  CHECK_FALSE(oversizedStatus.ok);
  CHECK(oversizedStatus.message == "chunk has too many gaussians");
  CHECK_FALSE(oversizedHandle.IsValid());
  oversizedStatus = harness.renderer().UpdateUploadedChunk(sceneHandle, chunkHandle, oversizedChunk);
  CHECK_FALSE(oversizedStatus.ok);
  CHECK(oversizedStatus.message == "chunk has too many gaussians");
  REQUIRE(harness.renderer().GetUploadedSceneChunks(sceneHandle, chunkHandles).ok);
  CHECK(chunkHandles.size() == 1u);

  CHECK_FALSE(harness.renderer().RemoveUploadedChunk(sceneHandle, UploadedChunkHandle{chunkHandle.value + 1000u}).ok);
  CHECK_FALSE(harness.renderer().SetUploadedChunkEnabled(sceneHandle, UploadedChunkHandle{chunkHandle.value + 1000u}, true).ok);
  CHECK_FALSE(harness.renderer().SetUploadedChunkScalingModifier(sceneHandle, UploadedChunkHandle{chunkHandle.value + 1000u}, 1.0f).ok);
  REQUIRE(harness.renderer().RemoveUploadedChunk(sceneHandle, chunkHandle).ok);
  CHECK_FALSE(harness.renderer().IsUploadedChunkValid(sceneHandle, chunkHandle));
  CHECK_FALSE(harness.renderer().GetUploadedChunkInfo(sceneHandle, chunkHandle, chunkInfo).ok);
  REQUIRE(harness.renderer().GetUploadedSceneChunks(sceneHandle, chunkHandles).ok);
  CHECK(chunkHandles.empty());
  REQUIRE(harness.renderer().GetUploadedSceneInfo(sceneHandle, sceneInfo).ok);
  CHECK(sceneInfo.chunkCount == 0u);
  CHECK(sceneInfo.gaussianCount == 0u);
  REQUIRE(harness.renderer().DestroyUploadedScene(sceneHandle).ok);
}

TEST_CASE("Renderer mutation tokens expire after EndSceneMutation") {
  RenderHarness harness;
  const Status init = harness.Initialize();
  REQUIRE_MESSAGE(init.ok, init.message);

  UploadedSceneHandle sceneHandle{};
  std::vector<UploadedChunkHandle> chunks;
  Scene scene = MakeTinyScene();
  REQUIRE(harness.renderer().CreateUploadedScene(scene, sceneHandle, &chunks).ok);
  REQUIRE(chunks.size() == 1u);

  SceneMutationToken token{};
  REQUIRE(harness.renderer().BeginSceneMutation(sceneHandle, token).ok);
  REQUIRE(harness.renderer().EndSceneMutation(token).ok);

  UploadedChunkHandle added{};
  CHECK_FALSE(harness.renderer().AddUploadedChunk(token, scene.splatSets.front(), added).ok);
  CHECK_FALSE(added.IsValid());
  CHECK_FALSE(harness.renderer().UpdateUploadedScene(token, scene).ok);
  CHECK_FALSE(harness.renderer().UpdateUploadedChunk(token, chunks.front(), scene.splatSets.front()).ok);
  CHECK_FALSE(harness.renderer().RemoveUploadedChunk(token, chunks.front()).ok);
  CHECK_FALSE(harness.renderer().SetUploadedChunkEnabled(token, chunks.front(), false).ok);
  CHECK_FALSE(harness.renderer().SetUploadedChunkScalingModifier(token, chunks.front(), 0.5f).ok);
  CHECK_FALSE(harness.renderer().DestroyUploadedScene(token).ok);

  UploadedSceneInfo sceneInfo{};
  REQUIRE(harness.renderer().GetUploadedSceneInfo(sceneHandle, sceneInfo).ok);
  CHECK(sceneInfo.chunkCount == 1u);
  CHECK(sceneInfo.gaussianCount == 2u);

  REQUIRE(harness.renderer().DestroyUploadedScene(sceneHandle).ok);
}

TEST_CASE("Renderer chunk handles are scoped to their uploaded scene") {
  RenderHarness harness;
  const Status init = harness.Initialize();
  REQUIRE_MESSAGE(init.ok, init.message);

  Scene sceneA = MakeTinyScene();
  Scene sceneB = MakeSceneWithColor(0.3f, 0.1f, 0.4f, 1.0f);
  UploadedSceneHandle sceneAHandle{};
  UploadedSceneHandle sceneBHandle{};
  std::vector<UploadedChunkHandle> chunksA;
  std::vector<UploadedChunkHandle> chunksB;
  REQUIRE(harness.renderer().CreateUploadedScene(sceneA, sceneAHandle, &chunksA).ok);
  REQUIRE(harness.renderer().CreateUploadedScene(sceneB, sceneBHandle, &chunksB).ok);
  REQUIRE(chunksA.size() == 1u);
  REQUIRE(chunksB.size() == 1u);

  CHECK(harness.renderer().IsUploadedChunkValid(sceneAHandle, chunksA.front()));
  CHECK(harness.renderer().IsUploadedChunkValid(sceneBHandle, chunksB.front()));
  CHECK_FALSE(harness.renderer().IsUploadedChunkValid(sceneBHandle, chunksA.front()));

  UploadedChunkInfo chunkInfo{};
  CHECK_FALSE(harness.renderer().GetUploadedChunkInfo(sceneBHandle, chunksA.front(), chunkInfo).ok);
  CHECK_FALSE(harness.renderer().UpdateUploadedChunk(sceneBHandle, chunksA.front(), sceneB.splatSets.front()).ok);
  CHECK_FALSE(harness.renderer().RemoveUploadedChunk(sceneBHandle, chunksA.front()).ok);
  CHECK_FALSE(harness.renderer().SetUploadedChunkEnabled(sceneBHandle, chunksA.front(), false).ok);
  CHECK_FALSE(harness.renderer().SetUploadedChunkScalingModifier(sceneBHandle, chunksA.front(), 0.5f).ok);

  UploadedSceneInfo sceneInfo{};
  REQUIRE(harness.renderer().GetUploadedSceneInfo(sceneAHandle, sceneInfo).ok);
  CHECK(sceneInfo.chunkCount == 1u);
  CHECK(sceneInfo.gaussianCount == 2u);
  REQUIRE(harness.renderer().GetUploadedSceneInfo(sceneBHandle, sceneInfo).ok);
  CHECK(sceneInfo.chunkCount == 1u);
  CHECK(sceneInfo.gaussianCount == 2u);

  REQUIRE(harness.renderer().DestroyUploadedScene(sceneAHandle).ok);
  REQUIRE(harness.renderer().DestroyUploadedScene(sceneBHandle).ok);
}

TEST_CASE("Renderer rejects stale and invalid frame contexts") {
  RenderHarness harness;
  const Status init = harness.Initialize();
  REQUIRE_MESSAGE(init.ok, init.message);

  UploadedSceneHandle sceneHandle{};
  REQUIRE(harness.renderer().CreateUploadedScene(MakeTinyScene(), sceneHandle).ok);
  const RenderInput input = MakeRenderInput(64, 64);
  OffscreenFrame frame{};
  REQUIRE(harness.CreateOffscreenFrame(64, 64, frame).ok);

  auto expectRejected = [&](const RenderFrameContext& frameContext) {
    RenderPreparationResult preparation{};
    CHECK_FALSE(harness.renderer().PrepareSceneForRender(sceneHandle, input, frameContext, &preparation).ok);
    RenderResult renderResult{};
    CHECK_FALSE(harness.renderer().Render(harness.commandList(), frame.binding, sceneHandle, input, frameContext, renderResult).ok);
  };

  RenderFrameContext missingFence = harness.FrameContext();
  missingFence.fence = nullptr;
  expectRejected(missingFence);

  RenderFrameContext missingSubmission = harness.FrameContext();
  missingSubmission.submissionFenceValue = 0;
  expectRejected(missingSubmission);

  ComPtr<ID3D12Fence> otherFence;
  REQUIRE(SUCCEEDED(harness.device()->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(otherFence.GetAddressOf()))));
  RenderFrameContext wrongFence = harness.FrameContext();
  wrongFence.fence = otherFence.Get();
  expectRejected(wrongFence);

  RenderFrameContext stale = harness.FrameContext();
  REQUIRE(harness.SignalFenceOnly().ok);
  stale.completedFenceValue = stale.submissionFenceValue;
  expectRejected(stale);
}

TEST_CASE("Renderer rejects reused in-flight submission fence values") {
  RenderHarness harness;
  const Status init = harness.Initialize();
  REQUIRE_MESSAGE(init.ok, init.message);

  UploadedSceneHandle sceneHandle{};
  REQUIRE(harness.renderer().CreateUploadedScene(MakeTinyScene(), sceneHandle).ok);
  const RenderInput input = MakeRenderInput(64, 64);
  RenderFrameContext frameContext = harness.FrameContext();
  frameContext.submissionFenceValue = 2;

  RenderPreparationResult preparation{};
  REQUIRE(harness.renderer().PrepareSceneForRender(sceneHandle, input, frameContext, &preparation).ok);

  OffscreenFrame frame{};
  REQUIRE(harness.CreateOffscreenFrame(64, 64, frame).ok);
  REQUIRE(harness.ResetCommandList().ok);

  RenderResult renderResult{};
  REQUIRE(harness.renderer().Render(harness.commandList(),
                                    frame.binding,
                                    sceneHandle,
                                    input,
                                    frameContext,
                                    renderResult)
              .ok);
  CHECK(renderResult.submission.submissionRequired);

  RenderFrameContext duplicate = frameContext;
  duplicate.frameIndex++;
  RenderResult duplicateResult{};
  CHECK_FALSE(harness.renderer().Render(harness.commandList(),
                                        frame.binding,
                                        sceneHandle,
                                        input,
                                        duplicate,
                                        duplicateResult)
                  .ok);

  RenderFrameContext decreasing = frameContext;
  decreasing.submissionFenceValue--;
  decreasing.frameIndex += 2;
  UploadedSceneGpuResources resources{};
  CHECK_FALSE(harness.renderer().AcquireUploadedSceneGpuResources(sceneHandle, decreasing, resources).ok);

  REQUIRE(harness.ExecuteCommandList(harness.commandList(),
                                     renderResult.submission.uploadSyncPoint,
                                     frameContext.submissionFenceValue)
              .ok);
  REQUIRE(harness.WaitForSubmittedWork().ok);
  REQUIRE(harness.renderer().DestroyUploadedScene(sceneHandle).ok);
}

TEST_CASE("Renderer renders one uploaded scene across queued frames") {
  RenderHarness harness;
  const Status init = harness.Initialize();
  REQUIRE_MESSAGE(init.ok, init.message);

  UploadedSceneHandle sceneHandle{};
  REQUIRE(harness.renderer().CreateUploadedScene(MakeTinyScene(), sceneHandle).ok);

  constexpr size_t kQueuedFrameCount = 3;
  std::array<OffscreenFrame, kQueuedFrameCount> frames{};
  std::array<ComPtr<ID3D12CommandAllocator>, kQueuedFrameCount> allocators{};
  std::array<ComPtr<ID3D12GraphicsCommandList>, kQueuedFrameCount> commandLists{};

  for (size_t i = 0; i < frames.size(); ++i) {
    RenderFrameContext frameContext = harness.FrameContext();
    frameContext.frameIndex = i;
    RenderInput input = MakeRenderInput(64, 64);
    input.frameIndex = frameContext.frameIndex;

    RenderPreparationResult preparation{};
    REQUIRE(harness.renderer().PrepareSceneForRender(sceneHandle, input, frameContext, &preparation).ok);
    CHECK(preparation.stats.gaussiansTotal == 2u);

    REQUIRE(harness.CreateOffscreenFrame(64, 64, frames[i]).ok);
    REQUIRE(harness.CreateCommandList(allocators[i], commandLists[i]).ok);

    RenderResult renderResult{};
    REQUIRE(harness.renderer().Render(commandLists[i].Get(), frames[i].binding, sceneHandle, input, frameContext, renderResult).ok);
    CHECK(renderResult.submission.submissionRequired);

    harness.QueueColorReadback(commandLists[i].Get(), frames[i]);
    REQUIRE(harness.ExecuteCommandList(commandLists[i].Get(),
                                       renderResult.submission.uploadSyncPoint,
                                       frameContext.submissionFenceValue)
                .ok);
  }

  REQUIRE(harness.WaitForSubmittedWork().ok);
  CHECK_MESSAGE(harness.DeviceRemovedReason() == S_OK, harness.DebugMessages());

  for (const OffscreenFrame& frame : frames) {
    const std::vector<uint8_t> pixels = harness.ReadbackColor(frame);
    REQUIRE_FALSE(pixels.empty());
    CHECK(CountNonZeroPixels(pixels) > 0u);
  }

  REQUIRE(harness.renderer().DestroyUploadedScene(sceneHandle).ok);
}

TEST_CASE("Renderer prepares negative view-space Z residency") {
  RenderHarness harness;
  const Status init = harness.Initialize();
  REQUIRE_MESSAGE(init.ok, init.message);

  UploadedSceneHandle sceneHandle{};
  REQUIRE(harness.renderer().CreateUploadedScene(MakeTinyScene(), sceneHandle).ok);
  RenderInput input = MakeRenderInput(64, 64);
  input.view = LookAt({0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -1.0f}, {0.0f, 1.0f, 0.0f});
  input.settings.positiveViewSpaceZ = false;
  RenderPreparationResult preparation{};
  REQUIRE(harness.renderer().PrepareSceneForRender(sceneHandle, input, harness.FrameContext(), &preparation).ok);
  CHECK(preparation.stats.residentGaussians == 2u);
  CHECK(preparation.stats.residentChunks == 1u);
}

TEST_CASE("Renderer preserves projected covariance with negative view-space Z") {
  RenderHarness harness;
  const Status init = harness.Initialize();
  REQUIRE_MESSAGE(init.ok, init.message);

  UploadedSceneHandle sceneHandle{};
  REQUIRE(harness.renderer().CreateUploadedScene(MakeAnisotropicScene(), sceneHandle).ok);

  auto render = [&](bool positiveViewSpaceZ) {
    RenderInput input = MakeRenderInput(96, 96);
    input.settings.positiveViewSpaceZ = positiveViewSpaceZ;
    if (!positiveViewSpaceZ) {
      input.view.m[10] = -input.view.m[10];
      input.proj.m[10] = -input.proj.m[10];
      input.proj.m[14] = -input.proj.m[14];
    }

    RenderPreparationResult preparation{};
    RenderResult renderResult{};
    const RenderFrameContext frameContext = harness.FrameContext();
    REQUIRE(harness.renderer().PrepareSceneForRender(sceneHandle, input, frameContext, &preparation).ok);
    OffscreenFrame frame{};
    REQUIRE(harness.CreateOffscreenFrame(96, 96, frame).ok);
    REQUIRE(harness.ResetCommandList().ok);
    REQUIRE(harness.renderer().Render(harness.commandList(), frame.binding, sceneHandle, input, frameContext, renderResult).ok);
    harness.QueueColorReadback(frame);
    REQUIRE(harness.ExecuteAndWait(renderResult.submission.uploadSyncPoint).ok);
    return harness.ReadbackColor(frame);
  };

  const std::vector<uint8_t> positiveZ = render(true);
  const std::vector<uint8_t> negativeZ = render(false);
  REQUIRE_FALSE(positiveZ.empty());
  REQUIRE_FALSE(negativeZ.empty());
  CHECK(negativeZ == positiveZ);

  REQUIRE(harness.renderer().DestroyUploadedScene(sceneHandle).ok);
}

TEST_CASE("Renderer reports dirty render errors as requiring submission") {
  RenderHarness harness;
  const Status init = harness.Initialize();
  REQUIRE_MESSAGE(init.ok, init.message);

  UploadedSceneHandle sceneHandle{};
  REQUIRE(harness.renderer().CreateUploadedScene(MakeTinyScene(), sceneHandle).ok);
  RenderPreparationResult preparation{};
  const RenderInput input = MakeRenderInput(64, 64);
  RenderFrameContext frameContext = harness.FrameContext();
  REQUIRE(harness.renderer().PrepareSceneForRender(sceneHandle, input, frameContext, &preparation).ok);

  OffscreenFrame frame{};
  REQUIRE(harness.CreateOffscreenFrame(64, 64, frame).ok);
  REQUIRE(harness.ResetCommandList().ok);

  RenderHooks hooks{};
  hooks.beforePrepare = [](const RenderHookContext&) { throw std::runtime_error("forced render hook failure"); };
  AdvancedRenderOptions options{};
  options.hooks = &hooks;
  RenderResult result{};
  const Status rendered = harness.renderer().Render(harness.commandList(), frame.binding, sceneHandle, input, options, frameContext, result);
  CHECK_FALSE(rendered.ok);
  CHECK(result.submission.submissionRequired);
  CHECK(result.submission.submissionRequired);
  REQUIRE(harness.ExecuteAndWait(result.submission.uploadSyncPoint).ok);
  CHECK(harness.renderer().Reset().ok);
}

TEST_CASE("Renderer GPU resource snapshot does not publish a direct fence lease") {
  RenderHarness harness;
  const Status init = harness.Initialize();
  REQUIRE_MESSAGE(init.ok, init.message);

  UploadedSceneHandle sceneHandle{};
  REQUIRE(harness.renderer().CreateUploadedScene(MakeTinyScene(), sceneHandle).ok);
  RenderPreparationResult preparation{};
  const RenderInput input = MakeRenderInput(64, 64);
  RenderFrameContext frameContext = harness.FrameContext();
  REQUIRE(harness.renderer().PrepareSceneForRender(sceneHandle, input, frameContext, &preparation).ok);

  UploadedSceneGpuResources resources{};
  RenderFrameContext stale = frameContext;
  stale.completedFenceValue = stale.submissionFenceValue;
  CHECK_FALSE(harness.renderer().GetUploadedSceneGpuResources(sceneHandle, stale, resources).ok);
  CHECK_FALSE(harness.renderer().AcquireUploadedSceneGpuResources(sceneHandle, stale, resources).ok);

  resources = {};
  REQUIRE(harness.renderer().GetUploadedSceneGpuResources(sceneHandle, frameContext, resources).ok);
  CHECK(resources.scene == sceneHandle);
  CHECK(resources.leaseFence == nullptr);
  CHECK(resources.leaseFenceValue == 0);
  CHECK_FALSE(resources.submission.submissionRequired);
  CHECK(resources.sceneGaussians.IsValid());
  CHECK(resources.sceneIndexToChunk.IsValid());
  CHECK_FALSE(resources.sceneGaussians.callerMayTransition);
  CHECK_FALSE(resources.sceneIndexToChunk.callerMayTransition);
  CHECK_FALSE(resources.sortedSceneIndices.IsValid());
  CHECK_FALSE(resources.visibleCounter.IsValid());
  REQUIRE(resources.chunks.size() == 1u);
  CHECK(resources.chunks.front().gaussianData.IsValid());
  CHECK_FALSE(resources.chunks.front().gaussianData.callerMayTransition);
  CHECK(harness.renderer().Reset().ok);
}

TEST_CASE("Renderer GPU resource lease holds destruction until the caller fence is signaled") {
  RenderHarness harness;
  const Status init = harness.Initialize();
  REQUIRE_MESSAGE(init.ok, init.message);

  UploadedSceneHandle sceneHandle{};
  REQUIRE(harness.renderer().CreateUploadedScene(MakeTinyScene(), sceneHandle).ok);
  RenderPreparationResult preparation{};
  const RenderInput input = MakeRenderInput(64, 64);
  const RenderFrameContext frameContext = harness.FrameContext();
  REQUIRE(harness.renderer().PrepareSceneForRender(sceneHandle, input, frameContext, &preparation).ok);

  UploadedSceneGpuResources resources{};
  REQUIRE(harness.renderer().AcquireUploadedSceneGpuResources(sceneHandle, frameContext, resources).ok);
  CHECK(resources.scene == sceneHandle);
  CHECK(resources.leaseFence == frameContext.fence);
  CHECK(resources.leaseFenceValue == frameContext.submissionFenceValue);
  CHECK(resources.submission.submissionRequired);
  CHECK(resources.sceneGaussians.IsValid());
  CHECK(resources.sceneGaussians.lifetime == GpuViewLifetime::UploadedSceneLifetime);
  CHECK(resources.sceneGaussians.access == GpuResourceAccess::ReadOnly);
  CHECK_FALSE(resources.sceneGaussians.callerMayTransition);
  CHECK_FALSE(resources.sceneGaussians.callerMayWrite);
  REQUIRE(resources.chunks.size() == 1u);
  CHECK(resources.chunks.front().gaussianData.IsValid());
  CHECK(resources.chunks.front().gaussianData.lifetime == GpuViewLifetime::UploadedSceneLifetime);
  CHECK(resources.chunks.front().gaussianData.access == GpuResourceAccess::ReadOnly);
  CHECK_FALSE(resources.chunks.front().gaussianData.callerMayTransition);
  CHECK_FALSE(resources.chunks.front().gaussianData.callerMayWrite);
  auto destroyFuture = std::async(std::launch::async, [&]() {
    return harness.renderer().DestroyUploadedScene(sceneHandle);
  });
  CHECK(destroyFuture.wait_for(std::chrono::milliseconds(25)) == std::future_status::timeout);
  REQUIRE(harness.SignalFenceOnly().ok);
  REQUIRE(destroyFuture.wait_for(std::chrono::seconds(2)) == std::future_status::ready);
  CHECK(destroyFuture.get().ok);
}

TEST_CASE("Renderer reset refuses outstanding mutation tokens and recovers after end") {
  RenderHarness harness;
  const Status init = harness.Initialize();
  REQUIRE_MESSAGE(init.ok, init.message);

  UploadedSceneHandle sceneHandle{};
  REQUIRE(harness.renderer().CreateUploadedScene(MakeTinyScene(), sceneHandle).ok);
  SceneMutationToken token{};
  REQUIRE(harness.renderer().BeginSceneMutation(sceneHandle, token).ok);
  SceneAccessInfo accessInfo{};
  REQUIRE(harness.renderer().GetSceneAccessInfo(sceneHandle, accessInfo).ok);
  CHECK(accessInfo.mutationActive);
  CHECK_FALSE(harness.renderer().Reset().ok);
  REQUIRE(harness.renderer().EndSceneMutation(token).ok);
  CHECK(harness.renderer().Reset().ok);
}

TEST_CASE("Renderer device loss wakes blocked scene access calls") {
  RenderHarness harness;
  const Status init = harness.Initialize();
  REQUIRE_MESSAGE(init.ok, init.message);

  UploadedSceneHandle sceneHandle{};
  REQUIRE(harness.renderer().CreateUploadedScene(MakeTinyScene(), sceneHandle).ok);
  SceneMutationToken token{};
  REQUIRE(harness.renderer().BeginSceneMutation(sceneHandle, token).ok);

  const RenderInput input = MakeRenderInput(64, 64);
  RenderFrameContext frameContext = harness.FrameContext();
  auto prepareFuture = std::async(std::launch::async, [&]() {
    RenderPreparationResult preparation{};
    return harness.renderer().PrepareSceneForRender(sceneHandle, input, frameContext, &preparation);
  });

  SceneMutationToken blockedToken{};
  auto mutationFuture = std::async(std::launch::async, [&]() {
    return harness.renderer().BeginSceneMutation(sceneHandle, blockedToken);
  });

  CHECK(prepareFuture.wait_for(std::chrono::milliseconds(25)) == std::future_status::timeout);
  CHECK(mutationFuture.wait_for(std::chrono::milliseconds(25)) == std::future_status::timeout);

  harness.renderer().NotifyDeviceLost();

  REQUIRE(prepareFuture.wait_for(std::chrono::seconds(2)) == std::future_status::ready);
  REQUIRE(mutationFuture.wait_for(std::chrono::seconds(2)) == std::future_status::ready);
  CHECK_FALSE(prepareFuture.get().ok);
  CHECK_FALSE(mutationFuture.get().ok);
  CHECK_FALSE(harness.renderer().EndSceneMutation(token).ok);
}

TEST_CASE("Raster device-lost shutdown releases retained resources") {
  RenderHarness harness;
  const Status init = harness.Initialize();
  REQUIRE_MESSAGE(init.ok, init.message);

  GaussianRasterPipeline raster;
  Status rasterInit = raster.Initialize(harness.device(), harness.queue(), harness.fence(), nullptr, nullptr, true);
  REQUIRE_MESSAGE(rasterInit.ok, rasterInit.message);

  Scene scene = MakeTinyScene();
  std::vector<uint64_t> chunkIds{1u};
  REQUIRE(raster.CreateOrUpdateScene(1u, scene, chunkIds).ok);
  raster.NotifyDeviceLost();
  CHECK_FALSE(raster.ShutdownDeviceLost().ok);
  CHECK(raster.Initialize(harness.device(), harness.queue(), harness.fence(), nullptr, nullptr, true).ok);
  CHECK(raster.Shutdown().ok);
}

TEST_CASE("Renderer uploads and exposes every VRAM format combination") {
  RenderHarness harness;
  const Status init = harness.Initialize();
  REQUIRE_MESSAGE(init.ok, init.message);

  const std::array<VramAttributeFormat, 3> formats{
      VramAttributeFormat::Float32,
      VramAttributeFormat::Float16,
      VramAttributeFormat::Uint8,
  };

  for (VramAttributeFormat rgbaFormat : formats) {
    for (VramAttributeFormat shFormat : formats) {
      Scene scene = MakeTinyScene();
      scene.vramFormat = {rgbaFormat, shFormat};
      UploadedSceneHandle sceneHandle{};
      REQUIRE(harness.renderer().CreateUploadedScene(scene, sceneHandle).ok);
      RenderPreparationResult preparation{};
      const RenderInput input = MakeRenderInput(48, 48);
      const RenderFrameContext frameContext = harness.FrameContext();
      REQUIRE(harness.renderer().PrepareSceneForRender(sceneHandle, input, frameContext, &preparation).ok);
      UploadedSceneGpuResources resources{};
      REQUIRE(harness.renderer().GetUploadedSceneGpuResources(sceneHandle, frameContext, resources).ok);
      CHECK(resources.vramFormat.rgbaFormat == rgbaFormat);
      CHECK(resources.vramFormat.shFormat == shFormat);
      CHECK(resources.packedStrideBytes == EstimatePackedGaussianStrideBytes(scene.vramFormat));
      CHECK(resources.sceneGaussians.strideBytes == resources.packedStrideBytes);
      REQUIRE(harness.SignalFenceOnly().ok);
      REQUIRE(harness.renderer().DestroyUploadedScene(sceneHandle).ok);
    }
  }
}

TEST_CASE("Renderer renders every VRAM format combination") {
  RenderHarness harness;
  const Status init = harness.Initialize();
  REQUIRE_MESSAGE(init.ok, init.message);

  const std::array<VramAttributeFormat, 3> formats{
      VramAttributeFormat::Float32,
      VramAttributeFormat::Float16,
      VramAttributeFormat::Uint8,
  };

  for (VramAttributeFormat rgbaFormat : formats) {
    for (VramAttributeFormat shFormat : formats) {
      Scene scene = MakeDirectionalShScene();
      scene.vramFormat = {rgbaFormat, shFormat};
      UploadedSceneHandle sceneHandle{};
      REQUIRE(harness.renderer().CreateUploadedScene(scene, sceneHandle).ok);
      RenderPreparationResult preparation{};
      RenderResult renderResult{};
      const RenderInput input = MakeRenderInput(64, 64);
      const RenderFrameContext frameContext = harness.FrameContext();
      REQUIRE(harness.renderer().PrepareSceneForRender(sceneHandle, input, frameContext, &preparation).ok);
      OffscreenFrame frame{};
      REQUIRE(harness.CreateOffscreenFrame(64, 64, frame).ok);
      REQUIRE(harness.ResetCommandList().ok);
      REQUIRE(harness.renderer().Render(harness.commandList(), frame.binding, sceneHandle, input, frameContext, renderResult).ok);
      harness.QueueColorReadback(frame);
      REQUIRE(harness.ExecuteAndWait(renderResult.submission.uploadSyncPoint).ok);
      const std::vector<uint8_t> pixels = harness.ReadbackColor(frame);
      REQUIRE_FALSE(pixels.empty());
      uint8_t peakColor = 0;
      for (size_t i = 0; i + 3u < pixels.size(); i += 4u) {
        peakColor = std::max({peakColor, pixels[i], pixels[i + 1u], pixels[i + 2u]});
      }
      CHECK(peakColor > 100u);
      CHECK(peakColor < 155u);
      REQUIRE(harness.renderer().DestroyUploadedScene(sceneHandle).ok);
    }
  }
}

TEST_CASE("Renderer final render path works") {
  RenderHarness harness;
  const Status init = harness.Initialize();
  REQUIRE_MESSAGE(init.ok, init.message);

  UploadedSceneHandle sceneHandle{};
  REQUIRE(harness.renderer().CreateUploadedScene(MakeTinyScene(), sceneHandle).ok);

  RenderPreparationResult preparation{};
  RenderResult renderResult{};
  RenderInput input = MakeRenderInput(96, 96);
  input.settings.maxAxisPixels = 512.0f;
  const RenderFrameContext frameContext = harness.FrameContext();
  REQUIRE(harness.renderer().PrepareSceneForRender(sceneHandle, input, frameContext, &preparation).ok);

  OffscreenFrame frame{};
  REQUIRE(harness.CreateOffscreenFrame(96, 96, frame).ok);
  REQUIRE(harness.ResetCommandList().ok);
  REQUIRE(harness.renderer().Render(harness.commandList(), frame.binding, sceneHandle, input, frameContext, renderResult).ok);
  harness.QueueColorReadback(frame);
  REQUIRE(harness.ExecuteAndWait(renderResult.submission.uploadSyncPoint).ok);
  const std::vector<uint8_t> pixels = harness.ReadbackColor(frame);
  REQUIRE_FALSE(pixels.empty());
  CHECK(CountNonZeroPixels(pixels) > 0u);

  REQUIRE(harness.renderer().DestroyUploadedScene(sceneHandle).ok);
}

TEST_CASE("Renderer renders color alpha and depth modes") {
  RenderHarness harness;
  const Status init = harness.Initialize();
  REQUIRE_MESSAGE(init.ok, init.message);

  UploadedSceneHandle sceneHandle{};
  REQUIRE(harness.renderer().CreateUploadedScene(MakeTinyScene(), sceneHandle).ok);

  auto renderMode = [&](RenderType type) {
    RenderPreparationResult preparation{};
    RenderResult renderResult{};
    RenderInput input = MakeRenderInput(96, 96);
    input.settings.maxAxisPixels = 512.0f;
    input.settings.renderType = type;
    const RenderFrameContext frameContext = harness.FrameContext();
    REQUIRE(harness.renderer().PrepareSceneForRender(sceneHandle, input, frameContext, &preparation).ok);

    OffscreenFrame frame{};
    REQUIRE(harness.CreateOffscreenFrame(96, 96, frame).ok);
    REQUIRE(harness.ResetCommandList().ok);
    REQUIRE(harness.renderer().Render(harness.commandList(), frame.binding, sceneHandle, input, frameContext, renderResult).ok);
    harness.QueueColorReadback(frame);
    REQUIRE(harness.ExecuteAndWait(renderResult.submission.uploadSyncPoint).ok);
    std::vector<uint8_t> pixels = harness.ReadbackColor(frame);
    REQUIRE_FALSE(pixels.empty());
    CHECK(CountNonZeroPixels(pixels) > 0u);
    return pixels;
  };

  const std::vector<uint8_t> color = renderMode(RenderType::Color);
  const std::vector<uint8_t> alpha = renderMode(RenderType::Alpha);
  const std::vector<uint8_t> depth = renderMode(RenderType::Depth);

  CHECK(ImagesDiffer(color, alpha));
  CHECK(ImagesDiffer(color, depth));

  REQUIRE(harness.renderer().DestroyUploadedScene(sceneHandle).ok);
}

TEST_CASE("Renderer survives repeated upload update render and reset cycles") {
  RenderHarness harness;
  const Status init = harness.Initialize();
  REQUIRE_MESSAGE(init.ok, init.message);

  UploadedSceneHandle sceneHandle{};
  std::vector<UploadedChunkHandle> chunks;
  REQUIRE(harness.renderer().CreateUploadedScene(MakeSceneWithColor(-0.15f, 1.0f, 0.1f, 0.1f), sceneHandle, &chunks).ok);
  REQUIRE(chunks.size() == 1u);

  for (uint32_t i = 0; i < 10u; ++i) {
    const float t = static_cast<float>(i) / 9.0f;
    Scene replacement = MakeSceneWithColor(-0.2f + t * 0.4f, 0.1f + t * 0.8f, 0.8f - t * 0.5f, 0.2f + t * 0.6f);
    REQUIRE(harness.renderer().UpdateUploadedChunk(sceneHandle, chunks.front(), replacement.splatSets.front()).ok);
    if ((i % 3u) == 1u) {
      UploadedChunkHandle extra{};
      REQUIRE(harness.renderer().AddUploadedChunk(sceneHandle, MakeSceneWithColor(0.25f, 0.1f, 0.5f, 1.0f).splatSets.front(), extra).ok);
      REQUIRE(harness.renderer().RemoveUploadedChunk(sceneHandle, extra).ok);
    }

    RenderPreparationResult preparation{};
    RenderResult renderResult{};
    const RenderInput input = MakeRenderInput(72, 72);
    const RenderFrameContext frameContext = harness.FrameContext();
    REQUIRE(harness.renderer().PrepareSceneForRender(sceneHandle, input, frameContext, &preparation).ok);
    OffscreenFrame frame{};
    REQUIRE(harness.CreateOffscreenFrame(72, 72, frame).ok);
    REQUIRE(harness.ResetCommandList().ok);
    REQUIRE(harness.renderer().Render(harness.commandList(), frame.binding, sceneHandle, input, frameContext, renderResult).ok);
    harness.QueueColorReadback(frame);
    REQUIRE(harness.ExecuteAndWait(renderResult.submission.uploadSyncPoint).ok);
    const std::vector<uint8_t> pixels = harness.ReadbackColor(frame);
    REQUIRE_FALSE(pixels.empty());
    CHECK(CountNonZeroPixels(pixels) > 0u);
  }

  REQUIRE(harness.renderer().DestroyUploadedScene(sceneHandle).ok);
  CHECK(harness.renderer().Reset().ok);
}

TEST_CASE("Renderer finalizes managed target state on dirty late render errors") {
  RenderHarness harness;
  const Status init = harness.Initialize();
  REQUIRE_MESSAGE(init.ok, init.message);

  UploadedSceneHandle sceneHandle{};
  REQUIRE(harness.renderer().CreateUploadedScene(MakeTinyScene(), sceneHandle).ok);
  RenderPreparationResult preparation{};
  const RenderInput input = MakeRenderInput(64, 64);
  const RenderFrameContext frameContext = harness.FrameContext();
  REQUIRE(harness.renderer().PrepareSceneForRender(sceneHandle, input, frameContext, &preparation).ok);

  OffscreenFrame frame{};
  REQUIRE(harness.CreateOffscreenFrame(64, 64, frame).ok);
  REQUIRE(harness.ResetCommandList().ok);

  RenderHooks hooks{};
  hooks.afterRaster = [](const RenderHookContext&) { throw std::runtime_error("forced late render hook failure"); };
  AdvancedRenderOptions options{};
  options.hooks = &hooks;
  RenderResult result{};
  const Status rendered = harness.renderer().Render(harness.commandList(), frame.binding, sceneHandle, input, options, frameContext, result);
  CHECK_FALSE(rendered.ok);
  CHECK(result.submission.submissionRequired);
  harness.QueueColorReadback(frame);
  REQUIRE(harness.ExecuteAndWait(result.submission.uploadSyncPoint).ok);
  const std::vector<uint8_t> pixels = harness.ReadbackColor(frame);
  REQUIRE_FALSE(pixels.empty());
  CHECK_MESSAGE(harness.DeviceRemovedReason() == S_OK, harness.DebugMessages());
}

TEST_CASE("Renderer initializes, uploads a tiny scene, and renders offscreen") {
  RenderHarness harness;
  const Status init = harness.Initialize();
  REQUIRE_MESSAGE(init.ok, init.message);

  UploadedSceneHandle sceneHandle{};
  std::vector<UploadedChunkHandle> chunkHandles;
  const Scene scene = MakeTinyScene();
  REQUIRE(harness.renderer().CreateUploadedScene(scene, sceneHandle, &chunkHandles).ok);
  REQUIRE(sceneHandle.IsValid());
  CHECK(chunkHandles.size() == 1u);

  RenderPreparationResult preparation{};
  RenderResult renderResult{};
  const RenderInput input = MakeRenderInput(128, 128);
  const RenderFrameContext frameContext = harness.FrameContext();
  REQUIRE(harness.renderer().PrepareSceneForRender(sceneHandle, input, frameContext, &preparation).ok);

  OffscreenFrame frame{};
  REQUIRE(harness.CreateOffscreenFrame(128, 128, frame).ok);
  REQUIRE(harness.ResetCommandList().ok);
  REQUIRE(harness.renderer().Render(harness.commandList(), frame.binding, sceneHandle, input, frameContext, renderResult).ok);
  harness.QueueColorReadback(frame);
  REQUIRE(harness.ExecuteAndWait(renderResult.submission.uploadSyncPoint).ok);
  CHECK_MESSAGE(harness.DeviceRemovedReason() == S_OK, harness.DebugMessages());

  const std::vector<uint8_t> pixels = harness.ReadbackColor(frame);
  REQUIRE_FALSE(pixels.empty());
  CHECK(CountNonZeroPixels(pixels) > 0u);
}

}  // namespace directxsplat
