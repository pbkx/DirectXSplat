# DirectXSplat

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

A C++/Direct3D 12 library for rendering 3D Gaussian Splats in D3D12.

[DirectXSplat Demo](https://github.com/user-attachments/assets/da40e9d1-d06b-4d12-b3cc-2b7a7f1459e6)

This is a C++/Direct3D 12 library for rendering scenes captured via the techniques described in [3D Gaussian Splatting for Real-Time Radiance Field Rendering](https://repo-sam.inria.fr/fungraph/3d-gaussian-splatting/). It lets you load PLY, SPZ, `.splat`, SOG, and LOD metadata scenes and render them from a host D3D12 application.

## Overview
- Fast rendering: GPU-accelerated D3D12 rasterization for high performance
- Up to 2.4x faster than [gsplat](https://github.com/nerfstudio-project/gsplat) in matched-quality resident-scene benchmarks
- Native embeddable D3D12 renderer with host-owned device, queue, command list, fences, and render targets
- Memory-conscious: persistent uploaded scenes and renderer-owned resource reuse
- No CUDA dependency for DirectXSplat rendering
- Compatible with trained 3DGS scenes in PLY, SPZ, `.splat`, SOG, and LOD metadata formats
- Convenience APIs included: interactive viewer via `Show(...)` and offscreen image capture via `Draw(...)`

## Requirements

- Windows 10/11 with a Direct3D 12-capable GPU
- Visual Studio 2022 with MSVC C++20 tools and Windows SDK
- CMake >= 3.25
- Python >= 3.10 only for benchmark scripts
- Benchmark Python packages via `pip install -r benches/requirements.txt`

## Quick Start

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 `
  -DDIRECTXSPLAT_BUILD_EXAMPLES=ON `
  -DDIRECTXSPLAT_BUILD_TESTS=ON

cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

Run the convenience viewer:

```powershell
.\build\bin\Release\DirectXSplatViewer.exe path\to\point_cloud.ply
```

## Pre-trained Dataset

This project can be used with scenes from [Gaussian Splatting](https://github.com/graphdeco-inria/gaussian-splatting). The official pre-trained models are available [here](https://repo-sam.inria.fr/fungraph/3d-gaussian-splatting/datasets/pretrained/models.zip).

Download the datasets into the `models/` directory. For a scene laid out like:

```text
models/garden/
  cameras.json
  point_cloud/iteration_30000/point_cloud.ply
```

The commands below assume the release executables are on your `PATH` or in the current directory. If you built from source, prefix them with `.\build\bin\Release\`.

run the C++ viewer with the trained 3DGS scene:

```powershell
$scene = "garden"
DirectXSplatViewer.exe "models\$scene\point_cloud\iteration_30000\point_cloud.ply"
```

To open the same scene with the dataset camera set, build examples and run:

```powershell
$scene = "garden"
DirectXSplatCameraViewerExample.exe `
  "models\$scene\point_cloud\iteration_30000\point_cloud.ply" `
  "models\$scene\cameras.json"
```

Replace `garden` with another downloaded scene, such as `bicycle`, `bonsai`, `counter`, `kitchen`, `room`, or `stump`.

## Build Library

```powershell
cmake -S . -B build-lib -G "Visual Studio 17 2022" -A x64 `
  -DDIRECTXSPLAT_BUILD_EXAMPLES=OFF `
  -DDIRECTXSPLAT_BUILD_TESTS=OFF

cmake --build build-lib --config Release --target directxsplat
```

## Use From CMake

Fetch the `v0.1.1` release:

```cmake
include(FetchContent)

FetchContent_Declare(
  DirectXSplat
  GIT_REPOSITORY https://github.com/pbkx/DirectXSplat.git
  GIT_TAG v0.1.1
)
FetchContent_MakeAvailable(DirectXSplat)

target_link_libraries(my_app PRIVATE DirectXSplat::DirectXSplat)
```

Install:

```powershell
cmake -S . -B build-install -G "Visual Studio 17 2022" -A x64 `
  -DDIRECTXSPLAT_BUILD_EXAMPLES=OFF `
  -DDIRECTXSPLAT_BUILD_TESTS=OFF `
  -DCMAKE_INSTALL_PREFIX="$PWD\install"

cmake --build build-install --config Release --target install
```

Consume:

```cmake
find_package(DirectXSplat CONFIG REQUIRED)

add_executable(my_app main.cpp)
target_link_libraries(my_app PRIVATE DirectXSplat::DirectXSplat)
target_compile_features(my_app PRIVATE cxx_std_20)
```

Configure the consuming project with the DirectXSplat install prefix:

```powershell
cmake -S my-app -B my-app-build -DCMAKE_PREFIX_PATH="$PWD\install"
cmake --build my-app-build --config Release
```

As a subproject, examples and tests are off by default:

```cmake
add_subdirectory(external/directxsplat)
target_link_libraries(my_app PRIVATE DirectXSplat::DirectXSplat)
```

## Primary API

Use the renderer API when DirectXSplat is embedded into an existing D3D12 renderer. Your application provides the D3D12 device, direct queue, command lists, fences, render targets, and frame loop. DirectXSplat records splat rendering commands into the command list you provide.

```cpp
#include <directxsplat/context.h>
#include <directxsplat/io.h>
#include <directxsplat/renderer.h>

struct DirectXSplatSession {
  directxsplat::D3D12Context context;
  directxsplat::Renderer renderer;
  directxsplat::UploadedSceneHandle scene;
};

directxsplat::Status CreateDirectXSplatSession(DirectXSplatSession& session,
                                               ID3D12Device* device,
                                               ID3D12CommandQueue* directQueue,
                                               ID3D12Fence* directFence,
                                               const char* scenePath) {
  directxsplat::Status status = session.context.Initialize(device, directQueue, directFence);
  if (!status.ok) {
    return status;
  }

  status = session.renderer.Initialize(session.context);
  if (!status.ok) {
    return status;
  }

  directxsplat::StatusOr<directxsplat::Scene> loaded = directxsplat::LoadSceneFromFile(scenePath);
  if (!loaded.ok()) {
    return loaded.status;
  }

  return session.renderer.CreateUploadedScene(loaded.value, session.scene);
}
```

Pass a reset, open direct command list to the frame function. It records the DirectXSplat work, closes the command list, submits it on the same direct queue used to initialize the context, and signals the frame fence.

```cpp
directxsplat::Status RenderDirectXSplatFrame(DirectXSplatSession& session,
                                             ID3D12GraphicsCommandList* commandList,
                                             ID3D12CommandQueue* directQueue,
                                             ID3D12Fence* directFence,
                                             ID3D12Resource* colorTarget,
                                             D3D12_CPU_DESCRIPTOR_HANDLE colorRtv,
                                             DXGI_FORMAT colorFormat,
                                             D3D12_RESOURCE_STATES colorStateBefore,
                                             D3D12_RESOURCE_STATES colorStateAfter,
                                             D3D12_VIEWPORT viewport,
                                             D3D12_RECT scissor,
                                             const directxsplat::RenderInput& input,
                                             uint64_t submissionFenceValue) {
  directxsplat::RenderFrameContext frameContext{};
  frameContext.fence = directFence;
  frameContext.completedFenceValue = directFence->GetCompletedValue();
  frameContext.submissionFenceValue = submissionFenceValue;
  frameContext.frameIndex = input.frameIndex;

  directxsplat::RenderPreparationResult preparation{};
  directxsplat::Status status =
      session.renderer.PrepareSceneForRender(session.scene, input, frameContext, &preparation);
  if (!status.ok) {
    return status;
  }

  directxsplat::RenderTargetBinding target{};
  target.colorTarget = colorTarget;
  target.colorRtv = colorRtv;
  target.colorFormat = colorFormat;
  target.colorStateBefore = colorStateBefore;
  target.colorStateAfter = colorStateAfter;
  target.transitionMode = directxsplat::ResourceTransitionMode::LibraryManaged;
  target.viewport = viewport;
  target.scissor = scissor;

  directxsplat::RenderResult result{};
  status = session.renderer.Render(commandList, target, session.scene, input, frameContext, result);
  if (!status.ok && !result.submission.submissionRequired) {
    return status;
  }

  if (result.submission.uploadSyncPoint.IsValid()) {
    HRESULT hr = directQueue->Wait(result.submission.uploadSyncPoint.fence,
                                   result.submission.uploadSyncPoint.value);
    if (FAILED(hr)) {
      session.renderer.NotifyDeviceLost();
      return directxsplat::Status::Error("failed waiting for DirectXSplat upload sync point");
    }
  }

  HRESULT hr = commandList->Close();
  if (FAILED(hr)) {
    session.renderer.NotifyDeviceLost();
    return directxsplat::Status::Error("failed closing DirectXSplat command list");
  }

  ID3D12CommandList* commandLists[] = {commandList};
  directQueue->ExecuteCommandLists(1, commandLists);
  hr = directQueue->Signal(directFence, frameContext.submissionFenceValue);
  if (FAILED(hr)) {
    session.renderer.NotifyDeviceLost();
    return directxsplat::Status::Error("failed signaling DirectXSplat frame fence");
  }

  return status;
}
```

Shutdown is explicit:

```cpp
session.renderer.DestroyUploadedScene(session.scene);
session.renderer.Shutdown();
session.context.Shutdown();
```

## Ownership and Threading

- `D3D12Context` stores non-owning pointers and does not retain COM references. Keep the supplied device, queues, and fences alive until `Renderer::Shutdown()` and `D3D12Context::Shutdown()` complete.
- `Renderer::Initialize(...)` retains internal COM references to the D3D12 objects it uses. The host still owns command allocators, command lists, queue submission order, fence values, render targets, and device-lost recovery.
- Renderer calls may come from separate host threads, but the host must synchronize its own D3D12 objects. Never record into the same command list or reset the same command allocator concurrently.
- Rendering and mutation are mutually exclusive for the same uploaded scene. `BeginSceneMutation(...)` may wait for active render encoding to finish, and every successful call must be paired with `EndSceneMutation(...)`.
- Multiple `Renderer` instances may share one context when the host coordinates command lists, queue ordering, and unique fence values across all instances.

## Frame Submission Contract

`RenderFrameContext` is required for rendering. DirectXSplat uses it to track when renderer-owned GPU resources can be reused or released.

Required fields:

- `fence` must be the same direct-queue fence passed to `D3D12Context::Initialize(...)`.
- `completedFenceValue` must be the latest known completed value for that fence, normally read from `directFence->GetCompletedValue()`.
- `submissionFenceValue` must be nonzero, unique for the submitted frame, greater than every value previously signaled on that fence, and greater than `completedFenceValue`.
- `frameIndex` should be the host frame index for the render input.

Submission rules:

- Call `PrepareSceneForRender(...)` before `Render(...)` for the same scene, input, and frame context.
- Pass a reset, open command list to `Render(...)`, then close it before submission.
- If `RenderResult::submission.uploadSyncPoint` is valid, make the direct queue wait on it before executing the command list.
- Execute the command list and signal `submissionFenceValue` on the same direct queue used to initialize the context. Signal only after all work for that frame has been submitted.
- Pass the fence's resulting completion progress back through `completedFenceValue` on future frames.
- If `Render` returns an error with `submissionRequired == true`, command-list work may already be recorded. Submit/signal it or enter device-lost cleanup.
- If queue execution or signaling fails after renderer work was recorded, call `renderer.NotifyDeviceLost()` before shutdown.
- Use `ResourceTransitionMode::LibraryManaged` when DirectXSplat should transition the target from `colorStateBefore` to `colorStateAfter`. Use `CallerManaged` when your renderer handles those transitions.

## Render Target Contract

- `colorRtv` must describe `colorTarget`, and `colorFormat` must match the RTV's single-sampled render-target format. Unsupported formats fail pipeline creation.
- With `ResourceTransitionMode::LibraryManaged`, the resource must be in `colorStateBefore` when rendering begins; DirectXSplat transitions it to `colorStateAfter`. With `CallerManaged`, the host must perform the required color, depth, and motion-vector transitions.
- `viewport` and `scissor` must be valid for the bound target and stay within its dimensions.
- Color output uses source-alpha-over blending. The host owns render-target color-space selection and presentation.
- Depth output is optional and contains approximate splat depth. When `RenderSettings::outputDepth` is enabled, provide a matching single-sampled `depthTarget`, `depthDsv`, `depthFormat`, and valid resource states.

## Renderer Configuration

```cpp
directxsplat::RendererConfig config{};
config.residencyBudgetGaussians = 16ull * 1024ull * 1024ull;
config.uploadBudgetGaussians = 650000;
config.warmUploadBudgetGaussians = 1200000;
config.maxUploadsPerFrame = 16;
config.chunkTargetSize = 65536;
config.lod0ScreenRadius = 72.0f;
config.lod1ScreenRadius = 24.0f;
config.cullScreenRadius = 0.35f;
config.residencyCacheFrames = 120;
config.enableGpuTiming = true;

renderer.Initialize(context, config);
```

Most apps can start with defaults.

## Scene Loading

Advanced renderer integrations load `directxsplat::Scene` directly:

```cpp
directxsplat::StatusOr<directxsplat::Scene> LoadSceneForDirectXSplat(const char* scenePath,
                                                                     const char* sourceImagesPath) {
  directxsplat::SceneLoadOptions options{};
  if (sourceImagesPath != nullptr) {
    options.sourceImageDirectory = sourceImagesPath;
  }
  return directxsplat::LoadSceneFromFile(scenePath, options);
}
```

`LoadSceneFromFile(...)` detects PLY, compressed PLY, SPZ, `.splat`, SOG, and hierarchical LOD metadata. The convenience loaders `LoadFromFile(...)`, `LoadFromPly(...)`, and `LoadFromSpz(...)` return `GaussianSplats` for the convenience wrappers.

## Scene Updates

Whole scene:

```cpp
directxsplat::Status status = renderer.UpdateUploadedScene(sceneHandle, newScene);
if (!status.ok) {
  return status;
}
```

Chunk mutation:

```cpp
directxsplat::SceneMutationToken token;
directxsplat::Status status = renderer.BeginSceneMutation(sceneHandle, token);
if (!status.ok) {
  return status;
}

status = renderer.AddUploadedChunk(token, gaussianSet, outChunkHandle);
directxsplat::Status endStatus = renderer.EndSceneMutation(token);
if (!status.ok) {
  return status;
}
return endStatus;
```

Available chunk operations:

- `AddUploadedChunk`
- `UpdateUploadedChunk`
- `RemoveUploadedChunk`
- `SetUploadedChunkEnabled`
- `SetUploadedChunkScalingModifier`
- `GetUploadedSceneChunks`
- `GetUploadedChunkInfo`

## GPU Resource Interop

`GetUploadedSceneGpuResources(...)` returns a non-leasing snapshot. `AcquireUploadedSceneGpuResources(...)` returns a lease for external GPU work that references renderer-owned resources.
Reset and open `externalCommandList` before recording external work. Submit every command that references the leased resources before signaling the lease fence.

```cpp
directxsplat::UploadedSceneGpuResources resources{};
directxsplat::Status status =
    renderer.AcquireUploadedSceneGpuResources(sceneHandle, frameContext, resources);
if (!status.ok) return status;

if (resources.submission.uploadSyncPoint.IsValid()) {
  HRESULT hr = directQueue->Wait(resources.submission.uploadSyncPoint.fence,
                                 resources.submission.uploadSyncPoint.value);
  if (FAILED(hr)) {
    renderer.NotifyDeviceLost();
    return directxsplat::Status::Error("failed waiting for DirectXSplat upload sync point");
  }
}

// Record external commands that reference resources into externalCommandList.

HRESULT hr = externalCommandList->Close();
if (FAILED(hr)) {
  renderer.NotifyDeviceLost();
  return directxsplat::Status::Error("failed closing external command list");
}

ID3D12CommandList* commandLists[] = {externalCommandList};
directQueue->ExecuteCommandLists(1, commandLists);

// Release the lease only after submitting every command that uses its resources.
hr = directQueue->Signal(resources.leaseFence, resources.leaseFenceValue);
if (FAILED(hr)) {
  renderer.NotifyDeviceLost();
  return directxsplat::Status::Error("failed signaling DirectXSplat resource lease fence");
}
```

Returned resources are renderer-owned. Respect `callerMayTransition` and `callerMayWrite`, and do not cache raw resource pointers beyond the snapshot or lease lifetime.
Use `GetUploadedSceneGpuResources` only for a non-leasing snapshot; use `AcquireUploadedSceneGpuResources` before recording external GPU work that references the returned resources. After submitting that external GPU work, signal `resources.leaseFence` with `resources.leaseFenceValue`.

## Convenience Wrappers

The convenience API creates the viewer or an owned runtime for you. It is useful for tools, smoke tests, screenshots, and simple applications that do not need to integrate with an existing D3D12 frame loop.

```cpp
#include <directxsplat/directxsplat.h>

auto splats = directxsplat::LoadFromPly("point_cloud.ply");
if (!splats.ok()) {
  return splats.status;
}

directxsplat::Status status = directxsplat::Show(splats.value);
if (!status.ok) {
  return status;
}
```

`Draw(...)` is an owned-runtime convenience capture path. It creates its own D3D12 runtime, renders one image, reads pixels back to the CPU, and returns `ImageRgba8`. It is not the production render loop and is not the benchmark timing path.

```cpp
#include <directxsplat/directxsplat.h>

auto splats = directxsplat::LoadFromPly("point_cloud.ply");
if (!splats.ok()) {
  return splats.status;
}

directxsplat::CameraSet cameras = directxsplat::MakeOrbitCameraSet(splats.value, 1, 512, 512);

directxsplat::DrawOptions options{};
options.width = 512;
options.height = 512;

auto image = directxsplat::Draw(splats.value, cameras.cameras[0], options);
if (!image.ok()) {
  return image.status;
}
```

## Examples

| Target | Executable | Purpose |
|---|---|---|
| `DirectXSplatHostD3D12RenderExample` | `DirectXSplatHostD3D12RenderExample.exe` | Host-owned D3D12 render submission. |
| `DirectXSplatOffscreenCaptureExample` | `DirectXSplatOffscreenCaptureExample.exe` | Direct renderer capture to PPM. |
| `DirectXSplatSceneUpdatesExample` | `DirectXSplatSceneUpdatesExample.exe` | Uploaded scene/chunk mutation. |
| `DirectXSplatGpuResourceInteropExample` | `DirectXSplatGpuResourceInteropExample.exe` | Renderer-owned GPU resource lease. |
| `directxsplat_viewer` | `DirectXSplatViewer.exe` | Convenience interactive viewer. |
| `DirectXSplatBasicViewerExample` | `DirectXSplatBasicViewerExample.exe` | Convenience scene viewer. |
| `DirectXSplatCameraViewerExample` | `DirectXSplatCameraViewerExample.exe` | Convenience viewer with camera set. |
| `DirectXSplatBasicDrawExample` | `DirectXSplatBasicDrawExample.exe` | Convenience one-image capture. |

## Benchmarks

Tested on NVIDIA GeForce RTX 4070 SUPER, Windows 11. FPS measures resident-scene, single-camera GPU render throughput; scene loading, initial upload, warmup, GPU-to-CPU readback, UI, and presentation are excluded.

| Implementation | Dataset | #imgs | Resolution | #splats | PSNR | FPS |
|---|---|---:|---:|---:|---:|---:|
| DirectXSplat | bicycle | 194 | 1237x822 | 6,131,954 | 19.17 +/- 1.71 | **382.51** |
| gsplat | bicycle | 194 | 1237x822 | 6,131,954 | 19.17 +/- 1.60 | 165.82 |
| DirectXSplat | garden | 185 | 1297x840 | 5,834,784 | **18.99 +/- 0.75** | **319.14** |
| gsplat | garden | 185 | 1297x840 | 5,834,784 | 18.96 +/- 0.74 | 131.68 |
| DirectXSplat | treehill | 141 | 1267x832 | 3,783,761 | **20.24 +/- 2.64** | **491.86** |
| gsplat | treehill | 141 | 1267x832 | 3,783,761 | 20.20 +/- 2.69 | 224.86 |

See [benches/README.md](benches/README.md) for setup, methodology, full results, and gsplat tuning details.

## Build Options

| Option | Default | Description |
|---|---:|---|
| `DIRECTXSPLAT_BUILD_TESTS` | `ON` top-level, `OFF` as subproject | Build tests. |
| `DIRECTXSPLAT_BUILD_VIEWER` | `ON` top-level, `OFF` as subproject | Build the DirectXSplat viewer executable. |
| `DIRECTXSPLAT_BUILD_EXAMPLES` | `OFF` | Build examples. |
| `DIRECTXSPLAT_BUILD_BENCHES` | `OFF` | Build benchmark harnesses. |
| `DIRECTXSPLAT_ENABLE_GPU_TESTS` | `ON` | Enable renderer GPU tests when supported. |
| `DIRECTXSPLAT_ENABLE_WARNINGS` | `ON` | Enable warning flags. |
| `DIRECTXSPLAT_WARNINGS_AS_ERRORS` | `OFF` | Treat warnings as errors. |

The previous `DXSPLAT_*` option names are still accepted as compatibility aliases.

## License

DirectXSplat is licensed under the [MIT License](LICENSE), copyright (c) 2026 pbkx.

## 3rd-Party Licenses

| Library | URL | License |
|--------------|---------|--|
| **Microsoft D3DX12 helper headers** | https://github.com/microsoft/DirectX-Headers | [MIT](https://github.com/microsoft/DirectX-Headers/blob/main/LICENSE) |
| **ghc::filesystem** | https://github.com/gulrak/filesystem | [MIT](https://github.com/gulrak/filesystem/blob/master/LICENSE) |
| **miniz** | https://github.com/richgel999/miniz | [Unlicense / Public Domain](https://github.com/richgel999/miniz/blob/master/LICENSE) |
| **stb_image** | https://github.com/nothings/stb | [MIT or Public Domain](https://github.com/nothings/stb/blob/master/LICENSE) |
| **GPUSorting** | https://github.com/b0nes164/GPUSorting | [MIT](3rdparty/GPUSorting/LICENSE) |

The bundled GPUSorting package also includes notices for incorporated third-party components:

| Project | URL | License |
|--------------|---------|--|
| **FidelityFX SDK** | https://github.com/GPUOpen-LibrariesAndSDKs/FidelityFX-SDK | [MIT](3rdparty/GPUSorting/LICENSE) |
| **CUB** | https://github.com/NVIDIA/cub | [BSD 3-Clause](3rdparty/GPUSorting/LICENSE) |
| **DirectStorage** | https://github.com/microsoft/DirectStorage | [MIT](3rdparty/GPUSorting/LICENSE) |
| **bb_segsort** | https://github.com/vtsynergy/bb_segsort | [LGPL 2.1](3rdparty/GPUSorting/LICENSE) |

Additional dependencies fetched by CMake:

| Library | URL | License |
|--------------|---------|--|
| **nlohmann/json** | https://github.com/nlohmann/json | [MIT](https://github.com/nlohmann/json/blob/develop/LICENSE.MIT) |
| **Dear ImGui** | https://github.com/ocornut/imgui | [MIT](https://github.com/ocornut/imgui/blob/master/LICENSE.txt) |
| **doctest** | https://github.com/doctest/doctest | [MIT](https://github.com/doctest/doctest/blob/master/LICENSE.txt) |
