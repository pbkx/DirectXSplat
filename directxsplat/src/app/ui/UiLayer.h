#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>

#include "app/CameraController.h"
#include "app/CameraPathAnimator.h"
#include "directxsplat/scene.h"
#include "directxsplat/settings.h"
#include "metrics/ImageMetrics.h"

namespace directxsplat {

struct UiActions {
  std::function<void()> nextScene;
  std::function<void()> prevScene;
  std::function<void(int32_t)> selectCamera;
};

struct CameraUiState {
  bool showCameraFrames = false;
  float frameSize = 0.1f;
  int32_t index = 0;
};

struct UiFrameData {
  RenderSettings* settings = nullptr;
  CameraUiState* cameraUi = nullptr;
  AnimationUiState* animationUi = nullptr;
  int32_t* selectedInputCamera = nullptr;
  uint32_t* renderWidthOverride = nullptr;
  uint32_t* renderHeightOverride = nullptr;
  bool* vsyncEnabled = nullptr;
  bool* paused = nullptr;
  bool* showMetrics = nullptr;
  bool* guiVisible = nullptr;
  bool fullscreen = false;
  std::string gpuName;
  CameraController* camera = nullptr;
  const Scene* scene = nullptr;
  FrameStats* stats = nullptr;
  const ViewerGraphData* graphData = nullptr;
  float fps = 0.0f;
  float frameMs = 0.0f;
  uint32_t renderWidth = 0;
  uint32_t renderHeight = 0;
  size_t cameraCount = 0;
  bool traversalEnabled = false;
  size_t traversalSceneCount = 0;
  size_t traversalCurrentIndex = 0;
  std::string statusMessage;
  ImageComparison comparison{};
  bool comparisonAvailable = false;
};

std::string FormatPinnedFps(float fps);
std::string FormatPinnedSize(uint32_t width, uint32_t height);
std::string FormatPinnedSplats(uint64_t total);
std::string FormatPinnedVisible(uint64_t visible, uint64_t total);
std::array<const char*, 5> UiSectionLabels();
std::array<const char*, 5> UiGraphicLabels();
std::array<const char*, 11> UiSceneLabels();
std::array<const char*, 8> UiCameraLabels();
std::array<const char*, 2> UiAnimationLabels();
void ClampCameraUiState(CameraUiState& state, size_t cameraCount);

class UiLayer {
 public:
  void Render(UiFrameData& frame, UiActions& actions);
};

}  // namespace directxsplat
