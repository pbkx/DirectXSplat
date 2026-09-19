#include "ui/UiLayer.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <imgui.h>
#include <string>
#include <vector>

namespace directxsplat {

std::string FormatPinnedFps(float fps) {
  char buffer[96]{};
  std::snprintf(buffer, sizeof(buffer), "%-13s: %.3f", "fps", fps);
  return buffer;
}

std::string FormatPinnedSize(uint32_t width, uint32_t height) {
  char buffer[96]{};
  std::snprintf(buffer, sizeof(buffer), "%-13s: %u x %u", "size", width, height);
  return buffer;
}

std::string FormatPinnedSplats(uint64_t total) {
  char buffer[96]{};
  std::snprintf(buffer, sizeof(buffer), "%-13s: %llu", "splats", static_cast<unsigned long long>(total));
  return buffer;
}

std::string FormatPinnedVisible(uint64_t visible, uint64_t total) {
  const double percent = total > 0 ? static_cast<double>(visible) * 100.0 / static_cast<double>(total) : 0.0;
  char buffer[128]{};
  std::snprintf(buffer,
                sizeof(buffer),
                "%-13s: %llu splats (%.2f%%)",
                "visible",
                static_cast<unsigned long long>(visible),
                percent);
  return buffer;
}

std::array<const char*, 5> UiSectionLabels() {
  return {"Graphic", "Scene", "Camera", "Animation", "Statistics"};
}

std::array<const char*, 5> UiGraphicLabels() {
  return {"VSync", "Fast culling", "Gamma correction", "AA", "aa"};
}

std::array<const char*, 11> UiSceneLabels() {
  return {"Render type", "color", "alpha", "depth", "Background", "R", "G", "B", "scale", "projection", "dilation"};
}

std::array<const char*, 8> UiCameraLabels() {
  return {"move speed", "turn speed", "Acceleration", "near clip", "far clip", "Show camera frames", "frame size", "index"};
}

std::array<const char*, 2> UiAnimationLabels() {
  return {"Animation", "fps"};
}

void ClampCameraUiState(CameraUiState& state, size_t cameraCount) {
  state.frameSize = std::clamp(state.frameSize, 0.001f, 10.0f);
  if (cameraCount == 0) {
    state.index = 0;
    return;
  }
  state.index = std::clamp(state.index, 0, static_cast<int32_t>(cameraCount - 1u));
}

namespace {

constexpr float kControlsWidth = 306.0f;
constexpr float kStatsMaxWidth = 300.0f;
constexpr float kPi = 3.14159265358979323846f;
constexpr float kGraphPlotLeft = 64.0f;
constexpr float kGraphPlotRight = 8.0f;
constexpr float kGraphTickLabelWidth = 36.0f;

ImU32 GraphPanelColor() {
  return IM_COL32(17, 38, 62, 255);
}

ImU32 GraphPlotColor() {
  return IM_COL32(12, 14, 16, 255);
}

ImU32 GraphGridColor() {
  return IM_COL32(86, 92, 100, 112);
}

ImU32 GraphBorderColor() {
  return IM_COL32(83, 88, 96, 210);
}

ImU32 GraphLineColor() {
  return IM_COL32(74, 128, 190, 255);
}

ImU32 GraphBarColor() {
  return GraphLineColor();
}

uint64_t VisibleSplats(const UiFrameData& frame) {
  return frame.stats != nullptr ? frame.stats->gaussiansVisible : 0;
}

uint64_t CountSceneSplats(const Scene* scene) {
  if (scene == nullptr) {
    return 0;
  }
  uint64_t total = 0;
  for (const GaussianSet& set : scene->splatSets) {
    total += static_cast<uint64_t>(set.gaussians.size());
  }
  return total;
}

uint64_t TotalSplats(const UiFrameData& frame) {
  const uint64_t sceneSplats = CountSceneSplats(frame.scene);
  return frame.stats != nullptr && frame.stats->gaussiansTotal > 0 ? frame.stats->gaussiansTotal : sceneSplats;
}

void RenderTraversalControls(UiFrameData& frame, UiActions& actions);

float MaxGraphValue(const std::vector<float>& values) {
  float maxValue = 0.0f;
  for (float value : values) {
    maxValue = std::max(maxValue, value);
  }
  return std::max(maxValue, 1.0f);
}

float MaxHistogramValue(const HistogramData& histogram) {
  float maxValue = 0.0f;
  for (float value : histogram.bins) {
    maxValue = std::max(maxValue, value);
  }
  return maxValue;
}

struct GraphTick {
  float value = 0.0f;
  bool showLabel = true;
  std::string label;
  ImVec2 labelSize{};
};

double NiceNumber(double value, bool roundValue) {
  if (value <= 0.0f) {
    return 1.0f;
  }
  const double exponent = std::floor(std::log10(value));
  const double fraction = value / std::pow(10.0, exponent);
  double niceFraction = 1.0;
  if (roundValue) {
    if (fraction < 1.5f) {
      niceFraction = 1.0f;
    } else if (fraction < 3.0f) {
      niceFraction = 2.0f;
    } else if (fraction < 7.0f) {
      niceFraction = 5.0f;
    } else {
      niceFraction = 10.0f;
    }
  } else {
    if (fraction <= 1.0f) {
      niceFraction = 1.0f;
    } else if (fraction <= 2.0f) {
      niceFraction = 2.0f;
    } else if (fraction <= 5.0f) {
      niceFraction = 5.0f;
    } else {
      niceFraction = 10.0f;
    }
  }
  return niceFraction * std::pow(10.0, exponent);
}

std::string FormatScientificTick(double value) {
  char buffer[32]{};
  std::snprintf(buffer, sizeof(buffer), "%.0e", value);
  std::string out = buffer;
  const size_t e = out.find('e');
  if (e == std::string::npos) {
    return out;
  }
  std::string mantissa = out.substr(0, e);
  std::string exponent = out.substr(e + 1u);
  bool negative = false;
  if (!exponent.empty() && (exponent[0] == '+' || exponent[0] == '-')) {
    negative = exponent[0] == '-';
    exponent.erase(exponent.begin());
  }
  while (exponent.size() > 1u && exponent[0] == '0') {
    exponent.erase(exponent.begin());
  }
  return mantissa + (negative ? "e-" : "e") + exponent;
}

std::string FormatTick(double value, float maxWidth) {
  char buffer[32]{};
  std::snprintf(buffer, sizeof(buffer), "%g", value);
  std::string label = buffer;
  if (maxWidth > 0.0f && ImGui::CalcTextSize(label.c_str()).x > maxWidth && value != 0.0) {
    label = FormatScientificTick(value);
  }
  return label;
}

GraphTick MakeGraphTick(double value, bool showLabel, bool constrainLabel) {
  GraphTick tick{};
  tick.value = static_cast<float>(value);
  tick.showLabel = showLabel;
  if (showLabel) {
    tick.label = FormatTick(value, constrainLabel ? kGraphTickLabelWidth : 0.0f);
    tick.labelSize = ImGui::CalcTextSize(tick.label.c_str());
  }
  return tick;
}

bool ContainsTick(double minValue, double maxValue, double value) {
  return value >= minValue && value <= maxValue;
}

std::vector<GraphTick> BuildImPlotTicks(float minValue, float maxValue, float pixels, bool vertical) {
  if (minValue == maxValue) {
    return {};
  }

  const int nMinor = 10;
  const int nMajor = std::max(2, static_cast<int>(std::round(pixels / (vertical ? 300.0f : 400.0f))));
  const double niceRange = NiceNumber(static_cast<double>(maxValue - minValue) * 0.99, false);
  const double interval = NiceNumber(niceRange / static_cast<double>(nMajor - 1), true);
  if (interval <= 0.0) {
    return {};
  }

  const double graphMin = std::floor(static_cast<double>(minValue) / interval) * interval;
  const double graphMax = std::ceil(static_cast<double>(maxValue) / interval) * interval;
  bool firstMajorSet = false;
  int firstMajorIndex = 0;
  ImVec2 totalSize{};
  std::vector<GraphTick> ticks;
  for (double major = graphMin; major < graphMax + 0.5 * interval; major += interval) {
    if (major - interval < 0.0 && major + interval > 0.0) {
      major = 0.0;
    }
    if (ContainsTick(minValue, maxValue, major)) {
      if (!firstMajorSet) {
        firstMajorIndex = static_cast<int>(ticks.size());
        firstMajorSet = true;
      }
      GraphTick tick = MakeGraphTick(major, true, vertical);
      totalSize.x += tick.labelSize.x;
      totalSize.y += tick.labelSize.y;
      ticks.push_back(tick);
    }
    for (int i = 1; i < nMinor; ++i) {
      const double minor = major + i * interval / static_cast<double>(nMinor);
      if (ContainsTick(minValue, maxValue, minor)) {
        GraphTick tick = MakeGraphTick(minor, true, vertical);
        totalSize.x += tick.labelSize.x;
        totalSize.y += tick.labelSize.y;
        ticks.push_back(tick);
      }
    }
  }

  if ((!vertical && totalSize.x > pixels * 0.8f) || (vertical && totalSize.y > pixels)) {
    for (int i = firstMajorIndex - 1; i >= 0; i -= 2) {
      ticks[static_cast<size_t>(i)].showLabel = false;
    }
    for (size_t i = static_cast<size_t>(firstMajorIndex + 1); i < ticks.size(); i += 2u) {
      ticks[i].showLabel = false;
    }
  }
  return ticks;
}

void DrawRotatedText(ImDrawList* drawList, ImVec2 pos, const char* text, ImU32 color, float angle, float textScale) {
  ImFont* font = ImGui::GetFont();
  const float fontSize = ImGui::GetFontSize();
  const float scale = fontSize / font->FontSize * textScale;
  const float cosA = std::cos(angle);
  const float sinA = std::sin(angle);
  auto rotate = [&](float x, float y) {
    return ImVec2(pos.x + x * cosA - y * sinA, pos.y + x * sinA + y * cosA);
  };

  drawList->PushTextureID(font->ContainerAtlas->TexID);
  float x = 0.0f;
  for (const char* c = text; *c != '\0'; ++c) {
    const unsigned char ch = static_cast<unsigned char>(*c);
    if (ch == ' ') {
      x += font->GetCharAdvance(ch) * scale;
      continue;
    }
    const ImFontGlyph* glyph = font->FindGlyph(static_cast<ImWchar>(ch));
    if (glyph == nullptr) {
      continue;
    }
    const float x0 = x + glyph->X0 * scale;
    const float y0 = glyph->Y0 * scale;
    const float x1 = x + glyph->X1 * scale;
    const float y1 = glyph->Y1 * scale;
    drawList->PrimReserve(6, 4);
    drawList->PrimQuadUV(rotate(x0, y0),
                         rotate(x1, y0),
                         rotate(x1, y1),
                         rotate(x0, y1),
                         ImVec2(glyph->U0, glyph->V0),
                         ImVec2(glyph->U1, glyph->V0),
                         ImVec2(glyph->U1, glyph->V1),
                         ImVec2(glyph->U0, glyph->V1),
                         color);
    x += glyph->AdvanceX * scale;
  }
  drawList->PopTextureID();
}

void DrawGraphTitle(ImDrawList* drawList, ImVec2 panelMin, ImVec2 plotMin, ImVec2 plotMax, const char* title) {
  const ImVec2 titleSize = ImGui::CalcTextSize(title);
  const float titleX = plotMin.x + (plotMax.x - plotMin.x - titleSize.x) * 0.5f;
  const float titleY = panelMin.y + (plotMin.y - panelMin.y - titleSize.y) * 0.5f;
  drawList->AddText(ImVec2(titleX, titleY), ImGui::GetColorU32(ImGuiCol_Text), title);
}

float PlotY(float value, float minValue, float maxValue, float top, float bottom) {
  const float range = std::max(maxValue - minValue, 1e-6f);
  const float t = std::clamp((value - minValue) / range, 0.0f, 1.0f);
  return bottom - t * (bottom - top);
}

float PlotX(float value, float minValue, float maxValue, float left, float right) {
  const float range = std::max(maxValue - minValue, 1e-6f);
  const float t = std::clamp((value - minValue) / range, 0.0f, 1.0f);
  return left + t * (right - left);
}

void DrawGraphFrame(ImDrawList* drawList,
                    ImVec2 plotMin,
                    ImVec2 plotMax,
                    const char* rowTitle,
                    const char* columnTitle,
                    const std::vector<GraphTick>& yTicks,
                    const std::vector<GraphTick>& xTicks,
                    float yMin,
                    float yMax,
                    float xMin,
                    float xMax,
                    bool showXLabels) {
  const ImU32 plotBg = GraphPlotColor();
  const ImU32 border = GraphBorderColor();
  const ImU32 grid = GraphGridColor();
  const ImU32 text = ImGui::GetColorU32(ImGuiCol_Text);

  drawList->AddRectFilled(plotMin, plotMax, plotBg);
  drawList->AddRect(plotMin, plotMax, border);

  for (const GraphTick& tick : yTicks) {
    const float y = PlotY(tick.value, yMin, yMax, plotMin.y, plotMax.y);
    drawList->AddLine(ImVec2(plotMin.x, y), ImVec2(plotMax.x, y), grid);
    if (tick.showLabel) {
      const float labelRight = plotMin.x - 7.0f;
      drawList->AddText(ImVec2(labelRight - tick.labelSize.x, y - tick.labelSize.y * 0.5f),
                        text,
                        tick.label.c_str());
    }
  }

  for (const GraphTick& tick : xTicks) {
    const float x = PlotX(tick.value, xMin, xMax, plotMin.x, plotMax.x);
    drawList->AddLine(ImVec2(x, plotMin.y), ImVec2(x, plotMax.y), grid);
    if (showXLabels && tick.showLabel) {
      drawList->AddText(ImVec2(x - tick.labelSize.x * 0.5f, plotMax.y + 5.0f), text, tick.label.c_str());
    }
  }

  const ImVec2 rowSize = ImGui::CalcTextSize(rowTitle);
  const float plotHeight = plotMax.y - plotMin.y;
  const float rowScale = std::min(1.0f, (plotHeight - 4.0f) / std::max(rowSize.x, 1.0f));
  const float rowCenterX = plotMin.x - 48.0f;
  const float rowCenterY = plotMin.y + plotHeight * 0.5f;
  DrawRotatedText(drawList,
                  ImVec2(rowCenterX - ImGui::GetFontSize() * rowScale * 0.5f,
                         rowCenterY + rowSize.x * rowScale * 0.5f),
                  rowTitle,
                  text,
                  -kPi * 0.5f,
                  rowScale);
  if (showXLabels) {
    const ImVec2 columnSize = ImGui::CalcTextSize(columnTitle);
    drawList->AddText(ImVec2(plotMin.x + (plotMax.x - plotMin.x - columnSize.x) * 0.5f,
                             plotMax.y + ImGui::GetFontSize() + 10.0f),
                      text,
                      columnTitle);
  }
}

void BeginGraphPanel(const char* id, float height, ImVec2& contentMin, float& contentWidth) {
  ImGui::PushID(id);
  contentMin = ImGui::GetCursorScreenPos();
  contentWidth = std::max(ImGui::GetContentRegionAvail().x, 1.0f);
  ImGui::Dummy(ImVec2(contentWidth, height));
  ImGui::GetWindowDrawList()->AddRectFilled(contentMin,
                                            ImVec2(contentMin.x + contentWidth, contentMin.y + height),
                                            GraphPanelColor());
}

void EndGraphPanel() {
  ImGui::PopID();
}

void RenderLineGraph(StatisticsGraph graph, const GraphSeries& series) {
  const char* title = StatisticsGraphTitle(graph);
  const char* columnTitle = StatisticsGraphColumnTitle(graph);
  const char* rowTitle = StatisticsGraphRowTitle(graph);
  std::vector<float> values = OrderedGraphSamples(series);
  if (values.empty()) {
    values.push_back(0.0f);
  }

  ImVec2 contentMin{};
  float contentWidth = 0.0f;
  const float panelHeight = 100.0f;
  BeginGraphPanel((std::string("##panel") + title).c_str(), panelHeight, contentMin, contentWidth);

  const float dataMax = MaxGraphValue(values);
  const float yMargin = dataMax * 0.05f;
  const float yMin = graph == StatisticsGraph::Visible ? -5.0f : -yMargin;
  const float yMax = graph == StatisticsGraph::Visible ? 105.0f : dataMax + yMargin;
  const float plotHeight = 61.0f;
  const std::vector<GraphTick> yTicks = BuildImPlotTicks(yMin, yMax, plotHeight, true);
  const ImVec2 plotMin(contentMin.x + kGraphPlotLeft, contentMin.y + 30.0f);
  const ImVec2 plotMax(std::max(plotMin.x + 1.0f, contentMin.x + contentWidth - kGraphPlotRight),
                       plotMin.y + plotHeight);
  const float xMax = static_cast<float>(std::max<size_t>(values.size(), 2u) - 1u);
  const std::vector<GraphTick> xTicks = BuildImPlotTicks(0.0f, xMax, plotMax.x - plotMin.x, false);
  DrawGraphTitle(ImGui::GetWindowDrawList(), contentMin, plotMin, plotMax, title);
  DrawGraphFrame(ImGui::GetWindowDrawList(),
                 plotMin,
                 plotMax,
                 rowTitle,
                 columnTitle,
                 yTicks,
                 xTicks,
                 yMin,
                 yMax,
                 0.0f,
                 xMax,
                 false);

  std::vector<ImVec2> points;
  points.reserve(values.size());
  for (size_t i = 0; i < values.size(); ++i) {
    const float x = PlotX(static_cast<float>(i), 0.0f, xMax, plotMin.x, plotMax.x);
    const float y = PlotY(values[i], yMin, yMax, plotMin.y, plotMax.y);
    points.push_back(ImVec2(x, y));
  }
  ImGui::GetWindowDrawList()->PushClipRect(plotMin, plotMax, true);
  if (points.size() > 1u) {
    ImGui::GetWindowDrawList()->AddPolyline(points.data(),
                                            static_cast<int>(points.size()),
                                            GraphLineColor(),
                                            0,
                                            1.0f);
  } else if (!points.empty()) {
    ImGui::GetWindowDrawList()->AddCircleFilled(points[0], 1.5f, GraphLineColor());
  }
  ImGui::GetWindowDrawList()->PopClipRect();

  EndGraphPanel();
}

void RenderHistogramGraph(StatisticsGraph graph, const HistogramData& histogram) {
  const char* title = StatisticsGraphTitle(graph);
  const char* columnTitle = StatisticsGraphColumnTitle(graph);
  const char* rowTitle = StatisticsGraphRowTitle(graph);

  ImVec2 contentMin{};
  float contentWidth = 0.0f;
  const float panelHeight = 200.0f;
  BeginGraphPanel((std::string("##panel") + title).c_str(), panelHeight, contentMin, contentWidth);

  const float maxCount = MaxHistogramValue(histogram);
  const bool empty = maxCount <= 0.0f;
  const float yMin = empty ? -0.5f : 0.0f;
  const float yMax = empty ? 0.5f : maxCount;
  const float plotHeight = 126.0f;
  const std::vector<GraphTick> yTicks = BuildImPlotTicks(yMin, yMax, plotHeight, true);
  const bool alphaGraph = graph == StatisticsGraph::SplatAlphaHistogram;
  const size_t binCount = alphaGraph ? kSplatAlphaHistogramBins : kProjectionActiveThreadHistogramBins;
  const float barWidth = alphaGraph ? (1.0f / static_cast<float>(binCount)) * 0.67f : 0.67f;
  const float xMin = alphaGraph ? 0.0f - barWidth * 0.5f : 1.0f - barWidth * 0.5f;
  const float xMax =
      alphaGraph ? (static_cast<float>(binCount - 1u) / static_cast<float>(binCount)) + barWidth * 0.5f
                 : static_cast<float>(binCount) + barWidth * 0.5f;
  const ImVec2 plotMin(contentMin.x + kGraphPlotLeft, contentMin.y + 34.0f);
  const ImVec2 plotMax(std::max(plotMin.x + 1.0f, contentMin.x + contentWidth - kGraphPlotRight),
                       plotMin.y + plotHeight);
  const std::vector<GraphTick> xTicks = BuildImPlotTicks(xMin, xMax, plotMax.x - plotMin.x, false);
  DrawGraphTitle(ImGui::GetWindowDrawList(), contentMin, plotMin, plotMax, title);
  DrawGraphFrame(ImGui::GetWindowDrawList(),
                 plotMin,
                 plotMax,
                 rowTitle,
                 columnTitle,
                 yTicks,
                 xTicks,
                 yMin,
                 yMax,
                 xMin,
                 xMax,
                 true);

  const float zeroY = PlotY(0.0f, yMin, yMax, plotMin.y, plotMax.y);
  const ImU32 barColor = GraphBarColor();
  ImGui::GetWindowDrawList()->PushClipRect(plotMin, plotMax, true);
  for (size_t i = 0; i < binCount; ++i) {
    const float center = alphaGraph ? static_cast<float>(i) / static_cast<float>(binCount) : static_cast<float>(i + 1u);
    const float halfWidth = barWidth * 0.5f;
    const float x0 = PlotX(center - halfWidth, xMin, xMax, plotMin.x, plotMax.x);
    const float x1 = PlotX(center + halfWidth, xMin, xMax, plotMin.x, plotMax.x);
    const float y = PlotY(histogram.bins[i], yMin, yMax, plotMin.y, plotMax.y);
    if (histogram.bins[i] > 0.0f) {
      ImGui::GetWindowDrawList()->AddRectFilled(ImVec2(x0, y), ImVec2(std::max(x0, x1), zeroY), barColor);
    }
  }
  ImGui::GetWindowDrawList()->PopClipRect();

  EndGraphPanel();
}

void RenderStatisticsGraphs(const UiFrameData& frame) {
  ViewerGraphData emptyGraphs{};
  const ViewerGraphData& graphs = frame.graphData != nullptr ? *frame.graphData : emptyGraphs;

  RenderLineGraph(StatisticsGraph::Fps, graphs.fps);
  RenderLineGraph(StatisticsGraph::Visible, graphs.visible);
  RenderHistogramGraph(StatisticsGraph::SplatAlphaHistogram, graphs.splatAlpha);
  RenderHistogramGraph(StatisticsGraph::ProjectionActiveThreads, graphs.projectionActiveThreads);
}

void RenderGraphicSection(UiFrameData& frame) {
  const auto labels = UiGraphicLabels();
  if (frame.vsyncEnabled != nullptr) {
    ImGui::Checkbox(labels[0], frame.vsyncEnabled);
  }
  ImGui::Checkbox(labels[1], &frame.settings->fastCulling);
  ImGui::Checkbox(labels[2], &frame.settings->gammaCorrection);
  ImGui::Checkbox(labels[3], &frame.settings->antialiasing);
  ImGui::SetNextItemWidth(128.0f);
  ImGui::SliderFloat("##aastrength", &frame.settings->antialiasingStrength, 0.0f, 2.5f, "%.2f");
  ImGui::SameLine();
  ImGui::TextUnformatted(labels[4]);
}

void RenderSceneSection(UiFrameData& frame, UiActions& actions) {
  const auto labels = UiSceneLabels();
  RenderType renderType = SanitizeRenderType(frame.settings->renderType);
  ImGui::SetNextItemWidth(128.0f);
  if (ImGui::BeginCombo("##rendertype", RenderTypeLabel(renderType))) {
    const RenderType options[] = {RenderType::Color, RenderType::Alpha, RenderType::Depth};
    for (RenderType option : options) {
      const bool selected = renderType == option;
      if (ImGui::Selectable(RenderTypeLabel(option), selected)) {
        renderType = option;
        frame.settings->renderType = option;
      }
      if (selected) {
        ImGui::SetItemDefaultFocus();
      }
    }
    ImGui::EndCombo();
  }
  ImGui::SameLine();
  ImGui::TextUnformatted(labels[0]);

  float background[3] = {
      frame.settings->backgroundColor.x,
      frame.settings->backgroundColor.y,
      frame.settings->backgroundColor.z,
  };
  ImGui::SetNextItemWidth(176.0f);
  if (ImGui::ColorEdit3(labels[4], background, ImGuiColorEditFlags_Float)) {
    frame.settings->backgroundColor = {background[0], background[1], background[2]};
  }

  ImGui::SetNextItemWidth(128.0f);
  ImGui::SliderFloat("##scalemod", &frame.settings->gaussianScalingModifier, 0.001f, 2.0f, "%.3f");
  ImGui::SameLine();
  ImGui::TextUnformatted(labels[8]);
  ImGui::SetNextItemWidth(128.0f);
  ImGui::SliderFloat("##maxaxispixels", &frame.settings->maxAxisPixels, 1.0f, 512.0f, "%.0f");
  ImGui::SameLine();
  ImGui::TextUnformatted(labels[9]);
  ImGui::SetNextItemWidth(128.0f);
  ImGui::SliderFloat("##frustumdilation", &frame.settings->frustumDilation, 0.0f, 0.25f, "%.3f");
  ImGui::SameLine();
  ImGui::TextUnformatted(labels[10]);
  RenderTraversalControls(frame, actions);
}

void RenderTraversalControls(UiFrameData& frame, UiActions& actions) {
  if (!frame.traversalEnabled || !actions.prevScene || !actions.nextScene) {
    return;
  }
  if (ImGui::Button("prev")) {
    actions.prevScene();
  }
  ImGui::SameLine();
  if (ImGui::Button("next")) {
    actions.nextScene();
  }
  ImGui::SameLine();
  ImGui::Text("%llu / %llu", static_cast<unsigned long long>(frame.traversalCurrentIndex + 1),
              static_cast<unsigned long long>(frame.traversalSceneCount));
}

void RenderCameraSection(UiFrameData& frame, UiActions& actions) {
  CameraState cameraState = frame.camera->State();
  bool cameraStateChanged = false;

  const auto labels = UiCameraLabels();
  ImGui::SetNextItemWidth(128.0f);
  cameraStateChanged |= ImGui::SliderFloat("##movespeed",
                                          &cameraState.movementSpeed,
                                          0.001f,
                                          100.0f,
                                          "%.3f",
                                          ImGuiSliderFlags_Logarithmic);
  ImGui::SameLine();
  ImGui::TextUnformatted(labels[0]);
  ImGui::SetNextItemWidth(128.0f);
  cameraStateChanged |= ImGui::SliderFloat("##turnspeed",
                                          &cameraState.rotationSpeed,
                                          0.05f,
                                          5.0f,
                                          "%.2f",
                                          ImGuiSliderFlags_Logarithmic);
  ImGui::SameLine();
  ImGui::TextUnformatted(labels[1]);
  cameraStateChanged |= ImGui::Checkbox(labels[2], &cameraState.useAcceleration);
  ImGui::SetNextItemWidth(128.0f);
  if (ImGui::SliderFloat("##nearclip",
                         &cameraState.nearPlane,
                         0.001f,
                         10.0f,
                         "%.4f",
                         ImGuiSliderFlags_Logarithmic)) {
    cameraState.farPlane = std::max(cameraState.farPlane, cameraState.nearPlane + 0.001f);
    cameraStateChanged = true;
  }
  ImGui::SameLine();
  ImGui::TextUnformatted(labels[3]);
  ImGui::SetNextItemWidth(128.0f);
  const float minFarPlane = std::max(1.0f, cameraState.nearPlane + 0.001f);
  if (ImGui::SliderFloat("##farclip",
                         &cameraState.farPlane,
                         minFarPlane,
                         10000.0f,
                         "%.1f",
                         ImGuiSliderFlags_Logarithmic)) {
    cameraStateChanged = true;
  }
  ImGui::SameLine();
  ImGui::TextUnformatted(labels[4]);
  if (cameraStateChanged) {
    frame.camera->SetState(cameraState);
  }

  ImGui::Separator();
  CameraUiState localState{};
  CameraUiState& state = frame.cameraUi != nullptr ? *frame.cameraUi : localState;
  ClampCameraUiState(state, frame.cameraCount);

  const bool disabled = frame.cameraCount == 0;
  if (disabled) {
    ImGui::BeginDisabled();
  }
  ImGui::Checkbox(labels[5], &state.showCameraFrames);
  ImGui::SliderFloat(labels[6], &state.frameSize, 0.001f, 10.0f, "%.3f", ImGuiSliderFlags_Logarithmic);
  const int maxIndex = frame.cameraCount > 0 ? static_cast<int>(frame.cameraCount - 1u) : 0;
  int index = state.index;
  if (ImGui::SliderInt(labels[7], &index, 0, maxIndex)) {
    state.index = index;
    ClampCameraUiState(state, frame.cameraCount);
    if (actions.selectCamera) {
      actions.selectCamera(state.index);
    }
  }
  if (disabled) {
    ImGui::EndDisabled();
  }
}

void RenderAnimationSection(UiFrameData& frame) {
  AnimationUiState localState{};
  AnimationUiState& state = frame.animationUi != nullptr ? *frame.animationUi : localState;
  ClampAnimationUiState(state, frame.cameraCount);

  const auto labels = UiAnimationLabels();
  const bool disabled = frame.cameraCount == 0;
  if (disabled) {
    ImGui::BeginDisabled();
  }
  ImGui::Checkbox("Animation##animation_enabled", &state.enabled);
  ImGui::SliderFloat(labels[1], &state.fps, 1.0f, 30.0f, "%.2f");
  ClampAnimationUiState(state, frame.cameraCount);
  if (disabled) {
    ImGui::EndDisabled();
  }
}

void RenderPinnedStatsWindow(const UiFrameData& frame) {
  const auto labels = UiSectionLabels();
  const ImGuiIO& io = ImGui::GetIO();
  ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x - 10.0f, 10.0f), ImGuiCond_Always, ImVec2(1.0f, 0.0f));
  ImGui::SetNextWindowSizeConstraints(ImVec2(0.0f, 0.0f), ImVec2(kStatsMaxWidth, 1000.0f));
  const ImGuiWindowFlags flags = ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings |
                                 ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoTitleBar |
                                 ImGuiWindowFlags_NoCollapse;
  if (ImGui::Begin("DirectXSplatStats", nullptr, flags)) {
    const uint64_t total = TotalSplats(frame);
    ImGui::TextUnformatted(FormatPinnedFps(frame.fps).c_str());
    ImGui::TextUnformatted(FormatPinnedSize(frame.renderWidth, frame.renderHeight).c_str());
    ImGui::TextUnformatted(FormatPinnedSplats(total).c_str());
    ImGui::TextUnformatted(FormatPinnedVisible(VisibleSplats(frame), total).c_str());
    if (ImGui::CollapsingHeader(labels[4])) {
      RenderStatisticsGraphs(frame);
    }
  }
  ImGui::End();
}

void RenderLeftControlsWindow(UiFrameData& frame, UiActions& actions) {
  const auto labels = UiSectionLabels();
  ImGui::SetNextWindowPos(ImVec2(10.0f, 10.0f), ImGuiCond_Always);
  ImGui::SetNextWindowSizeConstraints(ImVec2(kControlsWidth, 0.0f), ImVec2(kControlsWidth, 1000.0f));
  const ImGuiWindowFlags flags = ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings |
                                 ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoTitleBar |
                                 ImGuiWindowFlags_NoCollapse;
  if (ImGui::Begin("DirectXSplatControls", nullptr, flags)) {
    if (ImGui::CollapsingHeader(labels[0])) {
      RenderGraphicSection(frame);
    }
    if (ImGui::CollapsingHeader(labels[1])) {
      RenderSceneSection(frame, actions);
    }
    if (ImGui::CollapsingHeader(labels[2])) {
      RenderCameraSection(frame, actions);
    }
    if (ImGui::CollapsingHeader(labels[3])) {
      RenderAnimationSection(frame);
    }
  }
  ImGui::End();
}

void PushOverlayStyle() {
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(7.0f, 5.0f));
  ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(4.0f, 3.0f));
  ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(5.0f, 2.0f));
  ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
  ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 0.0f);
  ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.025f, 0.027f, 0.025f, 0.90f));
  ImGui::PushStyleColor(ImGuiCol_TitleBg, ImVec4(0.010f, 0.011f, 0.012f, 0.96f));
  ImGui::PushStyleColor(ImGuiCol_TitleBgActive, ImVec4(0.010f, 0.011f, 0.012f, 0.96f));
  ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.070f, 0.145f, 0.235f, 0.95f));
  ImGui::PushStyleColor(ImGuiCol_FrameBgHovered, ImVec4(0.095f, 0.210f, 0.340f, 0.95f));
  ImGui::PushStyleColor(ImGuiCol_FrameBgActive, ImVec4(0.120f, 0.265f, 0.430f, 0.95f));
  ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.075f, 0.155f, 0.250f, 0.95f));
  ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.115f, 0.240f, 0.380f, 0.95f));
  ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.145f, 0.295f, 0.455f, 0.95f));
  ImGui::PushStyleColor(ImGuiCol_CheckMark, ImVec4(0.360f, 0.680f, 1.000f, 1.00f));
}

void PopOverlayStyle() {
  ImGui::PopStyleColor(10);
  ImGui::PopStyleVar(5);
}

}  

void UiLayer::Render(UiFrameData& frame, UiActions& actions) {
  if (frame.settings == nullptr || frame.camera == nullptr) {
    return;
  }

  if (frame.showMetrics != nullptr) {
    *frame.showMetrics = true;
  }

  PushOverlayStyle();
  RenderLeftControlsWindow(frame, actions);
  RenderPinnedStatsWindow(frame);
  PopOverlayStyle();
}

}  
