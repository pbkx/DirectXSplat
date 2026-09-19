#include "io/scene_loader.h"

#include "filesystem.hpp"
#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <limits>
#include <new>
#include <numeric>
#include <stdexcept>

#include <nlohmann/json.hpp>

#include "api/CameraSetInternal.h"
#include "io/common/string_util.h"
#include "io/formats/ply/loader.h"
#include "io/formats/sog/loader.h"
#include "io/formats/splat/loader.h"
#include "io/formats/spz/loader.h"

namespace directxsplat::io {

namespace fs = ghc::filesystem;

namespace {

constexpr size_t kMaxLodFiles = 1024u * 1024u;
constexpr size_t kMaxLodTreeDepth = 256u;
constexpr size_t kMaxInputCameras = 65536u;
constexpr size_t kMaxInputCameraStringBytes = 4096u;

StatusOr<fs::path> ResolveMetadataPath(const fs::path& base, const std::string& name) {
  if (name.empty()) {
    return StatusOr<fs::path>::Error("invalid metadata path");
  }
  const fs::path relative(name);
  if (relative.is_absolute() || relative.has_root_name() || relative.has_root_directory()) {
    return StatusOr<fs::path>::Error("metadata path escapes scene folder");
  }
  for (const auto& part : relative) {
    if (part == "..") {
      return StatusOr<fs::path>::Error("metadata path escapes scene folder");
    }
  }
  try {
    fs::path canonicalBase = fs::weakly_canonical(base);
    fs::path resolved = fs::weakly_canonical((base / relative).lexically_normal());
    std::string baseString = canonicalBase.lexically_normal().generic_string();
    std::string resolvedString = resolved.lexically_normal().generic_string();
#ifdef _WIN32
    std::transform(baseString.begin(), baseString.end(), baseString.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    std::transform(resolvedString.begin(), resolvedString.end(), resolvedString.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
#endif
    while (!baseString.empty() && (baseString.back() == '/' || baseString.back() == '\\')) {
      baseString.pop_back();
    }
    while (!resolvedString.empty() && (resolvedString.back() == '/' || resolvedString.back() == '\\')) {
      resolvedString.pop_back();
    }
    if (resolvedString != baseString &&
        (resolvedString.size() <= baseString.size() || resolvedString.compare(0, baseString.size(), baseString) != 0 ||
         (resolvedString[baseString.size()] != '/' && resolvedString[baseString.size()] != '\\'))) {
      return StatusOr<fs::path>::Error("metadata path escapes scene folder");
    }
    return StatusOr<fs::path>::Ok(std::move(resolved));
  } catch (const fs::filesystem_error&) {
    return StatusOr<fs::path>::Error("metadata filesystem error");
  } catch (const std::exception&) {
    return StatusOr<fs::path>::Error("metadata path resolution failed");
  }
}

std::vector<fs::path> CollectSceneCandidates(const fs::path& folder) {
  std::vector<fs::path> out;
  if (!fs::exists(folder) || !fs::is_directory(folder)) {
    return out;
  }
  for (const auto& entry : fs::directory_iterator(folder)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    const std::string name = ToLower(entry.path().filename().string());
    if (EndsWithCaseInsensitive(name, ".ply") || EndsWithCaseInsensitive(name, ".compressed.ply") ||
        EndsWithCaseInsensitive(name, ".sog") || EndsWithCaseInsensitive(name, ".spz") ||
        EndsWithCaseInsensitive(name, ".splat") || name == "meta.json" || name == "lod-meta.json") {
      out.push_back(entry.path());
    }
  }
  std::sort(out.begin(), out.end());
  return out;
}

void LoadInputCameras(const fs::path& scenePath, const std::string& imageOverrideDir, Scene& scene);
Aabb MergeBounds(const std::vector<GaussianSet>& sets);

std::string DefaultSetName(const fs::path& path) {
  const std::string lower = ToLower(path.filename().string());
  if (lower == "meta.json" && path.has_parent_path()) {
    return path.parent_path().filename().string();
  }
  if (lower == "lod-meta.json" && path.has_parent_path()) {
    return path.parent_path().filename().string();
  }
  return path.stem().string();
}

StatusOr<GaussianSet> LoadSingleSet(const fs::path& scenePath, const std::string& setName) {
  const std::string lowerName = ToLower(scenePath.filename().string());
  if (EndsWithCaseInsensitive(lowerName, ".ply") || EndsWithCaseInsensitive(lowerName, ".compressed.ply")) {
    PlyLoader ply;
    const auto result = ply.Load(scenePath.string(), setName);
    if (!result.ok()) {
      return StatusOr<GaussianSet>::Error(result.status.message);
    }
    return StatusOr<GaussianSet>::Ok(std::move(result.value.set));
  }
  if (EndsWithCaseInsensitive(lowerName, ".sog") || lowerName == "meta.json") {
    SogLoader sog;
    const auto result = sog.Load(scenePath.string(), setName);
    if (!result.ok()) {
      return StatusOr<GaussianSet>::Error(result.status.message);
    }
    return StatusOr<GaussianSet>::Ok(std::move(result.value));
  }
  if (EndsWithCaseInsensitive(lowerName, ".spz")) {
    SpzLoader spz;
    const auto result = spz.Load(scenePath.string(), setName);
    if (!result.ok()) {
      return StatusOr<GaussianSet>::Error(result.status.message);
    }
    return StatusOr<GaussianSet>::Ok(std::move(result.value));
  }
  if (EndsWithCaseInsensitive(lowerName, ".splat")) {
    SplatLoader splat;
    const auto result = splat.Load(scenePath.string(), setName);
    if (!result.ok()) {
      return StatusOr<GaussianSet>::Error(result.status.message);
    }
    return StatusOr<GaussianSet>::Ok(std::move(result.value));
  }
  return StatusOr<GaussianSet>::Error("unsupported scene format");
}

Aabb ParseBound(const nlohmann::json& boundJson) {
  Aabb bounds{};
  if (!boundJson.is_object()) {
    return bounds;
  }
  if (!boundJson.contains("min") || !boundJson.contains("max")) {
    return bounds;
  }
  const auto& minJson = boundJson["min"];
  const auto& maxJson = boundJson["max"];
  if (!minJson.is_array() || !maxJson.is_array() || minJson.size() < 3 || maxJson.size() < 3) {
    return bounds;
  }
  try {
    bounds.min = {minJson.at(0).get<float>(), minJson.at(1).get<float>(), minJson.at(2).get<float>()};
    bounds.max = {maxJson.at(0).get<float>(), maxJson.at(1).get<float>(), maxJson.at(2).get<float>()};
  } catch (const nlohmann::json::exception&) {
    return {};
  }
  bounds.valid = true;
  return bounds;
}

bool CollectFinestFileIndices(const nlohmann::json& node, std::vector<size_t>& out, size_t maxCount, size_t depth) {
  if (depth >= kMaxLodTreeDepth) {
    return false;
  }
  if (!node.is_object()) {
    return true;
  }
  if (node.contains("lods") && node["lods"].is_object()) {
    int bestLod = std::numeric_limits<int>::max();
    size_t bestFile = std::numeric_limits<size_t>::max();
    for (auto it = node["lods"].begin(); it != node["lods"].end(); ++it) {
      if (!it.value().is_object() || !it.value().contains("file") || !it.value()["file"].is_number_unsigned()) {
        continue;
      }
      int lod = std::numeric_limits<int>::max();
      try {
        lod = std::stoi(it.key());
      } catch (...) {
        continue;
      }
      if (lod < bestLod) {
        bestLod = lod;
        bestFile = static_cast<size_t>(it.value().at("file").get<uint32_t>());
      }
    }
    if (bestFile != std::numeric_limits<size_t>::max() && out.size() < maxCount) {
      out.push_back(bestFile);
    }
  }
  if (node.contains("children") && node["children"].is_array()) {
    for (const auto& child : node["children"]) {
      if (out.size() >= maxCount) {
        break;
      }
      if (!CollectFinestFileIndices(child, out, maxCount, depth + 1u)) {
        return false;
      }
    }
  }
  return true;
}

StatusOr<Scene> LoadHierarchicalLodScene(const fs::path& manifestPath, const SceneLoadOptions& options) try {
  std::ifstream file(manifestPath.string());
  if (!file.is_open()) {
    return StatusOr<Scene>::Error("failed to open lod-meta.json");
  }

  nlohmann::json json;
  try {
    file >> json;
  } catch (...) {
    return StatusOr<Scene>::Error("invalid lod-meta.json");
  }

  if (!json.contains("filenames") || !json["filenames"].is_array()) {
    return StatusOr<Scene>::Error("lod-meta.json missing filenames");
  }
  if (json["filenames"].size() > kMaxLodFiles) {
    return StatusOr<Scene>::Error("lod-meta.json has too many files");
  }

  std::vector<std::string> filenames;
  filenames.reserve(json["filenames"].size());
  for (const auto& item : json["filenames"]) {
    if (item.is_string()) {
      filenames.push_back(item.get<std::string>());
    }
  }
  if (filenames.empty()) {
    return StatusOr<Scene>::Error("lod-meta.json has no chunk files");
  }

  std::vector<size_t> selectedFiles;
  if (json.contains("tree")) {
    if (!CollectFinestFileIndices(json["tree"], selectedFiles, filenames.size(), 0)) {
      return StatusOr<Scene>::Error("lod-meta.json tree is too deep");
    }
  }
  if (selectedFiles.empty()) {
    selectedFiles.resize(filenames.size());
    std::iota(selectedFiles.begin(), selectedFiles.end(), size_t{0});
  }
  std::sort(selectedFiles.begin(), selectedFiles.end());
  selectedFiles.erase(std::unique(selectedFiles.begin(), selectedFiles.end()), selectedFiles.end());

  Scene scene{};
  scene.sourcePath = manifestPath.parent_path().string();
  scene.splatSets.reserve(selectedFiles.size() + (json.contains("environment") ? 1u : 0u));
  const fs::path base = manifestPath.parent_path();
  for (size_t fileIndex : selectedFiles) {
    if (fileIndex >= filenames.size()) {
      continue;
    }
    const auto chunkPathResult = ResolveMetadataPath(base, filenames[fileIndex]);
    if (!chunkPathResult.ok()) {
      return StatusOr<Scene>::Error(chunkPathResult.status.message);
    }
    const fs::path chunkPath = chunkPathResult.value;
    const auto setResult = LoadSingleSet(chunkPath, DefaultSetName(chunkPath));
    if (!setResult.ok()) {
      return StatusOr<Scene>::Error(setResult.status.message);
    }
    scene.splatSets.push_back(std::move(setResult.value));
  }

  if (json.contains("environment")) {
    if (!json["environment"].is_string()) {
      return StatusOr<Scene>::Error("invalid lod-meta.json environment");
    }
    const auto environmentPathResult = ResolveMetadataPath(base, json["environment"].get<std::string>());
    if (!environmentPathResult.ok()) {
      return StatusOr<Scene>::Error(environmentPathResult.status.message);
    }
    const fs::path environmentPath = environmentPathResult.value;
    const auto environmentResult = LoadSingleSet(environmentPath, DefaultSetName(environmentPath));
    if (!environmentResult.ok()) {
      return StatusOr<Scene>::Error(environmentResult.status.message);
    }
    scene.splatSets.push_back(std::move(environmentResult.value));
  }

  if (scene.splatSets.empty()) {
    return StatusOr<Scene>::Error("lod-meta.json did not resolve any loadable chunk files");
  }

  if (json.contains("tree") && json["tree"].is_object() && json["tree"].contains("bound")) {
    scene.sceneBounds = ParseBound(json["tree"]["bound"]);
  }
  if (!scene.sceneBounds.valid) {
    scene.sceneBounds = MergeBounds(scene.splatSets);
  }
  LoadInputCameras(base, options.sourceImageDirectory, scene);
  return StatusOr<Scene>::Ok(std::move(scene));
} catch (const nlohmann::json::exception&) {
  return StatusOr<Scene>::Error("invalid lod-meta.json");
} catch (const std::bad_alloc&) {
  return StatusOr<Scene>::Error("lod-meta.json allocation failed");
} catch (const std::length_error&) {
  return StatusOr<Scene>::Error("lod-meta.json allocation failed");
}

void LoadInputCameras(const fs::path& scenePath, const std::string& imageOverrideDir, Scene& scene) {
  fs::path cameraPath;
  if (fs::is_directory(scenePath)) {
    cameraPath = scenePath / "cameras.json";
  } else {
    cameraPath = scenePath.parent_path() / "cameras.json";
  }

  if (!fs::exists(cameraPath)) {
    return;
  }

  std::ifstream file(cameraPath.string());
  if (!file.is_open()) {
    return;
  }

  nlohmann::json json;
  try {
    file >> json;
  } catch (...) {
    return;
  }

  if (!json.is_array()) {
    return;
  }
  if (json.size() > kMaxInputCameras) {
    return;
  }

  auto readCameraString = [](const nlohmann::json& item, const char* name, const char* fallback) {
    auto it = item.find(name);
    if (it == item.end() || !it->is_string()) {
      return std::string(fallback);
    }
    const std::string& value = it->get_ref<const std::string&>();
    if (value.size() > kMaxInputCameraStringBytes) {
      return std::string(fallback);
    }
    return value;
  };

  StatusOr<CameraSet> loadedCameras = directxsplat::LoadCameraSet(std::filesystem::path(cameraPath.string()));
  if (loadedCameras.ok()) {
    for (size_t i = 0; i < loadedCameras.value.cameras.size(); ++i) {
      InputCamera cam = directxsplat::InputCameraFromCameraParams(loadedCameras.value.cameras[i], i);
      if (i < json.size() && json[i].is_object()) {
        cam.sourceImage = readCameraString(json[i], "image", "");
      }
      if (!imageOverrideDir.empty() && !cam.sourceImage.empty()) {
        cam.sourceImage = (fs::path(imageOverrideDir) / cam.sourceImage).string();
      }
      scene.inputCameras.push_back(std::move(cam));
    }
    return;
  }

  for (const auto& j : json) {
    if (!j.is_object()) {
      continue;
    }
    InputCamera cam{};
    try {
      cam.name = readCameraString(j, "name", "camera");
      if (j.contains("position") && j["position"].is_array() && j["position"].size() >= 3) {
        const auto& position = j.at("position");
        cam.position = {position.at(0).get<float>(), position.at(1).get<float>(), position.at(2).get<float>()};
      }
      if (j.contains("rotation") && j["rotation"].is_array() && j["rotation"].size() >= 4) {
        const auto& rotation = j.at("rotation");
        cam.rotation = Normalize({rotation.at(0).get<float>(), rotation.at(1).get<float>(),
                                  rotation.at(2).get<float>(), rotation.at(3).get<float>()});
      }
      cam.fovYRadians = j.value("fovY", cam.fovYRadians);
      cam.sourceImage = readCameraString(j, "image", "");
    } catch (const nlohmann::json::exception&) {
      continue;
    }
    if (!imageOverrideDir.empty() && !cam.sourceImage.empty()) {
      cam.sourceImage = (fs::path(imageOverrideDir) / cam.sourceImage).string();
    }
    scene.inputCameras.push_back(std::move(cam));
  }
}

Aabb MergeBounds(const std::vector<GaussianSet>& sets) {
  Aabb out{};
  for (const auto& set : sets) {
    if (!set.bounds.valid) {
      continue;
    }
    if (!out.valid) {
      out = set.bounds;
      continue;
    }
    out.min = Min(out.min, set.bounds.min);
    out.max = Max(out.max, set.bounds.max);
  }
  return out;
}

}  

StatusOr<Scene> SceneLoader::Load(const std::string& path, const SceneLoadOptions& options) const try {
  const fs::path input(path);
  if (!fs::exists(input)) {
    return StatusOr<Scene>::Error("scene path not found");
  }

  fs::path scenePath = input;
  fs::path sourcePath = input;
  if (fs::is_directory(input)) {
    const auto candidates = CollectSceneCandidates(input);
    if (candidates.empty()) {
      return StatusOr<Scene>::Error("no supported scene file found in directory");
    }
    scenePath = candidates.front();
  }

  const std::string lowerName = ToLower(scenePath.filename().string());
  if (lowerName == "lod-meta.json") {
    return LoadHierarchicalLodScene(scenePath, options);
  }

  Scene scene{};
  scene.sourcePath = sourcePath.string();
  const auto setResult = LoadSingleSet(scenePath, DefaultSetName(scenePath));
  if (!setResult.ok()) {
    return StatusOr<Scene>::Error(setResult.status.message);
  }
  scene.splatSets.push_back(std::move(setResult.value));

  scene.sceneBounds = MergeBounds(scene.splatSets);
  LoadInputCameras(scenePath, options.sourceImageDirectory, scene);

  return StatusOr<Scene>::Ok(std::move(scene));
} catch (const fs::filesystem_error&) {
  return StatusOr<Scene>::Error("scene filesystem error");
} catch (const std::bad_alloc&) {
  return StatusOr<Scene>::Error("scene allocation failed");
} catch (const std::length_error&) {
  return StatusOr<Scene>::Error("scene allocation failed");
} catch (const std::exception&) {
  return StatusOr<Scene>::Error("scene load failed");
}

}
