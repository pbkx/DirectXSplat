#pragma once

#include <optional>
#include <string>
#include <vector>

#include "directxsplat/status.h"

namespace directxsplat::internal {

struct CliOptions {
  std::optional<std::string> scenePath;
  std::optional<std::string> folderTraversalPath;
  std::optional<std::string> imagePathOverride;
  std::optional<uint32_t> renderWidthOverride;
  std::optional<uint32_t> renderHeightOverride;
  bool enableDebugLayer = false;
  bool showHelp = false;
};

StatusOr<CliOptions> ParseCliOptions(const std::vector<std::string>& args);

}  // namespace directxsplat::internal
