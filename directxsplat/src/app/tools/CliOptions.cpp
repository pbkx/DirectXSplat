#include "tools/CliOptions.h"

#include <charconv>

namespace directxsplat::internal {

namespace {

std::optional<uint32_t> ParseUint(std::string_view s) {
  uint32_t v = 0;
  auto [ptr, ec] = std::from_chars(s.data(), s.data() + s.size(), v);
  if (ec != std::errc() || ptr != s.data() + s.size()) {
    return std::nullopt;
  }
  return v;
}

}  

StatusOr<CliOptions> ParseCliOptions(const std::vector<std::string>& args) {
  CliOptions out{};
  std::vector<std::string> positional;

  for (size_t i = 0; i < args.size(); ++i) {
    const std::string& arg = args[i];
    if (arg == "--help" || arg == "-h") {
      out.showHelp = true;
      continue;
    }
    if (arg == "--debug-layer") {
      out.enableDebugLayer = true;
      continue;
    }
    if (arg == "--images-path" && i + 1 < args.size()) {
      out.imagePathOverride = args[++i];
      continue;
    }
    if (arg == "--scene-folder" && i + 1 < args.size()) {
      out.folderTraversalPath = args[++i];
      continue;
    }
    if (arg == "--render-size" && i + 1 < args.size()) {
      const std::string s = args[++i];
      const size_t x = s.find('x');
      if (x == std::string::npos) {
        return StatusOr<CliOptions>::Error("invalid --render-size format");
      }
      auto w = ParseUint(s.substr(0, x));
      auto h = ParseUint(s.substr(x + 1));
      if (!w.has_value() || !h.has_value() || *w == 0 || *h == 0) {
        return StatusOr<CliOptions>::Error("invalid --render-size value");
      }
      out.renderWidthOverride = *w;
      out.renderHeightOverride = *h;
      continue;
    }
    if (!arg.empty() && arg[0] == '-' && arg != "-") {
      return StatusOr<CliOptions>::Error("unknown option: " + arg);
    }

    if (!arg.empty()) {
      positional.push_back(arg);
      continue;
    }
  }

  if (!positional.empty()) {
    std::string scenePath = positional.front();
    for (size_t i = 1; i < positional.size(); ++i) {
      scenePath += " ";
      scenePath += positional[i];
    }
    out.scenePath = std::move(scenePath);
  }

  return StatusOr<CliOptions>::Ok(std::move(out));
}

}  // namespace directxsplat::internal
