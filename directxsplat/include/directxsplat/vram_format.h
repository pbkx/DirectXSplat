#pragma once

#include <cstdint>

namespace directxsplat {

enum class VramAttributeFormat : uint32_t {
  Float32 = 0,
  Float16 = 1,
  Uint8 = 2,
};

struct VramFormatSettings {
  VramAttributeFormat rgbaFormat = VramAttributeFormat::Float16;
  VramAttributeFormat shFormat = VramAttributeFormat::Float16;
};

constexpr VramAttributeFormat SanitizeVramAttributeFormat(VramAttributeFormat format) {
  switch (format) {
    case VramAttributeFormat::Float32:
    case VramAttributeFormat::Float16:
    case VramAttributeFormat::Uint8:
      return format;
    default:
      return VramAttributeFormat::Float16;
  }
}

constexpr VramFormatSettings SanitizeVramFormatSettings(VramFormatSettings settings) {
  settings.rgbaFormat = SanitizeVramAttributeFormat(settings.rgbaFormat);
  settings.shFormat = SanitizeVramAttributeFormat(settings.shFormat);
  return settings;
}

constexpr uint32_t AttributeFormatSizeBytes(VramAttributeFormat format) {
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

constexpr uint32_t AlignPackedOffset4(uint32_t value) {
  return (value + 3u) & ~3u;
}

constexpr uint32_t EstimatePackedGaussianStrideBytes(VramFormatSettings settings) {
  settings = SanitizeVramFormatSettings(settings);
  constexpr uint32_t commonBytes = 24u;  // position, scale/filter, rotation
  constexpr uint32_t shCoeffCount = 45u; // Degree 1-3 RGB coefficients; SH DC is in RGBA.rgb and opacity is in RGBA.a.
  const uint32_t rgbaBytes = 4u * AttributeFormatSizeBytes(settings.rgbaFormat);
  const uint32_t shOffset = AlignPackedOffset4(commonBytes + rgbaBytes);
  const uint32_t idOffset = AlignPackedOffset4(shOffset + shCoeffCount * AttributeFormatSizeBytes(settings.shFormat));
  return AlignPackedOffset4(idOffset + 2u * sizeof(uint32_t));
}

}  // namespace directxsplat
