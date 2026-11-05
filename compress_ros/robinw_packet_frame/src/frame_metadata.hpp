#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <vector>

#include <zstd.h>

namespace robinw
{
// Zstd skippable frame ID 0xf, six-byte payload: "RBWF", version, level.
inline constexpr std::array<std::uint8_t, 12> kMetadataHeader{
  0x5f, 0x2a, 0x4d, 0x18, 6, 0, 0, 0, 'R', 'B', 'W', 'F'};
inline constexpr std::uint8_t kMetadataVersion = 1;
inline constexpr std::size_t kMetadataSize = kMetadataHeader.size() + 2;

inline void append_frame_metadata(std::vector<std::uint8_t> & data, int zstd_level)
{
  data.insert(data.end(), kMetadataHeader.begin(), kMetadataHeader.end());
  data.push_back(kMetadataVersion);
  data.push_back(static_cast<std::uint8_t>(zstd_level));
}

inline std::optional<int> frame_zstd_level(const std::vector<std::uint8_t> & data)
{
  if (data.size() < kMetadataSize) {
    return std::nullopt;
  }
  const auto metadata_offset = data.size() - kMetadataSize;
  const auto * metadata = data.data() + metadata_offset;
  if (!std::equal(kMetadataHeader.begin(), kMetadataHeader.end(), metadata) ||
    ZSTD_findFrameCompressedSize(data.data(), data.size()) != metadata_offset ||
    metadata[kMetadataHeader.size()] != kMetadataVersion)
  {
    return std::nullopt;
  }
  const int level = metadata[kMetadataSize - 1];
  if (level == 0) {
    throw std::runtime_error("Invalid recorded Zstd level");
  }
  return level;
}
}  // namespace robinw
