#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include <openssl/evp.h>

namespace robinw
{
using CalibrationId = std::array<std::uint8_t, 16>;
inline constexpr std::size_t kCalibrationHeaderSize = 70;
inline constexpr std::size_t kAngleTableOffset = kCalibrationHeaderSize + 2 + 8;
inline constexpr std::size_t kAngleTableSize = 4 * 65 * 6 * 8 * 4;
inline constexpr std::size_t kCalibrationSize = kAngleTableOffset + kAngleTableSize + 512;

inline CalibrationId calibration_id(const std::vector<std::uint8_t> & packet)
{
  if (packet.size() < kCalibrationSize) {
    throw std::runtime_error("Truncated angle table");
  }
  // The packet header, device ID and reserved bytes do not affect the angles.
  std::array<std::uint8_t, 2 + kAngleTableSize> geometry;
  std::copy_n(packet.begin() + kCalibrationHeaderSize, 2, geometry.begin());
  std::copy_n(packet.begin() + kAngleTableOffset, kAngleTableSize, geometry.begin() + 2);
  std::array<std::uint8_t, 32> digest;
  std::size_t size = 0;
  if (EVP_Q_digest(nullptr, "SHA256", nullptr, geometry.data(), geometry.size(),
    digest.data(), &size) != 1 || size != digest.size())
  {
    throw std::runtime_error("Could not hash angle table");
  }
  CalibrationId id;
  std::copy_n(digest.begin(), id.size(), id.begin());
  if (id == CalibrationId{}) {
    throw std::runtime_error("Reserved angle table identifier");
  }
  return id;
}

inline std::string calibration_id_hex(const CalibrationId & id)
{
  constexpr char digits[] = "0123456789abcdef";
  std::string text;
  text.reserve(id.size() * 2);
  for (auto byte : id) {
    text.push_back(digits[byte >> 4]);
    text.push_back(digits[byte & 0xf]);
  }
  return text;
}
}  // namespace robinw
