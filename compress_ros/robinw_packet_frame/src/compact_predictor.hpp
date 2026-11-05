#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace robinw::compact
{
using Bytes = std::vector<std::uint8_t>;
inline constexpr std::size_t kBlockHeader = 10;
inline constexpr std::size_t kSingle = 42;
inline constexpr std::size_t kDouble = 74;
inline constexpr std::uint32_t kRadiusMask = (1U << 18) - 1;
inline constexpr std::uint8_t kMagic[] = {'R', 'B', 'P', '2'};

inline std::uint32_t read_word(const std::uint8_t * data, std::size_t bytes)
{
  std::uint32_t result = 0;
  for (std::size_t byte = 0; byte < bytes; ++byte) {
    result |= std::uint32_t(data[byte]) << (8 * byte);
  }
  return result;
}

inline std::uint32_t scan_key(const std::uint8_t * header)
{
  const auto line = read_word(header + 8, 2) & 0x39ffU;
  return (line << 16) | read_word(header + 6, 2);
}

inline std::vector<std::uint64_t> scan_order(const Bytes & blocks, std::size_t width)
{
  std::vector<std::uint64_t> order(blocks.size() / width);
  for (std::size_t i = 0; i < order.size(); ++i) {
    order[i] = (std::uint64_t(scan_key(blocks.data() + i * width)) << 32) | i;
  }
  std::sort(order.begin(), order.end());
  return order;
}

inline std::size_t bitmap_size(std::size_t count) { return (count + 7) / 8; }

inline std::uint32_t bit_at(const std::uint8_t * bitmap, std::size_t i)
{
  return (bitmap[i / 8] >> (i % 8)) & 1U;
}

inline void set_bit(std::uint8_t * bitmap, std::size_t i, std::uint32_t bit)
{
  bitmap[i / 8] |= (bit & 1U) << (i % 8);
}

inline void append_planes(Bytes & out, const std::vector<std::uint32_t> & values, unsigned bits)
{
  if (values.empty()) {
    return;
  }
  const auto count = values.size();
  const auto whole_bytes = bits / 8;
  const auto mask_bytes = bitmap_size(count);
  const auto start = out.size();
  out.resize(start + whole_bytes * count + (bits % 8) * mask_bytes, 0);
  auto * dest = out.data() + start;
  for (unsigned byte = 0; byte < whole_bytes; ++byte) {
    for (std::size_t i = 0; i < count; ++i) {
      dest[byte * count + i] = values[i] >> (8 * byte);
    }
  }
  dest += whole_bytes * count;
  for (unsigned bit = 0; bit < bits % 8; ++bit) {
    for (std::size_t i = 0; i < count; ++i) {
      set_bit(dest + bit * mask_bytes, i, values[i] >> (8 * whole_bytes + bit));
    }
  }
}

class Reader
{
public:
  Reader(const std::uint8_t * data, std::size_t size) : current_(data), remaining_(size) {}
  std::size_t remaining() const { return remaining_; }

  const std::uint8_t * take(std::size_t bytes)
  {
    if (bytes > remaining_) {
      throw std::runtime_error("Truncated RBP2 block stream");
    }
    const auto * result = current_;
    current_ += bytes;
    remaining_ -= bytes;
    return result;
  }

  void read_planes(std::vector<std::uint32_t> & values, std::size_t count, unsigned bits)
  {
    const auto whole_bytes = bits / 8;
    const auto mask_bytes = bitmap_size(count);
    const auto * bytes = take(whole_bytes * count);
    const auto * masks = take((bits % 8) * mask_bytes);
    values.assign(count, 0);
    for (unsigned byte = 0; byte < whole_bytes; ++byte) {
      for (std::size_t i = 0; i < count; ++i) {
        values[i] |= std::uint32_t(bytes[byte * count + i]) << (8 * byte);
      }
    }
    for (unsigned bit = 0; bit < bits % 8; ++bit) {
      for (std::size_t i = 0; i < count; ++i) {
        values[i] |= bit_at(masks + bit * mask_bytes, i) << (8 * whole_bytes + bit);
      }
    }
  }

private:
  const std::uint8_t * current_;
  std::size_t remaining_;
};

inline void encode_blocks(const Bytes & blocks, std::size_t width, Bytes & out)
{
  const auto count = blocks.size() / width;
  if (count == 0) {
    return;
  }
  const auto start = out.size();
  out.resize(start + kBlockHeader * count);
  for (std::size_t byte = 0; byte < kBlockHeader; ++byte) {
    for (std::size_t i = 0; i < count; ++i) {
      out[start + byte * count + i] = blocks[i * width + byte];
    }
  }
  const auto order = scan_order(blocks, width);
  std::vector<std::uint32_t> reflectance(count), flags(count), deltas;
  deltas.reserve(count);
  for (std::size_t slot = kBlockHeader; slot < width; slot += 4) {
    const auto mask_start = out.size();
    out.resize(mask_start + bitmap_size(count), 0);
    auto * mask = out.data() + mask_start;
    deltas.clear();
    std::uint32_t previous_line = UINT32_MAX;
    std::int32_t previous_radius = 0;
    for (std::size_t i = 0; i < count; ++i) {
      const auto value = read_word(blocks.data() + std::uint32_t(order[i]) * width + slot, 4);
      const auto radius = static_cast<std::int32_t>((value >> 12) & kRadiusMask);
      reflectance[i] = value & 0xfffU;
      flags[i] = value >> 30;
      if (radius == 0) {
        continue;
      }
      set_bit(mask, i, 1);
      const auto line = static_cast<std::uint32_t>(order[i] >> 48);
      const auto delta = radius - (line == previous_line ? previous_radius : 0);
      const auto magnitude = static_cast<std::uint32_t>(delta < 0 ? -delta : delta);
      deltas.push_back((magnitude << 1) - (delta < 0));
      previous_line = line;
      previous_radius = radius;
    }
    append_planes(out, deltas, 19);
    append_planes(out, reflectance, 12);
    append_planes(out, flags, 2);
  }
}

inline Bytes decode_blocks(Reader & in, std::size_t count, std::size_t width)
{
  Bytes blocks(count * width);
  if (count == 0) {
    return blocks;
  }
  const auto * headers = in.take(count * kBlockHeader);
  for (std::size_t byte = 0; byte < kBlockHeader; ++byte) {
    for (std::size_t i = 0; i < count; ++i) {
      blocks[i * width + byte] = headers[byte * count + i];
    }
  }
  const auto order = scan_order(blocks, width);
  std::vector<std::uint32_t> deltas, reflectance, flags;
  for (std::size_t slot = kBlockHeader; slot < width; slot += 4) {
    const auto * mask = in.take(bitmap_size(count));
    std::size_t used = 0;
    for (std::size_t i = 0; i < count; ++i) {
      used += bit_at(mask, i);
    }
    in.read_planes(deltas, used, 19);
    in.read_planes(reflectance, count, 12);
    in.read_planes(flags, count, 2);
    std::size_t next_delta = 0;
    std::uint32_t previous_line = UINT32_MAX;
    std::int32_t previous_radius = 0;
    for (std::size_t i = 0; i < count; ++i) {
      std::int32_t radius = 0;
      if (bit_at(mask, i)) {
        const auto line = static_cast<std::uint32_t>(order[i] >> 48);
        const auto encoded = deltas[next_delta++];
        const auto delta = static_cast<std::int32_t>(encoded >> 1) ^
          -static_cast<std::int32_t>(encoded & 1U);
        radius = (line == previous_line ? previous_radius : 0) + delta;
        if (radius <= 0 || static_cast<std::uint32_t>(radius) > kRadiusMask) {
          throw std::runtime_error("Invalid RBP2 distance residual");
        }
        previous_line = line;
        previous_radius = radius;
      }
      const auto value = reflectance[i] | (std::uint32_t(radius) << 12) | (flags[i] << 30);
      auto * dest = blocks.data() + std::uint32_t(order[i]) * width + slot;
      for (unsigned byte = 0; byte < 4; ++byte) {
        dest[byte] = value >> (8 * byte);
      }
    }
  }
  return blocks;
}
}  // namespace robinw::compact
