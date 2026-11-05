#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>
#if defined(__aarch64__) && defined(__ARM_NEON) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#include <arm_neon.h>
#endif

namespace robinw::compact
{
using Bytes = std::vector<std::uint8_t>;
inline constexpr std::size_t kBlockHeader = 10;
inline constexpr std::size_t kSingle = 42;
inline constexpr std::size_t kDouble = 74;
inline constexpr std::uint32_t kRadiusMask = (1U << 18) - 1;
inline constexpr std::uint8_t kMagic[] = {'R', 'B', 'P', '3'};

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
  std::size_t first_tail = 0;
#if defined(__aarch64__) && defined(__ARM_NEON) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  const auto remainder = bits % 8;
  if (remainder != 0) {
    const auto shift = vdupq_n_s32(-static_cast<int>(8 * whole_bytes));
    for (; first_tail + 8 <= count; first_tail += 8) {
      const auto * src = values.data() + first_tail;
      const auto lower = vmovn_u32(vshlq_u32(vld1q_u32(src), shift));
      const auto upper = vmovn_u32(vshlq_u32(vld1q_u32(src + 4), shift));
      const auto bytes = vmovn_u16(vcombine_u16(lower, upper));
      auto lanes = vget_lane_u64(vreinterpret_u64_u8(bytes), 0);
      // Transpose eight bytes so each output byte contains one complete bit plane.
      auto swaps = (lanes ^ (lanes >> 7)) & UINT64_C(0x00aa00aa00aa00aa);
      lanes ^= swaps ^ (swaps << 7);
      swaps = (lanes ^ (lanes >> 14)) & UINT64_C(0x0000cccc0000cccc);
      lanes ^= swaps ^ (swaps << 14);
      swaps = (lanes ^ (lanes >> 28)) & UINT64_C(0x00000000f0f0f0f0);
      lanes ^= swaps ^ (swaps << 28);
      auto * plane = dest + first_tail / 8;
      for (unsigned bit = 0; bit < remainder; ++bit) {
        plane[bit * mask_bytes] = static_cast<std::uint8_t>(lanes);
        lanes >>= 8;
      }
    }
  }
#endif
  for (unsigned bit = 0; bit < bits % 8; ++bit) {
    for (std::size_t i = first_tail; i < count; ++i) {
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
      throw std::runtime_error("Truncated RBP3 block stream");
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

inline std::uint32_t zigzag(std::int32_t value)
{
  return (std::uint32_t(value < 0 ? -value : value) << 1) - (value < 0);
}

inline std::int32_t wrapped(std::int32_t value, unsigned bits)
{
  const auto half = std::int32_t(1) << (bits - 1);
  return ((value + half) & (2 * half - 1)) - half;
}

inline void encode_blocks(const Bytes & blocks, std::size_t width, Bytes & output)
{
  const std::size_t count = blocks.size() / width;
  if (count == 0) {
    return;
  }
  const auto slots = (width - kBlockHeader) / 4;
  // Block headers remain in wire order so the decoder can derive the scan permutation.
  const auto order = scan_order(blocks, width);
  std::vector<std::uint32_t> delta(count);
  for (std::size_t offset = 0; offset < kBlockHeader; offset += 2) {
    std::int32_t previous = 0;
    for (std::size_t i = 0; i < count; ++i) {
      const auto word = static_cast<std::int32_t>(read_word(blocks.data() + i * width + offset, 2));
      delta[i] = zigzag(wrapped(word - previous, 16));
      previous = word;
    }
    append_planes(output, delta, 16);
  }
  const auto masks_start = output.size();
  output.resize(masks_start + slots * bitmap_size(count), 0);
  std::vector<std::uint32_t> low, high, reflectance, flags;
  low.reserve(slots * count);
  high.reserve(slots * count);
  reflectance.reserve(slots * count);
  flags.reserve(slots * count);
  for (std::size_t slot = 0; slot < slots; ++slot) {
    auto * mask = output.data() + masks_start + slot * bitmap_size(count);
    std::uint32_t previous_line = UINT32_MAX, previous_valid_line = UINT32_MAX;
    std::int32_t previous_reflectance = 0, previous_radius = 0;
    for (std::size_t i = 0; i < count; ++i) {
      const auto * block = blocks.data() + std::uint32_t(order[i]) * width;
      const auto line = std::uint32_t(order[i] >> 48);
      const auto value = read_word(block + kBlockHeader + slot * 4, 4);
      const auto reflect = static_cast<std::int32_t>(value & 0xfff);
      reflectance.push_back(zigzag(wrapped(reflect - (line == previous_line ? previous_reflectance : 0), 12)));
      flags.push_back(value >> 30);
      previous_reflectance = reflect;
      previous_line = line;
      const auto radius = static_cast<std::int32_t>((value >> 12) & kRadiusMask);
      if (radius == 0) {
        continue;
      }
      const auto residual = zigzag(radius - (line == previous_valid_line ? previous_radius : 0));
      set_bit(mask, i, 1);
      low.push_back(residual & 0x7f);
      high.push_back(residual >> 7);
      previous_valid_line = line;
      previous_radius = radius;
    }
  }
  // All channels share the field streams. Nonzero high parts are stored as value minus one.
  append_planes(output, low, 7);
  const auto high_mask_start = output.size();
  output.resize(high_mask_start + bitmap_size(high.size()), 0);
  auto * high_mask = output.data() + high_mask_start;
  std::vector<std::uint32_t> sparse;
  for (std::size_t i = 0; i < high.size(); ++i) {
    if (high[i] != 0) {
      set_bit(high_mask, i, 1);
      sparse.push_back(high[i] - 1);
    }
  }
  append_planes(output, sparse, 12);
  append_planes(output, reflectance, 12);
  append_planes(output, flags, 2);
}

inline std::int32_t unzigzag(std::uint32_t value)
{
  return std::int32_t(value >> 1) ^ -std::int32_t(value & 1U);
}

inline Bytes decode_blocks(Reader & in, std::size_t count, std::size_t width)
{
  Bytes blocks(count * width);
  if (count == 0) {
    return blocks;
  }
  const auto slots = (width - kBlockHeader) / 4;
  std::vector<std::uint32_t> words;
  for (std::size_t offset = 0; offset < kBlockHeader; offset += 2) {
    in.read_planes(words, count, 16);
    std::int32_t previous = 0;
    for (std::size_t i = 0; i < count; ++i) {
      previous = (previous + unzigzag(words[i])) & 0xffff;
      blocks[i * width + offset] = previous;
      blocks[i * width + offset + 1] = previous >> 8;
    }
  }

  const auto order = scan_order(blocks, width);
  const auto mask_width = bitmap_size(count);
  const auto * occupancy = in.take(mask_width * slots);
  std::size_t valid = 0;
  for (std::size_t slot = 0; slot < slots; ++slot) {
    for (std::size_t i = 0; i < count; ++i) {
      valid += bit_at(occupancy + slot * mask_width, i);
    }
  }
  std::vector<std::uint32_t> low, sparse, reflectance, flags;
  in.read_planes(low, valid, 7);
  const auto * high_mask = in.take(bitmap_size(valid));
  std::size_t high_count = 0;
  for (std::size_t i = 0; i < valid; ++i) {
    high_count += bit_at(high_mask, i);
  }
  in.read_planes(sparse, high_count, 12);
  in.read_planes(reflectance, count * slots, 12);
  in.read_planes(flags, count * slots, 2);

  std::size_t valid_index = 0, high_index = 0;
  for (std::size_t slot = 0; slot < slots; ++slot) {
    std::uint32_t previous_line = UINT32_MAX, previous_valid_line = UINT32_MAX;
    std::int32_t previous_radius = 0, previous_reflectance = 0;
    const auto * mask = occupancy + slot * mask_width;
    for (std::size_t i = 0; i < count; ++i) {
      const auto line = std::uint32_t(order[i] >> 48);
      const auto reflect = (unzigzag(reflectance[slot * count + i]) +
        (line == previous_line ? previous_reflectance : 0)) & 0xfff;
      previous_reflectance = reflect;
      previous_line = line;

      std::int32_t radius = 0;
      if (bit_at(mask, i)) {
        auto residual = low[valid_index];
        if (bit_at(high_mask, valid_index)) {
          residual |= (sparse[high_index++] + 1) << 7;
        }
        ++valid_index;
        radius = unzigzag(residual) + (line == previous_valid_line ? previous_radius : 0);
        if (radius <= 0 || std::uint32_t(radius) > kRadiusMask) {
          throw std::runtime_error("Invalid RBP3 distance residual");
        }
        previous_radius = radius;
        previous_valid_line = line;
      }
      auto * block = blocks.data() + std::uint32_t(order[i]) * width;
      const auto value = std::uint32_t(reflect) | (std::uint32_t(radius) << 12) |
        (flags[slot * count + i] << 30);
      for (unsigned byte = 0; byte < 4; ++byte) {
        block[kBlockHeader + slot * 4 + byte] = value >> (8 * byte);
      }
    }
  }
  return blocks;
}

}  // namespace robinw::compact
