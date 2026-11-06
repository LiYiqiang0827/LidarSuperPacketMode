#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
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


inline std::vector<std::uint64_t> encoder_scan_order(const Bytes & blocks, std::size_t width)
{
  constexpr std::size_t kBins = 1024;
  std::array<std::array<std::size_t, kBins>, 3> counts{};
  std::vector<std::uint64_t> order(blocks.size() / width);
  for (std::size_t i = 0; i < order.size(); ++i) {
    const auto key = scan_key(blocks.data() + i * width);
    order[i] = (std::uint64_t(key) << 32) | i;
    for (unsigned pass = 0; pass < counts.size(); ++pass) {
      ++counts[pass][(key >> (10 * pass)) & (kBins - 1)];
    }
  }
  std::vector<std::uint64_t> scratch(order.size());
  for (unsigned pass = 0; pass < counts.size(); ++pass) {
    auto & offsets = counts[pass];
    std::size_t prefix = 0;
    bool all_same = false;
    for (auto & offset : offsets) {
      all_same |= offset == order.size();
      const auto count = offset;
      offset = prefix;
      prefix += count;
    }
    if (all_same) {
      continue;
    }
    for (const auto value : order) {
      const auto bucket = (value >> (32 + 10 * pass)) & (kBins - 1);
      scratch[offsets[bucket]++] = value;
    }
    order.swap(scratch);
  }
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

template<class Word>
inline void append_planes(Bytes & out, const Word * values, std::size_t count, unsigned bits)
{
  if (count == 0) {
    return;
  }
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
      const auto * src = values + first_tail;
      uint8x8_t bytes;
      if constexpr (sizeof(Word) == sizeof(std::uint16_t)) {
        const auto shift16 = vdupq_n_s16(-static_cast<int>(8 * whole_bytes));
        bytes = vmovn_u16(vshlq_u16(vld1q_u16(src), shift16));
      } else {
        const auto lower = vmovn_u32(vshlq_u32(vld1q_u32(src), shift));
        const auto upper = vmovn_u32(vshlq_u32(vld1q_u32(src + 4), shift));
        bytes = vmovn_u16(vcombine_u16(lower, upper));
      }
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
  const auto order = encoder_scan_order(blocks, width);
  std::vector<std::uint32_t> delta(count);
  for (std::size_t offset = 0; offset < kBlockHeader; offset += 2) {
    std::int32_t previous = 0;
    for (std::size_t i = 0; i < count; ++i) {
      const auto word = static_cast<std::int32_t>(read_word(blocks.data() + i * width + offset, 2));
      delta[i] = zigzag(wrapped(word - previous, 16));
      previous = word;
    }
    append_planes(output, delta.data(), delta.size(), 16);
  }
  const auto masks_start = output.size();
  output.resize(masks_start + slots * bitmap_size(count), 0);
  const auto points = slots * count;
  // The emitted streams contain at most 12 bits per value.
  std::unique_ptr<std::uint16_t[]> low(new std::uint16_t[points]);
  std::unique_ptr<std::uint16_t[]> sparse(new std::uint16_t[points]);
  std::unique_ptr<std::uint16_t[]> reflectance(new std::uint16_t[points]);
  std::unique_ptr<std::uint16_t[]> flags(new std::uint16_t[points]);
  Bytes high_mask(bitmap_size(points), 0);
  std::size_t valid_count = 0, high_count = 0;
  // Reset the predictors once per scan line and reuse the ordered point addresses.
  struct ScanLine
  {
    std::size_t end;
  };
  std::vector<const std::uint8_t *> ordered_points;
  std::vector<ScanLine> scan_lines;
  ordered_points.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    ordered_points.push_back(blocks.data() + std::uint32_t(order[i]) * width + kBlockHeader);
    if (i + 1 == count || (order[i] >> 48) != (order[i + 1] >> 48)) {
      scan_lines.push_back({i + 1});
    }
  }
  for (std::size_t slot = 0; slot < slots; ++slot) {
    auto * mask = output.data() + masks_start + slot * bitmap_size(count);
    auto * reflect_out = reflectance.get() + slot * count;
    auto * flags_out = flags.get() + slot * count;
    std::size_t i = 0;
    for (const auto & line : scan_lines) {
      std::int32_t previous_reflectance = 0, previous_radius = 0;
      for (; i < line.end; ++i) {
        std::uint32_t value;
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
        std::memcpy(&value, ordered_points[i] + slot * 4, sizeof value);
#else
        value = read_word(ordered_points[i] + slot * 4, 4);
#endif
        const auto reflect = static_cast<std::int32_t>(value & 0xfff);
        reflect_out[i] = zigzag(wrapped(reflect - previous_reflectance, 12));
        flags_out[i] = value >> 30;
        previous_reflectance = reflect;
        const auto radius = static_cast<std::int32_t>((value >> 12) & kRadiusMask);
        if (radius == 0) {
          continue;
        }
        const auto residual = zigzag(radius - previous_radius);
        previous_radius = radius;
        set_bit(mask, i, 1);
        low[valid_count] = residual & 0x7f;
        const auto high = residual >> 7;
        // A zero high part leaves the cursor in place, so its slot is overwritten or omitted.
        sparse[high_count] = high - 1;
        set_bit(high_mask.data(), valid_count, high != 0);
        ++valid_count;
        high_count += high != 0;
      }
    }
  }
  append_planes(output, low.get(), valid_count, 7);
  output.insert(output.end(), high_mask.begin(), high_mask.begin() + bitmap_size(valid_count));
  append_planes(output, sparse.get(), high_count, 12);
  append_planes(output, reflectance.get(), points, 12);
  append_planes(output, flags.get(), points, 2);
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
