#include "compact_predictor.hpp"

#include <cassert>
#include <iostream>
#include <random>

using namespace robinw::compact;

Bytes fixture(std::size_t width, std::size_t count, std::mt19937 & random)
{
  Bytes result(width * count);
  for (std::size_t row = 0; row < count; ++row) {
    auto * block = result.data() + row * width;
    for (std::size_t i = 0; i < kBlockHeader; ++i) {
      block[i] = random();
    }
    // Repeated scan lines, tied scan positions, zero and maximum range,
    // and non-zero flags on zero-range slots.
    block[6] = (row * 3) % 13;
    block[7] = 0;
    block[8] = row % 4;
    block[9] = (row % 2) << 3;
    for (std::size_t slot = kBlockHeader; slot < width; slot += 4) {
      const auto radius = slot == kBlockHeader || row % 5 == 0 ? 0 :
        row % 7 == 0 ? kRadiusMask : random() & kRadiusMask;
      const auto value = (random() & 0xfffU) | (radius << 12) | ((random() & 3U) << 30);
      for (unsigned i = 0; i < 4; ++i) {
        block[slot + i] = value >> (i * 8);
      }
    }
  }
  return result;
}

int main()
{
  std::mt19937 random(42);
  for (const std::size_t count : {0U, 1U, 7U, 8U, 9U, 513U}) {
    const auto single = fixture(kSingle, count, random);
    const auto dual = fixture(kDouble, count, random);
    Bytes data;
    encode_blocks(single, kSingle, data);
    encode_blocks(dual, kDouble, data);
    if (data.empty()) {
      assert(count == 0);
      continue;
    }
    Reader input(data.data(), data.size());
    assert(decode_blocks(input, count, kSingle) == single);
    assert(decode_blocks(input, count, kDouble) == dual);
    assert(input.remaining() == 0);
    Reader truncated(data.data(), data.size() - 1);
    assert(decode_blocks(truncated, count, kSingle) == single);
    bool rejected = false;
    try {
      decode_blocks(truncated, count, kDouble);
    } catch (const std::runtime_error &) {
      rejected = true;
    }
    assert(rejected);
  }
  std::cout << "PASS: RBP3 42/74-byte blocks, mixed streams, empty/edge bitmaps, and truncated input\n";
}
