#include "point_cloud_decoder.hpp"

#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>

#include <zstd.h>

#include "sdk_common/inno_lidar_packet_utils.h"

namespace robinw
{
namespace
{
using Bytes = std::vector<std::uint8_t>;
using Scan = seyond::msg::SeyondScan;
constexpr std::size_t kHeaderSize = sizeof(InnoDataPacket);
constexpr std::size_t kCalibrationSize = kHeaderSize + sizeof(InnoAngleHVTable);
constexpr std::size_t kMaxPacketCount = 65536;  // The sensor sub-index is uint16.
constexpr std::size_t kMaxFrameSize = 64 * 1024 * 1024;
static_assert(kHeaderSize == 70 && kCalibrationSize == 50512);

void decompress(const Bytes & compressed, std::size_t max_size, Bytes & decoded)
{
  const auto size = ZSTD_getFrameContentSize(compressed.data(), compressed.size());
  if (size == ZSTD_CONTENTSIZE_ERROR || size == ZSTD_CONTENTSIZE_UNKNOWN || size > max_size) {
    throw std::runtime_error("Invalid or oversized Zstd content");
  }
  decoded.resize(size);
  const auto result = ZSTD_decompress(
    decoded.data(), decoded.size(), compressed.data(), compressed.size());
  if (ZSTD_isError(result)) {
    throw std::runtime_error(ZSTD_getErrorName(result));
  }
  if (result != decoded.size()) {
    throw std::runtime_error("Wrong decompressed size");
  }
}

Bytes decode_calibration(const Calibration & message)
{
  Bytes table;
  decompress(message.data, kCalibrationSize, table);
  if (message.uncompressed_size != kCalibrationSize || table.size() != kCalibrationSize) {
    throw std::runtime_error("Wrong type-100 calibration size");
  }
  const auto & packet = *reinterpret_cast<const InnoDataPacket *>(table.data());
  if (packet.type != INNO_ROBINW_ITEM_TYPE_ANGLEHV_TABLE ||
    packet.common.size != table.size() ||
    !InnoDataPacketUtils::check_data_packet(packet, table.size()))
  {
    throw std::runtime_error("Invalid type-100 calibration packet");
  }
  return table;
}

Scan::SharedPtr restore_packets(const Frame & frame, Bytes & normalized)
{
  const std::size_t count = frame.packet_count;
  const std::size_t raw_size = frame.uncompressed_size;
  const std::size_t width = frame.shuffle_block_size;
  if (!count || count > kMaxPacketCount || raw_size < count * kHeaderSize ||
    raw_size > kMaxFrameSize ||
    (width != 0 && width != sizeof(InnoCoBlock1) && width != sizeof(InnoCoBlock2)))
  {
    throw std::runtime_error("Invalid compressed frame metadata");
  }
  decompress(frame.data, raw_size + kHeaderSize + 1, normalized);
  if (normalized.size() < kHeaderSize + 1) {
    throw std::runtime_error("Truncated packet header template");
  }
  const std::size_t changes = normalized.front();
  const std::size_t prefix_size = 1 + kHeaderSize + changes * (count + 1);
  const std::size_t payload_size = raw_size - count * kHeaderSize;
  if (changes > kHeaderSize || normalized.size() != prefix_size + payload_size ||
    (width && payload_size % width != 0))
  {
    throw std::runtime_error("Invalid packet header deltas or payload size");
  }
  const auto * header_template = normalized.data() + 1;
  const auto * offsets = header_template + kHeaderSize;
  for (std::size_t i = 0; i < changes; ++i) {
    if (offsets[i] >= kHeaderSize || (i && offsets[i] <= offsets[i - 1])) {
      throw std::runtime_error("Invalid header delta offset");
    }
  }
  const auto * deltas = offsets + changes;
  const auto * payload = normalized.data() + prefix_size;
  const auto total_blocks = width ? payload_size / width : 0;

  auto scan = std::make_shared<Scan>();
  scan->timestamp = frame.sensor_timestamp_us;
  scan->size = frame.packet_count;
  scan->is_last_scan = true;
  scan->packets.reserve(count);
  std::size_t payload_offset = 0;
  for (std::size_t i = 0; i < count; ++i) {
    std::array<std::uint8_t, kHeaderSize> header{};
    std::memcpy(header.data(), header_template, header.size());
    for (std::size_t change = 0; change < changes; ++change) {
      header[offsets[change]] = *deltas++;
    }
    const auto & info = *reinterpret_cast<const InnoDataPacket *>(header.data());
    const std::size_t packet_size = info.common.size;
    if (packet_size < kHeaderSize || packet_size - kHeaderSize > payload_size - payload_offset ||
      info.type != INNO_ROBINW_ITEM_TYPE_COMPACT_POINTCLOUD || info.idx != frame.frame_id ||
      (width && (info.item_size != width || (packet_size - kHeaderSize) % width != 0)))
    {
      throw std::runtime_error("Invalid restored packet header");
    }
    const std::size_t size = packet_size - kHeaderSize;
    auto & data = scan->packets.emplace_back().data;
    data.resize(packet_size);
    std::memcpy(data.data(), header.data(), kHeaderSize);
    auto * blocks = data.data() + kHeaderSize;
    if (width) {
      for (std::size_t lane = 0; lane < width; ++lane) {
        const auto * source = payload + lane * total_blocks + payload_offset / width;
        for (std::size_t block = 0; block < size / width; ++block) {
          blocks[block * width + lane] = source[block];
        }
      }
    } else {
      std::memcpy(blocks, payload + payload_offset, size);
    }
    payload_offset += size;
    const auto & packet = *reinterpret_cast<const InnoDataPacket *>(data.data());
    if (!InnoDataPacketUtils::check_data_packet(packet, data.size())) {
      throw std::runtime_error("Restored packet failed Seyond SDK validation");
    }
  }
  if (payload_offset != payload_size) {
    throw std::runtime_error("Unconsumed frame payload");
  }
  return scan;
}

}  // namespace

bool PointCloudDecoder::has_calibration() const
{
  return converter_.hasAngleTable();
}

void PointCloudDecoder::update_calibration(const Calibration & message)
{
  converter_.setAngleTable(decode_calibration(message));
}

sensor_msgs::msg::PointCloud2::SharedPtr PointCloudDecoder::decode(const Frame & frame)
{
  if (!has_calibration()) {
    return nullptr;
  }
  if (!std::isfinite(frame.sensor_timestamp_us) || frame.sensor_timestamp_us < 0 ||
    frame.sensor_timestamp_us >= (1.0 + std::numeric_limits<std::int32_t>::max()) * 1e6)
  {
    throw std::runtime_error("Invalid sensor timestamp");
  }
  auto scan = restore_packets(frame, normalized_);
  sensor_msgs::msg::PointCloud2::SharedPtr cloud;
  if (!converter_.PacketsToPointCloud2(scan, cloud) || !cloud) {
    throw std::runtime_error("Seyond decoder produced no point cloud");
  }
  return cloud;
}
}  // namespace robinw
