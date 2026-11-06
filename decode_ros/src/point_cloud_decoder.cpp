#include "point_cloud_decoder.hpp"

#include "compact_predictor.hpp"
#include "frame_metadata.hpp"

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
constexpr std::size_t kMaxPacketCount = 65536;  // The sensor sub-index is uint16.
constexpr std::size_t kMaxFrameSize = 64 * 1024 * 1024;
static_assert(kHeaderSize == kCalibrationHeaderSize &&
  kHeaderSize + sizeof(InnoAngleHVTable) == kCalibrationSize);

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
  if (!frame_zstd_level(frame.data)) {
    throw std::runtime_error("Unsupported compressed frame format; expected RBP3");
  }
  const auto payload_size = raw_size - count * kHeaderSize;
  constexpr auto magic_size = sizeof(compact::kMagic);
  decompress(frame.data, raw_size + payload_size / 8 + 1024, normalized);
  if (normalized.size() < magic_size + kHeaderSize + 1 ||
    !std::equal(compact::kMagic, compact::kMagic + magic_size, normalized.begin()))
  {
    throw std::runtime_error("Missing RBP3 packet header template");
  }
  const auto * normalized_headers = normalized.data() + magic_size;
  const std::size_t changes = normalized_headers[0];
  const std::size_t prefix_size = magic_size + 1 + kHeaderSize + changes * (count + 1);
  if (changes > kHeaderSize || prefix_size > normalized.size()) {
    throw std::runtime_error("Invalid RBP3 packet header deltas");
  }
  const auto * header_template = normalized_headers + 1;
  const auto * offsets = header_template + kHeaderSize;
  for (std::size_t i = 0; i < changes; ++i) {
    if (offsets[i] >= kHeaderSize || (i && offsets[i] <= offsets[i - 1])) {
      throw std::runtime_error("Invalid header delta offset");
    }
  }
  const auto * deltas = offsets + changes;
  auto scan = std::make_shared<Scan>();
  scan->timestamp = frame.sensor_timestamp_us;
  scan->size = frame.packet_count;
  scan->is_last_scan = true;
  scan->packets.reserve(count);
  std::size_t single_size = 0, dual_size = 0, total_size = 0;
  bool saw_single = false, saw_dual = false;
  for (std::size_t i = 0; i < count; ++i) {
    std::array<std::uint8_t, kHeaderSize> header{};
    std::memcpy(header.data(), header_template, header.size());
    for (std::size_t change = 0; change < changes; ++change) {
      header[offsets[change]] = *deltas++;
    }
    const auto & info = *reinterpret_cast<const InnoDataPacket *>(header.data());
    const std::size_t packet_size = info.common.size;
    const auto packet_width = static_cast<std::size_t>(info.item_size);
    if (packet_size < kHeaderSize || packet_size - kHeaderSize > payload_size - total_size ||
      info.type != INNO_ROBINW_ITEM_TYPE_COMPACT_POINTCLOUD || info.idx != frame.frame_id ||
      (packet_width != compact::kSingle && packet_width != compact::kDouble) ||
      packet_size - kHeaderSize != info.item_number * packet_width)
    {
      throw std::runtime_error("Invalid restored packet header");
    }
    const auto size = packet_size - kHeaderSize;
    saw_single |= packet_width == compact::kSingle;
    saw_dual |= packet_width == compact::kDouble;
    (packet_width == compact::kSingle ? single_size : dual_size) += size;
    total_size += size;
    auto & data = scan->packets.emplace_back().data;
    data.resize(packet_size);
    std::memcpy(data.data(), header.data(), kHeaderSize);
  }
  const auto actual_width = saw_single && saw_dual ? 0 : saw_single ? compact::kSingle : compact::kDouble;
  if (total_size != payload_size || width != actual_width) {
    throw std::runtime_error("RBP3 packet payload size or block width mismatch");
  }
  compact::Reader reader(normalized.data() + prefix_size, normalized.size() - prefix_size);
  const auto single = compact::decode_blocks(reader, single_size / compact::kSingle, compact::kSingle);
  const auto dual = compact::decode_blocks(reader, dual_size / compact::kDouble, compact::kDouble);
  if (reader.remaining() != 0) {
    throw std::runtime_error("Unconsumed RBP3 frame payload");
  }
  std::size_t single_offset = 0, dual_offset = 0;
  for (auto & packet : scan->packets) {
    auto & data = packet.data;
    const auto packet_width = compact::read_word(data.data() + 42, 2);
    const bool is_single = packet_width == compact::kSingle;
    const auto & stream = is_single ? single : dual;
    auto & offset = is_single ? single_offset : dual_offset;
    const auto size = data.size() - kHeaderSize;
    if (size) {
      std::memcpy(data.data() + kHeaderSize, stream.data() + offset, size);
      offset += size;
    }
    const auto & restored = *reinterpret_cast<const InnoDataPacket *>(data.data());
    if (!InnoDataPacketUtils::check_data_packet(restored, data.size())) {
      throw std::runtime_error("Restored packet failed Seyond SDK validation");
    }
  }
  return scan;
}

}  // namespace

void PointCloudDecoder::update_calibration(const Calibration & message)
{
  auto bytes = decode_calibration(message);
  const auto id = calibration_id(bytes);
  if (id != message.calibration_id) {
    throw std::runtime_error("Calibration ID does not match angle table");
  }
  const auto old = std::find_if(tables_.begin(), tables_.end(), [&id](const auto & table) {
    return table.id == id;
  });
  if (old != tables_.end()) {
    tables_.erase(old);
  }
  tables_.push_back(Table{id, std::move(bytes)});
  if (capacity_ && tables_.size() > capacity_) {
    tables_.pop_front();
  }
}

sensor_msgs::msg::PointCloud2::SharedPtr PointCloudDecoder::decode(const Frame & frame)
{
  const auto table = std::find_if(tables_.begin(), tables_.end(), [&frame](const auto & value) {
    return value.id == frame.calibration_id;
  });
  if (table == tables_.end()) {
    return nullptr;
  }
  if (applied_ != frame.calibration_id) {
    converter_.setAngleTable(table->data);
    applied_ = frame.calibration_id;
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
