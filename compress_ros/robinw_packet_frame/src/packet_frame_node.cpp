#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <zstd.h>

#include "calibration_id.hpp"
#include "compact_predictor.hpp"
#include "frame_metadata.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "robinw_packet_frame/msg/compressed_angle_calibration.hpp"
#include "robinw_packet_frame/msg/compressed_packet_frame.hpp"
#include "seyond/msg/seyond_scan.hpp"
#include "sdk_common/inno_lidar_packet_utils.h"

namespace
{
using Bytes = std::vector<std::uint8_t>;
using CompressedFrame = robinw_packet_frame::msg::CompressedPacketFrame;
using Calibration = robinw_packet_frame::msg::CompressedAngleCalibration;
using Scan = seyond::msg::SeyondScan;
using Clock = std::chrono::steady_clock;

constexpr std::size_t kHeaderSize = 70;
constexpr std::uint16_t kDataMagic = 0x176a;
constexpr std::uint8_t kCompactType = 13;
constexpr std::uint8_t kCalibrationType = 100;
constexpr std::uint16_t kLastPacket = 1U << 7;

template <typename UInt>
UInt read_le(const Bytes & bytes, std::size_t offset)
{
  UInt result = 0;
  for (std::size_t i = 0; i < sizeof(UInt); ++i) {
    result |= static_cast<UInt>(bytes[offset + i]) << (8U * i);
  }
  return result;
}

bool has_header(const Bytes & bytes, std::uint8_t type)
{
  return bytes.size() >= kHeaderSize && read_le<std::uint16_t>(bytes, 0) == kDataMagic &&
         bytes[38] == type;
}

struct Frame
{
  std::uint64_t frame_id{0};
  double sensor_timestamp_us{0};
  std::uint16_t shuffle_block_size{0};
  robinw::CalibrationId calibration{};
  std::vector<std::uint32_t> packet_sizes;
  Bytes data;
};

struct PacketInfo
{
  std::uint64_t frame_id;
  double sensor_timestamp_us;
  std::uint16_t sub_idx;
  std::uint16_t block_size;
  bool is_last;
};

bool read_packet_info(const Bytes & bytes, PacketInfo & info)
{
  if (!has_header(bytes, kCompactType)) {
    return false;
  }
  const auto declared_size = read_le<std::uint32_t>(bytes, 10);
  const auto block_count = read_le<std::uint32_t>(bytes, 38) >> 8;
  const auto block_size = read_le<std::uint16_t>(bytes, 42);
  const auto flags = read_le<std::uint16_t>(bytes, 48);
  const auto return_mode = (flags >> 2) & 7U;
  const auto expected_block_size = return_mode == 1 ? 42U : return_mode == 2 ? 74U : 0U;
  if (expected_block_size == 0 || block_size != expected_block_size ||
    declared_size != bytes.size() ||
    bytes.size() != kHeaderSize + static_cast<std::size_t>(block_count) * block_size)
  {
    return false;
  }
  info.frame_id = read_le<std::uint64_t>(bytes, 26);
  const auto time_bits = read_le<std::uint64_t>(bytes, 16);
  std::memcpy(&info.sensor_timestamp_us, &time_bits, sizeof(time_bits));
  info.sub_idx = read_le<std::uint16_t>(bytes, 34);
  info.block_size = block_size;
  info.is_last = (flags & kLastPacket) != 0;
  return true;
}
// RBP3 preserves original packet headers and predicts points independently in each frame.
void normalize_frame(const Frame & frame, Bytes & normalized)
{
  const auto * raw = frame.data.data();
  std::array<bool, kHeaderSize> changes{};
  std::size_t offset = frame.packet_sizes.front();
  for (std::size_t i = 1; i < frame.packet_sizes.size(); ++i) {
    for (std::size_t byte = 0; byte < kHeaderSize; ++byte) {
      changes[byte] |= raw[byte] != raw[offset + byte];
    }
    offset += frame.packet_sizes[i];
  }
  std::array<std::uint8_t, kHeaderSize> changed_offsets{};
  std::size_t changed_count = 0;
  for (std::size_t byte = 0; byte < kHeaderSize; ++byte) {
    if (changes[byte]) {
      changed_offsets[changed_count++] = static_cast<std::uint8_t>(byte);
    }
  }

  const auto packet_count = frame.packet_sizes.size();
  constexpr auto magic_size = sizeof(robinw::compact::kMagic);
  const auto prefix_size = magic_size + 1 + kHeaderSize + changed_count * (packet_count + 1);
  const auto payload_size = frame.data.size() - packet_count * kHeaderSize;
  normalized.clear();
  normalized.reserve(prefix_size + payload_size + payload_size / 8 + 1024);
  normalized.resize(prefix_size);
  std::memcpy(normalized.data(), robinw::compact::kMagic, magic_size);
  auto * output = normalized.data() + magic_size;
  output[0] = static_cast<std::uint8_t>(changed_count);
  std::memcpy(output + 1, raw, kHeaderSize);
  std::memcpy(output + 1 + kHeaderSize, changed_offsets.data(), changed_count);
  auto * deltas = output + 1 + kHeaderSize + changed_count;

  Bytes single, dual;
  if (frame.shuffle_block_size == robinw::compact::kSingle) {
    single.reserve(payload_size);
  } else if (frame.shuffle_block_size == robinw::compact::kDouble) {
    dual.reserve(payload_size);
  }
  offset = 0;
  for (const auto packet_size : frame.packet_sizes) {
    const auto * packet = raw + offset;
    for (std::size_t i = 0; i < changed_count; ++i) {
      *deltas++ = packet[changed_offsets[i]];
    }
    const auto width = robinw::compact::read_word(packet + 42, 2);
    auto & blocks = width == robinw::compact::kSingle ? single : dual;
    blocks.insert(blocks.end(), packet + kHeaderSize, packet + packet_size);
    offset += packet_size;
  }
  robinw::compact::encode_blocks(single, robinw::compact::kSingle, normalized);
  robinw::compact::encode_blocks(dual, robinw::compact::kDouble, normalized);
}
}  // namespace

class PacketFrameNode : public rclcpp::Node
{
public:
  PacketFrameNode()
  : Node("packet_frame_node")
  {
    const auto topic = declare_parameter<std::string>("input_topic", "/iv_packets_front_mid");
    const auto timeout = declare_parameter<int>("flush_timeout_ms", 500);
    const auto input_reliable = declare_parameter<bool>("input_reliable", true);
    if (timeout < 0) {
      throw std::invalid_argument("flush_timeout_ms must be nonnegative");
    }
    if (!zstd_context_) {
      throw std::runtime_error("Could not create Zstd compression context");
    }
    rcl_interfaces::msg::ParameterDescriptor level;
    level.description = "Zstd level for frames and calibration tables";
    rcl_interfaces::msg::IntegerRange valid_levels;
    valid_levels.from_value = 1;
    valid_levels.to_value = ZSTD_maxCLevel();
    valid_levels.step = 1;
    level.integer_range.push_back(valid_levels);
    declare_parameter<int>("zstd_level", 1, level);

    frame_publisher_ =
      create_publisher<CompressedFrame>("compressed_packet_frame", rclcpp::QoS(16).reliable());
    calibration_publisher_ = create_publisher<Calibration>(
      "compressed_angle_calibration", rclcpp::QoS(4).reliable().transient_local());

    auto input_qos = rclcpp::QoS(128);
    if (!input_reliable) {
      input_qos.best_effort();
    }
    input_ = create_subscription<Scan>(
      topic, input_qos,
      [this](Scan::ConstSharedPtr scan) { consume(*scan); });

    if (timeout > 0) {
      timeout_ = std::chrono::milliseconds(timeout);
      timer_ = create_wall_timer(
        std::min(timeout_, std::chrono::milliseconds(50)),
        [this]() {
          if (frame_ && Clock::now() - last_packet_ >= timeout_) {
            publish_frame(false);
          }
        });
    }
  }

private:
  int compress(const std::uint8_t * data, std::size_t size, Bytes & output)
  {
    output.resize(ZSTD_compressBound(size));
    const auto level = static_cast<int>(get_parameter("zstd_level").as_int());
    const auto result = ZSTD_compressCCtx(
      zstd_context_.get(), output.data(), output.size(), data, size, level);
    if (ZSTD_isError(result)) {
      throw std::runtime_error(ZSTD_getErrorName(result));
    }
    output.resize(result);
    return level;
  }

  void accept_calibration(const Bytes & bytes)
  {
    if (!has_header(bytes, kCalibrationType) ||
      read_le<std::uint32_t>(bytes, 10) != robinw::kCalibrationSize ||
      bytes.size() < robinw::kCalibrationSize ||
      !InnoDataPacketUtils::check_data_packet(
        *reinterpret_cast<const InnoDataPacket *>(bytes.data()), robinw::kCalibrationSize))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Ignoring invalid type-100 table");
      return;
    }
    try {
      const auto id = robinw::calibration_id(bytes);
      if (active_calibration_ == id) {
        return;
      }
      auto table = std::make_unique<Calibration>();
      table->uncompressed_size = robinw::kCalibrationSize;
      table->calibration_id = id;
      compress(bytes.data(), robinw::kCalibrationSize, table->data);
      calibration_publisher_->publish(std::move(table));
      if (frame_) {
        // The source did not give a frame boundary for this change.
        discarded_frame_ = frame_->frame_id;
        frame_.reset();
        RCLCPP_WARN(get_logger(), "Discarding frame %llu at angle table change",
          static_cast<unsigned long long>(*discarded_frame_));
      }
      active_calibration_ = id;
      RCLCPP_INFO(get_logger(), "Published angle calibration %s",
        robinw::calibration_id_hex(id).c_str());
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Calibration processing failed: %s", error.what());
    }
  }

  void consume(const Scan & scan)
  {
    for (const auto & packet : scan.packets) {
      PacketInfo info{};
      const bool valid_packet = read_packet_info(packet.data, info);
      if (valid_packet && frame_ && frame_->frame_id != info.frame_id) {
        publish_frame(false);
      }
      if (packet.has_table) {
        accept_calibration(packet.table);
      }
      if (!valid_packet) {
        if (frame_) {
          continuous_ = false;
        }
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Ignoring invalid type-13 packet");
        continue;
      }
      if (discarded_frame_ == info.frame_id) {
        continue;
      }
      discarded_frame_.reset();
      if (!frame_) {
        frame_ = std::make_unique<Frame>();
        frame_->frame_id = info.frame_id;
        frame_->calibration = active_calibration_;
        frame_->sensor_timestamp_us = info.sensor_timestamp_us;
        frame_->shuffle_block_size = info.block_size;
        frame_->packet_sizes.reserve(packet_count_hint_);
        frame_->data.reserve(frame_bytes_hint_);
        continuous_ = true;
      }
      if (frame_->shuffle_block_size != info.block_size) {
        frame_->shuffle_block_size = 0;
      }
      continuous_ = continuous_ && info.sub_idx == frame_->packet_sizes.size();
      frame_->packet_sizes.push_back(static_cast<std::uint32_t>(packet.data.size()));
      frame_->data.insert(frame_->data.end(), packet.data.begin(), packet.data.end());
      last_packet_ = Clock::now();
      if (info.is_last) {
        publish_frame(continuous_);
      }
    }
    if (scan.is_last_scan && frame_) {
      publish_frame(false);
    }
  }

  void publish_frame(bool complete)
  {
    const auto frame = std::move(frame_);
    if (complete) {
      packet_count_hint_ = frame->packet_sizes.size();
      frame_bytes_hint_ = frame->data.size();
    }
    try {
      if (frame->data.size() > std::numeric_limits<std::uint32_t>::max()) {
        throw std::runtime_error("Frame exceeds the message size field");
      }
      normalize_frame(*frame, normalized_);
      auto output = std::make_unique<CompressedFrame>();
      output->frame_id = frame->frame_id;
      output->calibration_id = frame->calibration;
      output->sensor_timestamp_us = frame->sensor_timestamp_us;
      output->complete = complete;
      output->packet_count = static_cast<std::uint32_t>(frame->packet_sizes.size());
      output->uncompressed_size = static_cast<std::uint32_t>(frame->data.size());
      output->shuffle_block_size = frame->shuffle_block_size;
      const auto level = compress(normalized_.data(), normalized_.size(), output->data);
      robinw::append_frame_metadata(output->data, level);
      frame_publisher_->publish(std::move(output));
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "Frame %llu compression failed: %s",
        static_cast<unsigned long long>(frame->frame_id), error.what());
    }
  }

  rclcpp::Subscription<Scan>::SharedPtr input_;
  rclcpp::Publisher<CompressedFrame>::SharedPtr frame_publisher_;
  rclcpp::Publisher<Calibration>::SharedPtr calibration_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<Frame> frame_;
  Bytes normalized_;
  std::unique_ptr<ZSTD_CCtx, decltype(&ZSTD_freeCCtx)> zstd_context_{
    ZSTD_createCCtx(), ZSTD_freeCCtx};
  robinw::CalibrationId active_calibration_{};
  std::optional<std::uint64_t> discarded_frame_;
  Clock::time_point last_packet_;
  std::chrono::milliseconds timeout_{0};
  std::size_t packet_count_hint_{0};
  std::size_t frame_bytes_hint_{0};
  bool continuous_{true};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PacketFrameNode>());
  rclcpp::shutdown();
}
