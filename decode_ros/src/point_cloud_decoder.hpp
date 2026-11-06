#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <vector>

#include "calibration_id.hpp"
#include "packet_to_pointcloud.h"
#include "robinw_packet_frame/msg/compressed_angle_calibration.hpp"
#include "robinw_packet_frame/msg/compressed_packet_frame.hpp"

namespace robinw
{
using Frame = robinw_packet_frame::msg::CompressedPacketFrame;
using Calibration = robinw_packet_frame::msg::CompressedAngleCalibration;

class PointCloudDecoder
{
public:
  // Zero capacity keeps all tables for offline export.
  explicit PointCloudDecoder(std::size_t capacity = 8) : capacity_(capacity) {}
  void update_calibration(const Calibration & message);
  // Returns nullptr if the exact table for this frame is unavailable.
  sensor_msgs::msg::PointCloud2::SharedPtr decode(const Frame & frame);

private:
  struct Table
  {
    CalibrationId id;
    std::vector<std::uint8_t> data;
  };

  seyond::PacketToPointCloud converter_;
  std::vector<std::uint8_t> normalized_;
  const std::size_t capacity_;
  std::deque<Table> tables_;
  std::optional<CalibrationId> applied_;
};
}  // namespace robinw
