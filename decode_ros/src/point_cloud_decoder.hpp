#pragma once

#include <cstdint>
#include <vector>

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
  bool has_calibration() const;
  void update_calibration(const Calibration & message);
  // Returns nullptr until the first valid calibration has been received.
  sensor_msgs::msg::PointCloud2::SharedPtr decode(const Frame & frame);

private:
  seyond::PacketToPointCloud converter_;
  std::vector<std::uint8_t> normalized_;
};
}  // namespace robinw
