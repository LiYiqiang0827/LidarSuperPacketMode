/**********************************************************************************************************************
Copyright (c) 2024 Seyond
All rights reserved

By downloading, copying, installing or using the software you agree to this license. If you do not agree to this
license, do not download, install, copy or use the software.

License Agreement
For Seyond LiDAR SDK Library
(2-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**********************************************************************************************************************/

#pragma once

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cstdint>
#include <memory>
#include <vector>

#include "driver_lidar.h"
#include "seyond/msg/seyond_scan.hpp"

namespace seyond {

class PacketToPointCloud {
 public:
  PacketToPointCloud();
  explicit PacketToPointCloud(const LidarConfig& config);
  ~PacketToPointCloud() = default;

  bool PacketsToPointCloud2(const seyond::msg::SeyondScan::SharedPtr& msg,
                            sensor_msgs::msg::PointCloud2::SharedPtr& pcMsg);

  bool PacketsToPointCloud2(const std::vector<seyond::msg::SeyondScan::SharedPtr>& msgs,
                            sensor_msgs::msg::PointCloud2::SharedPtr& pcMsg);

  bool hasAngleTable() const;
  void setAngleTable(const std::vector<std::uint8_t>& table);
  static LidarConfig defaultConfig();

 private:
  void processPackets(const seyond::msg::SeyondScan& scan);
  sensor_msgs::msg::PointCloud2::SharedPtr buildPointCloud2(double timestamp);

  LidarConfig config_;
  std::unique_ptr<DriverLidar> driver_ptr_;
};

}  // namespace seyond
