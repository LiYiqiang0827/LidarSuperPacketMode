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

#include "packet_to_pointcloud.h"

#include <cstring>

namespace seyond {

LidarConfig PacketToPointCloud::defaultConfig() {
  LidarConfig config{};
  config.index = 0;
  config.replay_rosbag = true;
  config.packet_mode = true;
  config.enable_imu_msg = false;
  config.aggregate_num = 20;
  config.frame_id = "seyond";
  config.packet_topic = "";
  config.frame_topic = "";
  config.lidar_name = "seyond";
  config.lidar_ip = "";
  config.port = 0;
  config.udp_port = 0;
  config.reflectance_mode = true;
  config.multiple_return = 1;
  config.enable_falcon_ring = false;
  config.continue_live = false;
  config.inno_pc_file = "";
  config.pcap_file = "";
  config.hv_table_file = "";
  config.packet_rate = 0;
  config.file_rewind = 0;
  config.max_range = 2000.0;
  config.min_range = 0.1;
  config.name_value_pairs = "";
  config.coordinate_mode = 3;
  config.transform_enable = false;
  config.x = 0.0;
  config.y = 0.0;
  config.z = 0.0;
  config.pitch = 0.0;
  config.yaw = 0.0;
  config.roll = 0.0;
  config.transform_matrix = "";
  return config;
}

PacketToPointCloud::PacketToPointCloud()
    : PacketToPointCloud(defaultConfig()) {}

PacketToPointCloud::PacketToPointCloud(const LidarConfig& config)
    : config_(config) {
  config_.replay_rosbag = true;
  config_.packet_mode = true;
  driver_ptr_ = std::make_unique<DriverLidar>(config_);
}

bool PacketToPointCloud::hasAngleTable() const {
  return driver_ptr_->anglehv_table_init_;
}

void PacketToPointCloud::setAngleTable(const std::vector<std::uint8_t>& table) {
  driver_ptr_->anglehv_table_.resize(table.size());
  std::memcpy(driver_ptr_->anglehv_table_.data(), table.data(), table.size());
  driver_ptr_->anglehv_table_init_ = true;
}

void PacketToPointCloud::processPackets(const seyond::msg::SeyondScan& scan) {
  for (const auto& pkt : scan.packets) {
    if (pkt.has_table && !hasAngleTable()) {
      setAngleTable(pkt.table);
    }
    driver_ptr_->convert_and_parse(reinterpret_cast<const int8_t*>(pkt.data.data()));
  }
}

sensor_msgs::msg::PointCloud2::SharedPtr PacketToPointCloud::buildPointCloud2(double timestamp) {
  auto pcMsg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  driver_ptr_->transform_pointcloud();
  pcl::toROSMsg(*driver_ptr_->pcl_pc_ptr, *pcMsg);
  pcMsg->header.frame_id = config_.frame_id;
  int64_t ts_ns = static_cast<int64_t>(timestamp * 1000);
  pcMsg->header.stamp.sec = ts_ns / 1000000000;
  pcMsg->header.stamp.nanosec = ts_ns % 1000000000;
  pcMsg->width = driver_ptr_->pcl_pc_ptr->width;
  pcMsg->height = driver_ptr_->pcl_pc_ptr->height;
  driver_ptr_->pcl_pc_ptr->clear();
  return pcMsg;
}

bool PacketToPointCloud::PacketsToPointCloud2(const seyond::msg::SeyondScan::SharedPtr& msg,
                                              sensor_msgs::msg::PointCloud2::SharedPtr& pcMsg) {
  pcMsg = nullptr;
  if (!msg) {
    return false;
  }

  processPackets(*msg);

  if (msg->is_last_scan) {
    if (driver_ptr_->pcl_pc_ptr->empty()) {
      return false;
    }
    pcMsg = buildPointCloud2(msg->timestamp);
  }
  return true;
}

bool PacketToPointCloud::PacketsToPointCloud2(const std::vector<seyond::msg::SeyondScan::SharedPtr>& msgs,
                                              sensor_msgs::msg::PointCloud2::SharedPtr& pcMsg) {
  pcMsg = nullptr;
  if (msgs.empty()) {
    return false;
  }

  double last_timestamp = 0.0;
  for (const auto& msg : msgs) {
    if (!msg) {
      continue;
    }
    processPackets(*msg);
    if (msg->is_last_scan) {
      last_timestamp = msg->timestamp;
    }
  }

  if (driver_ptr_->pcl_pc_ptr->empty()) {
    return false;
  }

  if (last_timestamp == 0.0) {
    last_timestamp = msgs.back()->timestamp;
  }

  pcMsg = buildPointCloud2(last_timestamp);
  return true;
}

}  // namespace seyond
