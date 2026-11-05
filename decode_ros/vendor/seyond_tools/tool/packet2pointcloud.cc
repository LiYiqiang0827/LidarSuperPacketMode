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

#include <rosbag2_cpp/reader.hpp>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cstdio>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <unistd.h>

#include "packet_to_pointcloud.h"
#include "seyond/msg/seyond_scan.hpp"

namespace {

// PCD header 里一个字段的描述。PointCloud2 的 point_step 带对齐填充(实测 48 字节)，
// PCD 没有 offset 概念、只能表达紧凑布局，所以写盘时按字段顺序重新紧凑排列(31 字节/点)。
// 字段顺序/类型直接取自 PointCloud2 的 fields，保证与 pcl::toROSMsg 产出的布局一致。
struct PcdField {
  std::string name;
  uint32_t size;
  uint32_t count;
  char type;        // 'F' / 'U' / 'I'
  uint32_t offset;  // 在 PointCloud2 单点内的字节偏移
};

bool describeField(const sensor_msgs::msg::PointField& f, PcdField& out) {
  uint32_t size = 0;
  char type = 0;
  switch (f.datatype) {
    case sensor_msgs::msg::PointField::INT8:
      size = 1;
      type = 'I';
      break;
    case sensor_msgs::msg::PointField::UINT8:
      size = 1;
      type = 'U';
      break;
    case sensor_msgs::msg::PointField::INT16:
      size = 2;
      type = 'I';
      break;
    case sensor_msgs::msg::PointField::UINT16:
      size = 2;
      type = 'U';
      break;
    case sensor_msgs::msg::PointField::INT32:
      size = 4;
      type = 'I';
      break;
    case sensor_msgs::msg::PointField::UINT32:
      size = 4;
      type = 'U';
      break;
    case sensor_msgs::msg::PointField::FLOAT32:
      size = 4;
      type = 'F';
      break;
    case sensor_msgs::msg::PointField::FLOAT64:
      size = 8;
      type = 'F';
      break;
    default:
      return false;
  }
  out.name = f.name;
  out.size = size;
  out.count = (f.count == 0) ? 1u : f.count;
  out.type = type;
  out.offset = f.offset;
  return true;
}

// 与旧工具一致的文件名: 2026_11_03-10_48_13_957.pcd（本地时间）
std::string formatTimestamp(int32_t sec, uint32_t nanosec) {
  std::time_t tt = static_cast<std::time_t>(sec);
  std::tm tm_buf{};
  localtime_r(&tt, &tm_buf);
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%04d_%02d_%02d-%02d_%02d_%02d_%03u",
                tm_buf.tm_year + 1900, tm_buf.tm_mon + 1, tm_buf.tm_mday,
                tm_buf.tm_hour, tm_buf.tm_min, tm_buf.tm_sec,
                static_cast<unsigned>(nanosec / 1000000));
  return std::string(buf);
}

// 把一个 PointCloud2 写成紧凑的 binary PCD，字段名/顺序/类型原样保留。
bool writePcd(const std::string& path, const sensor_msgs::msg::PointCloud2& msg) {
  std::vector<PcdField> fields;
  fields.reserve(msg.fields.size());
  for (const auto& f : msg.fields) {
    PcdField desc{};
    if (!describeField(f, desc)) {
      std::cerr << "Error: unsupported PointField datatype " << int(f.datatype)
                << " for field '" << f.name << "'" << std::endl;
      return false;
    }
    fields.push_back(desc);
  }

  const size_t point_count = static_cast<size_t>(msg.width) * static_cast<size_t>(msg.height);
  size_t row_size = 0;
  for (const auto& f : fields) {
    row_size += static_cast<size_t>(f.size) * f.count;
  }

  std::ofstream ofs(path, std::ios::binary);
  if (!ofs) {
    std::cerr << "Error: cannot open for writing: " << path << std::endl;
    return false;
  }

  ofs << "# .PCD v0.7 - Point Cloud Data file format\n";
  ofs << "VERSION 0.7\n";
  ofs << "FIELDS";
  for (const auto& f : fields) ofs << ' ' << f.name;
  ofs << '\n';
  ofs << "SIZE";
  for (const auto& f : fields) ofs << ' ' << f.size;
  ofs << '\n';
  ofs << "TYPE";
  for (const auto& f : fields) ofs << ' ' << f.type;
  ofs << '\n';
  ofs << "COUNT";
  for (const auto& f : fields) ofs << ' ' << f.count;
  ofs << '\n';
  ofs << "WIDTH " << point_count << '\n';
  ofs << "HEIGHT 1\n";
  ofs << "VIEWPOINT 0 0 0 1 0 0 0\n";
  ofs << "POINTS " << point_count << '\n';
  ofs << "DATA binary\n";

  if (point_count == 0) {
    return static_cast<bool>(ofs);
  }

  std::vector<char> packed(point_count * row_size);
  const char* src = reinterpret_cast<const char*>(msg.data.data());
  const size_t step = msg.point_step;
  size_t dst = 0;
  for (size_t i = 0; i < point_count; ++i) {
    const char* p = src + i * step;
    for (const auto& f : fields) {
      const size_t n = static_cast<size_t>(f.size) * f.count;
      std::memcpy(packed.data() + dst, p + f.offset, n);
      dst += n;
    }
  }
  ofs.write(packed.data(), static_cast<std::streamsize>(packed.size()));
  return static_cast<bool>(ofs);
}

}  // namespace

static void printUsage(const char* prog) {
  std::cerr << "Usage: " << prog << " <bag_path> <topic_name> <out_dir>" << std::endl;
  std::cerr << "  bag_path:   path to rosbag2 directory" << std::endl;
  std::cerr << "  topic_name: e.g. /iv_packets_front_mid" << std::endl;
  std::cerr << "  out_dir:    write one binary PCD per frame into it" << std::endl;
  std::cerr << "              (file name: <YYYY_MM_DD-HH_MM_SS_mmm>.pcd, local time)." << std::endl;
}

static void logCallback(int32_t level, const char* header, const char* msg) {
  if (level <= 4) {
    fprintf(stderr, "[SDK] %s %s\n", header, msg);
  }
}

int main(int argc, char* argv[]) {
  if (argc < 4) {
    printUsage(argv[0]);
    return 1;
  }

  std::string bag_path = argv[1];
  std::string topic_name = argv[2];
  std::string out_dir = argv[3];

  if (!std::filesystem::exists(bag_path)) {
    std::cerr << "Error: bag path does not exist: " << bag_path << std::endl;
    return 1;
  }

  {
    std::error_code ec;
    std::filesystem::create_directories(out_dir, ec);
    if (ec) {
      std::cerr << "Error: cannot create output dir: " << out_dir << " (" << ec.message() << ")"
                << std::endl;
      return 1;
    }
  }

  // SDK 默认把日志打到 stdout，统一重定向到 stderr，保证 stdout 干净。
  dup2(2, 1);

  std::string log_level = "info";
  seyond::DriverLidar::init_log_s(log_level, logCallback);

  // ---- 第一遍：从包里找角度标定表并存到临时文件，保证从第 0 帧就走"有表"解码路径 ----
  std::string table_path = "/tmp/seyond_hv_table.bin";
  bool table_found = false;
  {
    rosbag2_cpp::Reader pre_reader;
    pre_reader.open(bag_path);
    rosbag2_storage::StorageFilter pre_filter;
    pre_filter.topics.push_back(topic_name);
    pre_reader.set_filter(pre_filter);
    rclcpp::Serialization<seyond::msg::SeyondScan> pre_serializer;
    while (pre_reader.has_next() && !table_found) {
      auto bag_msg = pre_reader.read_next();
      auto scan_msg = std::make_shared<seyond::msg::SeyondScan>();
      rclcpp::SerializedMessage sm(*bag_msg->serialized_data);
      pre_serializer.deserialize_message(&sm, scan_msg.get());
      for (const auto& pkt : scan_msg->packets) {
        if (pkt.has_table && !pkt.table.empty()) {
          std::ofstream ofs(table_path, std::ios::binary);
          ofs.write(reinterpret_cast<const char*>(pkt.table.data()), pkt.table.size());
          ofs.close();
          table_found = true;
          std::cerr << "Found angle table (" << pkt.table.size() << " bytes)" << std::endl;
          break;
        }
      }
    }
    pre_reader.close();
  }

  // ---- 用表构造解码器（有表则从第 0 帧就用"有表"路径）----
  seyond::LidarConfig config = seyond::PacketToPointCloud::defaultConfig();
  if (table_found) {
    config.hv_table_file = table_path;
  }
  seyond::PacketToPointCloud converter(config);

  // ---- 第二遍：解码 -> 逐帧写 PCD ----
  rosbag2_cpp::Reader reader;
  reader.open(bag_path);
  rosbag2_storage::StorageFilter filter;
  filter.topics.push_back(topic_name);
  reader.set_filter(filter);

  rclcpp::Serialization<seyond::msg::SeyondScan> scan_serializer;

  uint32_t frame_count = 0;
  uint32_t saved_count = 0;
  while (reader.has_next()) {
    auto bag_msg = reader.read_next();
    auto scan_msg = std::make_shared<seyond::msg::SeyondScan>();
    rclcpp::SerializedMessage scan_ser(*bag_msg->serialized_data);
    scan_serializer.deserialize_message(&scan_ser, scan_msg.get());

    sensor_msgs::msg::PointCloud2::SharedPtr pc_msg;
    bool ret = converter.PacketsToPointCloud2(scan_msg, pc_msg);
    if (!ret) {
      continue;
    }

    if (pc_msg) {
      frame_count++;
      // 用 db3 录制时间戳作为点云时间戳，便于 Python 端和 marker 对齐
      pc_msg->header.stamp.sec = static_cast<int32_t>(bag_msg->recv_timestamp / 1000000000);
      pc_msg->header.stamp.nanosec = static_cast<uint32_t>(bag_msg->recv_timestamp % 1000000000);

      // 按旧工具的文件名和字段布局落盘
      const std::string file_name =
          formatTimestamp(pc_msg->header.stamp.sec, pc_msg->header.stamp.nanosec) + ".pcd";
      const std::string file_path = (std::filesystem::path(out_dir) / file_name).string();
      if (!writePcd(file_path, *pc_msg)) {
        std::cerr << "Error: failed to write " << file_path << std::endl;
      } else {
        saved_count++;
        const size_t points =
            static_cast<size_t>(pc_msg->width) * static_cast<size_t>(pc_msg->height);
        std::cerr << "[" << frame_count << "] Saved: " << file_path << " (" << points
                  << " points)" << std::endl;
      }
    }
  }

  reader.close();
  std::cerr << "Done. Processed " << frame_count << " scan messages, saved " << saved_count
            << " PCD files to " << out_dir << std::endl;
  return 0;
}
