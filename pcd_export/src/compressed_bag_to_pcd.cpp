#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>

#include "frame_metadata.hpp"
#include "pcd_writer.hpp"
#include "point_cloud_decoder.hpp"

namespace
{
template<typename Message>
Message read_message(const rosbag2_storage::SerializedBagMessage & bag_message)
{
  Message message;
  rclcpp::SerializedMessage serialized(*bag_message.serialized_data);
  rclcpp::Serialization<Message>().deserialize_message(&serialized, &message);
  return message;
}

void export_clouds(const std::string & bag, const std::string & frame_topic,
  const std::string & calibration_topic, const std::filesystem::path & output)
{
  rosbag2_cpp::Reader reader;
  reader.open(bag);
  rosbag2_storage::StorageFilter filter;
  filter.topics = {frame_topic, calibration_topic};
  reader.set_filter(filter);
  if (!reader.has_next()) {
    throw std::runtime_error("Bag has no frames on " + frame_topic);
  }
  std::filesystem::create_directories(output);
  robinw::PointCloudDecoder decoder(0);
  {
    // A calibration can be recorded after its first frame on the other topic.
    rosbag2_cpp::Reader tables;
    tables.open(bag);
    rosbag2_storage::StorageFilter only_calibration;
    only_calibration.topics = {calibration_topic};
    tables.set_filter(only_calibration);
    while (tables.has_next()) {
      decoder.update_calibration(read_message<robinw::Calibration>(*tables.read_next()));
    }
  }
  std::size_t saved = 0;
  std::size_t skipped = 0;
  std::size_t table_updates = 0;
  // The reader merges both topics in ascending bag receive time.
  while (reader.has_next()) {
    const auto message = reader.read_next();
    if (message->topic_name == calibration_topic) {
      ++table_updates;
      continue;
    }
    const auto frame = read_message<robinw::Frame>(*message);
    try {
      const auto level = robinw::frame_zstd_level(frame.data);
      const auto cloud = decoder.decode(frame);
      if (!cloud) {
        ++skipped;
        continue;
      }
      const auto & stamp = cloud->header.stamp;
      const auto file = output / (formatTimestamp(stamp.sec, stamp.nanosec) + ".pcd");
      if (std::filesystem::exists(file) || !writePcd(file.string(), *cloud)) {
        throw std::runtime_error("Cannot create PCD " + file.string());
      }
      ++saved;
      std::cout << frame.frame_id << ' ' << cloud->width << " points "
                << (frame.complete ? "complete" : "partial") << ' ' << file.filename().string()
                << " zstd_level=" << (level ? std::to_string(*level) : "unknown") << '\n';
    } catch (const std::exception & error) {
      throw std::runtime_error("Frame " + std::to_string(frame.frame_id) + ": " + error.what());
    }
  }
  if (saved == 0 && skipped == 0) {
    throw std::runtime_error("Bag has no frames on " + frame_topic);
  }
  std::cout << "Saved " << saved << " PCD files to " << output << '\n'
            << "Skipped " << skipped << " frame(s) without matching calibration; read "
            << table_updates << " calibration update(s)\n";
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 5) {
    std::cerr << "Usage: " << argv[0] << " <bag_path> <compressed_frame_topic>"
              << " <compressed_calibration_topic> <out_dir>\n";
    return 2;
  }
  try {
    export_clouds(argv[1], argv[2], argv[3], argv[4]);
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
