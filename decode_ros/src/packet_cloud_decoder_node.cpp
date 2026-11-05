#include <exception>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "point_cloud_decoder.hpp"

namespace
{
using Cloud = sensor_msgs::msg::PointCloud2;

class PacketCloudDecoderNode : public rclcpp::Node
{
public:
  PacketCloudDecoderNode()
  : Node("packet_cloud_decoder_node"),
    frame_id_(declare_parameter<std::string>("frame_id", "seyond"))
  {
    publisher_ = create_publisher<Cloud>("points", rclcpp::QoS(4).reliable());
    calibration_ = create_subscription<robinw::Calibration>(
      "compressed_angle_calibration", rclcpp::QoS(1).reliable().transient_local(),
      [this](robinw::Calibration::ConstSharedPtr message) {
        try {
          decoder_.update_calibration(*message);
          RCLCPP_INFO(get_logger(), "Applied angle calibration table");
        } catch (const std::exception & error) {
          RCLCPP_WARN(get_logger(), "Ignoring invalid angle calibration: %s", error.what());
        }
      });
    frames_ = create_subscription<robinw::Frame>(
      "compressed_packet_frame", rclcpp::QoS(16).reliable(),
      [this](robinw::Frame::ConstSharedPtr message) {
        try {
          auto cloud = decoder_.decode(*message);
          if (!cloud) {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 5000, "Skipping frames while waiting for angle calibration");
            return;
          }
          cloud->header.frame_id = frame_id_;
          publisher_->publish(*cloud);
        } catch (const std::exception & error) {
          RCLCPP_WARN(
            get_logger(), "Skipping frame %llu: %s",
            static_cast<unsigned long long>(message->frame_id), error.what());
        }
      });
  }

private:
  robinw::PointCloudDecoder decoder_;
  const std::string frame_id_;
  rclcpp::Publisher<Cloud>::SharedPtr publisher_;
  rclcpp::Subscription<robinw::Calibration>::SharedPtr calibration_;
  rclcpp::Subscription<robinw::Frame>::SharedPtr frames_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PacketCloudDecoderNode>());
  rclcpp::shutdown();
}
