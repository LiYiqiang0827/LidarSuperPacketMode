#include <chrono>
#include <cstddef>
#include <deque>
#include <exception>
#include <memory>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include "point_cloud_decoder.hpp"

namespace
{
using Cloud = sensor_msgs::msg::PointCloud2;
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

class PacketCloudDecoderNode : public rclcpp::Node
{
public:
  PacketCloudDecoderNode()
  : Node("packet_cloud_decoder_node"),
    frame_id_(declare_parameter<std::string>("frame_id", "seyond"))
  {
    publisher_ = create_publisher<Cloud>("points", rclcpp::QoS(4).reliable());
    calibration_ = create_subscription<robinw::Calibration>(
      "compressed_angle_calibration", rclcpp::QoS(4).reliable().transient_local(),
      [this](robinw::Calibration::ConstSharedPtr message) {
        try {
          decoder_.update_calibration(*message);
          RCLCPP_INFO(get_logger(), "Applied angle calibration table %s",
            robinw::calibration_id_hex(message->calibration_id).c_str());
          drain();
        } catch (const std::exception & error) {
          RCLCPP_WARN(get_logger(), "Ignoring invalid angle calibration: %s", error.what());
        }
      });
    frames_ = create_subscription<robinw::Frame>(
      "compressed_packet_frame", rclcpp::QoS(16).reliable(),
      [this](robinw::Frame::ConstSharedPtr message) {
        if (message->calibration_id == robinw::CalibrationId{}) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000, "Skipping frames without a calibration ID");
          return;
        }
        drain();
        if (pending_.size() == kMaxPending || message->data.size() > kMaxBytes - pending_bytes_) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000, "Skipping frames: calibration wait queue full");
          return;
        }
        pending_bytes_ += message->data.size();
        pending_.push_back({std::move(message), Clock::now() + 500ms});
        drain();
      });
    retry_timer_ = create_wall_timer(50ms, [this] { drain(); });
  }

private:
  void drain()
  {
    while (!pending_.empty()) {
      const auto & next = pending_.front();
      const auto & frame = *next.frame;
      try {
        if (Clock::now() >= next.deadline) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "Skipping frame %llu: angle calibration wait expired for %s",
            static_cast<unsigned long long>(frame.frame_id),
            robinw::calibration_id_hex(frame.calibration_id).c_str());
        } else {
          auto cloud = decoder_.decode(frame);
          if (!cloud) {
            return;
          }
          cloud->header.frame_id = frame_id_;
          publisher_->publish(*cloud);
        }
      } catch (const std::exception & error) {
        RCLCPP_WARN(get_logger(), "Skipping frame %llu: %s",
          static_cast<unsigned long long>(frame.frame_id), error.what());
      }
      pending_bytes_ -= frame.data.size();
      pending_.pop_front();
    }
  }

  struct Pending
  {
    robinw::Frame::ConstSharedPtr frame;
    Clock::time_point deadline;
  };

  static constexpr std::size_t kMaxPending = 32;
  static constexpr std::size_t kMaxBytes = 64 * 1024 * 1024;
  const std::string frame_id_;
  robinw::PointCloudDecoder decoder_;
  std::deque<Pending> pending_;
  std::size_t pending_bytes_{0};
  rclcpp::Publisher<Cloud>::SharedPtr publisher_;
  rclcpp::Subscription<robinw::Calibration>::SharedPtr calibration_;
  rclcpp::Subscription<robinw::Frame>::SharedPtr frames_;
  rclcpp::TimerBase::SharedPtr retry_timer_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PacketCloudDecoderNode>());
  rclcpp::shutdown();
}
