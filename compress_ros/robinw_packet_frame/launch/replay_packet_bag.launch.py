"""Play the original SeyondScan recording with the ROS 2 bag player."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def replay(context):
    bag, rate, delay, front, rear = (
        LaunchConfiguration(name).perform(context)
        for name in ("bag", "rate", "delay", "front_topic", "rear_topic")
    )
    topics = [front]
    if rear:
        topics.append(rear)
    cmd = ["ros2", "bag", "play", bag, "--rate", rate, "--disable-keyboard-controls"]
    if delay:
        cmd.extend(["--delay", delay])
    cmd.extend(["--topics", *topics])
    return [ExecuteProcess(cmd=cmd, output="screen")]


def generate_launch_description():
    default_bag = Path.home() / "mikelee/seyond_capture_front_mid_2026_11_06-18_11_58/packet_rosbag"
    return LaunchDescription([
        DeclareLaunchArgument("bag", default_value=str(default_bag),
                              description="Path to the ROS 2 bag directory"),
        DeclareLaunchArgument("rate", default_value="1.0"),
        DeclareLaunchArgument("delay", default_value="", description="Optional positive delay in seconds"),
        DeclareLaunchArgument("front_topic", default_value="/iv_packets_front_mid"),
        DeclareLaunchArgument("rear_topic", default_value=""),
        OpaqueFunction(function=replay),
    ])
