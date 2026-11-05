"""Play original SeyondScan messages with the ROS 2 bag player."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("bag", description="Path to the ROS 2 bag directory"),
        DeclareLaunchArgument("rate", default_value="1.0"),
        DeclareLaunchArgument("delay", default_value="0.0"),
        DeclareLaunchArgument("front_topic", default_value="/iv_packets_front_mid"),
        DeclareLaunchArgument("rear_topic", default_value="/iv_packets_rear_mid"),
    ]
    replay = ExecuteProcess(
        cmd=[
            "ros2", "bag", "play", LaunchConfiguration("bag"),
            "--rate", LaunchConfiguration("rate"),
            "--delay", LaunchConfiguration("delay"),
            "--disable-keyboard-controls",
            "--topics", LaunchConfiguration("front_topic"), LaunchConfiguration("rear_topic"),
        ],
        output="screen",
    )
    return LaunchDescription([*arguments, replay])
