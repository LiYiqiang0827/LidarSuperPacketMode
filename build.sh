#!/usr/bin/env bash
set -eo pipefail
project_link=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
output=${1:-"$(dirname -- "$project_link")/lidar_super_packet_mode_build"}
project=$(cd -- "$project_link" && pwd -P)
mkdir -p "$output"
output=$(cd -- "$output" && pwd -P)
if [[ -z "${ROS_DISTRO:-}" ]]; then
  shopt -s nullglob
  ros_setups=(/opt/ros/*/setup.bash)
  if (( ${#ros_setups[@]} != 1 )); then
    echo 'Source the intended ROS 2 distribution before building.' >&2
    exit 2
  fi
  source "${ros_setups[0]}"
else
  source "/opt/ros/$ROS_DISTRO/setup.bash"
fi
cd "$project"
colcon --log-base "$output/log" build --base-paths "$project/compress_ros" \
  --build-base "$output/build" --install-base "$output/install" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
printf '\nRun: source %q\n' "$output/install/setup.bash"
