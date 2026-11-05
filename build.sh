#!/usr/bin/env bash
set -eo pipefail
project=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
output=${1:-"$project/../lidar_super_packet_mode_build"}
mkdir -p "$output"
output=$(cd -- "$output" && pwd)
source /opt/ros/jazzy/setup.bash
cd "$project"
colcon --log-base "$output/log" build --base-paths "$project/compress_ros" \
  --build-base "$output/build" --install-base "$output/install" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
printf '\nRun: source %q\n' "$output/install/setup.bash"
