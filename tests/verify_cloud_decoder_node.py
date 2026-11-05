#!/usr/bin/env python3
"""Verify the live point cloud decoder against actual recorded messages and PCDs."""
import argparse
from datetime import datetime
import hashlib
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
from zoneinfo import ZoneInfo

import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.serialization import deserialize_message
from robinw_packet_frame.msg import CompressedAngleCalibration, CompressedPacketFrame
from sensor_msgs.msg import PointCloud2

from verify_calibration_order import FRAME, REAR_TABLE, TABLE, export_fixture, messages

SENSORS = ("front_mid", "rear_mid")
ZONE = ZoneInfo("Asia/Shanghai")
FIELDS = {
    "x": "<f4", "y": "<f4", "z": "<f4", "timestamp": "<f8", "intensity": "<f4",
    "flags": "u1", "elongation": "u1", "scan_id": "<u2", "scan_idx": "<u2", "is_2nd_return": "u1",
}
ROS_TYPES = {"<f4": 7, "<f8": 8, "<u2": 4, "u1": 2}


def verify_payload(cloud, sensor, reference):
    stamp = cloud.header.stamp
    sec = datetime.fromtimestamp(stamp.sec, ZONE).strftime("%Y_%m_%d-%H_%M_%S")
    name = f"{sec}_{stamp.nanosec // 1_000_000:03}.pcd"
    assert cloud.header.frame_id == f"{sensor}_lidar"
    assert cloud.height == 1 and not cloud.is_bigendian
    assert [(f.name, f.datatype, f.count) for f in cloud.fields] == [
        (name, ROS_TYPES[fmt], 1) for name, fmt in FIELDS.items()
    ]
    dtype = np.dtype({"names": list(FIELDS), "formats": list(FIELDS.values()),
                      "offsets": [f.offset for f in cloud.fields], "itemsize": cloud.point_step})
    points = np.frombuffer(cloud.data, dtype=dtype, count=cloud.width)
    packed = points.astype(np.dtype(list(FIELDS.items()))).tobytes()
    with (reference / name).open("rb") as stream:
        for line in stream:
            if line == b"DATA binary\n":
                break
        assert hashlib.sha256(packed).digest() == hashlib.file_digest(stream, "sha256").digest()
    return name


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("reference", type=Path)
    args = parser.parse_args()
    os.environ.setdefault("ROS_DOMAIN_ID", "91")
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    rclpy.init()
    checker = rclpy.create_node("verify_robinw_cloud_decoder")
    processes = []
    seen = {sensor: [] for sensor in SENSORS}

    def wait_until(predicate, seconds=5):
        deadline = time.monotonic() + seconds
        while not predicate():
            assert time.monotonic() < deadline, "Timed out waiting for ROS output"
            rclpy.spin_once(checker, timeout_sec=0.05)

    def quiet(seconds=0.4):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(checker, timeout_sec=0.05)

    def start_decoder(sensor, namespace):
        return subprocess.Popen([
            "ros2", "run", "robinw_packet_frame", "packet_cloud_decoder_node",
            "--ros-args", "-r", f"__ns:=/{namespace}", "-p", f"frame_id:={sensor}_lidar",
        ], start_new_session=True)

    try:
        for sensor in SENSORS:
            processes.append(start_decoder(sensor, sensor))
            checker.create_subscription(
                PointCloud2, f"/{sensor}/points",
                lambda cloud, s=sensor: seen[s].append(verify_payload(cloud, s, args.reference / s)),
                16,
            )
        wait_until(lambda: all(checker.count_publishers(f"/{s}/points") for s in SENSORS))
        topics = [f"/{sensor}/{kind}" for sensor in SENSORS
                  for kind in ("compressed_packet_frame", "compressed_angle_calibration")]
        player = subprocess.Popen([
            "ros2", "bag", "play", str(args.bag), "--delay", "2", "--disable-keyboard-controls",
            "--wait-for-all-acked", "5000", "--topics", *topics,
        ], start_new_session=True)
        processes.append(player)
        wait_until(lambda: player.poll() is not None, seconds=35)
        assert player.returncode == 0
        quiet()
        for sensor in SENSORS:
            assert seen[sensor] == sorted(p.name for p in (args.reference / sensor).glob("*.pcd"))[12:]
            print(f"PASS: {sensor} published 136 point clouds on native bag replay; all point fields match PCD")

        namespace = "switch_calibration"
        processes.append(start_decoder("front_mid", namespace))
        received = []
        checker.create_subscription(PointCloud2, f"/{namespace}/points", received.append, 4)
        frame_pub = checker.create_publisher(CompressedPacketFrame, f"/{namespace}/compressed_packet_frame", 4)
        table_pub = checker.create_publisher(
            CompressedAngleCalibration, f"/{namespace}/compressed_angle_calibration",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        wait_until(lambda: frame_pub.get_subscription_count() == table_pub.get_subscription_count() == 1)
        first, = messages(args.bag, TABLE, 1)
        second, = messages(args.bag, REAR_TABLE, 1)
        frame, = messages(args.bag, FRAME, 1)
        packet = deserialize_message(frame, CompressedPacketFrame)
        frame_pub.publish(packet)
        quiet()
        assert not received
        with tempfile.TemporaryDirectory(prefix="robinw-cloud-switch.") as tmp:
            root = Path(tmp)
            for count, (name, data) in enumerate((("table_a", first), ("table_b", second)), 1):
                export_fixture(root, name, [(TABLE, data, 1_000_000_000), (FRAME, frame, 1_000_000_001)])
                table_pub.publish(deserialize_message(data, CompressedAngleCalibration))
                quiet()
                assert len(received) == count - 1
                frame_pub.publish(packet)
                wait_until(lambda: len(received) == count)
                verify_payload(received[-1], "front_mid", root / (name + "_pcd"))
                assert received[-1].header.stamp.sec * 1_000_000_000 + received[-1].header.stamp.nanosec == (
                    int(packet.sensor_timestamp_us * 1000)
                )
            assert received[0].data != received[1].data
            table_pub.publish(CompressedAngleCalibration(uncompressed_size=50512, data=[1, 2, 3]))
            quiet()
            frame_pub.publish(packet)
            wait_until(lambda: len(received) == 3)
            verify_payload(received[-1], "front_mid", root / "table_b_pcd")
        print("PASS: no table means no output; table replacement changes points; invalid table preserves last valid table")
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
        checker.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
