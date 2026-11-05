#!/usr/bin/env python3
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import signal
import sqlite3
import struct
import subprocess
import sys
import time

from ament_index_python.packages import get_package_prefix
import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from robinw_packet_frame.msg import CompressedAngleCalibration, CompressedPacketFrame

TESTS = Path(__file__).resolve().parent
BIN = Path(get_package_prefix("robinw_packet_frame")) / "lib/robinw_packet_frame"
from verify_cloud_decoder_node import FIELDS, ROS_TYPES, ZONE
from verify_rosbag import BAG, expected_from_sqlite, frame_metadata, decode_message, decode_calibration

SENSORS = ("front_mid", "rear_mid")
INPUTS = [f"/iv_packets_{sensor}" for sensor in SENSORS]
POINTS = [f"/{sensor}/points" for sensor in SENSORS]
COMPRESSED = [f"/{sensor}/{kind}" for sensor in SENSORS
              for kind in ("compressed_packet_frame", "compressed_angle_calibration")]
PLAY_QOS = TESTS / "raw_playback_qos.yaml"
RECORD_QOS = TESTS / "point_recorder_qos.yaml"
PACKED_DTYPE = np.dtype(list(FIELDS.items()))

def rows_for(bag, topic):
    file = next(bag.glob("*.db3"))
    with sqlite3.connect(file.as_uri() + "?mode=ro", uri=True) as db:
        yield from db.execute(
            "SELECT messages.data FROM messages JOIN topics ON topics.id=messages.topic_id "
            "WHERE topics.name=? ORDER BY messages.timestamp,messages.id", (topic,))

def stamp_name(stamp_ns):
    sec, ns = divmod(stamp_ns, 1_000_000_000)
    return f"{datetime.fromtimestamp(sec, ZONE):%Y_%m_%d-%H_%M_%S}_{ns // 1_000_000:03}.pcd"

def capture(level, target):
    output = target / f"pointcloud_zstd{level}"
    processes = {}
    node = None
    def start(name, command):
        with (target / f"{name}.log").open("w") as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
        processes[name] = process
        return process
    def stop(name):
        process = processes[name]
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                return process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
        return process.wait()
    audit = {sensor: [] for sensor in SENSORS}
    rclpy.init()
    try:
        node = rclpy.create_node("cloud_e2e_audit")
        def on_frame(frame, sensor):
            tail = bytes(frame.data[-14:])
            expected_tail = struct.pack("<II4sBB", 0x184D2A5F, 6, b"RBWF", 3, level)
            assert tail == expected_tail, (sensor, frame.frame_id, tail)
            audit[sensor].append(("frame", frame.frame_id, int(frame.sensor_timestamp_us * 1000),
                                  frame.complete, len(frame.data), hashlib.sha256(frame.data).hexdigest()))
        def on_table(table, sensor):
            audit[sensor].append(("calibration", hashlib.sha256(table.data).hexdigest()))
        for sensor, topic in zip(SENSORS, INPUTS):
            namespace = f"__ns:=/{sensor}"
            start(f"compress_{sensor}", [
                str(BIN / "packet_frame_node"), "--ros-args", "-r", namespace,
                "-p", f"input_topic:={topic}", "-p", f"zstd_level:={level}"])
            start(f"decode_{sensor}", [
                str(BIN / "packet_cloud_decoder_node"), "--ros-args", "-r", namespace,
                "-p", f"frame_id:={sensor}_lidar"])
            node.create_subscription(
                CompressedPacketFrame, f"/{sensor}/compressed_packet_frame",
                lambda msg, s=sensor: on_frame(msg, s), 32)
            node.create_subscription(
                CompressedAngleCalibration, f"/{sensor}/compressed_angle_calibration",
                lambda msg, s=sensor: on_table(msg, s),
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        start("recorder", [
            "ros2", "bag", "record", "--disable-keyboard-controls", "--storage", "sqlite3",
            "--max-cache-size", "134217728", "--qos-profile-overrides-path", str(RECORD_QOS),
            "-o", str(output), "--topics", *POINTS, *COMPRESSED])
        deadline = time.monotonic() + 15
        while not (
            all(node.count_subscribers(topic) == 1 for topic in INPUTS + POINTS) and
            all(node.count_publishers(topic) == 1 for topic in POINTS) and
            all(node.count_subscribers(f"/{sensor}/{kind}") == 3 for sensor in SENSORS
                for kind in ("compressed_packet_frame", "compressed_angle_calibration"))
        ):
            assert time.monotonic() < deadline and all(p.poll() is None for p in processes.values()), (
                "Recording graph did not become ready")
            rclpy.spin_once(node, timeout_sec=0.1)
        player = start("player", [
            "ros2", "bag", "play", str(BAG), "--rate", "1", "--delay", "2",
            "--disable-keyboard-controls", "--qos-profile-overrides-path", str(PLAY_QOS),
            "--wait-for-all-acked", "5000", "--topics", *INPUTS])
        deadline = time.monotonic() + 40
        while player.poll() is None:
            assert time.monotonic() < deadline, "Replay timed out"
            rclpy.spin_once(node, timeout_sec=0.1)
        assert player.returncode == 0
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert stop("recorder") == 0, "Recorder failed to close cleanly"
    finally:
        for name in reversed(processes):
            stop(name)
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
    (target / "compressed_audit.json").write_text(json.dumps(audit, indent=2) + "\n")
    with (target / "bag_info.log").open("w") as stream:
        subprocess.run(["ros2", "bag", "info", str(output)], stdout=stream, check=True)
    return output, audit

def pcd_payload_hash(path, expected_points):
    with path.open("rb") as stream:
        points = None
        for line in stream:
            if line.startswith(b"POINTS "):
                points = int(line.split()[1])
            if line == b"DATA binary\n":
                break
        assert points == expected_points
        return hashlib.file_digest(stream, "sha256").hexdigest()

def validate(level, output, audit, expected, tables, reference_dir):
    db = next(output.glob("*.db3"))
    with sqlite3.connect(db.as_uri() + "?mode=ro", uri=True) as conn:
        assert conn.execute("PRAGMA quick_check").fetchone() == ("ok",)
        assert {name for name, in conn.execute("SELECT name FROM topics")} == set(POINTS + COMPRESSED)
    results = {}
    for sensor in SENSORS:
        refs = {}
        payload_bytes = 0
        for i, (cdr,) in enumerate(rows_for(output, f"/{sensor}/compressed_packet_frame")):
            frame = deserialize_message(cdr, CompressedPacketFrame)
            frame_metadata(frame, level)
            packets = decode_message(frame)
            actual = (frame.frame_id, frame.sensor_timestamp_us, frame.complete, tuple(map(len, packets)),
                      hashlib.sha256(b"".join(packets)).digest())
            assert actual == expected[sensor][i]
            refs[frame.frame_id] = (int(frame.sensor_timestamp_us * 1000), frame.complete,
                                    len(frame.data), hashlib.sha256(frame.data).hexdigest())
            payload_bytes += len(frame.data)
        assert len(refs) == len(expected[sensor])
        (cdr,), = rows_for(output, f"/{sensor}/compressed_angle_calibration")
        table = deserialize_message(cdr, CompressedAngleCalibration)
        assert decode_calibration(table) == tables[sensor]
        calibration_hash = hashlib.sha256(table.data).hexdigest()
        sensor_pcd = reference_dir / f"level_{level}_attempt_1" / "pcd_verified" / sensor
        seen = audit[sensor]
        assert len(seen) == 149 and [item for item in seen if item[0] == "calibration"] == [
            ("calibration", calibration_hash)
        ]
        assert [(item[1], item[2:]) for item in seen if item[0] == "frame"] == list(refs.items())
        first = next(i for i, item in enumerate(seen) if item[0] == "calibration")
        assert first == 12, (level, sensor, first)
        available = [item[2] for item in seen[first + 1:]]
        seen_stamps = []
        manifest = {}
        total_points = 0
        for data, in rows_for(output, f"/{sensor}/points"):
            cloud = deserialize_message(data, PointCloud2)
            assert cloud.header.frame_id == f"{sensor}_lidar"
            assert cloud.height == 1 and not cloud.is_bigendian
            assert cloud.row_step == cloud.width * cloud.point_step == len(cloud.data)
            assert [(f.name, f.datatype, f.count) for f in cloud.fields] == [
                (name, ROS_TYPES[fmt], 1) for name, fmt in FIELDS.items()
            ]
            stamp = cloud.header.stamp.sec * 1_000_000_000 + cloud.header.stamp.nanosec
            seen_stamps.append(stamp)
            name = stamp_name(stamp)
            dtype = np.dtype({"names": list(FIELDS), "formats": list(FIELDS.values()),
                              "offsets": [f.offset for f in cloud.fields], "itemsize": cloud.point_step})
            packed = np.frombuffer(cloud.data, dtype=dtype, count=cloud.width).astype(PACKED_DTYPE)
            digest = hashlib.sha256(packed.tobytes()).hexdigest()
            assert digest == pcd_payload_hash(sensor_pcd / name, cloud.width), (level, sensor, name)
            manifest[name] = digest
            total_points += cloud.width
        assert seen_stamps == available, (level, sensor, len(seen_stamps), len(available))
        assert len(manifest) == len(seen_stamps) == 136
        results[sensor] = {"payload_bytes": payload_bytes, "compressed_frames": len(refs), "skipped_before_calibration": first,
                           "recorded_clouds": len(seen_stamps), "points": total_points, "payload_hashes": manifest}
    return db.stat().st_size, results

def main():
    parser = argparse.ArgumentParser(description="Record live ROS 2 point clouds and compare against reference PCD.")
    parser.add_argument("reference_dir", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--levels", type=int, nargs="+", choices=(1, 3, 7), default=(1, 3, 7))
    args = parser.parse_args()
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=False)
    raw_frames, tables, _, _ = expected_from_sqlite()
    summary = {}
    for level in args.levels:
        for attempt in range(1, 4):
            target = output_dir / f"zstd{level}_attempt{attempt}"
            target.mkdir()
            try:
                bag, audit = capture(level, target)
                size, results = validate(level, bag, audit, raw_frames, tables, args.reference_dir)
                break
            except (AssertionError, subprocess.SubprocessError) as error:
                print(f"Level {level}, attempt {attempt} failed: {error}", flush=True)
                if attempt == 3:
                    raise
        summary[str(level)] = {"bag": str(bag), "db3_bytes": size, "sensors": results}
        for sensor, data in results.items():
            print(f"Zstd {level} {sensor}: {data['compressed_frames']} compressed, "
                  f"{data['skipped_before_calibration']} skipped, {data['recorded_clouds']} point clouds "
                  f"({data['points']:,} points); all point fields byte-identical to PCD")
        print(f"Zstd {level}: {size:,} byte SQLite bag at {bag}", flush=True)
        (output_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    for sensor in SENSORS:
        first, *remaining = (run["sensors"][sensor]["payload_hashes"] for run in summary.values())
        assert all(hashes == first for hashes in remaining)
    count = sum(sensor["recorded_clouds"] for run in summary.values() for sensor in run["sensors"].values())
    print(f"PASS: all {count} recorded PointCloud2 messages match PCD references")

if __name__ == "__main__":
    main()
