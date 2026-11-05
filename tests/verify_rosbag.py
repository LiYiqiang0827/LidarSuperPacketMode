#!/usr/bin/env python3
"""Compare native ROS 2 replay output with a separate read-only db3 parser."""

import ctypes
from hashlib import md5, sha256
import os
from pathlib import Path
from ament_index_python.packages import get_package_prefix
import subprocess
import struct
import sys
import time

HOME = Path.home()
TRIAL = Path(os.environ.get("ROBINW_TEST_TRIAL", HOME / "packet_data_compression/1103beicai_10_51_46_trial"))
BAG = TRIAL / "input/rosbag2_2026_11_03-10_51_46"
WORKSPACE = Path(__file__).resolve().parents[1]
from raw_bag import load_zstd, read_calibration, source_frames, split_message, topic_records, zstd_result
from rbp2_reference import decode_message, frame_metadata

os.environ.setdefault("ROS_DOMAIN_ID", "83")
os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"

import rclpy
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from robinw_packet_frame.msg import CompressedAngleCalibration, CompressedPacketFrame
from seyond.msg import SeyondPacket, SeyondScan

SENSORS = ("front_mid", "rear_mid")
LEVEL = {"front_mid": 7, "rear_mid": 9}
TABLE_QOS = QoSProfile(
    depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL
)


def expected_from_sqlite():
    db = next(BAG.glob("*.db3"))
    frames = {}
    tables = {}
    repeated = {}
    for sensor in SENSORS:
        topic = f"/iv_packets_{sensor}"
        frames[sensor] = [
            (idx, stamp, complete, tuple(map(len, packets)), sha256(b"".join(packets)).digest())
            for idx, stamp, complete, packets in source_frames(db, topic)
        ]
        tables[sensor] = read_calibration(db, topic)[0]
        repeated[sensor] = [
            sha256(table).digest()
            for msg in topic_records(db, topic, tables_only=True)
            for _, table in split_message(msg) if table
        ]
    sample_packets = next(
        packets for _, _, complete, packets in source_frames(db, "/iv_packets_front_mid") if complete
    )
    return frames, tables, repeated, sample_packets


def decode_calibration(table):
    assert table.uncompressed_size == 50512
    compressed = bytes(table.data)
    zstd = load_zstd()
    output = ctypes.create_string_buffer(table.uncompressed_size)
    size = zstd.ZSTD_decompress(output, len(output), compressed, len(compressed))
    assert size == table.uncompressed_size
    return zstd_result(zstd, output, size)


def main():
    expected, tables, repeats, sample_packets = expected_from_sqlite()
    compressed_bytes = dict.fromkeys(SENSORS, 0)
    table_sizes = dict.fromkeys(SENSORS, 0)
    seen = dict.fromkeys(SENSORS, 0)
    seen_tables = dict.fromkeys(SENSORS, 0)
    latched_tables = dict.fromkeys(SENSORS, 0)
    logs = Path(os.environ.get("ROBINW_TEST_LOG_DIR", WORKSPACE / "tests/results"))
    logs.mkdir(parents=True, exist_ok=True)
    seen_repeats = {sensor: [] for sensor in SENSORS}
    frame_subscriptions = []
    input_subscriptions = []
    processes = []
    streams = []
    rclpy.init()
    checker = rclpy.create_node("verify_robinw_packet_frames")

    def start(name, command):
        stream = (logs / (name + ".log")).open("w")
        streams.append(stream)
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT)
        processes.append(process)
        return process

    def frame_callback(sensor):
        def callback(frame):
            index = seen[sensor]
            assert index < len(expected[sensor]), f"{sensor}: extra frame {frame.frame_id}"
            frame_metadata(frame, LEVEL[sensor])
            packets = decode_message(frame)
            actual = (frame.frame_id, frame.sensor_timestamp_us, frame.complete,
                      tuple(map(len, packets)), sha256(b"".join(packets)).digest())
            compressed_bytes[sensor] += len(frame.data)
            wanted = expected[sensor][index]
            assert actual == wanted, f"{sensor}: frame {index} got {actual[:3]} expected {wanted[:3]}"
            seen[sensor] += 1
        return callback

    def table_callback(sensor, counters):
        def callback(table):
            assert decode_calibration(table) == tables[sensor], f"{sensor}: calibration content mismatch"
            table_sizes[sensor] = len(table.data)
            counters[sensor] += 1
        return callback

    def replay_callback(sensor):
        def callback(scan):
            for packet in scan.packets:
                if packet.has_table:
                    seen_repeats[sensor].append(sha256(bytes(packet.table)).digest())
        return callback

    try:
        for sensor in SENSORS:
            frame_subscriptions.append(checker.create_subscription(
                CompressedPacketFrame, f"/{sensor}/compressed_packet_frame", frame_callback(sensor), 32
            ))
            input_subscriptions.append(checker.create_subscription(
                SeyondScan, f"/iv_packets_{sensor}", replay_callback(sensor), 128
            ))
            checker.create_subscription(
                CompressedAngleCalibration, f"/{sensor}/compressed_angle_calibration",
                table_callback(sensor, seen_tables), TABLE_QOS
            )
            start(sensor, [
                str(Path(get_package_prefix("robinw_packet_frame")) / "lib/robinw_packet_frame/packet_frame_node"),
                "--ros-args", "-r", f"__ns:=/{sensor}", "-p", f"input_topic:=/iv_packets_{sensor}",
                *([] if sensor == "front_mid" else ["-p", f"zstd_level:={LEVEL[sensor]}"]),
            ])
        bag = start("bag", [
            "ros2", "launch", "robinw_packet_frame", "replay_packet_bag.launch.py",
            f"bag:={BAG}", "delay:=2",
        ])
        deadline = time.monotonic() + 40
        while time.monotonic() < deadline:
            rclpy.spin_once(checker, timeout_sec=0.1)
            if bag.poll() is not None:
                assert bag.returncode == 0, f"rosbag replay failed: {bag.returncode}"
                if all(seen[s] == len(expected[s]) and len(seen_repeats[s]) == len(repeats[s])
                       for s in SENSORS):
                    break
        for sensor in SENSORS:
            assert seen[sensor] == len(expected[sensor]), f"{sensor}: missing frames: {seen}"
            assert seen_repeats[sensor] == repeats[sensor], f"{sensor}: replayed table attachments differ"
        for subscription in frame_subscriptions + input_subscriptions:
            checker.destroy_subscription(subscription)

        # Frame acknowledgements prove the assembler consumed both table probes.
        assert tables["front_mid"] != tables["rear_mid"]
        client = AsyncParameterClient(checker, "/front_mid/packet_frame_node")
        assert client.wait_for_services(timeout_sec=4)
        for value, allowed in [(0, False), (9, True)]:
            future = client.set_parameters([Parameter("zstd_level", value=value)])
            rclpy.spin_until_future_complete(checker, future, timeout_sec=4)
            assert future.done() and future.result().results[0].successful == allowed

        probe_frames = []
        def probe_callback(frame):
            frame_metadata(frame, 9)
            probe_frames.extend(decode_message(frame))

        checker.create_subscription(
            CompressedPacketFrame, "/front_mid/compressed_packet_frame", probe_callback, 4
        )
        publisher = checker.create_publisher(SeyondScan, "/iv_packets_front_mid", 4)
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline and publisher.get_subscription_count() < 1:
            rclpy.spin_once(checker, timeout_sec=0.1)
        assert publisher.get_subscription_count() >= 1
        def send_table_probe(table):
            probe_frames.clear()
            packets = [SeyondPacket(data=packet) for packet in sample_packets]
            packets[0].has_table = True
            packets[0].table = table
            publisher.publish(SeyondScan(size=len(packets), is_last_scan=True, packets=packets))
            deadline = time.monotonic() + 4
            while time.monotonic() < deadline and not probe_frames:
                rclpy.spin_once(checker, timeout_sec=0.1)
            assert probe_frames == sample_packets, "Assembler did not consume the calibration probe"

        front_table = tables["front_mid"]
        old_md5 = md5(front_table[80:50000]).hexdigest()
        new_md5 = md5(tables["rear_mid"][80:50000]).hexdigest()
        assert old_md5 != new_md5
        node_log = logs / "front_mid.log"
        changes = "Angle table content changed"
        replay_log = node_log.read_text()
        assert f"Published first angle table (MD5 {old_md5})" in replay_log
        assert changes not in replay_log

        new_metadata = bytearray(front_table)
        timestamp, = struct.unpack_from("<d", new_metadata, 16)
        struct.pack_into("<d", new_metadata, 16, timestamp + 1000000)
        new_metadata[6] ^= 1  # A changed checksum must also stay outside the digest.
        send_table_probe(bytes(new_metadata))
        assert changes not in node_log.read_text(), "Timestamp/CRC changed the angle table MD5"

        send_table_probe(tables["rear_mid"])
        send_table_probe(tables["rear_mid"])
        probe_log = node_log.read_text()
        assert probe_log.count(changes) == 1, probe_log
        assert f"{changes} (MD5 {old_md5} -> {new_md5})" in probe_log
        for sensor in SENSORS:
            checker.create_subscription(
                CompressedAngleCalibration, f"/{sensor}/compressed_angle_calibration",
                table_callback(sensor, latched_tables), TABLE_QOS
            )
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline and not all(latched_tables.values()):
            rclpy.spin_once(checker, timeout_sec=0.1)
        assert all(value == 1 for value in seen_tables.values()), seen_tables
        assert all(value == 1 for value in latched_tables.values()), latched_tables

        for sensor in SENSORS:
            complete = sum(item[2] for item in expected[sensor])
            packets = sum(len(item[3]) for item in expected[sensor])
            print(f"{sensor}: {seen[sensor]} frames, {complete} complete, {packets} packets, "
                  f"Zstd {LEVEL[sensor]} frame payload {compressed_bytes[sensor]} bytes, "
                  f"calibration 50512 -> {table_sizes[sensor]} bytes, "
                  f"{len(seen_repeats[sensor])} input tables, one published and latched", flush=True)
        print("PASS: timestamp/CRC excluded from MD5; angle changes detected, first table stays latched", flush=True)
        print("PASS: each frame records its actual Zstd level, including after runtime parameter updates", flush=True)
        print("PASS: native RBP2 replay restored raw packets byte for byte", flush=True)
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                process.terminate()
            try:
                process.wait(timeout=4)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        for stream in streams:
            stream.close()
        checker.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
