#!/usr/bin/env python3
"""Test that PCD export uses only the latest preceding calibration in a bag."""
import argparse
import hashlib
import os
from pathlib import Path
import sqlite3
import subprocess
import tempfile

from rosbag2_py import ConverterOptions, SequentialWriter, StorageOptions, TopicMetadata

FRAME = "/front_mid/compressed_packet_frame"
TABLE = "/front_mid/compressed_angle_calibration"
REAR_TABLE = "/rear_mid/compressed_angle_calibration"


def messages(bag, topic, limit):
    db = next(bag.glob("*.db3"))
    with sqlite3.connect(db.resolve().as_uri() + "?mode=ro", uri=True) as conn:
        return [row[0] for row in conn.execute(
            "SELECT messages.data FROM messages JOIN topics ON messages.topic_id=topics.id "
            "WHERE topics.name=? ORDER BY messages.timestamp,messages.id LIMIT ?",
            (topic, limit),
        )]


def export_fixture(root, name, rows):
    bag = root / name
    output = root / (name + "_pcd")
    writer = SequentialWriter()
    writer.open(StorageOptions(uri=str(bag), storage_id="sqlite3"), ConverterOptions("cdr", "cdr"))
    writer.create_topic(TopicMetadata(
        0, FRAME, "robinw_packet_frame/msg/CompressedPacketFrame", "cdr"))
    writer.create_topic(TopicMetadata(
        0, TABLE, "robinw_packet_frame/msg/CompressedAngleCalibration", "cdr"))
    for topic, data, stamp in rows:
        writer.write(topic, data, stamp)
    writer.close()
    result = subprocess.run(
        ["ros2", "run", "robinw_packet_frame", "compressed_bag_to_pcd",
         str(bag), FRAME, TABLE, str(output)],
        check=True, stdout=subprocess.PIPE, text=True,
        env=dict(os.environ, TZ="Asia/Shanghai"),
    )
    hashes = {}
    for path in sorted(output.glob("*.pcd")):
        with path.open("rb") as stream:
            hashes[path.name] = hashlib.file_digest(stream, "sha256").hexdigest()
    return hashes, result.stdout


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("compressed_bag", type=Path)
    args = parser.parse_args()
    frames = messages(args.compressed_bag, FRAME, 5)
    first, = messages(args.compressed_bag, TABLE, 1)
    second, = messages(args.compressed_bag, REAR_TABLE, 1)
    assert len(frames) == 5 and first != second
    time = 1_000_000_000
    frame_rows = [(FRAME, frame, time + i) for i, frame in enumerate(frames)]
    with tempfile.TemporaryDirectory(prefix="robinw-calibration-order.") as temp:
        root = Path(temp)
        ref_a, _ = export_fixture(root, "reference_a", [(TABLE, first, time - 1), *frame_rows])
        ref_b, _ = export_fixture(root, "reference_b", [(TABLE, second, time - 1), *frame_rows])
        assert len(ref_a) == len(ref_b) == 5
        keys = sorted(ref_a)
        assert all(ref_a[key] != ref_b[key] for key in keys)

        # Both table updates share a timestamp with an adjacent frame. SQLite
        # orders equal receive timestamps by the recorded message ID.
        actual, log = export_fixture(root, "interleaved", [
            frame_rows[0], (TABLE, first, time), *frame_rows[1:3],
            (TABLE, second, time + 2), *frame_rows[3:],
        ])
        expected = {key: ref_a[key] for key in keys[1:3]}
        expected.update({key: ref_b[key] for key in keys[3:]})
        assert actual == expected
        assert "Skipped 1 frame(s) before first calibration; applied 2 calibration update(s)" in log
        print("PASS: skipped the frame before the first table; switched tables in bag order, including timestamp ties")

        actual, log = export_fixture(root, "no_calibration", frame_rows)
        assert not actual
        assert "Skipped 5 frame(s) before first calibration; applied 0 calibration update(s)" in log
        print("PASS: a bag with no calibration exports no PCD")


if __name__ == "__main__":
    main()
