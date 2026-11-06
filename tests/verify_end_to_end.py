import argparse
from array import array
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import tempfile
import struct
import subprocess
import time
from uuid import uuid4

os.environ.setdefault("ROS_DOMAIN_ID", "118")
if os.environ.get("ROS_DISTRO") == "humble":
    os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")
else:
    os.environ.setdefault("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")

import numpy as np
import rclpy
import rosbag2_py as bag
import yaml
from ament_index_python.packages import get_package_prefix
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.serialization import deserialize_message, serialize_message
from robinw_packet_frame.msg import CompressedAngleCalibration, CompressedPacketFrame
from sensor_msgs.msg import PointCloud2
from seyond.msg import SeyondScan

capture = Path.home() / "mikelee/seyond_capture_front_mid_2026_11_06-18_11_58"
parser = argparse.ArgumentParser(description="Verify hash-based calibration and PCD export.")
parser.add_argument("--bag", type=Path, default=capture / "packet_rosbag")
parser.add_argument("--output", type=Path)
args = parser.parse_args()
OUT = args.output or Path(tempfile.mkdtemp(prefix="seyond-hash-"))
OUT.mkdir(parents=True, exist_ok=True)
BIN = Path(get_package_prefix("robinw_packet_frame")) / "lib/robinw_packet_frame"
CAP = args.bag.parent


def fix_crc(packet):
    crc = 0xffffffff
    for byte in packet[:6] + packet[10:]:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0x82f63b78 if crc & 1 else 0)
    struct.pack_into("<I", packet, 6, crc ^ 0xffffffff)


def topic_metadata(index, name, msg_type):
    kwargs = dict(name=name, type=msg_type, serialization_format="cdr")
    try:
        return bag.TopicMetadata(id=index, **kwargs)
    except TypeError:
        return bag.TopicMetadata(offered_qos_profiles="", **kwargs)


r = bag.SequentialReader()
metadata = yaml.safe_load((args.bag / "metadata.yaml").read_text())["rosbag2_bagfile_information"]
r.open(bag.StorageOptions(uri=str(args.bag), storage_id=metadata["storage_identifier"]),
       bag.ConverterOptions("", ""))
packet_filter = bag.StorageFilter()
packet_filter.topics = ["/iv_packets_front_mid"]
r.set_filter(packet_filter)
groups,current=[],[]
while len(groups)<4 and r.has_next():
    _,payload,_=r.read_next()
    scan=deserialize_message(payload,SeyondScan)
    for packet in scan.packets:
        packet.has_table=False
        packet.table=array("B")
    current.append(scan)
    if scan.is_last_scan:
        groups.append(current)
        current=[]
a,repeated,b=groups[1:4]
table_a=bytearray((CAP/"anglehv_table.bin").read_bytes())
same=bytearray(table_a)
same[20]^=1
fix_crc(same)
table_b=bytearray(table_a)
for offset in range(80,50000,4):
    value,=struct.unpack_from("<H",table_b,offset)
    struct.pack_into("<H",table_b,offset,(value+64)&65535)
fix_crc(table_b)
def id_for(table):
    return hashlib.sha256(table[70:72]+table[80:50000]).digest()[:16]
assert id_for(table_a)==id_for(same)!=id_for(table_b)
for batch,table in ((a,table_a),(repeated,same),(b,table_b)):
    p=batch[0].packets[0]
    p.has_table=True
    p.table=array("B",table)
del b[4]

rclpy.init()
n=rclpy.create_node("codec_hash_probe")
probe = f"/hash_probe_{uuid4().hex[:12]}"
raw_topic = f"{probe}/raw"
tx_namespace = f"{probe}/tx"
rx_namespace = f"{probe}/rx"
points_topic = f"{rx_namespace}/points"
reliable_table=QoSProfile(depth=4,durability=DurabilityPolicy.TRANSIENT_LOCAL)
raw=n.create_publisher(SeyondScan, raw_topic, 100)
txframes, txtables, clouds=[],[],[]
n.create_subscription(CompressedPacketFrame, f"{tx_namespace}/compressed_packet_frame", txframes.append, 16)
n.create_subscription(CompressedAngleCalibration, f"{tx_namespace}/compressed_angle_calibration",
                      txtables.append, reliable_table)
rxframes=n.create_publisher(CompressedPacketFrame, f"{rx_namespace}/compressed_packet_frame", 16)
rxtables=n.create_publisher(CompressedAngleCalibration, f"{rx_namespace}/compressed_angle_calibration",
                            reliable_table)
def pointcloud(m):
    xyz=np.ndarray((m.width,3),dtype="<f4",buffer=m.data,strides=(m.point_step,4))
    clouds.append({"stamp_ns":m.header.stamp.sec*10**9+m.header.stamp.nanosec,
                   "width":m.width,"xyz":hashlib.sha256(xyz.tobytes()).hexdigest()})
n.create_subscription(PointCloud2, points_topic, pointcloud, 16)

children=[]
def spin_for(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        rclpy.spin_once(n,timeout_sec=.015)
def wait_for(predicate, seconds=6):
    end=time.monotonic()+seconds
    while not predicate():
        assert time.monotonic()<end and all(p.poll() is None for p in children)
        rclpy.spin_once(n,timeout_sec=.025)
try:
    for exe, namespace, parameters in (
        ("packet_frame_node", tx_namespace, ["-p", f"input_topic:={raw_topic}"]),
        ("packet_cloud_decoder_node", rx_namespace, []),
    ):
        with (OUT/f"{exe}.log").open("w") as log:
            children.append(subprocess.Popen([str(BIN/exe),"--ros-args","-r",f"__ns:={namespace}",*parameters],
                                             stdout=log,stderr=subprocess.STDOUT))
    wait_for(lambda:raw.get_subscription_count()>0 and rxframes.get_subscription_count()>0 and
                    rxtables.get_subscription_count()>0 and n.count_publishers(points_topic)>0)
    spin_for(.4)
    for batch in (a,repeated,b):
        for scan in batch:
            raw.publish(scan)
            spin_for(.008)
    wait_for(lambda:len(txframes)==3 and len(txtables)==2)
    assert [bytes(x.calibration_id) for x in txframes]==[id_for(table_a),id_for(table_a),id_for(table_b)]
    assert [bytes(x.calibration_id) for x in txtables]==[id_for(table_a),id_for(table_b)]
    assert [x.complete for x in txframes]==[True,True,False]
    assert all(bytes(x.data[-14:])==struct.pack("<II4sBB",0x184d2a5f,6,b"RBWF",3,1) for x in txframes)
    rxtables.publish(txtables[0])
    rxframes.publish(txframes[0])
    wait_for(lambda:len(clouds)==1)
    rxframes.publish(txframes[2])
    spin_for(.12)
    assert len(clouds)==1, "decoded with old table"
    spoof=deepcopy(txtables[1])
    spoof.calibration_id=txtables[0].calibration_id
    rxtables.publish(spoof)
    spin_for(.08)
    assert len(clouds)==1, "accepted mismatched table ID"
    rxtables.publish(txtables[1])
    wait_for(lambda:len(clouds)==2, .45)
    rxframes.publish(txframes[0])
    wait_for(lambda:len(clouds)==3)
    assert clouds[2]==clouds[0], "previous matching table not reused"
    assert clouds[1]["stamp_ns"]==int(txframes[2].sensor_timestamp_us*1000)
    assert 0<clouds[1]["width"]<clouds[0]["width"]
    absent=deepcopy(txframes[0])
    absent.calibration_id=array("B",b"\xff"*16)
    rxframes.publish(absent)
    spin_for(.62)
    assert len(clouds)==3
finally:
    n.destroy_node()
    rclpy.shutdown()
    for child in children:
        child.terminate()
    for child in children:
        child.wait(5)

# Put the new table after its first frame on disk to exercise offline export.
writer=bag.SequentialWriter()
writer.open(bag.StorageOptions(uri=str(OUT/"out_of_order_bag"),storage_id="sqlite3"),bag.ConverterOptions("",""))
topics=("/hash_tx/compressed_packet_frame","/hash_tx/compressed_angle_calibration")
types=("robinw_packet_frame/msg/CompressedPacketFrame","robinw_packet_frame/msg/CompressedAngleCalibration")
for i,(name,type_name) in enumerate(zip(topics,types)):
    writer.create_topic(topic_metadata(i, name, type_name))
for i,(topic,msg) in enumerate(((1,txtables[0]),(0,txframes[0]),(0,txframes[2]),(1,txtables[1]))):
    writer.write(topics[topic],serialize_message(msg),1_793_959_919_000_000_000+i*100_000_000)
del writer
report={"calibration_ids":[bytes(t.calibration_id).hex() for t in txtables],
        "frame_complete":[f.complete for f in txframes],"clouds":clouds,
        "wait_for_matching_table":True,"mismatched_table_rejected":True,
        "missing_id_timed_out":True,"rbp3_trailer_unchanged":True}
(OUT/"result.json").write_text(json.dumps(report,indent=2)+"\n")
print(json.dumps(report,indent=2))

subprocess.run([str(BIN / "compressed_bag_to_pcd"), str(OUT / "out_of_order_bag"),
                *topics, str(OUT / "pcd")], check=True)
files = sorted((OUT / "pcd").glob("*.pcd"))
assert len(files) == 2
for file, expected in zip(files, clouds[:2]):
    header, payload = file.read_bytes().split(b"DATA binary\n", 1)
    fields = {key: values.split() for line in header.decode().splitlines()
              if not line.startswith("#") for key, values in (line.split(" ", 1),)}
    assert fields["FIELDS"][:3] == ["x", "y", "z"]
    stride = sum(int(s) * int(c) for s, c in zip(fields["SIZE"], fields["COUNT"]))
    count = int(fields["POINTS"][0])
    xyz = np.ndarray((count, 3), dtype="<f4", buffer=payload, strides=(stride, 4))
    assert count == expected["width"]
    assert hashlib.sha256(xyz.tobytes()).hexdigest() == expected["xyz"]
print(f"PCD export and online decode match in {OUT}")
