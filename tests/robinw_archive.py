#!/usr/bin/env python3
"""Archive a single Robin W type-13 topic, preserving its original packet bytes."""

import argparse
import ctypes
import ctypes.util
from functools import lru_cache
import hashlib
import itertools
from pathlib import Path
import sqlite3
import struct
import zlib

MAGIC = b"RBWPKT1\0"
MAGIC_ZSTD = b"RBWZST1\0"
MAGIC_ZSTD_SHUFFLE = b"RBWSHF1\0"
CODECS = {"zlib": MAGIC, "zstd": MAGIC_ZSTD, "zstd-shuffle": MAGIC_ZSTD_SHUFFLE}
FILE = struct.Struct("<8sIQQ")  # magic, calibration bytes, index offset, frame count
INDEX = struct.Struct("<QdQ")  # frame ID, first recorded packet sensor time (us), record offset
FRAME = struct.Struct("<BIII")  # complete, packet count, original bytes, compressed bytes
HEADER_SIZE = 70
TABLE_SIZE = 50512


@lru_cache(maxsize=1)
def load_zstd():
    library = ctypes.util.find_library("zstd")
    if not library:
        raise RuntimeError("The Zstd archive codec requires libzstd")
    zstd = ctypes.CDLL(library)
    size = ctypes.c_size_t
    pointer = ctypes.c_void_p
    signatures = {
        "ZSTD_compressBound": ([size], size),
        "ZSTD_compress": ([pointer, size, pointer, size, ctypes.c_int], size),
        "ZSTD_decompress": ([pointer, size, pointer, size], size),
        "ZSTD_isError": ([size], ctypes.c_uint),
        "ZSTD_getErrorName": ([size], ctypes.c_char_p),
    }
    for name, (args, result) in signatures.items():
        function = getattr(zstd, name)
        function.argtypes = args
        function.restype = result
    return zstd


def zstd_result(zstd, buffer, size):
    if zstd.ZSTD_isError(size):
        raise ValueError(zstd.ZSTD_getErrorName(size).decode())
    return buffer.raw[:size]


def shuffle_blocks(payload):
    return b"".join(payload[lane::42] for lane in range(42))


def unshuffle_blocks(payload):
    blocks, remainder = divmod(len(payload), 42)
    if remainder:
        raise ValueError("Invalid single-return block payload size")
    output = bytearray(len(payload))
    for lane in range(42):
        output[lane::42] = payload[lane * blocks:(lane + 1) * blocks]
    return bytes(output)


def read_exact(stream, size):
    data = stream.read(size)
    if len(data) != size:
        raise ValueError("Truncated archive")
    return data


def topic_records(db, topic, tables_only=False):
    with sqlite3.connect(db.resolve().as_uri() + "?mode=ro", uri=True) as conn:
        row = conn.execute("SELECT id,type,serialization_format FROM topics WHERE name=?", (topic,)).fetchone()
        if row is None or row[1:] != ("seyond/msg/SeyondScan", "cdr"):
            raise ValueError(f"Missing SeyondScan/CDR topic: {topic}")
        query = "SELECT data FROM messages WHERE topic_id=?"
        if tables_only:
            query += f" AND length(data)>={TABLE_SIZE}"
        for (data,) in conn.execute(query + " ORDER BY timestamp,id", (row[0],)):
            yield data


def split_message(data):
    if data[:2] != b"\x00\x01":
        raise ValueError("Expected little-endian CDR")
    size, = struct.unpack_from("<I", data, 12)
    count, = struct.unpack_from("<I", data, 20)
    if size != count:
        raise ValueError("Unexpected SeyondScan packet count")
    offset = 24
    for _ in range(count):
        has_table = data[offset]
        offset = (offset + 4) & ~3
        packet_size, = struct.unpack_from("<I", data, offset)
        offset += 4
        packet = data[offset:offset + packet_size]
        offset = (offset + packet_size + 3) & ~3
        table_size, = struct.unpack_from("<I", data, offset)
        offset += 4
        table = data[offset:offset + table_size]
        offset += table_size
        if bool(has_table) != bool(table_size) or offset > len(data):
            raise ValueError("Invalid SeyondPacket vector")
        yield packet, table
    if not 0 <= len(data) - offset <= 3:
        raise ValueError("Unexpected CDR tail")


def read_calibration(db, topic):
    calibration = None
    copies = vector_bytes = 0
    for data in topic_records(db, topic, tables_only=True):
        for _, table in split_message(data):
            if not table:
                continue
            packet_size, = struct.unpack_from("<I", table, 10)
            if table[:2] != b"\x6a\x17" or table[38] != 100 or packet_size != TABLE_SIZE or len(table) < packet_size:
                raise ValueError("Unexpected Robin W calibration packet")
            effective = table[:packet_size]
            if calibration is None:
                calibration = effective
            elif effective[HEADER_SIZE:] != calibration[HEADER_SIZE:]:
                raise ValueError("Calibration changed within topic")
            copies += 1
            vector_bytes += len(table)
    if calibration is None:
        raise ValueError(f"No calibration available in {topic}")
    return calibration, copies, vector_bytes


def packet_header(packet):
    if len(packet) < HEADER_SIZE or packet[:2] != b"\x6a\x17" or packet[38] != 13:
        raise ValueError("Expected Robin W Compact packet")
    size, = struct.unpack_from("<I", packet, 10)
    timestamp, = struct.unpack_from("<d", packet, 16)
    frame, sub = struct.unpack_from("<QH", packet, 26)
    blocks = int.from_bytes(packet[39:42], "little")
    block_size, flags = struct.unpack_from("<H4xH", packet, 42)
    return_mode = (flags >> 2) & 7
    if {1: 42, 2: 74}.get(return_mode) != block_size or size != len(packet) or size != HEADER_SIZE + blocks * block_size:
        raise ValueError("Inconsistent packet size or return mode")
    return frame, timestamp, sub, bool(flags & 128)


def source_frames(db, topic):
    frame_id = None
    packets = []
    last = False
    for message in topic_records(db, topic):
        for packet, _ in split_message(message):
            current, time_us, sub, last_packet = packet_header(packet)
            if current != frame_id:
                if packets:
                    yield frame_id, timestamp, continuous and last, packets
                packets = []
                frame_id = current
                timestamp = time_us
                continuous = True
            continuous &= (sub == len(packets))
            packets.append(packet)
            last = last_packet
    if packets:
        yield frame_id, timestamp, continuous and last, packets


def encode_frame(packets, magic, level):
    template = packets[0][:HEADER_SIZE]
    changed = bytes(index for index in range(HEADER_SIZE)
                    if any(packet[index] != template[index] for packet in packets))
    differences = bytes(packet[index] for packet in packets for index in changed)
    payload = b"".join(packet[HEADER_SIZE:] for packet in packets)
    if magic == MAGIC_ZSTD_SHUFFLE:
        if any(packet[42:44] != b"\x2a\x00" for packet in packets):
            raise ValueError("Block shuffle requires 42-byte single-return blocks")
        payload = shuffle_blocks(payload)
    frame = bytes([len(changed)]) + template + changed + differences + payload
    if magic == MAGIC:
        return zlib.compress(frame, level)
    zstd = load_zstd()
    capacity = zstd.ZSTD_compressBound(len(frame))
    output = ctypes.create_string_buffer(capacity)
    size = zstd.ZSTD_compress(output, capacity, frame, len(frame), level)
    return zstd_result(zstd, output, size)


def decode_frame(stream, offset, magic):
    stream.seek(offset)
    complete, count, raw_size, compressed_size = FRAME.unpack(read_exact(stream, FRAME.size))
    compressed = read_exact(stream, compressed_size)
    if magic == MAGIC:
        frame = zlib.decompress(compressed)
    else:
        # Header normalization adds at most one template and its offsets.
        capacity = raw_size + 2 * HEADER_SIZE + 1
        zstd = load_zstd()
        output = ctypes.create_string_buffer(capacity)
        size = zstd.ZSTD_decompress(output, capacity, compressed, len(compressed))
        frame = zstd_result(zstd, output, size)
    changed_count = frame[0]
    template = frame[1:1+HEADER_SIZE]
    changes_begin = 1 + HEADER_SIZE
    delta_start = changes_begin + changed_count
    changes = frame[changes_begin:delta_start]
    payload_offset = delta_start + count * changed_count
    if magic == MAGIC_ZSTD_SHUFFLE:
        frame = frame[:payload_offset] + unshuffle_blocks(frame[payload_offset:])
    packets = []
    for _ in range(count):
        header = bytearray(template)
        for index in changes:
            header[index] = frame[delta_start]
            delta_start += 1
        size, = struct.unpack_from("<I", header, 10)
        stop = payload_offset + size - HEADER_SIZE
        packets.append(bytes(header) + frame[payload_offset:stop])
        payload_offset = stop
    if payload_offset != len(frame) or sum(map(len, packets)) != raw_size:
        raise ValueError("Frame length mismatch")
    return bool(complete), packets


def open_index(stream):
    magic, calibration_size, index_offset, count = FILE.unpack(read_exact(stream, FILE.size))
    if magic not in CODECS.values():
        raise ValueError("Unknown archive format")
    calibration = read_exact(stream, calibration_size)
    stream.seek(index_offset)
    entries = [INDEX.unpack(read_exact(stream, INDEX.size)) for _ in range(count)]
    if stream.read(1):
        raise ValueError("Unexpected bytes after the frame index")
    return calibration, entries, magic


def write_archive(db, topic, output, calibration, magic, level):
    entries = []
    with output.open("xb") as stream:
        stream.write(FILE.pack(magic, len(calibration), 0, 0))
        stream.write(calibration)
        for frame, timestamp, complete, packets in source_frames(db, topic):
            entries.append((frame, timestamp, stream.tell()))
            encoded = encode_frame(packets, magic, level)
            stream.write(FRAME.pack(complete, len(packets), sum(map(len, packets)), len(encoded)))
            stream.write(encoded)
        index_offset = stream.tell()
        for entry in entries:
            stream.write(INDEX.pack(*entry))
        stream.seek(0)
        stream.write(FILE.pack(magic, len(calibration), index_offset, len(entries)))


def verify_archive(db, topic, archive, calibration, table_copies, table_bytes):
    total_packets = raw_bytes = complete_frames = 0
    frame_sizes = []
    incomplete = []
    digest = hashlib.sha256()
    with archive.open("rb") as stream:
        restored_table, entries, magic = open_index(stream)
        if restored_table != calibration:
            raise ValueError("Calibration round trip failed")
        for entry, original in itertools.zip_longest(entries, source_frames(db, topic)):
            if entry is None or original is None:
                raise ValueError("Frame count mismatch")
            frame, timestamp, offset = entry
            complete, packets = decode_frame(stream, offset, magic)
            if (frame, timestamp, complete, packets) != original:
                raise ValueError(f"Packet round trip failed on frame {frame}")
            stream.seek(offset)
            frame_sizes.append(FRAME.unpack(read_exact(stream, FRAME.size))[-1])
            if complete:
                complete_frames += 1
            else:
                incomplete.append(frame)
            total_packets += len(packets)
            raw_bytes += sum(map(len, packets))
            for packet in packets:
                digest.update(packet)
    if not entries:
        raise ValueError("No packets in archive")
    size = archive.stat().st_size
    print(f"{topic} frames={len(entries)} complete={complete_frames} incomplete={incomplete}")
    print(f"packets={total_packets} packet_bytes={raw_bytes} table_copies={table_copies} "
          f"table_vector_bytes={table_bytes} calibration_bytes={len(calibration)} archive_bytes={size}")
    print(f"frame_compressed_avg={sum(frame_sizes)/len(frame_sizes):.1f} "
          f"min={min(frame_sizes)} max={max(frame_sizes)} packet_sha256={digest.hexdigest()}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    actions = parser.add_subparsers(dest="action", required=True)
    for action in ("pack", "verify"):
        sub = actions.add_parser(action)
        sub.add_argument("db3", type=Path)
        sub.add_argument("topic")
        sub.add_argument("archive", type=Path)
        if action == "pack":
            sub.add_argument("--codec", choices=CODECS, default="zlib")
            sub.add_argument("--level", type=int, help="Default: 6 for zlib, 19 for Zstd")
    extract = actions.add_parser("extract")
    extract.add_argument("archive", type=Path)
    extract.add_argument("frame", type=int)
    extract.add_argument("packets", type=Path, help="Output packets, each prefixed with a little-endian uint32 length")
    extract.add_argument("calibration", type=Path)
    args = parser.parse_args()
    if args.action == "extract":
        with args.archive.open("rb") as stream:
            calibration, entries, magic = open_index(stream)
            matches = [entry for entry in entries if entry[0] == args.frame]
            if len(matches) != 1:
                raise ValueError("Frame ID missing or ambiguous")
            _, timestamp, offset = matches[0]
            complete, packets = decode_frame(stream, offset, magic)
            with args.packets.open("xb") as output:
                for packet in packets:
                    output.write(struct.pack("<I", len(packet)))
                    output.write(packet)
            with args.calibration.open("xb") as output:
                output.write(calibration)
            print(f"frame={args.frame} first_packet_sensor_us={timestamp:.6f} complete={complete} packets={len(packets)}")
        return
    calibration, copies, table_bytes = read_calibration(args.db3, args.topic)
    if args.action == "pack":
        staged = args.archive.with_suffix(args.archive.suffix + ".partial")
        if args.archive.exists():
            parser.error(f"Archive already exists: {args.archive}")
        default_level = 6 if args.codec == "zlib" else 19
        level = args.level if args.level is not None else default_level
        write_archive(args.db3, args.topic, staged, calibration, CODECS[args.codec], level)
        verify_archive(args.db3, args.topic, staged, calibration, copies, table_bytes)
        args.archive.hardlink_to(staged)
        staged.unlink()
    else:
        verify_archive(args.db3, args.topic, args.archive, calibration, copies, table_bytes)


if __name__ == "__main__":
    main()
