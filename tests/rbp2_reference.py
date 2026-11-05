"""Independent NumPy reader used to verify original packet bytes."""
import ctypes
import struct

import numpy as np
from raw_bag import load_zstd


def frame_metadata(frame, expected_level):
    payload = bytes(frame.data)
    trailer = struct.pack("<II4sBB", 0x184D2A5F, 6, b"RBWF", 2, expected_level)
    assert payload.endswith(trailer), (frame.frame_id, expected_level)
    return payload[:-len(trailer)]


def decode_message(frame):
    compressed = frame_metadata(frame, frame.data[-1])
    zstd = load_zstd()
    maximum = frame.uncompressed_size + frame.uncompressed_size // 8 + 1024
    buffer = ctypes.create_string_buffer(maximum)
    size = zstd.ZSTD_decompress(buffer, maximum, compressed, len(compressed))
    assert not zstd.ZSTD_isError(size) and size <= maximum
    data = memoryview(buffer.raw[:size])
    assert data[:4] == b"RBP2"
    changes = data[4]
    template = data[5:75]
    offsets = data[75:75 + changes]
    cursor = 75 + changes
    headers = []
    counts = {42: 0, 74: 0}
    for _ in range(frame.packet_count):
        header = bytearray(template)
        for offset in offsets:
            header[offset] = data[cursor]
            cursor += 1
        width, = struct.unpack_from("<H", header, 42)
        packet_size, = struct.unpack_from("<I", header, 10)
        assert width in counts and packet_size >= 70 and (packet_size - 70) % width == 0
        n = (packet_size - 70) // width
        counts[width] += n
        headers.append((header, width, n))

    def take(n):
        nonlocal cursor
        value = data[cursor:cursor + n]
        assert len(value) == n
        cursor += n
        return value

    def bits(n, width):
        result = np.zeros(n, np.uint32)
        for bit in range(0, width // 8 * 8, 8):
            result |= np.frombuffer(take(n), np.uint8).astype(np.uint32) << bit
        for bit in range(width // 8 * 8, width):
            plane = np.frombuffer(take((n + 7) // 8), np.uint8)
            result |= np.unpackbits(plane, count=n, bitorder="little").astype(np.uint32) << bit
        return result

    streams = {}
    for width, n in counts.items():
        hdr = np.frombuffer(take(n * 10), np.uint8).reshape(10, n).T
        hw = np.ascontiguousarray(hdr).view("<u2").reshape(n, 5)
        line = hw[:, 4] & 0x39ff
        order = np.lexsort((hw[:, 3], line))
        line = line[order]
        points = np.empty((n, (width - 10) // 4), "<u4")
        for column in points.T:
            mask = np.frombuffer(take((n + 7) // 8), np.uint8)
            keep = np.unpackbits(mask, count=n, bitorder="little").astype(bool)
            residual = bits(int(keep.sum()), 19)
            signed = (residual >> 1).astype(np.int32) ^ -(residual & 1).astype(np.int32)
            radius = np.zeros(n, np.uint32)
            if len(signed):
                groups = line[keep]
                start = np.r_[True, groups[1:] != groups[:-1]]
                cumsum = np.cumsum(signed, dtype=np.int64)
                base = np.r_[0, cumsum[:-1]]
                group_index = np.maximum.accumulate(np.where(start, np.arange(len(signed)), 0))
                radius[keep] = cumsum - base[group_index]
            column[order] = bits(n, 12) | (radius << 12) | (bits(n, 2) << 30)
        streams[width] = np.concatenate((hdr, points.view(np.uint8)), axis=1)
    assert cursor == len(data)
    positions = dict.fromkeys(counts, 0)
    packets = []
    for header, width, n in headers:
        start = positions[width]
        packets.append(bytes(header) + streams[width][start:start+n].tobytes())
        positions[width] += n
    assert sum(map(len, packets)) == frame.uncompressed_size
    return packets
