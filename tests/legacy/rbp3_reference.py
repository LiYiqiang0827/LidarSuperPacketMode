"""Independent NumPy reader used to verify original packet bytes."""
import ctypes
import struct

import numpy as np
from raw_bag import load_zstd


def frame_metadata(frame, expected_level):
    payload = bytes(frame.data)
    trailer = struct.pack("<II4sBB", 0x184D2A5F, 6, b"RBWF", 3, expected_level)
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
    assert data[:4] == b"RBP3"
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

    def signed(v):
        return (v >> 1).astype(np.int32) ^ -(v & 1).astype(np.int32)

    def restore(values, groups, mask):
        if not len(values):
            return values
        starts = np.r_[True, groups[1:] != groups[:-1]]
        cumulative = np.cumsum(values, dtype=np.int64)
        base = np.r_[0, cumulative[:-1]]
        indices = np.maximum.accumulate(np.where(starts, np.arange(len(values)), 0))
        return (cumulative - base[indices]) & mask

    def bitmap(n):
        return np.unpackbits(np.frombuffer(take((n + 7) // 8), np.uint8),
                             count=n, bitorder="little").astype(bool)

    streams = {}
    for width, n in counts.items():
        slots = (width - 10) // 4
        hw = np.stack([np.cumsum(signed(bits(n, 16)), dtype=np.int64) & 0xffff
                       for _ in range(5)], axis=1).astype("<u2")
        hdr = hw.view(np.uint8)
        line = hw[:, 4] & 0x39ff
        order = np.lexsort((hw[:, 3], line))
        line = line[order]
        masks = [bitmap(n) for _ in range(slots)]
        ends = np.cumsum([mask.sum() for mask in masks])
        valid = int(ends[-1])
        residual = bits(valid, 7)
        high = bitmap(valid)
        residual[high] |= (bits(int(high.sum()), 12) + 1) << 7
        deltas = np.split(signed(residual), ends[:-1])
        reflect = signed(bits(n * slots, 12)).reshape(slots, n)
        flags = bits(n * slots, 2).reshape(slots, n)
        points = np.empty((n, slots), "<u4")
        for col, keep, dr, refl, flag in zip(points.T, masks, deltas, reflect, flags):
            radius = np.zeros(n, np.uint32)
            radius[keep] = restore(dr, line[keep], 0x3ffff)
            intensities = restore(refl, line, 0xfff).astype(np.uint32)
            col[order] = intensities | (radius << 12) | (flag << 30)
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
