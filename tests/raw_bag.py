"""Read raw Robin W packets and calibration directly from SQLite bags."""
import ctypes
import ctypes.util
from functools import lru_cache
import sqlite3
import struct

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


