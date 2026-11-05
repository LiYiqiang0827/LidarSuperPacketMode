uint64 frame_id
# Sensor timestamp of the first available packet, in microseconds.
float64 sensor_timestamp_us
bool complete
uint32 packet_count
# Total byte count of the original packets, including their headers.
uint32 uncompressed_size
# Block width used for byte shuffling: 42 (single), 74 (double), 0 (unshuffled).
uint16 shuffle_block_size
# Independent Zstd frame containing the normalized headers and the block payload.
uint8[] data
