uint64 frame_id
# Sensor timestamp of the first available packet, in microseconds.
float64 sensor_timestamp_us
bool complete
uint32 packet_count
# Total byte count of the original packets, including their headers.
uint32 uncompressed_size
# Source block width: 42 (single), 74 (double), 0 (unshuffled).
uint16 shuffle_block_size
# Independent Zstd frame containing RBP3 headers and predicted point fields, plus a Zstd metadata trailer.
uint8[] data
# First 16 bytes of SHA-256 of the calibration version and effective angle table.
# All zeroes in a frame mean that the encoder had no valid calibration for it.
uint8[16] calibration_id
