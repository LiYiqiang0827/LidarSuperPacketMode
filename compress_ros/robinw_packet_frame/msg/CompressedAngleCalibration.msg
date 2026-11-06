# Byte count of the effective type-100 packet, including its 70-byte header.
uint32 uncompressed_size
# Independent Zstd frame containing that original packet verbatim.
uint8[] data
# First 16 bytes of SHA-256 of the calibration version and effective angle table.
uint8[16] calibration_id
