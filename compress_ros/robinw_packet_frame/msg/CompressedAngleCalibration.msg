# Byte count of the effective type-100 packet, including its 70-byte header.
uint32 uncompressed_size
# Independent Zstd frame containing that original packet verbatim.
uint8[] data
