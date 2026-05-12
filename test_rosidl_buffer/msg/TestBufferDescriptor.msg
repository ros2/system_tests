# Wire descriptor used by the in-tree test buffer backend.

uint64 size                    # number of elements in the original buffer
uint64 data_hash               # FNV-1a hash of the element bytes
uint8[] data                   # raw element bytes (size * sizeof(T))
