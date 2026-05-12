# Minimal payload message used by the pub/sub launch tests.
# The `data` field is generated as rosidl::Buffer<uint8_t>, which is the
# feature under test.

uint32 seq
uint8[] data
