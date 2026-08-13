#include "hcp2_frame.h"

namespace esphome {
namespace hcpbridge {

uint16_t hcp2Crc16(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; bit++)
      crc = (crc & 1) ? (uint16_t) ((crc >> 1) ^ 0xA001) : (uint16_t) (crc >> 1);
  }
  return crc;
}

size_t hcp2Validate(const uint8_t *buf, size_t len) {
  // An address, a function code and a checksum is the shortest thing that could
  // be a frame at all.
  if (len < 4)
    return 0;
  const uint16_t got = (uint16_t) buf[len - 2] | ((uint16_t) buf[len - 1] << 8);
  if (got != hcp2Crc16(buf, len - 2))
    return 0;
  return len - 2;
}

size_t hcp2Finish(uint8_t *buf, size_t len, size_t cap) {
  if (len == 0 || len + 2 > cap)
    return 0;
  const uint16_t crc = hcp2Crc16(buf, len);
  buf[len] = (uint8_t) (crc & 0xFF);
  buf[len + 1] = (uint8_t) (crc >> 8);
  return len + 2;
}

}  // namespace hcpbridge
}  // namespace esphome
