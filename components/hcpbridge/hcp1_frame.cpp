#include "hcp1_frame.h"

namespace esphome {
namespace hcpbridge {

uint8_t hcp1Crc8(const uint8_t *data, size_t len) {
  // 0xF3 rather than the 0x00 or 0xFF a checksum usually starts at. Both
  // sources say so, one having reached it a different way, so it is not one
  // reading repeated twice.
  uint8_t crc = 0xF3;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; bit++)
      crc = (crc & 0x80) ? (uint8_t) ((crc << 1) ^ 0x07) : (uint8_t) (crc << 1);
  }
  return crc;
}

bool hcp1Parse(const uint8_t *buf, size_t len, Hcp1Frame *out) {
  // Address, the byte holding counter and length, and a checksum.
  if (len < 3 || len > HCP1_MAX_FRAME)
    return false;
  const uint8_t length = buf[1] & 0x0F;
  // The frame says how long it is; anything else means the break was missed or
  // two frames ran together.
  if ((size_t) (length + 3) != len)
    return false;
  if (hcp1Crc8(buf, len - 1) != buf[len - 1])
    return false;

  out->address = buf[0];
  out->counter = (uint8_t) (buf[1] >> 4);
  out->length = length;
  out->data = buf + 2;
  return true;
}

size_t hcp1Build(uint8_t *buf, size_t cap, uint8_t address, uint8_t counter, const uint8_t *data, uint8_t length) {
  const size_t total = (size_t) length + 3;
  if (length > 0x0F || total > cap || total > HCP1_MAX_FRAME)
    return 0;
  buf[0] = address;
  buf[1] = (uint8_t) (((counter & 0x0F) << 4) | length);
  for (uint8_t i = 0; i < length; i++)
    buf[2 + i] = data[i];
  buf[total - 1] = hcp1Crc8(buf, total - 1);
  return total;
}

}  // namespace hcpbridge
}  // namespace esphome
