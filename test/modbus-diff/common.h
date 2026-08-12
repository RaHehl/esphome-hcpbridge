#pragma once
#include <cstdint>
#include <cstddef>
#include <cstdio>
// Modbus-CRC wie auf dem Draht: niederwertiges Byte zuerst
static inline uint16_t mbcrc(const uint8_t *d, size_t n) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < n; i++) {
    crc ^= d[i];
    for (int b = 0; b < 8; b++) crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
  }
  return crc;
}
