#pragma once
// Kleine Attrappe der seriellen Schnittstelle, damit der echte poll() laufen
// kann: Byte-Puffer plus ein Ereignis je eingespeistem Telegramm.
#include <cstdint>
#include <cstddef>
#include <cstring>
struct UartSim {
  uint8_t rx[512]; size_t rx_len = 0, rx_pos = 0;
  uint8_t tx[512]; size_t tx_len = 0;
  int pending = 0; size_t pending_size = 0;
  void feed(const uint8_t *d, size_t n) {
    memcpy(rx, d, n); rx_len = n; rx_pos = 0; pending = 1; pending_size = n; tx_len = 0;
  }
};
extern UartSim g_uart;
