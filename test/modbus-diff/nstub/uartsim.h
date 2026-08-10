#pragma once
// Small UART fake so the real poll() can run: byte buffer plus one event per
// frame fed in.
#include <cstdint>
#include <cstddef>
#include <cstring>
struct UartSim {
  uint8_t rx[512]; size_t rx_len = 0, rx_pos = 0;
  uint8_t tx[512]; size_t tx_len = 0;
  size_t ev[16]; int ev_n = 0, ev_i = 0;
  void feed(const uint8_t *d, size_t n) {
    memcpy(rx, d, n); rx_len = n; rx_pos = 0; tx_len = 0;
    ev_n = 0; ev_i = 0; ev[ev_n++] = n;
  }
  // Report a frame in chunks, as the driver does at the RX threshold
  void feed_chunks(const uint8_t *d, size_t n, size_t chunk) {
    memcpy(rx, d, n); rx_len = n; rx_pos = 0; tx_len = 0;
    ev_n = 0; ev_i = 0;
    for (size_t o = 0; o < n && ev_n < 16; o += chunk)
      ev[ev_n++] = (n - o < chunk) ? (n - o) : chunk;
  }
};
extern UartSim g_uart;
