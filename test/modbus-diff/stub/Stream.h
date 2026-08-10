#pragma once
#include <cstdint>
#include <cstddef>
#include <cstring>
// Minimale Stream-Schnittstelle, wie sie die Bibliothek benutzt.
class Stream {
 public:
  virtual int available() = 0;
  virtual int read() = 0;
  virtual size_t write(uint8_t b) = 0;
  virtual size_t write(const uint8_t *b, size_t n) = 0;
  virtual void flush() = 0;
  virtual ~Stream() {}
};
// Serielle Attrappe: nimmt ein ganzes Telegramm auf, sammelt die Antwort ein.
class FakeSerial : public Stream {
 public:
  uint8_t rx[600]; size_t rx_len = 0, rx_pos = 0;
  uint8_t tx[600]; size_t tx_len = 0;
  void begin(long, int, int, int) {}
  uint32_t baudRate() { return 57600; }
  void feed(const uint8_t *d, size_t n) { memcpy(rx, d, n); rx_len = n; rx_pos = 0; tx_len = 0; }
  int available() override { return (int)(rx_len - rx_pos); }
  int read() override { return rx_pos < rx_len ? rx[rx_pos++] : -1; }
  size_t write(uint8_t b) override { tx[tx_len++] = b; return 1; }
  size_t write(const uint8_t *b, size_t n) override { memcpy(tx + tx_len, b, n); tx_len += n; return n; }
  void flush() override {}
};
extern FakeSerial Serial2;
