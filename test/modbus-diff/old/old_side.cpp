#include "hoermann.h"   // original version
#include "common.h"
FakeSerial Serial2;
extern unsigned long g_millis;
unsigned long millis() { return g_millis; }
// micros() must advance or the library's wait loop never exits
static unsigned long g_micros = 0;
unsigned long micros() { g_micros += 250; return g_micros; }

// No reimplementation: feed the fake port, run the library's REAL task(),
// collect what it sent.
int old_exchange(ModbusRTU &mb, const uint8_t *adu, size_t n, uint8_t *out) {
  Serial2.feed(adu, n);
  mb.task();
  if (Serial2.tx_len == 0) return -1;
  memcpy(out, Serial2.tx, Serial2.tx_len);
  return (int)Serial2.tx_len;
}
