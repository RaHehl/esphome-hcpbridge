// Vergleicht die ECHTE crc16() der neuen Fassung mit der ECHTEN Tabellen-CRC
// der abgeloesten Bibliothek, auf Wire-Byte-Ebene.
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include "uartsim.h"
UartSim g_uart;
#include "modbus_rtu.h"
#define highByte(w) ((uint8_t)((w) >> 8))
#define lowByte(w)  ((uint8_t)((w) & 0xFF))
#include "auchcrc.inc"
static uint16_t lib_crc16(uint8_t address, const uint8_t *frame, uint8_t pduLen) {
  uint8_t i = 0xFF ^ address;
  uint16_t val = _auchCRC[i];
  uint8_t CRCHi = 0xFF ^ highByte(val);
  uint8_t CRCLo = lowByte(val);
  while (pduLen--) {
    i = CRCHi ^ *frame++;
    val = _auchCRC[i];
    CRCHi = CRCLo ^ highByte(val);
    CRCLo = lowByte(val);
  }
  return (uint16_t)((CRCHi << 8) | CRCLo);
}
int main() {
  unsigned bad = 0, n = 0;
  for (int t = 0; t < 300000; t++) {
    uint8_t buf[256];
    int len = 1 + rand() % 250;             // Adresse + PDU
    for (int i = 0; i < len; i++) buf[i] = rand() & 0xFF;
    // alte Seite: Adresse getrennt, PDU danach; auf dem Draht CRCHi zuerst
    uint16_t l = lib_crc16(buf[0], buf + 1, (uint8_t)(len - 1));
    uint8_t old_first = (uint8_t)(l >> 8), old_second = (uint8_t)(l & 0xFF);
    // neue Seite: echte Funktion, niederwertiges Byte zuerst
    uint16_t m = esphome::hcpbridge::ModbusRtuServer::crc16(buf, len);
    uint8_t new_first = (uint8_t)(m & 0xFF), new_second = (uint8_t)(m >> 8);
    n++;
    if (old_first != new_first || old_second != new_second) {
      if (bad < 3) printf("ABWEICHUNG len=%d alt=%02x%02x neu=%02x%02x\n", len, old_first, old_second, new_first, new_second);
      bad++;
    }
  }
  printf("%u Prueflaeufe, %u Abweichungen\n", n, bad);
  return bad != 0;
}
