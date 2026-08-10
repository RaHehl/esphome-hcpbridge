#include "hoermann.h"   // Urfassung
#include "common.h"
FakeSerial Serial2;
extern unsigned long g_millis;
unsigned long millis() { return g_millis; }

int ModbusRTU::exchange(const uint8_t *adu, size_t n, uint8_t *out) {
  uint8_t address = adu[0];
  bool valid_frame = (address == MODBUSRTU_BROADCAST || address == this->slaveId_);
  if (!valid_frame) return -1;              // fremde Adresse: keine Antwort
  if (n < 4) return -1;
  free(_frame);
  _len = (uint16_t)(n - 1);           // die Bibliothek liest das Adressbyte getrennt
  _frame = (uint8_t *)malloc(_len);
  memcpy(_frame, adu + 1, _len);
  uint16_t frameCrc = ((_frame[_len - 2] << 8) | _frame[_len - 1]);
  _len = _len - 2;
  uint16_t want = mbcrc(adu, n - 2);
  uint16_t wire = (uint16_t)(((want & 0xFF) << 8) | (want >> 8));  // so liest die Bibliothek
  if (frameCrc != wire) { free(_frame); _frame = nullptr; _len = 0; return -1; }
  _reply = Modbus::EX_PASSTHROUGH;
  slavePDU(_frame);
  int r = -1;
  if (address != MODBUSRTU_BROADCAST && _reply != Modbus::REPLY_OFF) {
    out[0] = address;
    memcpy(out + 1, _frame, _len);
    r = (int)_len + 1;
  }
  free(_frame); _frame = nullptr; _len = 0;
  return r;
}
