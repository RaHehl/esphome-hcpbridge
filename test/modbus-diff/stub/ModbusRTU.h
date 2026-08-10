#pragma once
#include "Modbus.h"
#define MODBUSRTU_BROADCAST 0
// Ersatz nur fuer die Uebertragungsschicht. Die Entscheidungslogik (slavePDU)
// bleibt der echte Bibliothekscode.
class ModbusRTU : public Modbus {
 public:
  uint8_t slaveId_ = 0;
  void begin(void *) {}
  void begin(void *, int, bool) {}
  void slave(uint8_t id) { slaveId_ = id; }
  bool task() { return true; }
  // wie ModbusAPI<T>::addHreg
  bool addHreg(uint16_t offset, uint16_t value = 0, uint16_t numregs = 1) {
    return this->addReg(HREG(offset), value, numregs);
  }
  uint16_t Hreg(uint16_t offset) { return this->Reg(HREG(offset)); }
  bool Hreg(uint16_t offset, uint16_t value) { return this->Reg(HREG(offset), value); }
  // Bildet ModbusRTUTemplate::task() ab: Adressfilter, CRC, slavePDU, Broadcast
  int exchange(const uint8_t *adu, size_t n, uint8_t *out);
};
