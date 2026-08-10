#define private public
#include "hoermann.h"      // neue Fassung
#undef private
#include "common.h"
#include "uartsim.h"
UartSim g_uart;
#include <string>
#include <iostream>
unsigned long g_millis = 1000;
namespace esphome { unsigned long millis() { return g_millis; } }

// Faehrt den ECHTEN Produktivpfad: Bytes in die Schnittstelle, poll() laufen
// lassen, gesendete Bytes einsammeln.
int exchange_new(HoermannGarageEngine &e, const uint8_t *adu, size_t n, uint8_t *out) {
  const char *cs = getenv("CHUNK");
  if (cs) g_uart.feed_chunks(adu, n, (size_t)atoi(cs)); else g_uart.feed(adu, n);
  for (int i = 0; i < 20; i++) e.mb.poll(0);   // Bus-Task ruft poll() in Schleife
  if (g_uart.tx_len == 0) return -1;
  memcpy(out, g_uart.tx, g_uart.tx_len);
  return (int)g_uart.tx_len;
}
int main() {
  auto &e = HoermannGarageEngine::getInstance();
  e.setup(18, 17, -1);
  std::string line;
  uint8_t req[300], resp[300];
  while (std::getline(std::cin, line)) {
    if (line.empty()) continue;
    if (line[0] == 'T') { g_millis = strtoul(line.c_str() + 1, nullptr, 10); continue; }
    if (line[0] == 'C') {
      static const HoermannCommand *cmds[7] = {
          &HoermannCommand::STARTOPENDOOR, &HoermannCommand::STARTCLOSEDOOR,
          &HoermannCommand::STARTIMPULSE,  &HoermannCommand::STARTOPENDOORHALF,
          &HoermannCommand::STARTVENTPOSITION, &HoermannCommand::STARTTOGGLELAMP,
          &HoermannCommand::WAITING};
      e.setCommand(true, cmds[strtol(line.c_str() + 1, nullptr, 10) % 7]);
      continue;
    }
    size_t n = 0;
    for (size_t i = 0; i + 1 < line.size(); i += 2)
      req[n++] = (uint8_t)strtol(line.substr(i, 2).c_str(), nullptr, 16);
    int r = exchange_new(e, req, n, resp);
    printf("R:");
    if (r < 0) printf("-");
    else { for (int i = 0; i < r; i++) printf("%02x", resp[i]); }
    printf(" G:");
    for (int i=0;i<3;i++) printf("%04x", e.regGet(0x9C41+i));
    for (int i=0;i<9;i++) printf("%04x", e.regGet(0x9D31+i));
    for (int i=0;i<8;i++) printf("%04x", e.regGet(0x9CB9+i));
    printf(" S:%d,%d,%.3f,%.3f,%d,%d,%d,%d,%.3f,%lu\n",
           (int)e.state->valid, (int)e.state->state, e.state->targetPosition,
           e.state->currentPosition, (int)e.state->lightOn, (int)e.state->relayOn,
           (int)e.state->changed, (int)e.state->debMessage, e.state->gotoPosition,
           e.state->lastModbusRespone);
  }
  return 0;
}
