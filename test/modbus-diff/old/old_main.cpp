#define private public
#include "hoermann.h"
#undef private
#include "common.h"
#include <string>
#include <iostream>
unsigned long g_millis = 1000;
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
    extern int old_exchange(ModbusRTU &, const uint8_t *, size_t, uint8_t *);
    int r = old_exchange(e.mb, req, n, resp);
    printf("R:");
    if (r < 0) printf("-");
    else { for (int i = 0; i < r; i++) printf("%02x", resp[i]); }
    printf(" G:");
    for (int i=0;i<3;i++) printf("%04x", e.mb.Hreg(0x9C41+i));
    for (int i=0;i<9;i++) printf("%04x", e.mb.Hreg(0x9D31+i));
    for (int i=0;i<8;i++) printf("%04x", e.mb.Hreg(0x9CB9+i));
    // debMessage/lastModbusRespone are gone from the new side: write-only,
    // never read by anything.
    printf(" S:%d,%d,%.3f,%.3f,%d,%d,%d,%.3f\n",
           (int)e.state->valid, (int)e.state->state, e.state->targetPosition,
           e.state->currentPosition, (int)e.state->lightOn, (int)e.state->relayOn,
           (int)e.state->changed, e.state->gotoPosition);
  }
  return 0;
}
