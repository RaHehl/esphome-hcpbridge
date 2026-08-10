#define private public
#include "hoermann.h"
#undef private
#include "common.h"
#include <string>
#include <iostream>
int main() {
  auto &e = HoermannGarageEngine::getInstance();
  e.setup(18, 17, -1);
  std::string line;
  uint8_t req[300], resp[300];
  while (std::getline(std::cin, line)) {
    if (line.empty()) continue;
    size_t n = 0;
    for (size_t i = 0; i + 1 < line.size(); i += 2)
      req[n++] = (uint8_t)strtol(line.substr(i, 2).c_str(), nullptr, 16);
    int r = e.mb.exchange(req, n, resp);
    printf("R:");
    if (r < 0) printf("-");
    else { for (int i = 0; i < r; i++) printf("%02x", resp[i]);
           uint16_t c = mbcrc(resp, r); printf("%02x%02x", c & 0xFF, c >> 8); }
    printf(" G:");
    for (int i=0;i<3;i++) printf("%04x", e.mb.Hreg(0x9C41+i));
    for (int i=0;i<9;i++) printf("%04x", e.mb.Hreg(0x9D31+i));
    for (int i=0;i<8;i++) printf("%04x", e.mb.Hreg(0x9CB9+i));
    printf(" S:%d,%d,%.3f,%.3f,%d,%d,%d,%d,%.3f,%lu\n",
           (int)e.state->valid, (int)e.state->state, e.state->targetPosition,
           e.state->currentPosition, (int)e.state->lightOn, (int)e.state->relayOn,
           (int)e.state->changed, (int)e.state->debMessage, e.state->gotoPosition,
           e.state->lastModbusRespone);
  }
  return 0;
}
