#define private public
#include "hoermann.h"      // neue Fassung
#undef private
#include "common.h"
#include <string>
#include <iostream>
static unsigned long g_millis = 1000;
namespace esphome { unsigned long millis() { return g_millis; } }

// Bildet die Annahmeentscheidungen aus ModbusRtuServer::poll() nach.
int exchange_new(HoermannGarageEngine &e, const uint8_t *adu, size_t n, uint8_t *out) {
  if (n < 4) return -1;
  uint16_t got = (uint16_t)adu[n - 2] | ((uint16_t)adu[n - 1] << 8);
  if (got != mbcrc(adu, n - 2)) return -1;
  const uint8_t addr = adu[0];
  const bool broadcast = (addr == 0);
  if (!broadcast && addr != SLAVE_ID) return -1;
  uint8_t tx[300];
  size_t rlen = e.onFrame(adu, n - 2, tx);
  if (rlen == 0 || broadcast) return -1;
  memcpy(out, tx, rlen);
  return (int)rlen;
}
int main() {
  auto &e = HoermannGarageEngine::getInstance();
  std::string line;
  uint8_t req[300], resp[300];
  while (std::getline(std::cin, line)) {
    if (line.empty()) continue;
    size_t n = 0;
    for (size_t i = 0; i + 1 < line.size(); i += 2)
      req[n++] = (uint8_t)strtol(line.substr(i, 2).c_str(), nullptr, 16);
    int r = exchange_new(e, req, n, resp);
    printf("R:");
    if (r < 0) printf("-");
    else { for (int i = 0; i < r; i++) printf("%02x", resp[i]);
           uint16_t c = mbcrc(resp, r); printf("%02x%02x", c & 0xFF, c >> 8); }
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
