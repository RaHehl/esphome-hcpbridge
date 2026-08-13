// Drives the codec the way the transport does on the device: feed the port,
// let it decide, collect what went out and what it now says about the door.
#include "hcp2_codec.h"
#include "modbus_rtu.h"
using namespace esphome::hcpbridge;
#include "uartsim.h"
#include <cstring>
UartSim g_uart;
// The port the codec no longer owns. On the device this lives in the transport;
// here the harness holds it, which is the same arrangement seen from the desk.
static ModbusRtuServer g_mb;
static Hcp2Codec *g_codec = nullptr;
#include <string>
#include <iostream>
uint32_t g_millis = 1000;
namespace esphome {
uint32_t millis() { return g_millis; }
}  // namespace esphome

// The production path, not a paraphrase of it: the same codec, the same frame
// server, only the port is faked.
int exchange_new(Hcp2Codec &e, const uint8_t *adu, size_t n, uint8_t *out) {
  const char *cs = getenv("CHUNK");
  if (cs)
    g_uart.feed_chunks(adu, n, (size_t) atoi(cs));
  else
    g_uart.feed(adu, n);
  for (int i = 0; i < 20; i++)
    g_mb.poll(0);  // the bus task calls poll() in a loop
  if (g_uart.tx_len == 0)
    return -1;
  memcpy(out, g_uart.tx, g_uart.tx_len);
  return (int) g_uart.tx_len;
}
int main() {
  // Line buffered, so this can be driven turn by turn and not only fed a file
  // and read afterwards. Into a pipe the default is to buffer in blocks, and a
  // caller waiting for the answer to the frame it just sent would wait for a
  // buffer that only flushes when it is full.
  setvbuf(stdout, nullptr, _IOLBF, 0);
  static Hcp2Codec e;
  g_codec = &e;
  g_mb.set_handler([](const uint8_t *req, size_t len, uint8_t *resp) -> size_t {
    return g_codec->onFrame(g_millis, req, len, resp);
  });
  // The port is the transport's on the device; here it is the harness's, which
  // is the same arrangement seen from the desk.
  g_mb.begin((uart_port_t) 2, 18, 17, -1, 57600, SLAVE_ID);
  // No port here: the codec has none. Frames arrive through g_uart below.
  std::string line;
  uint8_t req[300], resp[300];
  while (std::getline(std::cin, line)) {
    if (line.empty())
      continue;
    if (line[0] == 'T') {
      g_millis = (uint32_t) strtoull(line.c_str() + 1, nullptr, 10);
      continue;
    }
    if (line[0] == 'S') {
      e.stopDoor(g_millis);
      continue;
    }
    // What the ESPHome component does on its own schedule rather than on a
    // frame. Without a way to ask for it, everything hanging off it - the link
    // going stale above all - is unreachable from a test.
    if (line[0] == 'L') {
      printf("%s\n", hcpLinkName(e.state->link));
      continue;
    }
    if (line[0] == 'U') {
      e.checkBusSilence(g_millis);
      e.publishIdentity();
      continue;
    }
    // Go to a partial position, the one entry point that arms a target the
    // door is later stopped at.
    if (line[0] == 'P') {
      e.setPosition(g_millis, (int) strtol(line.c_str() + 1, nullptr, 10));
      continue;
    }
    if (line[0] == 'C') {
      static const HoermannCommand cmds[7] = {HoermannCommand::OPEN, HoermannCommand::CLOSE, HoermannCommand::IMPULSE,
                                              HoermannCommand::HALF, HoermannCommand::VENT,  HoermannCommand::LIGHT_ON,
                                              HoermannCommand::NONE};
      e.setCommand(g_millis, true, cmds[strtol(line.c_str() + 1, nullptr, 10) % 7]);
      continue;
    }
    size_t n = 0;
    for (size_t i = 0; i + 1 < line.size(); i += 2)
      req[n++] = (uint8_t) strtol(line.substr(i, 2).c_str(), nullptr, 16);
    int r = exchange_new(e, req, n, resp);
    printf("R:");
    if (r < 0)
      printf("-");
    else {
      for (int i = 0; i < r; i++)
        printf("%02x", resp[i]);
    }
    printf(" S:%d,%d,%.3f,%.3f,%d,%d,%d,%.3f ", (int) e.state->valid, (int) e.state->state, e.state->targetPosition,
           e.state->currentPosition, (int) e.state->lightOn, (int) e.state->relayOn, (int) e.state->changed,
           e.state->gotoPosition);
    // As a word, so a test can say what it expects without repeating the
    // enum's order back at the code it is testing.
    printf("N:%s\n", hoermannStateName(e.state->state));
  }
  return 0;
}
