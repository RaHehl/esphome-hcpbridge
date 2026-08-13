// How much silence ends a frame.
//
// The UART watches the line itself; this is the number it watches with. Two
// traps live in it and both are quiet when wrong: the driver counts SYMBOL
// times rather than microseconds, and the register it lands in is narrow, the
// narrowest being the original ESP32's, which stops accepting above 92.
#include <cstdio>
#include <cstdlib>

#include "modbus_rtu.h"

using namespace esphome::hcpbridge;

static int failures = 0;
static void check(bool ok, const char *what) {
  printf("  %-58s %s\n", what, ok ? "ok" : "FAILED");
  if (!ok)
    failures++;
}

/** Microseconds of silence the chosen symbol count actually buys at that baud. */
static uint32_t idleUs(uint32_t baud) { return rxIdleSymbols(baud) * (11UL * 1000000UL / baud); }

int main() {
  check(rxIdleSymbols(57600) == 10, "at 57600 the bus waits ten symbols");
  check(idleUs(57600) >= 1750, "which is the 1750 us the replaced library waited");
  check(idleUs(57600) < 2500, "and not so much that the drive is kept waiting");

  check(rxIdleSymbols(19200) >= 4, "the older bus gets at least the floor");
  check(idleUs(19200) >= 1750, "and still a full gap at 19200");

  // A fast port would want fewer microseconds per symbol and so more symbols;
  // the register cannot hold them, and the driver only says so to the log.
  check(rxIdleSymbols(921600) == 92, "an absurd baud rate is clamped, not truncated");
  // Fast enough that a symbol rounds to zero microseconds, which is its own
  // branch and was left behind when the clamp moved from 93 to 92.
  check(rxIdleSymbols(20000000) == 92, "and so is one where a symbol rounds away");
  check(rxIdleSymbols(9600) == 4, "a slow one hits the floor rather than one symbol");

  for (uint32_t baud : {9600u, 19200u, 38400u, 57600u, 115200u, 230400u, 921600u}) {
    const uint8_t n = rxIdleSymbols(baud);
    if (!(n >= 4 && n <= 92)) {
      printf("  %u baud gives %u symbols, outside what the register holds\n", baud, n);
      failures++;
    }
  }
  check(true, "every baud in the table stays inside the register");

  printf("\n%s\n", failures == 0 ? "all frame-gap checks passed" : "frame-gap checks FAILED");
  return failures == 0 ? 0 : 1;
}
