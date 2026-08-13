// The task the newer bus runs on, and the wiring between its two halves.
//
// Not in the data path itself: the port belongs to modbus_rtu, which is what
// the wire actually reaches, and the deciding belongs to hcp2_codec, which has
// no port, no clock and no thread. This holds the two, hands the codec the
// clock once per frame, runs the task that must not miss one, and does the
// waiting that shutting down needs.
//
// Split so the deciding half can be driven from a desk. It also happens to be
// the same seam the older bus was born with, which is what stops two protocols
// meaning two copies of everything.
#pragma once

#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "hcp2_codec.h"
#include "hcp_bus_task.h"
#include "modbus_rtu.h"

namespace esphome {
namespace hcpbridge {

static constexpr uint32_t HCP_BAUD = 57600;

class Hcp2Transport {
 public:
  /** False when the port or the task could not be brought up. */
  bool begin(int8_t rx, int8_t tx, int8_t rts, uint8_t uartNum);

  /**
   * Tells the drive we are about to go quiet and waits for it to acknowledge.
   *
   * False if it stayed silent. Carry on anyway: a restart must not hang on a
   * bus that is already gone.
   */
  bool announcePause(uint32_t timeoutMs);

  /** Waits for a gap, so a restart does not land inside a telegram. */
  void quiesce() { this->settleBeforeRestart(); }

  /** One receive and answer cycle. The task calls this in a loop. */
  void serveOnce() { this->mb_.poll(); }

  Hcp2Codec codec;

 protected:
  /** Waits out any frame still on the wire before the caller restarts. */
  void settleBeforeRestart();

  ModbusRtuServer mb_;  // our own RTU server, no third-party library
  // One task per instance, so two doors on one board each get their own.
  TaskHandle_t busTask_{nullptr};
};

}  // namespace hcpbridge
}  // namespace esphome
