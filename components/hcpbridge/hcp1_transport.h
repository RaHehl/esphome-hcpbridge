// The task the older bus runs on.
//
// Same shape as the newer one: hcp1_serial owns the port and decides where a
// frame ends, hcp1_codec decides what to answer, and this holds the task and
// reads the clock. What differs between the two buses is confined to those
// first two.
#pragma once

#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "hcp1_codec.h"
#include "hcp1_serial.h"
#include "hcp_bus_task.h"

namespace esphome {
namespace hcpbridge {

// Here rather than next to the codec: how fast the port runs is a property of
// the wire, and the codec is not supposed to know there is one.
static constexpr uint32_t HCP1_BAUD = 19200;

class Hcp1Transport {
 public:
  /** False when the port or the task could not be brought up. */
  bool begin(int8_t rx, int8_t tx, int8_t rts, uint8_t uartNum);

  /** One receive and answer cycle. The task calls this in a loop. */
  void serveOnce() { this->serial_.poll(); }

  /**
   * There is no pause handshake on this bus, so there is nobody to tell.
   *
   * Here so that whoever is shutting down can ask either bus the same question
   * and get an honest answer from both.
   */
  bool announcePause(uint32_t timeoutMs) { return false; }

  /**
   * The half of going quiet this bus can do.
   *
   * There is nobody to tell, but waiting for a gap needs no handshake, and
   * without it a restart here lands inside a telegram as easily as on the other
   * bus. This used to sit behind the pause acknowledgement, which meant the
   * protocol without one got no protection at all.
   */
  void quiesce() {
    settleUntilQuiet([this] { return this->codec.lastFrameAt(); });
  }

  Hcp1Codec codec;

 protected:
  Hcp1SerialServer serial_;
  // One task per instance, so two doors on one board each get their own.
  TaskHandle_t busTask_{nullptr};
};

}  // namespace hcpbridge
}  // namespace esphome
