// Where a frame starts and stops on the older bus.
//
// The counterpart to modbus_rtu, and the same job: collect bytes off a port,
// decide where one frame ends, hand the whole thing to whoever asked, send back
// what they give. Neither knows what a frame means, and neither touches a
// checksum - that belongs with the codec, which is what lets the codec be asked
// about real wire bytes without a port in sight.
//
// What differs is only the delimiter. Modbus RTU ends a frame by letting the
// line fall idle. This bus puts a break in front of every frame, so the end of
// one is announced by the start of the next.
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "hcp1_frame.h"

namespace esphome {
namespace hcpbridge {

class Hcp1SerialServer {
 public:
  // Called with everything collected since the last break, checksum included.
  // Returns the number of bytes to send back, checksum already on them, or 0
  // to stay quiet.
  using Handler = std::function<size_t(const uint8_t *frame, size_t len, uint8_t *out)>;

  bool begin(uart_port_t port, int rx_pin, int tx_pin, int rts_pin, uint32_t baud);
  void set_handler(Handler handler) { this->handler_ = std::move(handler); }

  /** Blocking receive and answer cycle; call from a dedicated task. */
  void poll(uint32_t timeoutMs = 20);

 private:
  /**
   * Sends one frame, break first.
   *
   * The driver's own call appends a break after what it is given and offers no
   * way to send one alone, so the break that has to arrive first is carried by
   * a throwaway byte in a separate call. Reusing the trailing break of the
   * previous transmission is not an option: the drive puts its own frames on
   * the wire in between, each with a break of its own, so ours is long gone by
   * the time we may speak again.
   */
  void send(const uint8_t *frame, size_t len);

  // A noisy line raises one of these per event, and logging each would block
  // the task the drive is waiting on. Same throttle as the newer bus.
  bool warn_due();

  // Only ever the value begin() was given; the initialiser is here so the
  // field is never read uninitialised. UART_NUM_0 because it is the one
  // port every variant has - a C3 has no UART 2 and will not compile a
  // mention of one.
  uart_port_t port_{UART_NUM_0};
  QueueHandle_t queue_{nullptr};
  Handler handler_;
  uint8_t rx_buf_[HCP1_MAX_FRAME];
  uint8_t tx_buf_[HCP1_MAX_FRAME];
  size_t rx_len_{0};
  uint32_t warn_seen_{0};
};

}  // namespace hcpbridge
}  // namespace esphome
