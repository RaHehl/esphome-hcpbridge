// A Modbus RTU responder: framing, checksum, and handing a complete request to
// whoever asked to be told. It knows no register and no function code of its
// own - what exists on the newer Hoermann bus and how it answers is in
// hcp2_codec, and what owns this port and its task is hcp2_transport.
//
// A frame ends on an idle line, detected by the driver rather than polled for
// in software as the replaced library had to.
//
// With rts_pin set the driver's RS485 half duplex mode drives the direction
// pin. Off by default and untested on hardware.
#pragma once

#include <cstdint>
#include <cstddef>
#include <functional>

#include "driver/uart.h"

#include "hcp2_frame.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

namespace esphome {
namespace hcpbridge {

// Driver RX FIFO threshold. Frames longer than this arrive split, see poll().
static constexpr int RX_FULL_THRESHOLD = 120;
// Growing either limit past the buffer would write into whatever follows.
static_assert(3 + 2 * 0x7D + 2 <= 256, "response can exceed the transmit buffer");

class ModbusRtuServer {
 public:
  // Called with everything that arrived between two idle gaps, checksum
  // included. Returns the number of bytes to send back, which the handler has
  // already put a checksum on, or 0 to stay silent.
  using Handler = std::function<size_t(const uint8_t *req, size_t len, uint8_t *response)>;

  bool begin(uart_port_t port, int rx_pin, int tx_pin, int rts_pin, uint32_t baud, uint8_t slave_id);
  void set_handler(Handler handler) { this->handler_ = std::move(handler); }

  // Blocking receive/answer cycle; call from a dedicated task.
  void poll(uint32_t timeout_ms = 20);

 private:
  // Only ever the value begin() was given; the initialiser is here so the
  // field is never read uninitialised. UART_NUM_0 because it is the one
  // port every variant has - a C3 has no UART 2 and will not compile a
  // mention of one.
  uart_port_t port_{UART_NUM_0};
  QueueHandle_t queue_{nullptr};
  uint8_t slave_id_{2};
  // Twice as long as the longest frame this port can carry.
  uint32_t tx_wait_ms_{50};
  Handler handler_;
  uint8_t rx_buf_[HCP2_MAX_FRAME];
  uint8_t tx_buf_[HCP2_MAX_FRAME];
  // Per port, not shared: with two buses on one board a single counter would
  // have them throttling each other's warnings.
  uint32_t warn_seen_{0};
  bool warn_due();
};

}  // namespace hcpbridge
}  // namespace esphome
