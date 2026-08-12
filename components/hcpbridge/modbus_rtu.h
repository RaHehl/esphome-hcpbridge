// Modbus RTU server for the Hoermann HCP bus.
//
// Replaces the Arduino library "emelianov/modbus-esp8266". The transport
// lives here; which function codes exist and how they answer is in
// hoermann.cpp, deliberately matching what that library did (see
// test/modbus-diff/run.sh, which compiles the old library and compares).
//
// Frame detection uses the ESP-IDF UART driver's RX timeout: Modbus RTU
// separates frames by a gap of 3.5 character times, and the driver raises
// an event once the line has been idle that long. Polling for that in
// software - as the Arduino version had to - is far less reliable.
//
// One deliberate difference: with rts_pin configured this uses the driver's
// RS485 half duplex mode, where the hardware drives the direction pin. The
// Arduino version toggled that pin from software around each send. The
// option is off by default and untested on hardware.
#pragma once

#include <cstdint>
#include <cstddef>
#include <functional>

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

namespace esphome {
namespace hcpbridge {

static constexpr size_t MODBUS_MAX_FRAME = 256;

// Driver RX FIFO threshold. Frames longer than this arrive split, see poll().
static constexpr int RX_FULL_THRESHOLD = 120;
// The largest answer is address, function code, byte count, the words, and the
// checksum. Growing either limit past the buffer would write into whatever
// follows it.
static_assert(3 + 2 * 0x7D + 2 <= 256, "response can exceed the transmit buffer");

class ModbusRtuServer {
 public:
  // Called with the complete request (without CRC). Returns the number of
  // bytes written into `response`, or 0 to stay silent (broadcast).
  using Handler = std::function<size_t(const uint8_t *req, size_t len, uint8_t *response)>;

  bool begin(uart_port_t port, int rx_pin, int tx_pin, int rts_pin, uint32_t baud, uint8_t slave_id);
  void set_handler(Handler handler) { this->handler_ = std::move(handler); }

  // Blocking receive/answer cycle; call from a dedicated task.
  void poll(uint32_t timeout_ms = 20);

  static uint16_t crc16(const uint8_t *data, size_t len);

 private:
  uart_port_t port_{UART_NUM_2};
  QueueHandle_t queue_{nullptr};
  uint8_t slave_id_{2};
  Handler handler_;
  uint8_t rx_buf_[MODBUS_MAX_FRAME];
  uint8_t tx_buf_[MODBUS_MAX_FRAME];
};

}  // namespace hcpbridge
}  // namespace esphome
