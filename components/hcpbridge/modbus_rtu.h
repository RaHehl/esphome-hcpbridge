// Minimal Modbus RTU server for the Hoermann HCP bus.
//
// Replaces the Arduino library "emelianov/modbus-esp8266". Only what the
// door drive actually asks for is implemented: function code 0x17
// (read/write multiple registers) and 0x10 (write multiple registers).
//
// Frame detection uses the ESP-IDF UART driver's RX timeout: Modbus RTU
// separates frames by a gap of 3.5 character times, and the driver raises
// an event once the line has been idle that long. Polling for that in
// software - as the Arduino version had to - is far less reliable.
#pragma once

#include <cstdint>
#include <cstddef>
#include <functional>

#include "driver/uart.h"

namespace esphome {
namespace hcpbridge {

static constexpr size_t MODBUS_MAX_FRAME = 256;

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
  uint8_t slave_id_{2};
  Handler handler_;
  uint8_t rx_buf_[MODBUS_MAX_FRAME];
  uint8_t tx_buf_[MODBUS_MAX_FRAME];
};

}  // namespace hcpbridge
}  // namespace esphome
