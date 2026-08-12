#include "modbus_rtu.h"

#include "esphome/core/log.h"

namespace esphome {
namespace hcpbridge {

static const char *const TAG = "hcpbridge.rtu";

// Standard Modbus CRC16 (polynomial 0xA001, reflected 0x8005).
uint16_t ModbusRtuServer::crc16(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 1) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

bool ModbusRtuServer::begin(uart_port_t port, int rx_pin, int tx_pin, int rts_pin, uint32_t baud,
                            uint8_t slave_id) {
  this->port_ = port;
  this->slave_id_ = slave_id;

  uart_config_t cfg = {};
  cfg.baud_rate = static_cast<int>(baud);
  cfg.data_bits = UART_DATA_8_BITS;
  cfg.parity = UART_PARITY_EVEN;   // the drive uses 8E1
  cfg.stop_bits = UART_STOP_BITS_1;
  // Never hardware flow control: in RS485 half duplex the driver owns the
  // direction pin, so the two would fight over it.
  cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  cfg.rx_flow_ctrl_thresh = 0;
  cfg.source_clk = UART_SCLK_DEFAULT;

  if (uart_param_config(this->port_, &cfg) != ESP_OK) {
    ESP_LOGE(TAG, "uart_param_config failed");
    return false;
  }
  if (uart_set_pin(this->port_, tx_pin, rx_pin, rts_pin >= 0 ? rts_pin : UART_PIN_NO_CHANGE,
                   UART_PIN_NO_CHANGE) != ESP_OK) {
    ESP_LOGE(TAG, "uart_set_pin failed");
    return false;
  }
  // The event queue is what tells us a frame ended. Without it uart_read_bytes
  // just waits out its own timeout and merges two frames into one.
  if (uart_driver_install(this->port_, MODBUS_MAX_FRAME * 4, MODBUS_MAX_FRAME * 4, 20, &this->queue_,
                          0) != ESP_OK) {
    ESP_LOGE(TAG, "uart_driver_install failed");
    return false;
  }
  if (rts_pin >= 0) {
    // Half duplex transceiver: the driver toggles RTS around transmission.
    uart_set_mode(this->port_, UART_MODE_RS485_HALF_DUPLEX);
  }
  // Frame end is silence on the line. Careful with the unit: the driver takes
  // BIT times, not character times, so the value below is 10 bits = about
  // 174 us at this baud rate, not the 1750 us the arithmetic reads like. It has
  // always been that, and the drive answers within it, so it stays. Raising it
  // to a true 3.5 characters would be closer to the standard but changes when
  // we answer, which is not something to alter without a door to try it on.
  // Modbus RTU asks for 3.5 character times;
  // the replaced library always waited 1750 us. Keeping that longer gap on
  // purpose: a drive pausing mid-frame would otherwise yield two useless halves.
  const uint32_t symbol_us = 11UL * 1000000UL / baud;      // ~11 Bit je Zeichen
  uint32_t symbols = (1750UL + symbol_us - 1) / symbol_us; // aufrunden
  if (symbols < 4)
    symbols = 4;
  if (symbols > 100)
    symbols = 100;
  uart_set_rx_timeout(this->port_, static_cast<uint8_t>(symbols));
  // High threshold so short HCP frames are always ended by silence, not by a
  // half-full buffer.
  uart_set_rx_full_threshold(this->port_, RX_FULL_THRESHOLD);

  // Whatever was on the line during boot is not a frame we can use.
  uart_flush_input(this->port_);
  xQueueReset(this->queue_);

  ESP_LOGI(TAG, "RTU server on UART%d rx=%d tx=%d rts=%d %" PRIu32 " baud 8E1, slave id %u",
           static_cast<int>(this->port_), rx_pin, tx_pin, rts_pin, baud, this->slave_id_);
  return true;
}

void ModbusRtuServer::poll(uint32_t timeout_ms) {
  uart_event_t ev;
  if (this->queue_ == nullptr)
    return;
  if (xQueueReceive(this->queue_, &ev, pdMS_TO_TICKS(timeout_ms)) != pdTRUE)
    return;
  if (ev.type == UART_FIFO_OVF || ev.type == UART_BUFFER_FULL) {
    ESP_LOGW(TAG, "RX overflow, flushing");
    uart_flush_input(this->port_);
    xQueueReset(this->queue_);
    return;
  }
  if (ev.type != UART_DATA) {
    // Parity/framing errors: leaving those bytes in would shift every frame
    // that follows.
    ESP_LOGW(TAG, "uart event %d, flushing", static_cast<int>(ev.type));
    uart_flush_input(this->port_);
    return;
  }
  size_t to_read = ev.size > sizeof(this->rx_buf_) ? sizeof(this->rx_buf_) : ev.size;
  int len = uart_read_bytes(this->port_, this->rx_buf_, to_read, pdMS_TO_TICKS(2));
  if (len < 4)  // address + function + CRC is the shortest possible frame
    return;

  size_t n = static_cast<size_t>(len);
  // The driver also raises an event on a full RX buffer, not just on silence,
  // so frames above the threshold arrive in pieces that each fail the CRC.
  // Read on until the line is quiet, but only then: short frames - all the
  // drive ever sends - keep their zero extra latency.
  while (n >= RX_FULL_THRESHOLD && n < sizeof(this->rx_buf_)) {
    int more = uart_read_bytes(this->port_, this->rx_buf_ + n, sizeof(this->rx_buf_) - n,
                               pdMS_TO_TICKS(3));
    if (more <= 0)
      break;
    n += static_cast<size_t>(more);
  }
  if (n < 4)
    return;

  uint16_t got = static_cast<uint16_t>(this->rx_buf_[n - 2]) |
                 (static_cast<uint16_t>(this->rx_buf_[n - 1]) << 8);
  uint16_t want = crc16(this->rx_buf_, n - 2);
  if (got != want) {
    ESP_LOGW(TAG, "CRC mismatch (got %04X want %04X, %u bytes)", got, want, static_cast<unsigned>(n));
    return;
  }

  const uint8_t addr = this->rx_buf_[0];
  const bool broadcast = addr == 0;
  if (!broadcast && addr != this->slave_id_)
    return;  // not for us

  if (!this->handler_)
    return;

  size_t rlen = this->handler_(this->rx_buf_, n - 2, this->tx_buf_);
  if (rlen == 0 || broadcast)
    return;  // broadcasts are never answered

  if (rlen + 2 > sizeof(this->tx_buf_)) {
    ESP_LOGE(TAG, "response %u bytes does not fit", static_cast<unsigned>(rlen));
    return;
  }
  uint16_t crc = crc16(this->tx_buf_, rlen);
  this->tx_buf_[rlen++] = static_cast<uint8_t>(crc & 0xFF);
  this->tx_buf_[rlen++] = static_cast<uint8_t>(crc >> 8);
  uart_write_bytes(this->port_, reinterpret_cast<const char *>(this->tx_buf_), rlen);
  // Matches the library's flush(): on a half duplex bus we must not start
  // listening while our own bytes are still going out.
  uart_wait_tx_done(this->port_, pdMS_TO_TICKS(50));
}

}  // namespace hcpbridge
}  // namespace esphome
