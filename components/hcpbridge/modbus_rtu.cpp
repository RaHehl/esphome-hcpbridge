#include "modbus_rtu.h"

#include "esphome/core/log.h"

namespace esphome {
namespace hcpbridge {

static const char *const TAG = "hcpbridge.rtu";

// A bus with nothing on the other end produces one event per idle period, and
// logging each would block the task the drive is waiting on. Counted rather
// than timed, to keep this file free of anything but the UART driver.
static const uint32_t RTU_WARN_EVERY = 256;

bool ModbusRtuServer::warn_due() { return (this->warn_seen_++ % RTU_WARN_EVERY) == 0; }

bool ModbusRtuServer::begin(uart_port_t port, int rx_pin, int tx_pin, int rts_pin, uint32_t baud, uint8_t slave_id) {
  this->port_ = port;
  this->slave_id_ = slave_id;

  uart_config_t cfg = {};
  cfg.baud_rate = static_cast<int>(baud);
  cfg.data_bits = UART_DATA_8_BITS;
  cfg.parity = UART_PARITY_EVEN;  // the drive uses 8E1
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
  if (uart_set_pin(this->port_, tx_pin, rx_pin, rts_pin >= 0 ? rts_pin : UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) !=
      ESP_OK) {
    ESP_LOGE(TAG, "uart_set_pin failed");
    return false;
  }
  // The event queue is what tells us a frame ended. Without it uart_read_bytes
  // just waits out its own timeout and merges two frames into one.
  if (uart_driver_install(this->port_, HCP2_MAX_FRAME * 4, HCP2_MAX_FRAME * 4, 20, &this->queue_, 0) != ESP_OK) {
    ESP_LOGE(TAG, "uart_driver_install failed");
    return false;
  }
  if (rts_pin >= 0) {
    // Half duplex transceiver: the driver toggles RTS around transmission.
    // Unchecked, a failure here would leave us talking over the drive.
    if (uart_set_mode(this->port_, UART_MODE_RS485_HALF_DUPLEX) != ESP_OK) {
      ESP_LOGE(TAG, "RS485 half duplex mode could not be set");
      return false;
    }
  }
  // Mind the unit: the driver counts SYMBOL times, eleven bits at 8E1, so this
  // is about 1.9 ms. Deliberate, and the same wait the replaced library had: a
  // drive pausing mid frame would otherwise yield two useless halves. It is
  // also the largest delay in front of any answer, so look here first if the
  // drive ever turns out to be impatient.
  const uint32_t symbol_us = 11UL * 1000000UL / baud;       // eleven bits per symbol at 8E1
  uint32_t symbols = (1750UL + symbol_us - 1) / symbol_us;  // round up
  if (symbols < 4)
    symbols = 4;
  // The register holds ten bits, and the driver multiplies by the symbol
  // length, so anything above 1023/11 is silently refused.
  if (symbols > 93)
    symbols = 93;
  uart_set_rx_timeout(this->port_, static_cast<uint8_t>(symbols));
  // High threshold so short HCP frames are always ended by silence, not by a
  // half-full buffer.
  uart_set_rx_full_threshold(this->port_, RX_FULL_THRESHOLD);

  // Eleven bits per byte at 8E1, doubled so the wait is a safety net and never
  // the thing that cuts a transmission short.
  this->tx_wait_ms_ = 2 * (HCP2_MAX_FRAME * 11UL * 1000UL / baud) + 5;

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
    if (this->warn_due())
      ESP_LOGW(TAG, "RX overflow, flushing");
    uart_flush_input(this->port_);
    xQueueReset(this->queue_);
    return;
  }
  if (ev.type != UART_DATA) {
    // Parity/framing errors: leaving those bytes in would shift every frame
    // that follows.
    if (this->warn_due())
      ESP_LOGW(TAG, "uart event %d, flushing", static_cast<int>(ev.type));
    uart_flush_input(this->port_);
    xQueueReset(this->queue_);
    return;
  }
  size_t to_read = ev.size > sizeof(this->rx_buf_) ? sizeof(this->rx_buf_) : ev.size;
  int len = uart_read_bytes(this->port_, this->rx_buf_, to_read, pdMS_TO_TICKS(2));
  if (len < 4)  // address, function code and checksum, the shortest there is
    return;

  size_t n = static_cast<size_t>(len);
  // The driver also raises an event on a full buffer, so a long frame arrives
  // in pieces. Only then is it worth waiting for the rest.
  while (n >= RX_FULL_THRESHOLD && n < sizeof(this->rx_buf_)) {
    int more = uart_read_bytes(this->port_, this->rx_buf_ + n, sizeof(this->rx_buf_) - n, pdMS_TO_TICKS(3));
    if (more <= 0)
      break;
    n += static_cast<size_t>(more);
  }
  if (n < 4)
    return;

  const uint8_t addr = this->rx_buf_[0];
  const bool broadcast = addr == 0;
  if (!broadcast && addr != this->slave_id_)
    return;  // not for us

  if (!this->handler_)
    return;

  // The whole frame, checksum included: whether it is worth answering is the
  // handler's judgement, and so is putting a checksum on the answer.
  const size_t rlen = this->handler_(this->rx_buf_, n, this->tx_buf_);
  if (rlen == 0 || broadcast)
    return;  // broadcasts are never answered

  uart_write_bytes(this->port_, reinterpret_cast<const char *>(this->tx_buf_), rlen);
  // On a half duplex bus we must not start listening while our own bytes are
  // still going out. Derived from the baud rate rather than a fixed number: a
  // full frame at 57600 8E1 takes about 49 ms, and a flat 50 would have left a
  // millisecond of margin for a frame we never send today but might.
  uart_wait_tx_done(this->port_, pdMS_TO_TICKS(this->tx_wait_ms_));
}

}  // namespace hcpbridge
}  // namespace esphome
