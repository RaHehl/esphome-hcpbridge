#include "hcp1_serial.h"

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace hcpbridge {

static const char *const TAG = "hcpbridge.serial";

// A break is measured in bit times, not microseconds. The drive holds the line
// for about a millisecond, which at this baud rate is roughly twenty bits; a
// receiver counts anything past one character as a break, so being comfortably
// above that matters more than the exact number.
static constexpr int HCP1_BREAK_BITS = 20;
// After the break, before the frame: a receiver needs the line idle again to
// find the first start bit.
static constexpr uint32_t HCP1_MARK_US = 200;
// Whatever this carries is discarded; the receiver is still inside the break.
static constexpr uint8_t HCP1_BREAK_CARRIER = 0x00;

bool Hcp1SerialServer::begin(uart_port_t port, int rx_pin, int tx_pin, int rts_pin, uint32_t baud) {
  this->port_ = port;

  uart_config_t cfg = {};
  cfg.baud_rate = (int) baud;
  cfg.data_bits = UART_DATA_8_BITS;
  // No parity, unlike the newer bus. The checksum is eight bits and the only
  // thing guarding a frame, which is why the codec also refuses anything whose
  // length disagrees with itself.
  cfg.parity = UART_PARITY_DISABLE;
  cfg.stop_bits = UART_STOP_BITS_1;
  cfg.flow_ctrl = rts_pin >= 0 ? UART_HW_FLOWCTRL_RTS : UART_HW_FLOWCTRL_DISABLE;
  cfg.source_clk = UART_SCLK_DEFAULT;

  if (uart_param_config(this->port_, &cfg) != ESP_OK)
    return false;
  if (uart_set_pin(this->port_, tx_pin, rx_pin, rts_pin >= 0 ? rts_pin : UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) !=
      ESP_OK)
    return false;
  if (uart_driver_install(this->port_, 256, 256, 20, &this->queue_, 0) != ESP_OK)
    return false;
  if (rts_pin >= 0)
    uart_set_mode(this->port_, UART_MODE_RS485_HALF_DUPLEX);
  return true;
}

void Hcp1SerialServer::send(const uint8_t *frame, size_t len) {
  uart_write_bytes_with_break(this->port_, &HCP1_BREAK_CARRIER, 1, HCP1_BREAK_BITS);
  uart_wait_tx_done(this->port_, pdMS_TO_TICKS(20));
  esphome::delayMicroseconds(HCP1_MARK_US);
  uart_write_bytes(this->port_, frame, len);
  // On a half duplex bus we must not start listening while our own bytes are
  // still going out.
  uart_wait_tx_done(this->port_, pdMS_TO_TICKS(50));
}

void Hcp1SerialServer::poll(uint32_t timeoutMs) {
  uart_event_t event;
  if (this->queue_ == nullptr || xQueueReceive(this->queue_, &event, pdMS_TO_TICKS(timeoutMs)) != pdTRUE)
    return;

  switch (event.type) {
    case UART_BREAK:
      // A break is where one frame ends and the next begins. The driver has
      // already dropped what it had buffered, so nothing useful arrives with this
      // event: it is a marker, and whatever was collected before it is a frame.
      if (this->rx_len_ > 0 && this->handler_) {
        const size_t answer = this->handler_(this->rx_buf_, this->rx_len_, this->tx_buf_);
        if (answer > 0)
          this->send(this->tx_buf_, answer);
      }
      this->rx_len_ = 0;
      break;

    case UART_DATA: {
      size_t room = sizeof(this->rx_buf_) - this->rx_len_;
      if (room == 0) {
        // Longer than any frame on this bus can be, so the break that would have
        // ended it was missed. Start again at the next one rather than answering
        // something assembled out of two.
        this->rx_len_ = 0;
        room = sizeof(this->rx_buf_);
      }
      const int got =
          uart_read_bytes(this->port_, this->rx_buf_ + this->rx_len_, event.size < room ? event.size : room, 0);
      if (got > 0)
        this->rx_len_ += (size_t) got;
      break;
    }

    default:
      // A parity or frame error means what was collected cannot be trusted to be
      // one frame, and eight bits of checksum are too narrow to lean on for that.
      ESP_LOGD(TAG, "line error, waiting for the next break");
      uart_flush_input(this->port_);
      this->rx_len_ = 0;
      break;
  }
}

}  // namespace hcpbridge
}  // namespace esphome
