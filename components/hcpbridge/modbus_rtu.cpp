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
  cfg.flow_ctrl = rts_pin >= 0 ? UART_HW_FLOWCTRL_RTS : UART_HW_FLOWCTRL_DISABLE;
  cfg.rx_flow_ctrl_thresh = 122;
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
  if (uart_driver_install(this->port_, MODBUS_MAX_FRAME * 4, MODBUS_MAX_FRAME * 4, 0, nullptr, 0) !=
      ESP_OK) {
    ESP_LOGE(TAG, "uart_driver_install failed");
    return false;
  }
  if (rts_pin >= 0) {
    // Half duplex transceiver: the driver toggles RTS around transmission.
    uart_set_mode(this->port_, UART_MODE_RS485_HALF_DUPLEX);
  }
  // End of frame after 3.5 idle character times, as Modbus RTU requires.
  uart_set_rx_timeout(this->port_, 4);

  ESP_LOGI(TAG, "RTU server on UART%d rx=%d tx=%d rts=%d %" PRIu32 " baud 8E1, slave id %u",
           static_cast<int>(this->port_), rx_pin, tx_pin, rts_pin, baud, this->slave_id_);
  return true;
}

void ModbusRtuServer::poll(uint32_t timeout_ms) {
  int len = uart_read_bytes(this->port_, this->rx_buf_, sizeof(this->rx_buf_),
                            pdMS_TO_TICKS(timeout_ms));
  if (len < 4)  // address + function + CRC is the shortest possible frame
    return;

  size_t n = static_cast<size_t>(len);
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

  uint16_t crc = crc16(this->tx_buf_, rlen);
  this->tx_buf_[rlen++] = static_cast<uint8_t>(crc & 0xFF);
  this->tx_buf_[rlen++] = static_cast<uint8_t>(crc >> 8);
  uart_write_bytes(this->port_, reinterpret_cast<const char *>(this->tx_buf_), rlen);
}

}  // namespace hcpbridge
}  // namespace esphome
