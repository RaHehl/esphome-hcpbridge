#pragma once

#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "hoermann.h"

namespace esphome {
namespace hcpbridge {

class HCPBridge : public PollingComponent {
 public:
  void setup() override;
  // The drive polls from the moment it has power and counts the polls we do
  // not answer, so the port has to come up before everything else.
  float get_setup_priority() const override { return setup_priority::BUS; }
  void update() override;
  void on_shutdown() override;
  void dump_config() override;
  /** Tell the drive we are about to go quiet. Call from an update on_begin. */
  void announce_pause();
  void set_tx_pin(int8_t tx_pin) { this->tx_pin_ = tx_pin; }
  void set_rx_pin(int8_t rx_pin) { this->rx_pin_ = rx_pin; }
  void set_rts_pin(int8_t rts_pin) { this->rts_pin_ = rts_pin; }
  void set_uart_num(uint8_t uart_num) { this->uart_num_ = uart_num; }
  HoermannGarageEngine *engine{nullptr};
  void add_on_state_callback(std::function<void()> &&callback);

 protected:
  // -1 means "not configured"; the defaults then come from the variant.
  int8_t tx_pin_{-1};
  int8_t rx_pin_{-1};
  int8_t rts_pin_{-1};
  uint8_t uart_num_{2};
  CallbackManager<void()> state_callback_;
};
}  // namespace hcpbridge
}  // namespace esphome
