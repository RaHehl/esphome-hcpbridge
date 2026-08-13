#pragma once

#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "hcp_bus.h"

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
  /** True selects the older bus. Set before any component sets up. */
  void set_hcp1(bool hcp1) { this->hcp1_ = hcp1; }

  /**
   * What this door's bus can do.
   *
   * Answerable before setup, unlike anything reached through `engine`: an
   * entity is asked for its traits while it registers, and the bus is not
   * built until this component sets up.
   *
   * By value, not by reference: the two constants have internal linkage, so an
   * inline function handing out references to them would mean something
   * different in every translation unit that included this.
   */
  HcpCapabilities caps() const { return this->hcp1_ ? HCP1_CAPABILITIES : HCP2_CAPABILITIES; }

  // One bus per configured door, so a board with two runs two of these side by
  // side - and since the protocol is now chosen when the door is set up rather
  // than when the image is built, the two may differ.
  HcpBus *engine{nullptr};
  void add_on_state_callback(std::function<void()> &&callback);

 protected:
  /** Empties the protocol path's event ring into the log. */
  void drain_events();
  // -1 means no pin reached us; validation is what fills the variant defaults in.
  int8_t tx_pin_{-1};
  int8_t rx_pin_{-1};
  int8_t rts_pin_{-1};
  uint8_t uart_num_{2};
  bool hcp1_{false};
  CallbackManager<void()> state_callback_;
};
}  // namespace hcpbridge
}  // namespace esphome
