#pragma once

#include "esphome/core/component.h"
#include "esphome/components/cover/cover.h"
#include "../hcpbridge.h"

namespace esphome {
namespace hcpbridge {

class HCPBridgeCover : public cover::Cover, public Component {
 public:
  cover::CoverTraits get_traits() override;
  void control(const cover::CoverCall &call) override;
  void set_hcpbridge_parent(HCPBridge *parent) { this->parent_ = parent; }
  void setup() override;
  void on_event_triggered();
  bool get_light_state() { return this->parent_->engine->state()->lightOn; }
  void set_light_state(bool state) { this->parent_->engine->turnLight(millis(), state); }
  bool get_relay_state() { return this->parent_->engine->state()->relayOn; }
  HoermannState::State get_cover_state() { return this->parent_->engine->state()->state; }
  void on_go_to_half();
  void on_go_to_open();
  void on_go_to_close();
  void on_go_to_vent();

 protected:
  /** Says in the log whether a service call reached the drive. */
  void report(const char *what, bool sent);
  HCPBridge *parent_;
  // Whether Home Assistant has been told anything at all yet.
  bool published_ = false;
  float previousPosition_ = 0.0f;
  cover::CoverOperation previousOperation_ = cover::COVER_OPERATION_IDLE;
};
}  // namespace hcpbridge
}  // namespace esphome
