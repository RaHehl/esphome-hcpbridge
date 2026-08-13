#pragma once

#include "esphome/core/component.h"
#include "esphome/components/switch/switch.h"
#include "esphome/core/log.h"
#include "../hcpbridge.h"

namespace esphome {
namespace hcpbridge {

enum HCPBridgeSwitchType {
  HCPBRIDGE_SWITCH_VENT = 0,
  HCPBRIDGE_SWITCH_HALF,
};

class HCPBridgeSwitch : public switch_::Switch, public Component {
 public:
  void set_hcpbridge_parent(HCPBridge *parent) { this->parent_ = parent; }
  void set_switch_type(HCPBridgeSwitchType type) { this->type_ = type; }
  void setup() override;
  void on_event_triggered();
  void write_state(bool state) override;

 protected:
  /** The position this switch stands for, as the drive reports it. */
  HoermannState::State target_state() const {
    return this->type_ == HCPBRIDGE_SWITCH_HALF ? HoermannState::State::HALFOPEN : HoermannState::State::VENT;
  }
  bool send_open();
  // A dropped command must not leave Home Assistant showing a state the door
  // never reached.
  void command_dropped();

  HCPBridge *parent_{nullptr};
  HCPBridgeSwitchType type_{HCPBRIDGE_SWITCH_VENT};
};

}  // namespace hcpbridge
}  // namespace esphome
