#pragma once

#include "esphome/core/component.h"
#include "esphome/components/button/button.h"
#include "../hcpbridge.h"

namespace esphome {
namespace hcpbridge {

enum HCPBridgeButtonType {
  HCPBRIDGE_BUTTON_IMPULSE = 0,
  HCPBRIDGE_BUTTON_VENT,
  HCPBRIDGE_BUTTON_HALF,
};

class HCPBridgeButton : public button::Button, public Component {
 public:
  void set_hcpbridge_parent(HCPBridge *parent) { this->parent_ = parent; }
  void set_button_type(HCPBridgeButtonType type) { this->type_ = type; }
  void press_action() override;

 protected:
  HCPBridge *parent_{nullptr};
  HCPBridgeButtonType type_{HCPBRIDGE_BUTTON_IMPULSE};
};

}  // namespace hcpbridge
}  // namespace esphome
