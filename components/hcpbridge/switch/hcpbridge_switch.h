#pragma once

#include "esphome/core/component.h"
#include "esphome/components/switch/switch.h"
#include "../hcpbridge.h"
#include "esphome/core/log.h"

namespace esphome {
namespace hcpbridge {

class HCPBridgeSwitchVent : public switch_::Switch, public Component {
  public:
    void set_hcpbridge_parent(HCPBridge *parent) { this->parent_ = parent; }
    void setup() override;
    void on_event_triggered();
    void write_state(bool state) override;
    bool previousState_ = false;
  protected:
    // A dropped command must not leave Home Assistant showing a state the door
    // never reached.
    void command_dropped(const char *tag) {
      ESP_LOGW(tag, "command dropped, republishing actual state");
      this->publish_state(this->previousState_);
    }
    HCPBridge *parent_;
};

// Class for Half Switch
class HCPBridgeSwitchHalf : public switch_::Switch, public Component {
  public:
    void set_hcpbridge_parent(HCPBridge *parent) { this->parent_ = parent; }
    void setup() override;
    void on_event_triggered();
    void write_state(bool state) override;
    bool previousState_ = false;
  protected:
    // A dropped command must not leave Home Assistant showing a state the door
    // never reached.
    void command_dropped(const char *tag) {
      ESP_LOGW(tag, "command dropped, republishing actual state");
      this->publish_state(this->previousState_);
    }
    HCPBridge *parent_;
};

}  // namespace hcpbridge
}  // namespace esphome
