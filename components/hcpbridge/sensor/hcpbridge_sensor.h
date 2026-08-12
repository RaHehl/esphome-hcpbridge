#pragma once

#include "../hcpbridge.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"

namespace esphome {
namespace hcpbridge {

class HCPBridgeSensor : public sensor::Sensor, public Component {
 public:
  void set_hcpbridge_parent(HCPBridge *parent) { this->parent_ = parent; }
  void set_target_mode(bool target_mode) { this->target_mode_ = target_mode; }
  void setup() override;
  void dump_config() override;
  void update_state(float value);
  void on_event_triggered();

 protected:
  HCPBridge *parent_;
  bool target_mode_ = false;
  float previousPosition_ = 0.0f;
};

}  // namespace hcpbridge
}  // namespace esphome
