#pragma once

#include "esphome/core/component.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "../hcpbridge.h"

namespace esphome {
namespace hcpbridge {

enum HCPBridgeBinarySensorType {
  HCPBRIDGE_BINARY_IS_CONNECTED = 0,
  HCPBRIDGE_BINARY_RELAY_STATE,
  HCPBRIDGE_BINARY_ACTUATOR_ERROR,
};

class HCPBridgeBinarySensor : public binary_sensor::BinarySensor, public Component {
 public:
  void set_hcpbridge_parent(HCPBridge *parent) { this->parent_ = parent; }
  void set_sensor_type(HCPBridgeBinarySensorType type) { this->type_ = type; }
  void setup() override;
  void on_event_triggered();
  void dump_config() override;

 protected:
  HCPBridge *parent_{nullptr};
  HCPBridgeBinarySensorType type_{HCPBRIDGE_BINARY_IS_CONNECTED};
};

}  // namespace hcpbridge
}  // namespace esphome
