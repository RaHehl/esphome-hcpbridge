#pragma once

#include <string>

#include "esphome/core/component.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "../hcpbridge.h"

namespace esphome {
namespace hcpbridge {

enum HCPBridgeTextSensorType {
  HCPBRIDGE_TEXT_STATE = 0,
  HCPBRIDGE_TEXT_SERIAL_NUMBER,
  HCPBRIDGE_TEXT_FIRMWARE_VERSION,
};

class HCPBridgeTextSensor : public text_sensor::TextSensor, public Component {
  public:
      void set_hcpbridge_parent(HCPBridge *parent) { this->parent_ = parent; }
      void set_sensor_type(HCPBridgeTextSensorType type) { this->type_ = type; }
      void setup() override;
      void on_event_triggered();

  protected:
      HCPBridge *parent_{nullptr};
      HCPBridgeTextSensorType type_{HCPBRIDGE_TEXT_STATE};
      HoermannState::State previousState_{HoermannState::CLOSED};
      std::string previousText_;
      bool published_{false};
};

}  // namespace hcpbridge
}  // namespace esphome
