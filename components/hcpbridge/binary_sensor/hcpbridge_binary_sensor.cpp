#include "hcpbridge_binary_sensor.h"

namespace esphome {
namespace hcpbridge {

static const char *const TAG = "hcpbridge.binary_sensor";

void HCPBridgeBinarySensor::setup() {
  this->parent_->add_on_state_callback([this]() { this->on_event_triggered(); });
}

void HCPBridgeBinarySensor::on_event_triggered() {
  const HoermannState *state = this->parent_->engine->state();
  bool value;
  switch (this->type_) {
    case HCPBRIDGE_BINARY_RELAY_STATE:
      value = state->relayOn;
      break;
    case HCPBRIDGE_BINARY_ACTUATOR_ERROR:
      value = state->actuatorError;
      break;
    default:
      value = state->valid;
      break;
  }
  // ESPHome drops a repeat itself, and it gets the first one right where a
  // comparison against `state` cannot: that member starts out false, so a
  // bridge booting with a dead bus would publish nothing at all and leave the
  // connected sensor unknown in Home Assistant - which is the one moment it
  // exists to speak up.
  this->publish_state(value);
}

void HCPBridgeBinarySensor::dump_config() { LOG_BINARY_SENSOR("", "HCPBridge", this); }

}  // namespace hcpbridge
}  // namespace esphome
