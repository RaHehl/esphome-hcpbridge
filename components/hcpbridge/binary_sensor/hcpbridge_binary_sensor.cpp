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
    // Named rather than left to a default, so a type added later fails to
    // compile instead of quietly reporting the connection.
    case HCPBRIDGE_BINARY_IS_CONNECTED:
      value = state->valid;
      break;
  }
  // ESPHome drops a repeat itself, so publishing every time costs nothing. The
  // replaced code compared against the published state and seeded it with a
  // hardcoded false in setup(), which told Home Assistant the bus was down
  // before anyone had looked.
  this->publish_state(value);
}

void HCPBridgeBinarySensor::dump_config() { LOG_BINARY_SENSOR("", "HCPBridge", this); }

}  // namespace hcpbridge
}  // namespace esphome
