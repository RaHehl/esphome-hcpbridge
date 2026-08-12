#include "hcpbridge_sensor.h"
#include "esphome/core/log.h"

namespace esphome {
namespace hcpbridge {

static const char *TAG = "hcpbridge.sensor";

void HCPBridgeSensor::setup() {
  this->parent_->add_on_state_callback([this]() { this->on_event_triggered(); });
  this->update_state(0.0f);
}

void HCPBridgeSensor::dump_config() {
  ESP_LOGCONFIG(TAG, "HCPBridge Sensor: %s", this->target_mode_ ? "target position" : "position");
}

void HCPBridgeSensor::update_state(float value) {
  this->publish_state(value * 100);
  this->previousPosition_ = value;
  ESP_LOGD(TAG, "Published new state: %.2f", value);
}

void HCPBridgeSensor::on_event_triggered() {
  // The drive sends both in one register: high byte is where it is heading,
  // low byte where it is now.
  float position = this->target_mode_ ? this->parent_->engine->state->targetPosition
                                      : this->parent_->engine->state->currentPosition;

  if (this->previousPosition_ != position) {
    ESP_LOGD(TAG, "Position changed: %.2f -> %.2f", this->previousPosition_, position);
    this->update_state(position);
  }
}

}  // namespace hcpbridge
}  // namespace esphome
