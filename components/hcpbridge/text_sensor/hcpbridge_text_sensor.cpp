#include "hcpbridge_text_sensor.h"

namespace esphome {
namespace hcpbridge {

static const char *const TAG = "hcpbridge.text_sensor";

void HCPBridgeTextSensor::setup() {
  if (this->parent_ != nullptr) {
    this->parent_->add_on_state_callback([this]() { this->on_event_triggered(); });
  }
}

void HCPBridgeTextSensor::on_event_triggered() {
  if (this->parent_ == nullptr) {
    return;
  }
  const HoermannState *state = this->parent_->engine->state();

  if (this->type_ == HCPBRIDGE_TEXT_STATE) {
    if (this->published_ && state->state == this->previousState_) {
      return;
    }
    this->previousState_ = state->state;
    const std::string text = hoermannStateName(state->state);
    this->published_ = true;
    ESP_LOGD(TAG, "state - %s", text.c_str());
    this->publish_state(text);
    return;
  }

  if (this->type_ == HCPBRIDGE_TEXT_LINK) {
    const char *link = hcpLinkName(state->link);
    if (this->published_ && this->previousText_ == link) {
      return;
    }
    this->previousText_ = link;
    this->published_ = true;
    this->publish_state(link);
    return;
  }

  // The drive answers the identity request once, so nothing is published until
  // the answer has arrived.
  const char *text = this->type_ == HCPBRIDGE_TEXT_SERIAL_NUMBER ? state->serialNumber : state->firmwareVersion;
  if (text[0] == '\0' || (this->published_ && this->previousText_ == text)) {
    return;
  }
  this->previousText_ = text;
  this->published_ = true;
  this->publish_state(text);
}

}  // namespace hcpbridge
}  // namespace esphome
