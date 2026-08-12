#include "hcpbridge_textSensor.h"

namespace esphome {
namespace hcpbridge {

static const char *const TAG = "hcpbridge.text_sensor";

static std::string stateToText(HoermannState::State state) {
  switch (state) {
    case HoermannState::OPENING:
      return "Opening";
    case HoermannState::MOVE_VENTING:
      return "Move venting";
    case HoermannState::MOVE_HALF:
      return "Move half";
    case HoermannState::CLOSING:
      return "Closing";
    case HoermannState::OPEN:
      return "Open";
    case HoermannState::CLOSED:
      return "Closed";
    case HoermannState::STOPPED:
      return "Stopped";
    case HoermannState::HALFOPEN:
      return "Half open";
    case HoermannState::VENT:
      return "Venting";
    default:
      return "Unknown";
  }
}

void HCPBridgeTextSensor::setup() {
  if (this->parent_ != nullptr) {
    this->parent_->add_on_state_callback([this]() { this->on_event_triggered(); });
  }
}

void HCPBridgeTextSensor::on_event_triggered() {
  if (this->parent_ == nullptr) {
    return;
  }
  const HoermannState *state = this->parent_->engine->state;

  if (this->type_ == HCPBRIDGE_TEXT_STATE) {
    if (this->published_ && state->state == this->previousState_) {
      return;
    }
    this->previousState_ = state->state;
    const std::string text = stateToText(state->state);
    this->published_ = true;
    ESP_LOGD(TAG, "state - %s", text.c_str());
    this->publish_state(text);
    return;
  }

  // The drive answers the identity request once, so nothing is published until
  // the answer has arrived.
  const std::string &text = this->type_ == HCPBRIDGE_TEXT_SERIAL_NUMBER ? state->serialNumber
                                                                       : state->firmwareVersion;
  if (text.empty() || (this->published_ && text == this->previousText_)) {
    return;
  }
  this->previousText_ = text;
  this->published_ = true;
  this->publish_state(text);
}

}  // namespace hcpbridge
}  // namespace esphome
