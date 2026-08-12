#include "hcpbridge_cover.h"

namespace esphome {
namespace hcpbridge {
static const char *const TAG = "hcpbridge.cover";

void HCPBridgeCover::on_go_to_open() {
  ESP_LOGD(TAG, "HCPBridgeCover::on_go_to_open() - opening");
  this->parent_->engine->openDoor();
}

void HCPBridgeCover::on_go_to_close() {
  ESP_LOGD(TAG, "HCPBridgeCover::on_go_to_close() - closing");
  this->parent_->engine->closeDoor();
}

void HCPBridgeCover::on_go_to_half() {
  ESP_LOGD(TAG, "HCPBridgeCover::on_go_to_half() - half opening");
  this->parent_->engine->halfPositionDoor();
}

void HCPBridgeCover::on_go_to_vent() {
  ESP_LOGD(TAG, "HCPBridgeCover::on_go_to_vent() - venting");
  this->parent_->engine->ventilationPositionDoor();
}

cover::CoverTraits HCPBridgeCover::get_traits() {
  auto traits = cover::CoverTraits();
  traits.set_is_assumed_state(false);
  traits.set_supports_position(true);
  traits.set_supports_tilt(false);
  traits.set_supports_stop(true);
  traits.set_supports_toggle(true);
  return traits;
}

void HCPBridgeCover::control(const cover::CoverCall &call) {
  bool ok = true;
  if (call.get_stop()) {
    ok = this->parent_->engine->stopDoor() && ok;
  }
  if (call.get_position().has_value()) {
    if (call.get_position().value() == 1.0f) {
      ok = this->parent_->engine->openDoor() && ok;
    } else if (call.get_position().value() == 0.0f) {
      ok = this->parent_->engine->closeDoor() && ok;
    } else {
      ok = this->parent_->engine->setPosition(call.get_position().value() * 100.0f) && ok;
    }
  }
  if (call.get_toggle()) {
    ok = this->parent_->engine->impulseDoor() && ok;
  }
  if (!ok) {
    ESP_LOGW(TAG, "command dropped, the drive has not fetched the previous one yet");
    this->publish_state();  // keep Home Assistant on the real position
  }
}

void HCPBridgeCover::setup() {
  ESP_LOGD(TAG, "HCPBridgeCover::setup() - setup method calleds");
  this->parent_->add_on_state_callback([this]() { this->on_event_triggered(); });
}

void HCPBridgeCover::on_event_triggered() {
  if (!this->parent_->engine->state->valid) {
    if (!this->status_has_warning()) {
      ESP_LOGD(TAG,
               "HCPBridgeCover::on_event_triggered() - state is invalid, "
               "setting warning");
      this->status_set_warning();
      // Otherwise Home Assistant shows "Opening" for ever on a bus that died
      // mid-travel.
      this->current_operation = cover::COVER_OPERATION_IDLE;
      this->publish_state(false);
    }
    return;
  }

  if (this->status_has_warning()) {
    ESP_LOGD(TAG, "HCPBridgeCover::on_event_triggered() - clearing warning");
    this->status_clear_warning();
  }

  HoermannState *state = this->parent_->engine->state;
  float currentPosition = state->currentPosition;
  HoermannState::State stateValue = state->state;

  switch (stateValue) {
    case HoermannState::OPENING:
      this->current_operation = cover::COVER_OPERATION_OPENING;
      break;
    case HoermannState::MOVE_VENTING:
    case HoermannState::MOVE_HALF:
      // 1.0 is fully open, so a falling position is the door coming down.
      if (this->previousPosition_ > currentPosition) {
        this->current_operation = cover::COVER_OPERATION_CLOSING;
      } else {
        this->current_operation = cover::COVER_OPERATION_OPENING;
      }
      break;
    case HoermannState::CLOSING:
      this->current_operation = cover::COVER_OPERATION_CLOSING;
      break;
    default:
      this->current_operation = cover::COVER_OPERATION_IDLE;
      break;
  }

  this->position = currentPosition;

  bool position_changed = this->previousPosition_ != this->position;
  bool operation_changed = this->previousOperation_ != this->current_operation;

  if (operation_changed) {
    ESP_LOGV(TAG, "Operation changed: %d", this->current_operation);
    this->publish_state();  // Full publish on operation change
    this->previousOperation_ = this->current_operation;
  } else if (position_changed) {
    ESP_LOGV(TAG, "Position changed: %f", this->position);
    this->publish_state(false);  // Partial publish on position change
  }

  if (position_changed) {
    this->previousPosition_ = this->position;
  }
}
}  // namespace hcpbridge
}  // namespace esphome
