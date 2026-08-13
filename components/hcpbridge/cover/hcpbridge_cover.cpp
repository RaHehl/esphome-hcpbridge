#include "hcpbridge_cover.h"

namespace esphome {
namespace hcpbridge {
static const char *const TAG = "hcpbridge.cover";

// These are reached from a Home Assistant service, where a refusal has nowhere
// to go but the log. Dropping it silently is how "I pressed it and nothing
// happened" becomes unanswerable.
void HCPBridgeCover::report(const char *what, bool sent) {
  if (sent) {
    ESP_LOGD(TAG, "%s", what);
  } else {
    ESP_LOGW(TAG, "%s: the drive was not given that command", what);
  }
}

void HCPBridgeCover::on_go_to_open() { this->report("open", this->parent_->engine->openDoor(millis())); }

void HCPBridgeCover::on_go_to_close() { this->report("close", this->parent_->engine->closeDoor(millis())); }

void HCPBridgeCover::on_go_to_half() { this->report("half open", this->parent_->engine->halfPositionDoor(millis())); }

void HCPBridgeCover::on_go_to_vent() { this->report("vent", this->parent_->engine->ventilationPositionDoor(millis())); }

cover::CoverTraits HCPBridgeCover::get_traits() {
  auto traits = cover::CoverTraits();
  traits.set_is_assumed_state(false);
  // The older bus carries no position: the codec reports the two ends and
  // refuses anything between, because inventing a number is worse than not
  // having one. Offering a slider on top of that gives Home Assistant a control
  // that silently does nothing and a readout that never leaves the ends.
  traits.set_supports_position(this->parent_->caps().hasPosition);
  traits.set_supports_tilt(false);
  traits.set_supports_stop(true);
  traits.set_supports_toggle(true);
  return traits;
}

void HCPBridgeCover::control(const cover::CoverCall &call) {
  bool ok = true;
  if (call.get_stop()) {
    ok = this->parent_->engine->stopDoor(millis()) && ok;
  }
  if (call.get_position().has_value()) {
    if (call.get_position().value() == 1.0f) {
      ok = this->parent_->engine->openDoor(millis()) && ok;
    } else if (call.get_position().value() == 0.0f) {
      ok = this->parent_->engine->closeDoor(millis()) && ok;
    } else {
      ok = this->parent_->engine->setPosition(millis(), call.get_position().value() * 100.0f) && ok;
    }
  }
  if (call.get_toggle()) {
    ok = this->parent_->engine->impulseDoor(millis()) && ok;
  }
  if (!ok) {
    // A busy slot is the usual reason, but not the only one: a dead link
    // refuses too, and so does a position this bus cannot ask for.
    ESP_LOGW(TAG, "the drive was not given that command");
    this->publish_state();  // keep Home Assistant on the real position
  }
}

void HCPBridgeCover::setup() {
  this->parent_->add_on_state_callback([this]() { this->on_event_triggered(); });
}

void HCPBridgeCover::on_event_triggered() {
  if (!this->parent_->engine->state()->valid) {
    if (!this->status_has_warning()) {
      ESP_LOGD(TAG, "the link is down, so what the door is doing is no longer known");
      this->status_set_warning();
      // Otherwise Home Assistant shows "Opening" for ever on a bus that died
      // mid-travel.
      this->current_operation = cover::COVER_OPERATION_IDLE;
      this->publish_state(false);
    }
    return;
  }

  if (this->status_has_warning()) {
    ESP_LOGD(TAG, "the link is back");
    this->status_clear_warning();
  }

  HoermannState *state = this->parent_->engine->state();
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

  // A door that is shut and still when the bus comes up changes neither, and
  // without this nothing would ever be published: the entity would sit unknown
  // in Home Assistant until the door happened to move. It is only reached that
  // way when the drive is already answering at boot - after an update, say -
  // because otherwise the invalid branch above has published once already.
  if (!this->published_ || operation_changed) {
    this->published_ = true;
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
