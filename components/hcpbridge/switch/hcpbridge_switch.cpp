#include "hcpbridge_switch.h"

namespace esphome {
namespace hcpbridge {

static const char *const TAG = "hcpbridge.switch";

void HCPBridgeSwitch::setup() {
  this->parent_->add_on_state_callback([this]() { this->on_event_triggered(); });
}

void HCPBridgeSwitch::command_dropped() {
  ESP_LOGW(TAG, "%s was not given to the drive", this->get_name().c_str());
  // Home Assistant has already moved the toggle. Publishing the state the door
  // is really in would be dropped as a repeat of what was last published, and
  // the toggle would then stay wrong for as long as the door does not happen to
  // move on its own. Telling the deduplicator it no longer knows lets the
  // correction through.
  this->publish_dedup_.next_unknown();
  this->publish_state(this->parent_->engine->state()->state == this->target_state());
}

bool HCPBridgeSwitch::send_open() {
  return this->type_ == HCPBRIDGE_SWITCH_HALF ? this->parent_->engine->halfPositionDoor(millis())
                                              : this->parent_->engine->ventilationPositionDoor(millis());
}

void HCPBridgeSwitch::on_event_triggered() {
  if (this->parent_ == nullptr || this->parent_->engine == nullptr) {
    return;
  }
  // Nothing has been heard from the drive, so the door's position is unknown
  // rather than false.
  if (!this->parent_->engine->state()->valid) {
    if (!this->status_has_warning()) {
      this->status_set_warning();
    }
    return;
  }
  if (this->status_has_warning()) {
    this->status_clear_warning();
  }

  // Unconditional: ESPHome drops the repeats, and it publishes the first one
  // even when it is false, which a comparison against a false-initialised
  // member never would.
  this->publish_state(this->parent_->engine->state()->state == this->target_state());
}

void HCPBridgeSwitch::write_state(bool state) {
  const HoermannState::State now = this->parent_->engine->state()->state;
  if (state) {
    // Already there: asking again would be a second key press.
    if (now == this->target_state()) {
      return;
    }
    if (!this->send_open()) {
      this->command_dropped();
    }
    return;
  }
  if (now == HoermannState::State::CLOSED) {
    return;
  }
  if (!this->parent_->engine->closeDoor(millis())) {
    this->command_dropped();
  }
}

}  // namespace hcpbridge
}  // namespace esphome
