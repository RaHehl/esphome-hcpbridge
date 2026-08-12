#include "esphome/core/log.h"
#include "hcpbridge_light.h"

namespace esphome {
namespace hcpbridge {

static const char *TAG = "hcpbridge.light";

light::LightTraits HCPBridgeLight::get_traits() {
  auto traits = light::LightTraits();
  traits.set_supported_color_modes({light::ColorMode::ON_OFF});
  return traits;
}

void HCPBridgeLight::setup() {
    ESP_LOGD(TAG, "HCPBridgeLight::setup() - setup method calleds");
    this->parent_->add_on_state_callback([this]() { this->on_event_triggered(); });
}

void HCPBridgeLight::write_state(light::LightState *state) {
  bool binary;
  state->current_values_as_binary(&binary);
  if (binary)
    this->output_->turn_on();
  else
    this->output_->turn_off();

  // The light component publishes what was asked for, not what happened. With
  // no drive on the other end that published value is a claim about a lamp
  // nobody switched: the request was refused, yet Home Assistant shows the
  // light as on and keeps showing it, because the correction below only runs
  // when the drive reports something. Put the last thing it did report back.
  if (!this->parent_->engine->state->valid)
    this->sync_from_drive();
}

void HCPBridgeLight::on_event_triggered() {
  if (!this->parent_->engine->state->valid)
    return;
  this->sync_from_drive();
}

void HCPBridgeLight::sync_from_drive() {
  if (this->state_ == nullptr)
    return;
  const bool driveSaysOn = this->parent_->engine->state->lightOn;
  if (this->state_->current_values.is_on() == driveSaysOn)
    return;
  ESP_LOGD(TAG, "HCPBridgeBinaryLight::update() - adjusting state");
  if (driveSaysOn) {
    this->state_->turn_on().perform();
  } else {
    this->state_->turn_off().perform();
  }
}

void HCPBridgeLight::dump_config(){
  ESP_LOGCONFIG(TAG, "HCPBridgeBinaryLight");
}

} //namespace hcpbridge
} //namespace esphome