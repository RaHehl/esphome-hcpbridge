#include "hcpbridge_button.h"

namespace esphome {
namespace hcpbridge {

static const char *const TAG = "hcpbridge.button";

void HCPBridgeButtonVent::press_action() {
  ESP_LOGD(TAG, "HCPBridgeButtonVent::press_action() - Triggering vent position");
  if (!this->parent_->engine->ventilationPositionDoor())
    ESP_LOGW(TAG, "command dropped, the drive has not fetched the previous one yet");
}

void HCPBridgeButtonHalf::press_action() {
  ESP_LOGD(TAG, "HCPBridgeButtonHalf::press_action() - Triggering half-open position");
  if (!this->parent_->engine->halfPositionDoor())
    ESP_LOGW(TAG, "command dropped, the drive has not fetched the previous one yet");
}

void HCPBridgeButtonImpulse::press_action() {
  ESP_LOGD(TAG, "HCPBridgeButtonImpulse::press_action() - Triggering impulse action");
  if (!this->parent_->engine->impulseDoor())
    ESP_LOGW(TAG, "command dropped, the drive has not fetched the previous one yet");
}

}  // namespace hcpbridge
}  // namespace esphome
