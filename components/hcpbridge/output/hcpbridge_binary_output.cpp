#include "hcpbridge_binary_output.h"

#include "esphome/core/log.h"

namespace esphome {
namespace hcpbridge {

static const char *TAG = "hcpbridge.binary_output";

void HCPBridgeBinaryOutput::write_state(bool state) {
  if (state != this->parent_->engine->state()->lightOn) {
    ESP_LOGD(TAG, "turning the light %s", state ? "on" : "off");
    if (!this->parent_->engine->turnLight(millis(), state))
      ESP_LOGW(TAG, "the drive was not given that command");
  }
}

void HCPBridgeBinaryOutput::dump_config() { ESP_LOGCONFIG(TAG, "HCPBridge Light output"); }

}  // namespace hcpbridge
}  // namespace esphome
