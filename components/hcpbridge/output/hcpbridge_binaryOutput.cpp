#include "hcpbridge_binaryOutput.h"

#include "esphome/core/log.h"

namespace esphome {
namespace hcpbridge {

static const char *TAG = "hcpbridge.binary_output";

void HCPBridgeBinaryOutput::write_state(bool state) {
  if (state != this->parent_->engine->state->lightOn) {
    ESP_LOGD(TAG, "HCPBridgeBinaryOutput::write_state() - turning light %s", state ? "true" : "false");
    if (!this->parent_->engine->turnLight(state))
      ESP_LOGW(TAG, "command dropped, the drive has not fetched the previous one yet");
  } 
}

void HCPBridgeBinaryOutput::dump_config() {
  ESP_LOGCONFIG(TAG, "HCPBridgeBinaryOutput");
}

}  // namespace hcpbridge
}  // namespace esphome
