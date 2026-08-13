#include "hcpbridge_button.h"

namespace esphome {
namespace hcpbridge {

static const char *const TAG = "hcpbridge.button";

void HCPBridgeButton::press_action() {
  // Whichever bus was configured. Naming the concrete type here is how this
  // file went on referring to a class that had been renamed away.
  HcpBus *engine = this->parent_->engine;
  bool sent;
  const char *what;
  switch (this->type_) {
    case HCPBRIDGE_BUTTON_VENT:
      what = "vent";
      sent = engine->ventilationPositionDoor(millis());
      break;
    case HCPBRIDGE_BUTTON_HALF:
      what = "half open";
      sent = engine->halfPositionDoor(millis());
      break;
    default:
      what = "impulse";
      sent = engine->impulseDoor(millis());
      break;
  }
  if (!sent) {
    ESP_LOGW(TAG, "%s dropped, the drive has not fetched the previous command yet", what);
  }
}

}  // namespace hcpbridge
}  // namespace esphome
