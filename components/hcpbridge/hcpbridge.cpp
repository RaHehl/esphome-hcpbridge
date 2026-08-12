#include "hcpbridge.h"

namespace esphome {
namespace hcpbridge {

static const char *TAG = "hcpbridge";
void HCPBridge::setup() {
  int8_t rx = this->rx_pin_ == nullptr ? PIN_RXD : this->rx_pin_->get_pin();
  int8_t tx = this->tx_pin_ == nullptr ? PIN_TXD : this->tx_pin_->get_pin();
  int8_t rts = this->rts_pin_ == nullptr ? -1 : this->rts_pin_->get_pin();

  this->engine = &HoermannGarageEngine::getInstance();
  this->engine->setup(rx, tx, rts);
}
void HCPBridge::add_on_state_callback(std::function<void()> &&callback) {
  this->state_callback_.add(std::move(callback));
}

void HCPBridge::update() {
  this->engine->checkBusSilence();
  if (this->engine->state->changed) {
    this->engine->state->clearChanged();
    this->state_callback_.call();
  }
}

void HCPBridge::on_shutdown() {
  // Say goodbye before the restart, so the drive lets go of us in an orderly
  // way instead of finding an accessory that stopped answering mid frame.
  if (this->engine == nullptr) {
    return;
  }
  if (this->engine->leaveBus(LEAVE_WAIT_MS)) {
    ESP_LOGI(TAG, "drive confirmed we are leaving the bus");
  } else {
    ESP_LOGW(TAG, "drive did not confirm us leaving the bus");
  }
}
}  // namespace hcpbridge
}  // namespace esphome