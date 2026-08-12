#include "hcpbridge.h"

namespace esphome {
namespace hcpbridge {

static const char *TAG = "hcpbridge";
void HCPBridge::setup() {
  int8_t rx = this->rx_pin_ == nullptr ? PIN_RXD : this->rx_pin_->get_pin();
  int8_t tx = this->tx_pin_ == nullptr ? PIN_TXD : this->tx_pin_->get_pin();
  int8_t rts = this->rts_pin_ == nullptr ? -1 : this->rts_pin_->get_pin();

  this->engine = &HoermannGarageEngine::getInstance();
  if (!this->engine->setup(rx, tx, rts)) {
    // Without this the bridge looks healthy, offers a cover, and swallows every
    // command sent to it.
    this->mark_failed();
  }
}
void HCPBridge::add_on_state_callback(std::function<void()> &&callback) {
  this->state_callback_.add(std::move(callback));
}

void HCPBridge::update() {
  if (this->engine == nullptr) {
    return;
  }
  this->engine->publishIdentity();
  this->engine->publishCounterProbe();
  this->engine->checkBusSilence();
  if (this->engine->state->changed) {
    this->engine->state->clearChanged();
    this->state_callback_.call();
  }
}

void HCPBridge::announce_pause() {
  if (this->engine == nullptr) {
    return;
  }
  // Meant for an on_begin trigger of the update platform: by the time a restart
  // happens the bus has been unanswered for the whole transfer already, because
  // writing flash stops the serial driver from running.
  if (this->engine->announcePause(PAUSE_ACK_WAIT_MS)) {
    ESP_LOGI(TAG, "drive confirmed the pause");
  } else {
    ESP_LOGW(TAG, "drive did not confirm the pause");
  }
}

void HCPBridge::on_shutdown() {
  // Announce the pause before restarting, so the drive is told we are going
  // quiet instead of finding an accessory that stopped answering mid frame.
  if (this->engine == nullptr) {
    return;
  }
  if (this->engine->announcePause(PAUSE_ACK_WAIT_MS)) {
    ESP_LOGI(TAG, "drive confirmed the pause");
  } else {
    ESP_LOGW(TAG, "drive did not confirm the pause");
  }
}
}  // namespace hcpbridge
}  // namespace esphome