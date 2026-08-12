#include "hcpbridge.h"

namespace esphome {
namespace hcpbridge {

static const char *TAG = "hcpbridge";
void HCPBridge::setup() {
  const int8_t rx = this->rx_pin_ < 0 ? PIN_RXD : this->rx_pin_;
  const int8_t tx = this->tx_pin_ < 0 ? PIN_TXD : this->tx_pin_;
  const int8_t rts = this->rts_pin_;

  this->engine = &HoermannGarageEngine::getInstance();
  if (!this->engine->setup(rx, tx, rts, this->uart_num_)) {
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
  // For an ota on_begin trigger: writing flash stops the serial driver, so by
  // the time a restart happens the bus has been unanswered for the whole
  // transfer.
  if (this->engine->announcePause(PAUSE_ACK_WAIT_MS)) {
    ESP_LOGI(TAG, "drive confirmed the pause");
  } else {
    ESP_LOGW(TAG, "drive did not confirm the pause");
  }
}

void HCPBridge::on_shutdown() { this->announce_pause(); }
void HCPBridge::dump_config() {
  ESP_LOGCONFIG(TAG, "HCPBridge:");
  ESP_LOGCONFIG(TAG, "  UART%u  rx=%d tx=%d rts=%d", this->uart_num_,
                this->rx_pin_ < 0 ? PIN_RXD : this->rx_pin_,
                this->tx_pin_ < 0 ? PIN_TXD : this->tx_pin_, this->rts_pin_);
}

}  // namespace hcpbridge
}  // namespace esphome
