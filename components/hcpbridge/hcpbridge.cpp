#include "hcpbridge.h"

namespace esphome {
namespace hcpbridge {

static const char *TAG = "hcpbridge";
void HCPBridge::setup() {
  const int8_t rx = this->rx_pin_;
  const int8_t tx = this->tx_pin_;
  const int8_t rts = this->rts_pin_;

  // Built before anything can fail, and never torn down. The entities are
  // separate components with their own Home Assistant surface: mark_failed()
  // here does not stop a button from being pressed, so a bus that was never
  // built would be dereferenced through a null pointer at the first press. An
  // unstarted bus instead refuses every command and says why, which is the
  // behaviour that already exists for a drive that is not answering.
  this->engine = this->hcp1_ ? static_cast<HcpBus *>(new Hcp1Bus()) : static_cast<HcpBus *>(new Hcp2Bus());
  if (this->engine == nullptr) {
    // ESP-IDF builds without exceptions, where a failed new usually aborts
    // rather than returning here at all. Checking costs nothing and is what
    // this has to do if that ever changes.
    ESP_LOGE(TAG, "no memory for the bus");
    this->mark_failed();
    return;
  }

  // Which pins a variant defaults to is decided during validation, where it can
  // be shown in `esphome config` and checked against the rest of it. This only
  // catches a build that got past that. Refusing beats handing -1 to the
  // driver, which reads as "leave the pin alone" and gives a port that looks
  // configured and hears nothing.
  if (rx < 0 || tx < 0) {
    ESP_LOGE(TAG, "no pins were configured for this bus");
    this->mark_failed();
    return;
  }
  if (!this->engine->begin(rx, tx, rts, this->uart_num_)) {
    // Without this the bridge looks healthy, offers a cover, and swallows every
    // command sent to it.
    this->mark_failed();
  }
}
void HCPBridge::add_on_state_callback(std::function<void()> &&callback) {
  this->state_callback_.add(std::move(callback));
}

void HCPBridge::drain_events() {
  // Turning numbers into sentences happens here, on the loop task, where a slow
  // console costs nothing the drive is waiting for.
  HcpEventEntry batch[16];
  size_t n;
  do {
    n = this->engine->events()->drain(batch, sizeof(batch) / sizeof(batch[0]));
    for (size_t i = 0; i < n; i++) {
      const HcpEventEntry &e = batch[i];
      const char *fmt = hcpEventFormat(e.code);
      const bool trouble = hcpEventIsTrouble(e.code);
      if (hcpEventArgKind(e.code) == HcpArgKind::COMMAND) {
        const char *name = commandName((HoermannCommand) e.a);
        if (trouble) {
          ESP_LOGW(TAG_HCI, fmt, name, (unsigned) e.b);
        } else {
          ESP_LOGI(TAG_HCI, fmt, name, (unsigned) e.b);
        }
      } else if (trouble) {
        ESP_LOGW(TAG_HCI, fmt, (unsigned) e.a, (unsigned) e.b);
      } else {
        ESP_LOGI(TAG_HCI, fmt, (unsigned) e.a, (unsigned) e.b);
      }
    }
  } while (n == sizeof(batch) / sizeof(batch[0]));

  const uint32_t lost = this->engine->events()->takeDropped();
  if (lost != 0) {
    ESP_LOGW(TAG_HCI, "%u events did not fit and were dropped", (unsigned) lost);
  }
}

void HCPBridge::update() {
  if (this->engine == nullptr) {
    return;
  }
  this->drain_events();
  this->engine->publishIdentity();
  this->engine->checkBusSilence(millis());
  if (this->engine->state()->changed) {
    this->engine->state()->clearChanged();
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
  // On a bus with no such handshake this answers false without waiting, and
  // the drive simply notices the silence.
  if (this->engine->announcePause(PAUSE_ACK_WAIT_MS)) {
    ESP_LOGI(TAG, "drive confirmed the pause");
  } else {
    ESP_LOGW(TAG, "drive did not confirm the pause");
  }
}

void HCPBridge::on_shutdown() {
  this->announce_pause();
  // Asked of both buses. announce_pause already settles on the protocol that
  // has a handshake and returns straight away on the one that does not, which
  // left the older bus restarting mid telegram.
  if (this->engine != nullptr)
    this->engine->quiesce();
}
void HCPBridge::dump_config() {
  ESP_LOGCONFIG(TAG, "HCPBridge:");
  // Which bus this door speaks is a runtime answer now, so the log has to say
  // it: with two doors on one board they may differ.
  ESP_LOGCONFIG(TAG, "  %s on UART%u  rx=%d tx=%d rts=%d", this->hcp1_ ? "hcp1" : "hcp2", this->uart_num_,
                this->rx_pin_, this->tx_pin_, this->rts_pin_);
}

}  // namespace hcpbridge
}  // namespace esphome
