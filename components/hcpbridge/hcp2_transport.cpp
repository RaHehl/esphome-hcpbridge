#include <cstdio>

#include "hcp2_transport.h"

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace hcpbridge {

// Whose bus this task serves, handed over at creation rather than looked up, so
// two doors on one board do not end up both serving the first one's port.
static void busServeTask(void *parameter) {
  Hcp2Transport *transport = static_cast<Hcp2Transport *>(parameter);
  while (true) {
    transport->serveOnce();
  }
}

bool Hcp2Transport::begin(int8_t rx, int8_t tx, int8_t rts, uint8_t uartNum) {
  // The one place the bus task reads the clock. Everything the codec does with
  // time is handed this value, which is what lets the whole exchange be driven
  // from a test with no clock at all.
  this->mb_.set_handler([this](const uint8_t *req, size_t len, uint8_t *resp) -> size_t {
    return this->codec.onFrame(esphome::millis(), req, len, resp);
  });
  if (!this->mb_.begin((uart_port_t) uartNum, rx, tx, rts, HCP_BAUD, SLAVE_ID)) {
    // No port, no task: at top priority it would spin, because poll() returns
    // immediately.
    ESP_LOGE(TAG_HCI, "serial setup failed, bus task not started");
    return false;
  }

  // Named after the port, so two doors on one board can be told apart in a
  // crash report or a task list. FreeRTOS copies the name, so the buffer
  // may live on the stack.
  char taskName[configMAX_TASK_NAME_LEN];
  snprintf(taskName, sizeof(taskName), "hcp2-uart%u", (unsigned) uartNum);
  if (!startBusTask(taskName, busServeTask, this, &this->busTask_)) {
    ESP_LOGE(TAG_HCI, "bus task could not be created");
    return false;
  }
  return true;
}

bool Hcp2Transport::announcePause(uint32_t timeoutMs) {
  // On a bus with no such handshake there is nobody to tell and nothing to wait
  // for, and waiting anyway would hold up a restart for no reason.
  if (!this->codec.caps.hasPause)
    return false;

  const uint32_t last = this->codec.lastFrameAt();
  // Nobody to say it to: nothing ever arrived, or the bus has been quiet past
  // the point where we call it gone. A restart must not pay for that.
  if (last == 0 || (esphome::millis() - last) > BUS_SILENCE_MS)
    return false;

  // A waiting press is dropped rather than arriving after a restart with nobody
  // expecting it.
  this->codec.dropQueuedCommand();
  this->codec.requestPause();

  const uint32_t started = esphome::millis();
  // Subtract, never add: adding overflows when millis() wraps.
  while ((esphome::millis() - started) < timeoutMs) {
    esphome::App.feed_wdt();
    if (this->codec.pauseAcknowledged()) {
      // Going back to the ordinary status here would say "never mind" for two
      // seconds and then vanish anyway.
      this->settleBeforeRestart();
      this->codec.endPause();
      return true;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  // Back to answering normally: a restart that never happens must not leave the
  // bridge announcing a pause for ever.
  this->settleBeforeRestart();
  this->codec.endPause();
  return false;
}

// The restart has to land between telegrams, not inside one. The bus task keeps
// running, so a fixed sleep would prove nothing.
void Hcp2Transport::settleBeforeRestart() {
  const uint32_t started = esphome::millis();
  while ((esphome::millis() - started) < PAUSE_SETTLE_MS) {
    esphome::App.feed_wdt();
    const uint32_t last = this->codec.lastFrameAt();
    if (last != 0 && (esphome::millis() - last) > PAUSE_QUIET_MS)
      return;
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

}  // namespace hcpbridge
}  // namespace esphome
