#include <cstdio>

#include "hcp1_transport.h"

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace hcpbridge {

// Whose bus this task serves, handed over at creation rather than looked up, so
// two doors on one board do not end up both serving the first one's port.
static void hcp1ServeTask(void *parameter) {
  Hcp1Transport *transport = static_cast<Hcp1Transport *>(parameter);
  while (true) {
    transport->serveOnce();
  }
}

bool Hcp1Transport::begin(int8_t rx, int8_t tx, int8_t rts, uint8_t uartNum) {
  // The one place this bus reads the clock. Everything the codec does with time
  // is handed this value, which is what lets the whole exchange be driven from
  // a test with no clock at all.
  this->serial_.set_handler([this](const uint8_t *frame, size_t len, uint8_t *out) -> size_t {
    return this->codec.onFrame(esphome::millis(), frame, len, out);
  });
  if (!this->serial_.begin((uart_port_t) uartNum, rx, tx, rts, HCP1_BAUD)) {
    ESP_LOGE(TAG_HCI, "serial setup failed, bus task not started");
    return false;
  }

  // Named after the port, so two doors on one board can be told apart in a
  // crash report or a task list. FreeRTOS copies the name, so the buffer
  // may live on the stack.
  char taskName[configMAX_TASK_NAME_LEN];
  snprintf(taskName, sizeof(taskName), "hcp1-uart%u", (unsigned) uartNum);
  if (!startBusTask(taskName, hcp1ServeTask, this, &this->busTask_)) {
    ESP_LOGE(TAG_HCI, "bus task could not be created");
    return false;
  }
  return true;
}

}  // namespace hcpbridge
}  // namespace esphome
