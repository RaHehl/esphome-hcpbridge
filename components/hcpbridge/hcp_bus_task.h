// The task both buses answer from.
//
// Which frames go over the wire differs between the two protocols; the deadline
// does not. Both have a drive on the other end that counts unanswered polls, so
// both want the same priority, the same core, and the same refusal to guess at
// either. Kept in one place because two copies of a scheduling decision drift,
// and the copy that drifts is the one nobody has a drive for.
#pragma once

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "hcp_state.h"

namespace esphome {
namespace hcpbridge {

// Above the application, below the radio stack: the answer to a frame is short
// and time-critical, but starving Wi-Fi to deliver it would trade one problem
// for another. The task blocks on the driver's queue, so it only runs when a
// frame has actually arrived.
//
// The value holds on single-core variants too. The deadline belongs to the
// drive, and below the network stack (lwIP runs at 18) a busy one, an update
// download most of all, would sit in front of the answer. What makes that safe
// is that this task cannot spin: it blocks on the driver's queue and the work
// per frame is bounded.
static constexpr unsigned HCP_TASK_PRIO = 19;

// In bytes. The protocol call chain needs the room.
static constexpr unsigned HCP_TASK_STACK = 8192;

/**
 * Waits for a gap on the wire before the caller pulls the floor out.
 *
 * A restart landing inside a telegram is what makes a drive count an
 * unanswered poll, and neither bus can help that once the task is gone. This
 * belongs to both of them: only the pause handshake is a property of the newer
 * protocol, and waiting for quiet is a property of having a wire at all. It
 * used to live in the newer transport, which left the older one with nothing.
 *
 * Gives up after PAUSE_SETTLE_MS: a drive that never falls quiet must not stop
 * a shutdown from happening.
 */
template<typename LastFrameAt> void settleUntilQuiet(LastFrameAt lastFrameAt) {
  const uint32_t started = esphome::millis();
  while ((esphome::millis() - started) < PAUSE_SETTLE_MS) {
    esphome::App.feed_wdt();
    const uint32_t last = lastFrameAt();
    if (last != 0 && (esphome::millis() - last) > PAUSE_QUIET_MS)
      return;
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// An undefined macro is zero to the preprocessor, so a spelling mistake or a
// missing include here would silently take the single-core branch on a
// dual-core chip - a bus task with no affinity, sharing a core with Wi-Fi, and
// nothing to show for it but the occasional missed answer under load.
#ifndef configNUMBER_OF_CORES
#error "configNUMBER_OF_CORES is not visible here; the core count must not be guessed"
#endif

/**
 * Starts the task that serves one bus.
 *
 * `arg` is handed to the task rather than looked up, so two doors on one board
 * do not end up both serving the first one's port.
 */
inline bool startBusTask(const char *name, void (*serve)(void *), void *arg, TaskHandle_t *out) {
  const BaseType_t created =
#if configNUMBER_OF_CORES > 1
      // Wi-Fi and lwIP live on core 0, so the answer path gets the other one to
      // itself.
      xTaskCreatePinnedToCore(serve, name, HCP_TASK_STACK, arg, HCP_TASK_PRIO, out, 1);
#else
      // Nothing to pin to, and asking for core 1 anyway is not a no-op: the
      // affinity check inside FreeRTOS asserts. The priority still holds the
      // answer above the network stack, see HCP_TASK_PRIO.
      xTaskCreate(serve, name, HCP_TASK_STACK, arg, HCP_TASK_PRIO, out);
#endif
  return created == pdPASS && *out != nullptr;
}

}  // namespace hcpbridge
}  // namespace esphome
