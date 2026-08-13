// The seam between the two tasks, with both of them actually running.
//
// Everything else about this component is tested on one thread: the simulator
// interleaves a poll and a press by hand, so it can only ever see the orders it
// was told to try. On a device the ESPHome loop runs on one core and the bus
// task on the other, and the interesting orders are the ones nobody thought of.
//
// What this establishes is not that the timing is right, which no test on a
// desk can say. It is that the handover has no state a reader and a writer can
// disagree about: a press is either taken or refused and never half of both,
// what goes out is always something somebody asked for, and nothing the loop
// reads is ever a value the bus task never wrote.
//
// Run under the thread sanitiser as well, which is where a plain member shared
// across tasks stops being an argument and becomes a report.
#include <atomic>
#include <cstdio>
#include <cstring>
#include <thread>

#include "hcp2_codec.h"

using namespace esphome::hcpbridge;

namespace {

int failures = 0;

void check(bool ok, const char *what) {
  printf("  %-58s %s\n", what, ok ? "ok" : "FAILED");
  if (!ok)
    failures++;
}

/** The drive telling everyone where the door is. This is what moves the state
 *  the entities read, so without it the loop side of the seam is never
 *  exercised and a plain member would go unnoticed. */
size_t broadcast(uint8_t stateHi, uint8_t position, uint8_t *out, size_t cap) {
  uint8_t body[7 + 18] = {0x00, 0x10, 0x9D, 0x31, 0x00, 0x09, 18};
  body[7 + 3] = position;
  body[7 + 4] = stateHi;
  memcpy(out, body, sizeof(body));
  return hcp2Finish(out, sizeof(body), cap);
}

size_t statusPoll(uint8_t counter, uint8_t *out, size_t cap) {
  const uint8_t body[] = {
      0x02, 0x17, 0x9C, 0xB9, 0x00, 0x08, 0x9C, 0x41, 0x00, 0x02, 0x04, counter, CMD_STATUS, 0x00, 0x00,
  };
  memcpy(out, body, sizeof(body));
  return hcp2Finish(out, sizeof(body), cap);
}

constexpr int ROUNDS = 20000;

}  // namespace

int main() {
  Hcp2Codec codec;
  std::atomic<bool> done{false};
  // One clock for both, as on the device. Two that run independently would put
  // every press in the past the moment the other side looked at it.
  std::atomic<uint32_t> now{1000};
  std::atomic<int> accepted{0};
  std::atomic<int> sent{0};
  std::atomic<int> impossible{0};
  volatile int sink = 0;

  // The bus task: answers polls, which is the only thing that empties the slot.
  std::thread bus([&] {
    uint8_t frame[64];
    uint8_t answer[64];
    uint8_t counter = 1;
    while (!done.load()) {
      codec.onFrame(now.load(), frame, statusPoll(counter, frame, sizeof(frame)), answer);
      // Alternating, so the door really moves and the state fields really get
      // written while the loop is reading them.
      const uint8_t hi = (counter & 1) ? 0x01 : 0x02;
      codec.onFrame(now.load(), frame, broadcast(hi, (uint8_t) (counter & 0x7F), frame, sizeof(frame)), answer);
      counter = (uint8_t) (counter + 1);
      if (counter == 0)
        counter = 1;
      HcpEventEntry e;
      while (codec.events.drain(&e, 1) == 1) {
        if (e.code == HcpEvent::COMMAND_SENT || e.code == HcpEvent::COMMAND_REPEATED) {
          sent.fetch_add(1);
          // Only ever something an entity asked for. A value from neither the
          // enum nor the set below would mean the slot was read while it was
          // being written.
          const HoermannCommand c = (HoermannCommand) e.a;
          if (c != HoermannCommand::OPEN && c != HoermannCommand::CLOSE && c != HoermannCommand::IMPULSE &&
              c != HoermannCommand::LIGHT_ON && c != HoermannCommand::LIGHT_OFF)
            impossible.fetch_add(1);
        }
      }
    }
  });

  // The loop task: presses, and reads the state the entities publish.
  std::thread loop([&] {
    for (int i = 0; i < ROUNDS; i++) {
      // Advanced slowly enough that a press has polls to be collected by, the
      // way it does on a drive that asks several times a second.
      const uint32_t t = now.load() + (uint32_t) ((i & 0x0F) == 0);
      now.store(t);
      if (codec.openDoor(t))
        accepted.fetch_add(1);
      if (codec.closeDoor(t))
        accepted.fetch_add(1);
      if ((i & 0x3F) == 0 && codec.stopDoor(t))
        accepted.fetch_add(1);
      // What an entity does on every callback. Landed in a volatile sink so
      // the reads actually happen: discarded, the compiler is free to drop
      // them, and then there is nothing for the sanitiser to look at.
      sink = (int) codec.state->currentPosition.load();
      sink = (int) codec.state->state.load();
      sink = (int) codec.state->link.load();
      sink = (int) codec.state->lightOn.load();
    }
    done.store(true);
  });

  loop.join();
  bus.join();

  check(impossible.load() == 0, "nothing went out that no entity had asked for");
  check(accepted.load() > 0, "presses were taken while the bus task was answering");
  check(sent.load() > 0, "and the bus task carried some of them out");
  // Every press is either refused on the spot or handed over exactly once, so
  // the drive can never be told to do more than somebody pressed.
  check(sent.load() <= accepted.load(), "never more went out than was accepted");
  printf("  (%d accepted, %d sent, over %d rounds)\n", accepted.load(), sent.load(), ROUNDS);

  printf("\n%s\n", failures == 0 ? "all two task checks passed" : "two task checks FAILED");
  return failures == 0 ? 0 : 1;
}
