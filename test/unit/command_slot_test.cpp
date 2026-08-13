// The one command slot, and the clock that empties it.
//
// One press, one command, dropped if nobody collects it within CMD_STALE_MS.
// That expiry is the only thing between a press nobody could deliver and a door
// that moves minutes later, so the ways it can be defeated are worth naming.
//
// The two buses answer a second press differently and both are deliberate. The
// newer one defends the slot and refuses, because a counter echo lets it tell a
// delivered command from a lost one. The older one replaces, because it cannot,
// and the last thing somebody pressed is the safer guess.
//
// Watched the way the drive would: presses go in through the same calls an
// entity makes, and what came of them is read out of the event band. Nothing
// here reaches past an access specifier, so nothing here can pass by
// rearranging what it is testing.
//
// None of it is visible to the simulator, which never presses twice before the
// drive polls.
#include <cstdio>
#include <cstring>

#include "hcp1_codec.h"
#include "hcp2_codec.h"

using namespace esphome::hcpbridge;

namespace {

int failures = 0;

void check(bool ok, const char *what) {
  printf("  %-58s %s\n", what, ok ? "ok" : "FAILED");
  if (!ok)
    failures++;
}

uint8_t frame[64];
uint8_t answer[64];

/** One status poll, the frame the drive sends several times a second. */
size_t statusPoll(uint8_t counter, uint8_t *out) {
  const uint8_t body[] = {
      0x02, 0x17, 0x9C, 0xB9, 0x00, 0x08, 0x9C, 0x41, 0x00, 0x02, 0x04, counter, CMD_STATUS, 0x00, 0x00,
  };
  memcpy(out, body, sizeof(body));
  return hcp2Finish(out, sizeof(body), sizeof(frame));
}

struct Outcome {
  bool sent{false};
  bool expired{false};
};

/** Empties the band, so what follows reports only what it caused. */
void forgetEvents(Hcp2Codec &c) {
  HcpEventEntry e;
  while (c.events.drain(&e, 1) == 1) {
  }
  c.events.takeDropped();
}

/** Polls once and reports what became of anything waiting. */
Outcome poll(Hcp2Codec &c, uint32_t nowMs, uint8_t counter) {
  c.onFrame(nowMs, frame, statusPoll(counter, frame), answer);
  Outcome o;
  HcpEventEntry e;
  while (c.events.drain(&e, 1) == 1) {
    if (e.code == HcpEvent::COMMAND_SENT)
      o.sent = true;
    if (e.code == HcpEvent::COMMAND_STALE)
      o.expired = true;
  }
  return o;
}

}  // namespace

int main() {
  {
    Hcp2Codec c;
    poll(c, 100, 1);
    check(c.openDoor(1000), "a press is taken once the drive is answering");
    check(poll(c, 1100, 2).sent, "and the next poll carries it out");
  }

  {
    // The defect this file exists for. Somebody taps again because nothing has
    // happened yet. Each further tap is refused, and must not buy the first one
    // more time; left unchecked, the first press outlives any expiry and the
    // door runs whenever the drive next gets round to us.
    Hcp2Codec c;
    poll(c, 100, 1);
    c.openDoor(1000);
    bool everyTapRefused = true;
    for (uint32_t t = 1100; t < 60000; t += 100)
      everyTapRefused = everyTapRefused && !c.closeDoor(t);
    check(everyTapRefused, "further taps are refused while one is still waiting");
    // The refusals alone overrun a 32 slot band, and it keeps the oldest, so
    // without this the verdict below would have been dropped rather than false.
    forgetEvents(c);
    const Outcome o = poll(c, 60100, 2);
    check(!o.sent, "a press nobody could collect does not go out an hour later");
    check(o.expired, "it is dropped as expired instead");
  }

  {
    // A stop overwrites the slot rather than competing for it, so its stamp has
    // to arrive with it. Written the other way round the bus task can pair the
    // stop with the stamp that was there before - zero on a fresh boot, which
    // reads as long expired.
    Hcp2Codec c;
    poll(c, 100, 1);
    check(c.stopDoor(1000), "a stop is taken");
    check(!poll(c, 1100, 2).expired, "and is not mistaken for one that had been waiting");
  }

  {
    // Going quiet for an update is not a link. A press taken here would sit out
    // the restart window and go out the moment the pause ends, which is a door
    // starting while the flash is being written.
    Hcp2Codec c;
    poll(c, 100, 1);
    c.requestPause();
    check(!c.openDoor(1000), "a press during an announced pause is refused");
    check(!c.stopDoor(1000), "and so is a stop");
    c.endPause();
    check(!poll(c, 1100, 2).sent, "so nothing is waiting when the pause ends");
  }

  {
    Hcp1Codec c;
    // Nothing may be sent before the drive has handed out an address, so the
    // older bus refuses rather than reporting a stop it cannot deliver.
    check(!c.stopDoor(1000), "the older bus refuses a stop with no address");
  }

  printf("\n%s\n", failures == 0 ? "all command slot checks passed" : "command slot checks FAILED");
  return failures == 0 ? 0 : 1;
}
