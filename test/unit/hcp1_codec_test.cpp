// The older bus's exchange, played out on a desk.
//
// No drive here speaks this, so nothing below has been near a door. What it can
// establish is that the accessory answers what it must, stays quiet where it
// must, and - the part worth the most - never sends a command twice, on a bus
// that cannot tell it whether the first one arrived.
#include <cstdio>
#include <cstring>

#include "hcp1_codec.h"

using namespace esphome::hcpbridge;

namespace {

int failures = 0;

void check(bool ok, const char *what) {
  printf("  %-58s %s\n", what, ok ? "ok" : "FAILED");
  if (!ok)
    failures++;
}

uint8_t scratch[HCP1_MAX_FRAME];

size_t buildScan(uint8_t counter, uint8_t *out) {
  const uint8_t data[] = {HCP1_CMD_SCAN, HCP1_ADDR_MASTER};
  return hcp1Build(out, HCP1_MAX_FRAME, HCP1_ADDR_SELF, counter, data, 2);
}

size_t buildPoll(uint8_t counter, uint8_t *out) {
  const uint8_t data[] = {HCP1_CMD_STATUS_REQUEST};
  return hcp1Build(out, HCP1_MAX_FRAME, HCP1_ADDR_SELF, counter, data, 1);
}

size_t buildBroadcast(uint8_t counter, uint8_t d0, uint8_t *out) {
  const uint8_t data[] = {d0, 0x00};
  return hcp1Build(out, HCP1_MAX_FRAME, HCP1_ADDR_BROADCAST, counter, data, 2);
}

/** Drives one poll and hands back the command byte that went out. */
uint8_t poll(Hcp1Codec &c, uint32_t nowMs, uint8_t counter) {
  uint8_t frame[HCP1_MAX_FRAME];
  const size_t n = buildPoll(counter, frame);
  const size_t answered = c.onFrame(nowMs, frame, n, scratch);
  if (answered == 0)
    return 0xFF;  // silence, which is never a command
  Hcp1Frame f;
  if (!hcp1Parse(scratch, answered, &f) || f.length < 2)
    return 0xFF;
  return f.data[1];
}

void broadcast(Hcp1Codec &c, uint32_t nowMs, uint8_t counter, uint8_t d0) {
  uint8_t frame[HCP1_MAX_FRAME];
  const size_t n = buildBroadcast(counter, d0, frame);
  c.onFrame(nowMs, frame, n, scratch);
}

/** A codec the drive has already scanned, which is the ordinary state. */
void bringUp(Hcp1Codec &c, uint32_t nowMs = 1000) {
  uint8_t frame[HCP1_MAX_FRAME];
  const size_t n = buildScan(1, frame);
  c.onFrame(nowMs, frame, n, scratch);
}

}  // namespace

int main() {
  {
    Hcp1Codec c;
    uint8_t frame[HCP1_MAX_FRAME];
    const size_t n = buildScan(3, frame);
    const size_t answered = c.onFrame(1000, frame, n, scratch);
    Hcp1Frame f;
    check(answered > 0 && hcp1Parse(scratch, answered, &f), "a scan is answered");
    check(f.address == HCP1_ADDR_MASTER && f.counter == 4, "the answer goes to the drive, counting one past the scan");
    check(f.length == 2 && f.data[0] == HCP1_TYPE_UAP1 && f.data[1] == HCP1_ADDR_SELF,
          "and says what kind of accessory this is, and where");
  }

  {
    Hcp1Codec c;
    check(poll(c, 1000, 1) == 0xFF, "a poll before the drive has scanned us goes unanswered");
    bringUp(c);
    check(poll(c, 1200, 2) == 0, "after the scan, a poll gets an answer with no command");
  }

  {
    // What each press puts on the wire. The bytes are from the protocol
    // description, not read back out of the table under test - otherwise this
    // asserts that the code equals itself. Nothing watched this at all: asking
    // to vent could have sent the open bit, on the one bus here that has never
    // answered a real door.
    struct {
      HoermannCommand cmd;
      uint8_t expected;
      const char *what;
    } wanted[] = {
        {HoermannCommand::OPEN, 1 << 0, "open is bit 0"},
        {HoermannCommand::CLOSE, 1 << 1, "close is bit 1"},
        {HoermannCommand::IMPULSE, 1 << 2, "an impulse is bit 2"},
        {HoermannCommand::VENT, 1 << 4, "vent is bit 4, not one of the door's own"},
        {HoermannCommand::LIGHT_ON, 1 << 3, "the light is bit 3"},
        {HoermannCommand::LIGHT_OFF, 1 << 3, "and toggles, so both directions ask the same"},
    };
    for (const auto &w : wanted) {
      Hcp1Codec c;
      bringUp(c);
      check(c.submit(1200, w.cmd), "the press is accepted");
      check(poll(c, 1400, 2) == w.expected, w.what);
    }
  }

  {
    Hcp1Codec c;
    bringUp(c);
    check(c.submit(1200, HoermannCommand::OPEN), "an open is accepted once addressed");
    check(poll(c, 1300, 2) == HCP1_DO_OPEN, "and goes out on the next poll");
    check(poll(c, 1400, 3) == 0, "exactly once");
    check(poll(c, 1500, 4) == 0, "and not again after that");
  }

  {
    // The whole reason lostAnswerRepeats is zero here: nothing on this bus can
    // say whether the answer arrived, so sending again would be a second press
    // rather than a retry.
    Hcp1Codec c;
    bringUp(c);
    c.submit(1200, HoermannCommand::OPEN);
    uint8_t sent = 0;
    for (uint32_t t = 1300; t < 6000; t += 200)
      if (poll(c, t, (uint8_t) ((t / 200) & 0x0F)) == HCP1_DO_OPEN)
        sent++;
    check(sent == 1, "a drive that never reacts is still only asked once");
  }

  {
    Hcp1Codec c;
    bringUp(c);
    check(!c.submit(1200, HoermannCommand::HALF), "half open is refused, because nothing here says how to ask for it");
  }

  {
    Hcp1Codec c;
    check(!c.submit(1000, HoermannCommand::OPEN), "a press before the drive has scanned us is refused, not queued");
    bringUp(c, 1100);
    check(poll(c, 1200, 2) == 0, "so nothing fires once it does scan");
  }

  {
    Hcp1Codec c;
    bringUp(c);
    c.submit(1200, HoermannCommand::OPEN);
    // Nobody collected it; whoever pressed has walked away.
    check(poll(c, 1200 + CMD_STALE_MS + 500, 2) == 0,
          "a press the drive never collected goes stale instead of waiting");
  }

  {
    Hcp1Codec c;
    bringUp(c);
    c.submit(1200, HoermannCommand::OPEN);
    c.stopDoor(1250);
    check(poll(c, 1300, 2) == 0, "a stop cancels a press that has not gone out");
  }

  {
    Hcp1Codec c;
    bringUp(c);
    broadcast(c, 1100, 2, HCP1_BC_MOVING);  // opening
    c.stopDoor(1200);
    check(poll(c, 1300, 3) == HCP1_DO_IMPULSE, "a stop on a moving door goes out as an impulse");
  }

  {
    Hcp1Codec c;
    bringUp(c);
    broadcast(c, 1100, 2, HCP1_BC_CLOSED);
    check(c.state->state == HoermannState::CLOSED, "a shut door reads as shut");
    broadcast(c, 1200, 3, HCP1_BC_MOVING);
    check(c.state->state == HoermannState::OPENING, "moving without the direction bit is opening");
    broadcast(c, 1300, 4, (uint8_t) (HCP1_BC_MOVING | HCP1_BC_CLOSING));
    check(c.state->state == HoermannState::CLOSING, "and with it, closing");
    broadcast(c, 1400, 5, HCP1_BC_OPEN);
    check(c.state->state == HoermannState::OPEN, "an open door reads as open");
    broadcast(c, 1500, 6, HCP1_BC_VENTING);
    check(c.state->state == HoermannState::VENT, "and the venting position as venting");
    broadcast(c, 1600, 7, (uint8_t) (HCP1_BC_OPEN | HCP1_BC_LIGHT | HCP1_BC_ERROR));
    check(c.state->lightOn && c.state->actuatorError, "the light and a fault come through");
  }

  {
    Hcp1Codec c;
    bringUp(c);
    // A broadcast is addressed to everybody, so answering it would be two
    // accessories talking at once.
    uint8_t frame[HCP1_MAX_FRAME];
    const size_t n = buildBroadcast(2, HCP1_BC_CLOSED, frame);
    check(c.onFrame(1100, frame, n, scratch) == 0, "a broadcast is never answered");

    // Somebody else's accessory.
    const uint8_t data[] = {HCP1_CMD_STATUS_REQUEST};
    const size_t m = hcp1Build(frame, sizeof(frame), 0x29, 3, data, 1);
    check(c.onFrame(1200, frame, m, scratch) == 0, "nor is a frame for another address");

    // A frame whose checksum does not match cannot be trusted to say who it is
    // for, let alone what it wants.
    const size_t k = buildPoll(4, frame);
    frame[k - 1] ^= 0x01;
    check(c.onFrame(1300, frame, k, scratch) == 0, "nor one whose checksum is wrong");
  }

  {
    // Press stop while the door is moving, then have it arrive before the drive
    // collects the answer. Resolved when it was pressed, the impulse would go
    // out at a standing door and start it again.
    Hcp1Codec c;
    bringUp(c);
    broadcast(c, 1100, 2, HCP1_BC_MOVING);
    c.stopDoor(1150);
    broadcast(c, 1200, 3, HCP1_BC_OPEN);  // it got there on its own
    check(poll(c, 1250, 4) == 0, "a stop does not start a door that arrived while it was queued");
  }

  {
    // The distinction the whole link state exists for: a bus nothing has ever
    // arrived on wants the wiring checked, one that fell quiet wants the drive
    // power cycled. A single flag says "off" to both.
    Hcp1Codec c;
    check(c.state->link == HcpLink::NEVER_SEEN, "before anything, the link has never been seen");
    bringUp(c, 1000);
    check(c.state->link == HcpLink::ENUMERATING, "a frame arriving is not the same as being registered");
    poll(c, 1100, 2);
    check(c.state->link == HcpLink::REGISTERED, "being polled by address is registration");
    c.checkBusSilence(1100 + BUS_SILENCE_MS + 1000);
    check(c.state->link == HcpLink::SILENT, "and going quiet is told apart from never having spoken");
  }

  printf(failures ? "\nFAILED\n" : "\nall HCP1 exchange checks passed\n");
  return failures ? 1 : 0;
}
