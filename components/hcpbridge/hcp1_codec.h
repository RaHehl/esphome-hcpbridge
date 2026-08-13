// What to answer on the older bus, and what its broadcasts mean.
//
// Pure in the same sense as the frame layer below it: bytes and a time in,
// bytes and a state out. It holds no port, reads no clock and prints nothing,
// so the whole exchange can be played out on a desk.
//
// The shape of a command's life is the same as on the newer bus - one press,
// one command, dropped if nobody collects it - but one difference is a safety
// question rather than a preference. The newer bus echoes a counter, so a lost
// answer is visible and sending again is a retry. Here there is no such echo:
// an answer that vanished and one that arrived look identical, and sending
// again would be a second press at somebody's door. Hence lostAnswerRepeats
// being zero in the capabilities, and hence no retry anywhere in this file.
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "hcp1_frame.h"
#include "hcp_capabilities.h"
#include "hcp_events.h"
#include "hcp_state.h"

namespace esphome {
namespace hcpbridge {

// No delivery receipt, so no re-sending; no identity exchange, no pause
// handshake, and a door that is either shut, open or somewhere between with
// nothing on the wire to say where.
static constexpr HcpCapabilities HCP1_CAPABILITIES = {
    /* lostAnswerRepeats */ 0,
    // Also zero, and for a weaker reason than the one above: this bus does
    // carry the state word a repeat could be judged against, so asking again
    // once would not be unsafe the way re-sending a lost command is. It is not
    // done because nothing here has been near a drive, and a first version that
    // sends one command per press is the one whose behaviour can be predicted
    // from reading it. Worth revisiting once one of these has answered a door.
    /* unactedRepeats */ 0,
    /* hasIdentity */ false,
    /* hasPause */ false,
    /* hasPosition */ false,
};

class Hcp1Codec {
 public:
  /**
   * Answers one frame. Returns the length written, or 0 to stay quiet.
   *
   * Quiet is the normal answer to most of what crosses this bus: broadcasts go
   * to everyone and are never answered, and frames addressed to another
   * accessory are none of our business.
   */
  size_t onFrame(uint32_t nowMs, const uint8_t *buf, size_t len, uint8_t *out);

  /** Queues a press. False when there is nowhere to put it. */
  bool submit(uint32_t nowMs, HoermannCommand cmd);

  // The same names the newer bus's codec offers, so an entity calls one thing
  // and the choice of protocol is made once, when the image is built.
  bool openDoor(uint32_t nowMs) { return this->submit(nowMs, HoermannCommand::OPEN); }
  bool closeDoor(uint32_t nowMs) { return this->submit(nowMs, HoermannCommand::CLOSE); }
  bool impulseDoor(uint32_t nowMs) { return this->submit(nowMs, HoermannCommand::IMPULSE); }
  bool halfPositionDoor(uint32_t nowMs) { return this->submit(nowMs, HoermannCommand::HALF); }
  bool ventilationPositionDoor(uint32_t nowMs) { return this->submit(nowMs, HoermannCommand::VENT); }
  bool turnLight(uint32_t nowMs, bool on);
  /** Only the ends: nothing on this bus carries a position to stop at. */
  bool setPosition(uint32_t nowMs, int percent);
  /** Drops whatever is queued, and asks the door to stop if it is moving. */
  bool stopDoor(uint32_t nowMs);

  /** When a frame last arrived, so a restart can wait for a gap. */
  uint32_t lastFrameAt() const { return this->lastFrameAt_.load(); }

  /** Drops the link once the drive has gone quiet for too long. */
  void checkBusSilence(uint32_t nowMs);
  /** Nothing to publish: this bus has no serial number or firmware version. */
  void publishIdentity() {}

  HoermannState stateStorage;
  HoermannState *state = &this->stateStorage;
  HcpEventRing<32> events;
  HcpCapabilities caps = HCP1_CAPABILITIES;

 protected:
  void applyBroadcast(uint32_t nowMs, const uint8_t *data, uint8_t len);
  uint8_t commandByte(HoermannCommand cmd) const;
  size_t answerScan(uint8_t counter, uint8_t *out);
  size_t answerStatus(uint32_t nowMs, uint8_t counter, uint8_t *out);

  // Atomic for the same reason as the newer codec's slot: the loop task fills
  // these and the bus task empties them, and a plain member lets the compiler
  // pair a fresh press with the previous timestamp and drop it as stale. A
  // dropped press is a door that did not move; a dropped stop is worse.
  //
  // The drive assigns an address before an accessory may speak. Until it has,
  // anything we sent would be talking over somebody.
  std::atomic<bool> addressed_{false};

  // One slot, and what it was queued at. There is no second one: a press that
  // has not gone out yet is replaced rather than queued behind, because the
  // last thing somebody pressed is what they want.
  std::atomic<HoermannCommand> pending_{HoermannCommand::NONE};
  std::atomic<uint32_t> pendingAt_{0};

  // Bus task only: what went out last, purely so a repeat can be recognised.
  HoermannCommand lastSent_{HoermannCommand::NONE};
  uint32_t lastSentAt_{0};

  // When a frame last arrived, so a link that has gone away can be noticed.
  // Written by the bus task, read by the loop in checkBusSilence.
  std::atomic<uint32_t> lastFrameAt_{0};
};

}  // namespace hcpbridge
}  // namespace esphome
