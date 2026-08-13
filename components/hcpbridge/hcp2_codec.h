// Credits to https://github.com/Gifford47/HCPBridgeMqtt for the initial code base

#pragma once

#include <atomic>
#include <cstdint>

#include "hcp2_frame.h"
#include "hcp_capabilities.h"
#include "hcp_state.h"
#include "hcp_events.h"

namespace esphome {
namespace hcpbridge {

// The wire, and what answers on it. No port and no task live here: this reads
// frames and writes answers, which is what lets the whole exchange be played
// out on a desk. What owns the port is hcp2_transport.
//

// What this bus can do, read by the command lifecycle instead of being written
// into it. HCP2 echoes a counter, so a lost answer is visible and re-sending is
// a retry rather than a second press.
static constexpr HcpCapabilities HCP2_CAPABILITIES = {
    /* lostAnswerRepeats */ 3,
    /* unactedRepeats */ 1,
    /* hasIdentity */ true,
    /* hasPause */ true,
    /* hasPosition */ true,
};

// Hoermann bus register blocks: the drive writes commands to 0x9C41 and its
// state to 0x9D31, and reads our answer from 0x9CB9.
static constexpr uint16_t REG_CMD_BASE = 0x9C41;
// Nine, not three: the drive answers an identity request by writing its payload
// into this same block.
static constexpr uint32_t REG_CMD_COUNT = 9;
static constexpr uint16_t REG_BCAST_BASE = 0x9D31;
static constexpr uint32_t REG_BCAST_COUNT = 9;
static constexpr uint16_t REG_RESP_BASE = 0x9CB9;
static constexpr uint32_t REG_RESP_COUNT = 8;

// Identity exchange: which value rides along with a poll, what the drive sends
// back, and how long its answer is.
static constexpr uint16_t IDENT_REQ_SERIAL = 0x05;
static constexpr uint16_t IDENT_REQ_FIRMWARE = 0x06;
static constexpr uint16_t IDENT_SUB_SERIAL = 0x0C;
static constexpr uint16_t IDENT_SUB_FIRMWARE = 0x0D;
// A payload that outgrew the block would read past it instead of failing here.
static_assert(2 + SERIAL_FIRST_REGS <= REG_CMD_COUNT, "serial payload exceeds the command block");
static_assert(2 + SERIAL_SECOND_REGS <= REG_CMD_COUNT, "serial payload exceeds the command block");
static_assert(2 + IDENT_FIRMWARE_LEN / 2 <= REG_CMD_COUNT, "firmware payload exceeds the command block");
static constexpr uint32_t IDENT_RETRY_MS = 30000;
static constexpr uint32_t IDENT_MAX_ATTEMPTS = 3;
// Pausing: the drive is told on the next poll that we are about to go quiet,
// and confirms with its own sub code naming the address it is pausing.
static constexpr uint16_t IDENT_SUB_PAUSE_ACK = 0x19;
// A frame shape we cannot answer properly repeats as fast as the drive polls,
// so the same one is only reported this often. A different shape is immediate.
static constexpr uint32_t UNKNOWN_SHAPE_REPEAT_MS = 10000;

// What the drive is asking for, in the low byte of the first written register.
static constexpr uint16_t CMD_CONNECT = 0x02;
static constexpr uint16_t CMD_STATUS = 0x03;
static constexpr uint16_t CMD_TRANSFER = 0x04;

// Answer codes we put in the low byte of the second answer register.
static constexpr uint16_t RESP_STATUS = 0x01;
static constexpr uint16_t RESP_REQUEST = 0x22;
static constexpr uint16_t RESP_PAUSE = 0x29;
static constexpr uint16_t RESP_ACK = 0xFD;
static constexpr uint16_t RESP_NAK = 0xFE;

class Hcp2Codec {
 public:
  // A member the pointer points at, rather than an allocation: the state
  // lives exactly as long as the engine does, and nothing has to free it.
  HoermannState stateStorage;
  HoermannState *state = &this->stateStorage;

  /**
     * Answers one frame as it came off the wire, checksum and all.
     *
     * Returns the length of the answer, checksum included, or 0 to stay
     * silent - which is what a broken checksum gets, along with everything
     * else this bus has no answer for.
     */
  size_t onFrame(uint32_t nowMs, const uint8_t *req, size_t len, uint8_t *resp);

  void setCommandValuesToRead(uint32_t nowMs);
  void onDoorPositonChanged(uint16_t oldVal, uint16_t val);
  void onCurrentStateChanged(uint16_t oldVal, uint16_t val);
  void onRegSevenChanged(uint16_t oldVal, uint16_t val);

  // First register of 0x9C41: high byte counter, low byte command.
  void onCounterWrite(uint16_t val);
  void checkGotoTarget(uint32_t nowMs);
  bool gotoCheckPending = false;

  // After the whole write has landed, so a payload spread over several
  // registers can be read as one.
  void onWriteBlockComplete(uint32_t nowMs, uint16_t addr, uint16_t count);

  /** Ask the drive for its serial number, then its firmware version. */
  void requestDriveIdentity(uint32_t nowMs);

  /** Tell the drive on its next poll that this accessory is going quiet. */
  void requestPause() {
    this->pauseRequested.store(true);
    this->pauseConfirmed.store(false);
  }
  void endPause() { this->pauseRequested.store(false); }
  bool pauseAcknowledged() const { return this->pauseConfirmed.load(); }
  /** When a frame last arrived, or 0 if none ever has. */
  uint32_t lastFrameAt() const { return this->lastFrameOn.load(); }
  /** Drops a press nobody is standing there for any more. */
  void dropQueuedCommand() { this->nextCommand.store(HoermannCommand::NONE); }

  /** Drops the connected state once the drive has gone quiet for too long. */
  void checkBusSilence(uint32_t nowMs);

  /** Main task only, so the string the sensors read has one writer. */
  void publishIdentity();

  // Thirty-two is a few seconds of a talkative bus, which is what it takes to
  // still have the beginning of an episode once somebody notices the end.
  HcpEventRing<32> events;

  // Read wherever a limit is applied, so a second protocol brings different
  // numbers rather than different code.
  HcpCapabilities caps = HCP2_CAPABILITIES;

  // One map for all three blocks, so callbacks fire whichever code wrote.
  /**
   * The two blocks the drive writes, addressed by index rather than by address.
   * onFrame refuses any frame whose blocks are not exactly these three, so an
   * address map would be a general answer to a question with three fixed ones.
   */
  void writeCmd(uint8_t i, uint16_t val);
  void writeBcast(uint8_t i, uint16_t val);
  void onRequestHook(uint32_t nowMs, uint8_t fc, uint16_t c1, uint16_t c2, uint8_t command = 0);
  void reportUnknownShape(uint32_t nowMs, uint8_t fc, uint16_t c1, uint16_t c2);

  /** The counter is a delivery receipt: the drive holds its value until it
     *  has been answered, so one that fails to advance means ours was lost. */
  void syncCounter(uint8_t counterByte, uint8_t command);
  void advanceCounter();
  void rearmLostCommand();

  uint8_t txCounter = 0;      // goes into the answer
  uint8_t txCounterPrev = 0;  // the one to fall back on when an answer is lost
  bool txCounterValid = false;
  // What the last answer carried, so it can be put back if that answer was
  // lost. Mirrors the snapshot the count is checked against.
  HoermannCommand lastSentCommand = HoermannCommand::NONE;

  // Last frame shape we had no branch for, so a repeat can be told from a new
  // one. Only ever touched from the bus task.
  bool unknownSeen = false;
  uint8_t unknownFc = 0;
  uint16_t unknownC1 = 0, unknownC2 = 0;
  uint32_t unknownLoggedOn = 0;

  /** False when the slot was occupied and the command was dropped. */
  bool setCommand(uint32_t nowMs, bool cond, HoermannCommand command);

  bool stopDoor(uint32_t nowMs);
  bool closeDoor(uint32_t nowMs);
  bool openDoor(uint32_t nowMs);
  bool impulseDoor(uint32_t nowMs);
  bool halfPositionDoor(uint32_t nowMs);
  bool ventilationPositionDoor(uint32_t nowMs);
  bool turnLight(uint32_t nowMs, bool on);
  bool setPosition(uint32_t nowMs, int setPosition);

 private:
  uint16_t regCmd[REG_CMD_COUNT] = {0};      // 0x9C41, written by the drive
  uint16_t regBcast[REG_BCAST_COUNT] = {0};  // 0x9D31, drive state
  uint16_t regResp[REG_RESP_COUNT] = {0};    // 0x9CB9, what we answer with
  std::atomic<HoermannCommand> nextCommand{HoermannCommand::NONE};
  std::atomic<uint32_t> nextCommandOn{0};

  // Where a partway run is meant to stop, on its way from the loop task to the
  // bus task. Handed over rather than written into the shared state directly:
  // two tasks writing that one float meant a target could be lost, leaving the
  // door to run to the end stop, or arrive after its own run had finished and
  // halt the next one somewhere nobody asked for. Percent, or -1 for none.
  std::atomic<int> gotoRequest{-1};
  // Set by a stop on the main task, honoured by the bus task.
  std::atomic<bool> cancelWatch{false};  // shared with the bus task

  // Bus task only. What was sent and how the door looked then, so a missing
  // effect can be told apart from a delivered command.
  HoermannCommand awaitedCommand = HoermannCommand::NONE;
  HoermannCommand repeatCommand = HoermannCommand::NONE;
  uint32_t awaitedSince = 0;  // first time it went out; the deadline runs from here
  uint32_t lastSentAt = 0;    // most recent time; the confirmation runs from here
  uint8_t confirmRepeats = 0;
  // The drive's own state word, not our translation of it: a code we do not
  // translate must still count as the drive having reacted.
  uint8_t repeatsSent = 0;   // re-sends of the current command after a loss
  uint32_t stateWrites = 0;  // every change of the drive's state word
  uint32_t stateWritesWhenSent = 0;

  bool commandTookEffect() const;
  bool commandReached(HoermannCommand cmd) const;
  bool repeatStillMakesSense() const;

  /**
   * Empties the slot, but only if it still holds what this poll took out of it.
   *
   * A plain store would throw away a stop that the loop task put there while
   * this frame was being answered, after the caller had already been told it
   * was accepted. Losing a stop is the one loss somebody notices at the door.
   */
  void clearIfStill(HoermannCommand taken) {
    HoermannCommand expected = taken;
    this->nextCommand.compare_exchange_strong(expected, HoermannCommand::NONE);
  }
  void checkCommandEffect(uint32_t nowMs);

  // The drive only answers a request that rode along with a poll, and takes
  // it back out of the slot, so an unanswered one has to be repeated.
  uint8_t identityWanted = 0;  // 0 = nothing, else IDENT_REQ_*
  uint8_t identityAttempts = 0;
  uint32_t identityAskedOn = 0;
  // A separate flag, not identityAskedOn == 0: millis() really is 0 for the
  // first millisecond and wraps back through it every 49.7 days.
  bool identityAsked = false;
  bool identityStarted = false;

  // Handed from the bus task to the main task: buffer first, flag second.
  char identSerial[IDENT_SERIAL_LEN + 1] = {0};
  char identFirmware[IDENT_FIRMWARE_LEN + 1] = {0};
  std::atomic<bool> identSerialReady{false};
  std::atomic<bool> identFirmwareReady{false};
  uint8_t serialBuf[IDENT_SERIAL_LEN] = {0};
  bool serialFirstHalfSeen = false;

  // Set from the main task, read and answered by the bus task.
  std::atomic<bool> pauseRequested{false};
  std::atomic<bool> pauseConfirmed{false};
  // Written by the bus task on every frame, read by the main task.
  std::atomic<uint32_t> lastFrameOn{0};

  void copyRegsToBytes(uint8_t firstReg, uint8_t regCount, uint8_t *out);
  bool onIdentityData(uint32_t nowMs, uint8_t counterByte, uint8_t subCode, uint16_t count);
  void answerTransfer(uint8_t counterByte, uint8_t code);
  void armIdentityRequest(uint32_t nowMs, uint8_t request);
};

}  // namespace hcpbridge
}  // namespace esphome
