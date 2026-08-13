// Credits to https://github.com/Gifford47/HCPBridgeMqtt for the initial code base

#include <cstring>

#include "hcp2_codec.h"

namespace esphome {
namespace hcpbridge {

// Under the component name, so `logger: logs: hcpbridge.bus:` can reach it.

// The drive's command block, by index. Only the first register has anything
// hanging off it; the rest is payload the branches read straight out of the
// frame. The hook runs before the store, so it still sees the old value.
void Hcp2Codec::writeCmd(uint8_t i, uint16_t val) {
  if (i == 0)
    this->onCounterWrite(val);
  this->regCmd[i] = val;
}

// The drive's state block, by index. Three registers are watched for a change,
// which is why the old value is read before the new one lands.
void Hcp2Codec::writeBcast(uint8_t i, uint16_t val) {
  const uint16_t old = this->regBcast[i];
  if (i == 1) {
    this->onDoorPositonChanged(old, val);
  } else if (i == 2) {
    if (old != val)
      this->stateWrites++;
    this->onCurrentStateChanged(old, val);
  } else if (i == 6) {
    this->onRegSevenChanged(old, val);
  }
  this->regBcast[i] = val;
}

/**
 * A frame we have no branch for. It is answered with a plain status and an
 * empty payload, so the shape is the only thing worth knowing about it: it is
 * what a missing branch would have to be written against. Repeats are
 * throttled, but a shape not seen before is always reported, because the
 * interesting ones arrive in bursts while the drive takes an accessory back
 * onto the bus.
 */
void Hcp2Codec::reportUnknownShape(uint32_t nowMs, uint8_t fc, uint16_t c1, uint16_t c2) {
  const uint32_t now = nowMs;
  const bool sameShape = this->unknownSeen && this->unknownFc == fc && this->unknownC1 == c1 && this->unknownC2 == c2;
  // Subtract, never add: adding overflows when millis() wraps.
  if (sameShape && (now - this->unknownLoggedOn) < UNKNOWN_SHAPE_REPEAT_MS)
    return;
  this->unknownSeen = true;
  this->unknownFc = fc;
  this->unknownC1 = c1;
  this->unknownC2 = c2;
  this->unknownLoggedOn = now;
  this->events.push(HcpEvent::UNANSWERABLE_FRAME, c1, c2, nowMs);
}

/**
 * The former onRequest callback. The library ran it for every function code it
 * knew, before any validation.
 */
// Which blocks a frame touches is settled in onFrame, which answers nothing
// whose blocks are not exactly the two below. What is left to tell apart is how
// many registers were asked for and which command byte came with them.
void Hcp2Codec::onRequestHook(uint32_t nowMs, uint8_t fc, uint16_t c1, uint16_t c2, uint8_t command) {
  // The command byte decides, not the register counts alone: a transfer written
  // as two registers used to land here and take a queued press with it, and the
  // answer that would have carried it is overwritten later anyway.
  if (fc == 0x17 && c2 == 0x02 && c1 == 0x08 && command == CMD_STATUS) {
    this->regResp[0] = 0x0000;
    this->regResp[1] = RESP_STATUS;
    this->regResp[4] = 0x0000;
    this->regResp[5] = 0x0000;
    this->regResp[6] = 0x0000;
    this->regResp[7] = 0x0000;

    // Checked before the command slot is read, so a press that is still waiting
    // stays waiting instead of being consumed by a frame that does not carry it.
    if (this->pauseRequested.load()) {
      this->regResp[1] = RESP_PAUSE;
      this->regResp[2] = SLAVE_ID;
      this->regResp[3] = 0x0000;
      // This task owns them. A repeat inside the settle window would start the
      // door a second before we disappear.
      this->awaitedCommand = HoermannCommand::NONE;
      this->repeatCommand = HoermannCommand::NONE;
      this->state->setValid(true);
      this->state->setLink(HcpLink::PAUSED);
      return;
    }
    setCommandValuesToRead(nowMs);

    // The first answered poll means the drive is talking to us, so this is the
    // point to find out who it is.
    if (!this->identityStarted && this->caps.hasIdentity) {
      this->identityStarted = true;
      this->requestDriveIdentity(nowMs);
    }
    // A request travels in the same registers a command would, so it has to
    // wait until nothing is being pressed.
    if (this->identityWanted != 0 && this->regResp[2] == 0x0000 && this->regResp[3] == 0x0000) {
      const uint32_t now = nowMs;
      const bool firstTry = !this->identityAsked;
      // Subtract, never add: adding overflows when millis() wraps.
      if (firstTry || (now - this->identityAskedOn) > IDENT_RETRY_MS) {
        if (this->identityAttempts >= IDENT_MAX_ATTEMPTS) {
          this->events.push(HcpEvent::IDENTITY_UNANSWERED, this->identityWanted, 0, nowMs);
          this->identityWanted = 0;
        } else {
          this->identityAttempts++;
          this->identityAskedOn = now;
          this->identityAsked = true;
          this->regResp[1] = RESP_REQUEST;
          this->regResp[2] = (uint16_t) (this->identityWanted << 8);
          this->regResp[3] = 0x0000;
        }
      }
    }
  } else if (fc == 0x17 && c2 == 0x02 && c1 == 0x02) {
    this->regResp[0] = 0x0004;
    this->regResp[1] = 0x0000;
    this->events.push(HcpEvent::EMPTY_COMMAND, 0, 0, nowMs);
  } else if (fc == 0x17 && c2 == 0x03 && c1 == 0x05) {
    this->events.push(HcpEvent::BUSSCAN, 0, 0, nowMs);
    this->regResp[0] = 0x0000;
    this->regResp[1] = 0x0005;
    this->regResp[2] = 0x0430;
    this->regResp[3] = 0x10ff;
    this->regResp[4] = 0xa845;
  } else if (fc == 0x17 && c2 >= 0x03) {
    // The whole block: it is read back in this same frame, so a stale command
    // would go out as a key press.
    for (uint8_t i = 0; i < REG_RESP_COUNT; i++)
      this->regResp[i] = 0x0000;
    // One we understand is answered later and overwrites this. This is one we
    // do not, and zero is not a defined answer code.
    this->regResp[1] = RESP_STATUS;
  } else if (fc == 0x10) {
  } else {
    // Read back in the same frame, so a leftover command or signature would go
    // out a second time.
    for (uint8_t i = 0; i < REG_RESP_COUNT; i++)
      this->regResp[i] = 0x0000;
    // Zero is not a defined answer code. Plain status with an empty payload is
    // what this is.
    this->regResp[1] = RESP_STATUS;
    this->reportUnknownShape(nowMs, fc, c1, c2);
  }
  this->state->setValid(true);
  // A poll answered is what registration means on this bus; there is no
  // separate telegram for it. Not while going quiet, where saying "registered"
  // would undo what was just announced.
  if (this->state->link != HcpLink::PAUSED)
    this->state->setLink(HcpLink::REGISTERED);
}

static inline uint16_t rd16(const uint8_t *p) { return (uint16_t) (p[0] << 8) | p[1]; }
static inline void wr16(uint8_t *p, uint16_t v) {
  p[0] = (uint8_t) (v >> 8);
  p[1] = (uint8_t) (v & 0xFF);
}

/**
 * Answers one complete frame (CRC already stripped).
 *
 * Order follows the library: fill the response registers first, then apply the
 * drive's writes (which can still change 0x9CB9), then answer.
 */
size_t Hcp2Codec::onFrame(uint32_t nowMs, const uint8_t *raw, size_t rawLen, uint8_t *resp) {
  // A frame whose checksum does not match cannot be trusted to say who it is
  // addressed to, let alone what it wants.
  const size_t len = hcp2Validate(raw, rawLen);
  if (len == 0)
    return 0;
  const uint8_t *const req = raw;

  // Anything addressed to us is a sign of life. The clock can read 0 for the
  // first millisecond, which would look like "never".
  this->lastFrameOn.store(nowMs == 0 ? 1 : nowMs);
  if (this->state->link == HcpLink::NEVER_SEEN)
    this->state->setLink(HcpLink::ENUMERATING);

  const uint8_t fc = req[1];
  // A read/write frame to the broadcast address is never answered, so running
  // it would consume a queued press for nothing.
  if (req[0] == 0x00 && fc == 0x17)
    return 0;

  if (fc == 0x17) {
    if (len < 11)
      return 0;
    const uint16_t readAddr = rd16(req + 2);
    const uint16_t readCnt = rd16(req + 4);
    const uint16_t writeAddr = rd16(req + 6);
    const uint16_t writeCnt = rd16(req + 8);
    const uint8_t byteCnt = req[10];
    const uint8_t *wdata = req + 11;

    // All of it before anything is stored or answered. A frame that fails gets
    // no answer at all, there being no code for "your frame was wrong"; and
    // answering first let a malformed frame swallow a queued key press.
    if (readAddr != REG_RESP_BASE || writeAddr != REG_CMD_BASE) {
      // A wrong address shifts the payload by a register: counter read as
      // command, command as position.
      this->events.push(HcpEvent::WRONG_REGISTER_BLOCK, readAddr, writeAddr, nowMs);
      return 0;
    }
    if (readCnt < 1 || readCnt > REG_RESP_COUNT || writeCnt < 1 || (0xFFFF - readAddr) < readCnt ||
        byteCnt != 2 * writeCnt)
      return 0;
    // One to nine registers; anything longer has no room in the block.
    if (byteCnt < 2 || byteCnt > 2 * REG_CMD_COUNT)
      return 0;
    if (len < (size_t) (11 + byteCnt))
      return 0;
    // Before the hook that reads the queue, because a lost answer puts a
    // command back into it. High byte counter, low byte command.
    this->syncCounter(wdata[0], wdata[1]);
    // Only the status branch refills it. Left standing, a lost busscan answer
    // re-sent a door command that had already been delivered.
    this->lastSentCommand = HoermannCommand::NONE;

    this->onRequestHook(nowMs, fc, readCnt, writeCnt, wdata[1]);

    // Bounded above: byteCnt is 2 * writeCnt and no larger than the block.
    for (uint16_t i = 0; i < writeCnt; i++)
      this->writeCmd((uint8_t) i, rd16(wdata + 2 * i));
    this->onWriteBlockComplete(nowMs, writeAddr, writeCnt);

    size_t n = 0;
    resp[n++] = req[0];
    resp[n++] = fc;
    resp[n++] = (uint8_t) (readCnt * 2);
    for (uint16_t i = 0; i < readCnt; i++) {
      wr16(resp + n, this->regResp[i]);
      n += 2;
    }
    // Only for an answer that is going out. The byte we send stays the drive's
    // own, mirrored; the count exists to notice a loss, not to be sent.
    this->advanceCounter();
    return hcp2Finish(resp, n, HCP2_MAX_FRAME);
  }

  if (fc == 0x10) {
    if (len < 7)
      return 0;
    const uint16_t addr = rd16(req + 2);
    const uint16_t cnt = rd16(req + 4);
    const uint8_t byteCnt = req[6];
    const uint8_t *wdata = req + 7;

    // As above. A wrong address would land the door state where the position
    // is read from.
    if (addr != REG_BCAST_BASE) {
      this->events.push(HcpEvent::WRONG_BROADCAST_BLOCK, addr, 0, nowMs);
      return 0;
    }
    if (cnt < 1 || byteCnt != 2 * cnt || byteCnt < 2 || byteCnt > 2 * REG_BCAST_COUNT)
      return 0;
    if (len < (size_t) (7 + byteCnt))
      return 0;

    this->onRequestHook(nowMs, fc, cnt, 0, 0);

    for (uint16_t i = 0; i < cnt; i++)
      this->writeBcast((uint8_t) i, rd16(wdata + 2 * i));
    this->checkGotoTarget(nowMs);

    // Built for completeness; the frame layer never sends an answer to a
    // broadcast.
    size_t n = 0;
    resp[n++] = req[0];
    resp[n++] = fc;
    wr16(resp + n, addr);
    n += 2;
    wr16(resp + n, cnt);
    n += 2;
    return hcp2Finish(resp, n, HCP2_MAX_FRAME);
  }

  // Two function codes exist here. The catalogue the old library served handed
  // our whole state to anyone asking and let anyone write into it.
  return 0;
}

void Hcp2Codec::setCommandValuesToRead(uint32_t nowMs) {
  // Runs here, in the task that owns all of this, and it can arm a repeat,
  // which must not be decided from a half read state.
  this->checkCommandEffect(nowMs);

  uint16_t regPlug2Value = 0x0000;
  uint16_t regPlug3Value = 0x0000;

  // Written once, then gone: the answer is rebuilt from nothing on every poll,
  // so a command that is not put back in is simply not there any more.
  // The watch is the bus task's, so a stop from the other side asks rather than
  // reaches in.
  if (this->cancelWatch.exchange(false)) {
    this->awaitedCommand = HoermannCommand::NONE;
    this->repeatCommand = HoermannCommand::NONE;
  }

  HoermannCommand cmd = this->nextCommand.load();
  // A press only goes out while somebody could still be standing there. The
  // drive answers within one poll, so anything older than this waited for a
  // frame that never came: an enumeration burst, or a bus that was talking but
  // not to us.
  if (cmd != HoermannCommand::NONE && (nowMs - this->nextCommandOn.load()) > CMD_STALE_MS) {
    this->events.push(HcpEvent::COMMAND_STALE, (uint16_t) cmd, 0, nowMs);
    this->nextCommand.store(HoermannCommand::NONE);
    cmd = HoermannCommand::NONE;
  }
  // Resolved here, against the state as it stands now rather than as it stood
  // when somebody pressed the button.
  if (cmd == HoermannCommand::STOP) {
    const HoermannState::State now = this->state->state;
    const bool moving = now == HoermannState::CLOSING || now == HoermannState::OPENING ||
                        now == HoermannState::MOVE_HALF || now == HoermannState::MOVE_VENTING;
    // A standing door is stopped by there being nothing queued. An impulse
    // would start it.
    cmd = moving ? HoermannCommand::IMPULSE : HoermannCommand::NONE;
    this->nextCommand.store(HoermannCommand::NONE);
  }

  // Which of the two it is decides what gets said about it and whether the
  // watch starts over; putting the value on the wire is the same either way.
  HcpEvent why = HcpEvent::COMMAND_SENT;
  if (cmd != HoermannCommand::NONE) {
    // A press always wins over a repeat of our own.
    this->repeatCommand = HoermannCommand::NONE;
    this->nextCommand.store(HoermannCommand::NONE);
    // Start watching for the effect.
    this->awaitedCommand = cmd;
    this->awaitedSince = nowMs;
    this->confirmRepeats = 0;
    this->stateWritesWhenSent = this->stateWrites;
    this->repeatsSent = 0;
  } else if (this->repeatCommand != HoermannCommand::NONE && this->repeatStillMakesSense()) {
    // The deadline keeps running from the first attempt, or a command that
    // never takes effect never gives up.
    cmd = this->repeatCommand;
    this->repeatCommand = HoermannCommand::NONE;
    why = HcpEvent::COMMAND_REPEATED;
  } else if (this->repeatCommand != HoermannCommand::NONE) {
    this->events.push(HcpEvent::COMMAND_REPEAT_DROPPED, 0, 0, nowMs);
    this->repeatCommand = HoermannCommand::NONE;
  }

  if (cmd != HoermannCommand::NONE) {
    const HoermannCommandValues &v = commandValues(cmd);
    regPlug2Value = v.reg2;
    regPlug3Value = v.reg3;
    this->events.push(why, (uint16_t) cmd, v.reg2, nowMs);
    this->lastSentAt = nowMs;
  }
  // So a lost answer can be reconstructed. Nothing carried, nothing to put
  // back.
  this->lastSentCommand = cmd;
  this->regResp[2] = regPlug2Value;
  this->regResp[3] = regPlug3Value;
}

// Did the door do what it was told? A direction that is already reached counts,
// otherwise pressing "open" on an open door would look like a failure.
/** Is the door already where this command would send it? */
bool Hcp2Codec::commandReached(HoermannCommand cmd) const {
  const HoermannState::State now = this->state->state;
  switch (cmd) {
    case HoermannCommand::OPEN:
      return now == HoermannState::OPENING || now == HoermannState::OPEN;
    case HoermannCommand::CLOSE:
      return now == HoermannState::CLOSING || now == HoermannState::CLOSED;
    case HoermannCommand::HALF:
      return now == HoermannState::MOVE_HALF || now == HoermannState::HALFOPEN;
    case HoermannCommand::VENT:
      return now == HoermannState::MOVE_VENTING || now == HoermannState::VENT;
    case HoermannCommand::LIGHT_ON:
      return this->state->lightOn;
    case HoermannCommand::LIGHT_OFF:
      return !this->state->lightOn;
    default:
      // An impulse names no destination, so there is nothing to have reached.
      return false;
  }
}

bool Hcp2Codec::commandTookEffect() const {
  // Counted, not compared: a door that started and was stopped again between
  // two polls comes back to the same word, and a value test reads that as the
  // drive having done nothing.
  if (this->stateWrites != this->stateWritesWhenSent)
    return true;
  // Otherwise the destination counts, so asking an open door to open is not
  // taken for a failure.
  return this->commandReached(this->awaitedCommand);
}

/**
 * Between arming a repeat and sending it, the door can have moved on. An
 * impulse is the dangerous one: on a standing door it starts a run, so a repeat
 * armed while the door was travelling must not go out once it has stopped.
 */
bool Hcp2Codec::repeatStillMakesSense() const {
  const HoermannCommand cmd = this->repeatCommand;
  if (cmd == HoermannCommand::NONE)
    return false;
  if (cmd == HoermannCommand::IMPULSE)
    // An impulse has no destination to check against, so the question is
    // whether the drive did anything at all. It usually starts a standing door,
    // which is why "is it moving" was the wrong test: it threw away the repeat
    // for the ordinary toggle and told the caller the press had gone out.
    return this->stateWrites == this->stateWritesWhenSent;
  // Anything else names a destination, so it is still worth sending exactly
  // while the door is not already there.
  return !this->commandReached(cmd);
}

void Hcp2Codec::checkCommandEffect(uint32_t nowMs) {
  // Once, so the checks below all see the same command.
  const HoermannCommand awaited = this->awaitedCommand;
  if (awaited == HoermannCommand::NONE)
    return;
  if (this->commandTookEffect()) {
    this->awaitedCommand = HoermannCommand::NONE;
    // The repeat belongs to this command. After it took effect it is a second
    // key press.
    this->repeatCommand = HoermannCommand::NONE;
    return;
  }
  // A press the caller queued in the meantime takes over; repeating an old one
  // on top of it would be a second press nobody asked for.
  if (this->nextCommand.load() != HoermannCommand::NONE) {
    this->awaitedCommand = HoermannCommand::NONE;
    this->repeatCommand = HoermannCommand::NONE;
    return;
  }
  // Subtract, never add: adding overflows when millis() wraps.
  const uint32_t sinceFirstSent = nowMs - this->awaitedSince;
  const uint32_t sinceLastSent = nowMs - this->lastSentAt;
  if (sinceFirstSent > CMD_GIVEUP_MS) {
    this->events.push(HcpEvent::COMMAND_UNANSWERED, (uint16_t) awaited, 0, nowMs);
    this->awaitedCommand = HoermannCommand::NONE;
    this->repeatCommand = HoermannCommand::NONE;
    // A target outliving the command that created it hijacks the next run in
    // that direction, whoever started it.
    this->state->setGotoPosition(0.0f);
    return;
  }
  if (sinceLastSent > CMD_CONFIRM_MS && this->confirmRepeats < this->caps.unactedRepeats) {
    this->confirmRepeats++;
    this->repeatCommand = awaited;
  }
}

void Hcp2Codec::onDoorPositonChanged(uint16_t oldVal, uint16_t val) {
  // on First Byte changed (current)
  if ((oldVal & 0x00FF) != (val & 0x00FF)) {
    this->state->setCurrentPosition((float) (val & 0x00FF) / 200.0f);
    // Decided after the whole telegram, not here. Registers are applied in
    // address order, so the state this would test is still the previous
    // telegram's: one that reports both "past the target" and "stopped" would
    // fire an impulse at a door somebody had just stopped.
    this->gotoCheckPending = true;
  }
  // on Second Byte changed (target)
  if ((oldVal & 0xFF00) != (val & 0xFF00)) {
    this->state->setTargetPosition((float) ((val & 0xFF00) >> 8) / 200.0f);
  }
}

void Hcp2Codec::onCurrentStateChanged(uint16_t oldVal, uint16_t val) {
  // on First Byte changed
  if ((oldVal & 0xFF00) != (val & 0xFF00)) {
    this->events.push(HcpEvent::STATE_CHANGED, (uint16_t) ((val & 0xFF00) >> 8), val);

    switch ((val & 0xFF00) >> 8) {
      case 0x1:
        this->state->setState(HoermannState::State::OPENING);
        break;
      case 0x2:
        this->state->setState(HoermannState::State::CLOSING);
        break;
      case 0x20:
        this->state->setState(HoermannState::State::OPEN);
        break;
      case 0x40:
        this->state->setState(HoermannState::State::CLOSED);
        break;
      case 0x80:
        this->state->setState(HoermannState::State::HALFOPEN);
        break;
      case 0x09:
        this->state->setState(HoermannState::State::MOVE_VENTING);
        break;
      case 0x05:
        this->state->setState(HoermannState::State::MOVE_HALF);
        break;
      case 0x0A:
        this->state->setState(HoermannState::State::VENT);
        break;
      case 0x00:
        // Additional check on the low byte when the high byte is 0x00
        if ((val & 0x00FF) == 0x61) {
          this->state->setState(HoermannState::State::VENT);
        } else {
          this->state->setState(HoermannState::State::STOPPED);
        }
        break;
      default:
        this->events.push(HcpEvent::UNKNOWN_STATE, (uint16_t) ((val & 0xFF00) >> 8), val);
    }
  }
}

void Hcp2Codec::onRegSevenChanged(uint16_t oldVal, uint16_t val)
//Observed Values, last bit 4 is assumed could not be tested as I have no UAP HCP.
//0x00 0x00 Relay off - Light off
//0x02 0x00 Relay on  - light off
//0x02 0x10 Relay on  - light on
//0x00 0x10 Relay off - light on
//0x00 0x14 Relay on  - light on
//0x00 0x04 Relay on  - light off

{
  if ((oldVal & 0xFF00) != (val & 0xFF00)) {
    // The drive's own fault indication, whatever the relay bit below means on
    // a given installation.
    this->state->setActuatorError((val & 0x3000) != 0);
    // 0x02 happen when relay menu 30 is set to 06, 07, 10
    this->state->setRelayOn((val & 0xFF00) >> 8 == 0x02);
  }
  // On second byte changed
  if ((oldVal & 0x00FF) != (val & 0x00FF)) {
    this->events.push(HcpEvent::RELAY_REGISTER, 0, val);
    this->state->setLigthOn((val & 0x00FF) == 0x14 || (val & 0x00FF) == 0x10);
    this->state->setRelayOn((val & 0xFF00) >> 8 == 0x02 || (val & 0x00FF) == 0x14 || (val & 0x00FF) == 0x04);
  }
}

/**
 * Write on 0x9C41 , byte1: counter, byte2: command
 */
/** The go-to-position stop, once the whole broadcast has been applied. */
void Hcp2Codec::checkGotoTarget(uint32_t nowMs) {
  if (!this->gotoCheckPending)
    return;
  this->gotoCheckPending = false;
  const float target = this->state->gotoPosition;
  if (target <= 0.0f)
    return;
  const HoermannState::State now = this->state->state;
  const bool arrived = (now == HoermannState::CLOSING && target >= this->state->currentPosition) ||
                       (now == HoermannState::OPENING && target <= this->state->currentPosition);
  if (!arrived)
    return;
  // Only a stop that was actually issued may clear the target; otherwise the
  // door runs to the end stop with nothing left to retry from.
  if (this->stopDoor(nowMs))
    this->state->setGotoPosition(0.0f);
}

void Hcp2Codec::onCounterWrite(uint16_t val) {
  // Top bit selects the half of a split payload; it is not part of the count.
  uint16_t counter = val & 0x7F00;
  uint16_t command = (val & 0x00FF) << 8;
  this->regResp[0] |= counter;
  this->regResp[1] |= command;
}

/**
 * Called with the drive's counter byte before the answer is built, because the
 * decision it makes has to happen before a command is taken out of the queue.
 *
 * The byte that ends up on the wire is not changed by any of this. In step the
 * count equals what the drive just sent, and after a loss the fallback value is
 * the one the drive is repeating, which is the same byte again. Its whole
 * purpose is to notice the loss.
 */
void Hcp2Codec::syncCounter(uint8_t counterByte, uint8_t command) {
  // Top bit is the half selector of a split payload, not part of the count.
  const uint8_t rx = counterByte & 0x7F;

  if (!this->txCounterValid) {
    // First frame we ever see: adopt whatever the drive is counting.
    this->txCounter = rx;
    this->txCounterValid = true;
    return;
  }
  if (rx != this->txCounter) {
    // Only a repeat of the value we already answered is a loss, and only a
    // status poll carries a command.
    if (rx == this->txCounterPrev && command == CMD_STATUS)
      this->rearmLostCommand();
    // Anything else is the drive somewhere unexpected. Follow it: holding our
    // own value kept the mismatch, and the repeat, alive on every frame.
    this->txCounter = rx;
  }
}

void Hcp2Codec::advanceCounter() {
  this->txCounterPrev = this->txCounter;
  this->txCounter = this->txCounter == 0x7F ? 1 : (uint8_t) (this->txCounter + 1);
}

/**
 * Put back what the lost answer was carrying. Only the repeat slot is used, so
 * a press made in the meantime still wins: that one is newer and says what the
 * user wants now.
 */
void Hcp2Codec::rearmLostCommand() {
  if (this->lastSentCommand == HoermannCommand::NONE)
    return;  // the lost answer carried no command, nothing to put back
  if (this->nextCommand.load() != HoermannCommand::NONE)
    return;
  // While the drive repeats a counter our answer never arrived, so re-sending
  // is right and at most one copy can land. That rests on the drive holding its
  // counter until answered, which is a model of it, not a fact about it.
  // Capped, so a drive that behaves otherwise costs a bounded number of key
  // presses instead of one per poll for as long as it lasts.
  if (this->repeatsSent >= this->caps.lostAnswerRepeats) {
    this->events.push(HcpEvent::COMMAND_RESEND_GAVE_UP);
    this->lastSentCommand = HoermannCommand::NONE;
    return;
  }
  this->repeatsSent++;
  this->repeatCommand = this->lastSentCommand;
  // The drive told us the answer never arrived, so there is nothing left to
  // watch. Leaving the watch standing let the effect guess win over the
  // evidence: any unrelated change of the state word between the two polls
  // counted as "the drive acted" and threw the repeat away.
  this->awaitedCommand = HoermannCommand::NONE;
  this->lastSentCommand = HoermannCommand::NONE;
  this->events.push(HcpEvent::COMMAND_RESENT, (uint16_t) this->repeatCommand);
}

void Hcp2Codec::requestDriveIdentity(uint32_t nowMs) { this->armIdentityRequest(nowMs, IDENT_REQ_SERIAL); }

void Hcp2Codec::armIdentityRequest(uint32_t nowMs, uint8_t request) {
  this->identityWanted = request;
  this->identityAttempts = 0;
  this->identityAskedOn = 0;
  this->identityAsked = false;
  this->serialFirstHalfSeen = false;
}

// Registers hold two payload bytes each, high byte first.
void Hcp2Codec::copyRegsToBytes(uint8_t firstReg, uint8_t regCount, uint8_t *out) {
  for (uint8_t i = 0; i < regCount; i++) {
    const uint16_t v = this->regCmd[firstReg + i];
    out[2 * i] = (uint8_t) (v >> 8);
    out[2 * i + 1] = (uint8_t) (v & 0x00FF);
  }
}

// Trailing padding varies, and anything unprintable would only confuse a text
// sensor, so cut at the first byte that is neither.
static void identityToText(const uint8_t *data, size_t len, char *out, size_t outSize) {
  size_t at = 0;
  for (size_t i = 0; i < len && at + 1 < outSize; i++) {
    if (data[i] < 0x20 || data[i] > 0x7E)
      break;
    out[at++] = (char) data[i];
  }
  while (at > 0 && out[at - 1] == ' ')
    at--;
  out[at] = '\0';
}

// Silence would leave the code at zero, which the protocol does not define.
void Hcp2Codec::answerTransfer(uint8_t counterByte, uint8_t code) {
  for (uint8_t i = 2; i < REG_RESP_COUNT; i++)
    this->regResp[i] = 0x0000;
  // Only the lower seven bits are the drive's running counter, the top bit
  // marks which half of a split payload this was and must not be echoed.
  this->regResp[0] = (uint16_t) ((counterByte & 0x7F) << 8);
  this->regResp[1] = (uint16_t) (0x0400 | code);
}

void Hcp2Codec::onWriteBlockComplete(uint32_t nowMs, uint16_t addr, uint16_t count) {
  if (addr != REG_CMD_BASE || count < 2)
    return;
  // Low byte of the first register is what the drive wants from us, high byte
  // its running counter. Only the payload transfer is of interest here.
  const uint8_t command = (uint8_t) (this->regCmd[0] & 0x00FF);
  if (command != 0x04)
    return;
  const uint8_t counterByte = (uint8_t) (this->regCmd[0] >> 8);
  const uint8_t subCode = (uint8_t) (this->regCmd[1] >> 8);

  if (subCode == IDENT_SUB_PAUSE_ACK) {
    // The drive names the address it is pausing. It sits astride two
    // registers: low byte of the sub code register, high byte of the next.
    if (count >= 3) {
      const uint16_t named = (uint16_t) (((this->regCmd[1] & 0x00FF) << 8) | (this->regCmd[2] >> 8));
      if (named == SLAVE_ID)
        this->pauseConfirmed.store(true);
    }
    // Answered either way: we understood the question, and leaving it open is
    // worse than answering for an address that was not ours.
    this->answerTransfer(counterByte, RESP_ACK);
    return;
  }

  if (subCode == IDENT_SUB_SERIAL || subCode == IDENT_SUB_FIRMWARE) {
    // Acknowledging a payload we threw away tells the drive it landed, so it
    // never sends it again and the value only returns via our own retry.
    this->answerTransfer(counterByte, this->onIdentityData(nowMs, counterByte, subCode, count) ? RESP_ACK : RESP_NAK);
    return;
  }

  // Something we do not know. Say so rather than leaving an undefined code.
  this->events.push(HcpEvent::UNKNOWN_TRANSFER_SUB, subCode, 0, nowMs);
  this->answerTransfer(counterByte, RESP_NAK);
}

void Hcp2Codec::publishIdentity() {
  if (this->identSerialReady.exchange(false))
    this->state->setSerialNumber(this->identSerial);
  if (this->identFirmwareReady.exchange(false))
    this->state->setFirmwareVersion(this->identFirmware);
}

void Hcp2Codec::checkBusSilence(uint32_t nowMs) {
  const uint32_t last = this->lastFrameOn.load();
  if (last == 0)
    return;  // nothing ever arrived, the initial state already says so
  if ((nowMs - last) > BUS_SILENCE_MS) {
    this->state->setValid(false);
    this->state->setLink(HcpLink::SILENT);
    // The slot has no expiry, so a press made just before we called the link
    // dead would move the door whenever it returns.
    if (this->nextCommand.exchange(HoermannCommand::NONE) != HoermannCommand::NONE)
      this->events.push(HcpEvent::LINK_LOST_COMMAND_DROPPED, 0, 0, nowMs);
  }
}

bool Hcp2Codec::onIdentityData(uint32_t nowMs, uint8_t counterByte, uint8_t subCode, uint16_t count) {
  // A payload for a request that is no longer open is a late resend. Taking it
  // would restart the retry clock of whatever is outstanding now.
  const uint8_t answers = subCode == IDENT_SUB_SERIAL ? IDENT_REQ_SERIAL : IDENT_REQ_FIRMWARE;
  if (this->identityWanted != answers)
    return false;
  // The payload starts one register behind the sub code.
  const uint8_t firstPayloadReg = 2;
  const uint16_t payloadRegs = count > firstPayloadReg ? count - firstPayloadReg : 0;
  bool kept = false;

  if (subCode == IDENT_SUB_FIRMWARE && payloadRegs >= IDENT_FIRMWARE_LEN / 2) {
    uint8_t buf[IDENT_FIRMWARE_LEN];
    this->copyRegsToBytes(firstPayloadReg, IDENT_FIRMWARE_LEN / 2, buf);
    identityToText(buf, sizeof(buf), this->identFirmware, sizeof(this->identFirmware));
    this->identFirmwareReady.store(true);
    this->identityWanted = 0;
    this->events.push(HcpEvent::IDENTITY_FIRMWARE, 0, 0, nowMs);
    kept = true;
  } else if (subCode == IDENT_SUB_SERIAL) {
    // Top bit marks the first half. The halves differ in length, so each is
    // checked against its own; otherwise a short one pads with stale registers.
    const bool firstHalf = (counterByte & 0x80) != 0;
    if (firstHalf && payloadRegs >= SERIAL_FIRST_REGS) {
      this->copyRegsToBytes(firstPayloadReg, SERIAL_FIRST_REGS, this->serialBuf);
      this->serialFirstHalfSeen = true;
      // Half an answer is progress. The attempt count does not restart, or a
      // drive sending only the first half would be asked for ever.
      this->identityAskedOn = nowMs;
      this->identityAsked = true;
      kept = true;
    } else if (!firstHalf && this->serialFirstHalfSeen && payloadRegs >= SERIAL_SECOND_REGS) {
      this->copyRegsToBytes(firstPayloadReg, SERIAL_SECOND_REGS, this->serialBuf + 2 * SERIAL_FIRST_REGS);
      this->serialFirstHalfSeen = false;
      identityToText(this->serialBuf, IDENT_SERIAL_LEN, this->identSerial, sizeof(this->identSerial));
      this->identSerialReady.store(true);
      this->events.push(HcpEvent::IDENTITY_SERIAL, 0, 0, nowMs);
      // Identity is only complete with the firmware version, so go straight on.
      this->armIdentityRequest(nowMs, IDENT_REQ_FIRMWARE);
      kept = true;
    }
  }

  return kept;
}

/**
 * Helper to set next Command and *not* skip Current Command before end was sent
 */
bool Hcp2Codec::setCommand(uint32_t nowMs, bool cond, HoermannCommand command) {
  if (cond) {
    // Queueing with nobody listening does not delay a press, it arms one: the
    // slot has no expiry, so the door moves whenever the link returns. Refusing
    // lets the caller say so while the person who pressed is still there.
    if (!this->state->valid) {
      this->events.push(HcpEvent::COMMAND_REFUSED_NO_LINK, (uint16_t) command, 0, nowMs);
      return false;
    }
    this->nextCommandOn.store(nowMs);
    // Set from the main loop, read by the bus task. The slot holds a value now
    // rather than a pointer to one, so it fits an atomic that is lock free on
    // every target this builds for.
    HoermannCommand expected = HoermannCommand::NONE;
    if (!this->nextCommand.compare_exchange_strong(expected, command)) {
      this->events.push(HcpEvent::COMMAND_REFUSED_SLOT_BUSY, (uint16_t) command, 0, nowMs);
      return false;
    }
  }
  return true;
}

/**
 * Control Functions
 */
bool Hcp2Codec::stopDoor(uint32_t nowMs) {
  // A stop outranks whatever is queued. Competing for the slot meant the last
  // thing the user pressed could lose to the one before it: a stop while an
  // open was still waiting was reported as accepted and the door opened.
  if (!this->state->valid) {
    this->events.push(HcpEvent::COMMAND_REFUSED_NO_LINK, (uint16_t) HoermannCommand::STOP, 0, nowMs);
    return false;
  }
  // Stored as the intent, not as what it will turn into. Whether a stop needs
  // an impulse depends on the door still moving when the frame goes out, and
  // between here and there it can arrive - at which point the same impulse
  // would start it again.
  this->nextCommand.store(HoermannCommand::STOP);
  this->nextCommandOn.store(nowMs);
  // The watch belongs to the bus task, so it is asked to drop it rather than
  // reached into from here.
  this->cancelWatch.store(true);
  return true;
}
bool Hcp2Codec::closeDoor(uint32_t nowMs) { return setCommand(nowMs, true, HoermannCommand::CLOSE); }
bool Hcp2Codec::openDoor(uint32_t nowMs) { return setCommand(nowMs, true, HoermannCommand::OPEN); }
bool Hcp2Codec::impulseDoor(uint32_t nowMs) { return setCommand(nowMs, true, HoermannCommand::IMPULSE); }
bool Hcp2Codec::halfPositionDoor(uint32_t nowMs) { return setCommand(nowMs, true, HoermannCommand::HALF); }
bool Hcp2Codec::ventilationPositionDoor(uint32_t nowMs) { return setCommand(nowMs, true, HoermannCommand::VENT); }
bool Hcp2Codec::turnLight(uint32_t nowMs, bool on) {
  return setCommand(nowMs, on != this->state->lightOn, on ? HoermannCommand::LIGHT_ON : HoermannCommand::LIGHT_OFF);
}
bool Hcp2Codec::setPosition(uint32_t nowMs, int setPosition) {
  // First and last movement segments seem a bit inconsistent on Promatic4, so it's better to leave it to fully open or close.
  if (setPosition <= 5)
    return closeDoor(nowMs);
  if (setPosition >= 95)
    return openDoor(nowMs);

  // Stopping the door partway needs a position on the wire to stop against.
  // Without one this can only be open or shut, which the two branches above
  // already answered.
  if (!this->caps.hasPosition)
    return false;

  const float target = static_cast<float>(setPosition) / 100.0f;
  const float here = this->state->currentPosition;
  if (here == target)
    return true;  // already there

  // The target is what stops the door on the way, so it must not outlive the
  // run it belongs to. Armed only once the command it goes with has been
  // accepted: set first, it survived a refusal and then halted a later,
  // unrelated run at a position nobody had asked for.
  if (!setCommand(nowMs, true, here < target ? HoermannCommand::OPEN : HoermannCommand::CLOSE))
    return false;
  this->state->setGotoPosition(target);
  return true;
}

}  // namespace hcpbridge
}  // namespace esphome
