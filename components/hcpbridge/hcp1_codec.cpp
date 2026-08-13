#include "hcp1_codec.h"

namespace esphome {
namespace hcpbridge {

uint8_t Hcp1Codec::commandByte(HoermannCommand cmd) const {
  switch (cmd) {
    case HoermannCommand::OPEN:
      return HCP1_DO_OPEN;
    case HoermannCommand::CLOSE:
      return HCP1_DO_CLOSE;
    case HoermannCommand::IMPULSE:
      return HCP1_DO_IMPULSE;
    case HoermannCommand::VENT:
      return HCP1_DO_VENT;
    // One bit, and it toggles. There is no way to ask for the light to be on,
    // only for it to be other than it is, so the two directions collapse into
    // the same request and the caller's intent is checked before it gets here.
    case HoermannCommand::LIGHT_ON:
    case HoermannCommand::LIGHT_OFF:
      return HCP1_DO_LIGHT;
    // Half open has no bit on this bus. The drive has the position, but nothing
    // in either description says how to ask for it, and guessing would mean
    // guessing with a door.
    default:
      return 0;
  }
}

bool Hcp1Codec::submit(uint32_t nowMs, HoermannCommand cmd) {
  if (cmd != HoermannCommand::STOP && this->commandByte(cmd) == 0)
    return false;  // nothing on this bus says that
  if (!this->addressed_) {
    // Queueing here would arm a press for whenever the drive first speaks,
    // which could be minutes away and nobody standing there.
    this->events.push(HcpEvent::COMMAND_STALE, (uint16_t) cmd, 0, nowMs);
    return false;
  }
  this->pending_ = cmd;
  this->pendingAt_ = nowMs;
  return true;
}

bool Hcp1Codec::turnLight(uint32_t nowMs, bool on) {
  // One bit, and it toggles. Asking for the state it is already in would send a
  // press that turns it the other way.
  if (on == this->state->lightOn)
    return true;
  return this->submit(nowMs, on ? HoermannCommand::LIGHT_ON : HoermannCommand::LIGHT_OFF);
}

bool Hcp1Codec::setPosition(uint32_t nowMs, int percent) {
  if (percent <= 5)
    return this->closeDoor(nowMs);
  if (percent >= 95)
    return this->openDoor(nowMs);
  return false;
}

void Hcp1Codec::checkBusSilence(uint32_t nowMs) {
  if (this->lastFrameAt_ == 0)
    return;  // nothing ever arrived, the initial state already says so
  if ((nowMs - this->lastFrameAt_) > BUS_SILENCE_MS) {
    this->state->setValid(false);
    this->state->setLink(HcpLink::SILENT);
    // A press made just before the link was called dead would otherwise move
    // the door whenever it returns.
    this->pending_ = HoermannCommand::NONE;
    this->addressed_ = false;
  }
}

bool Hcp1Codec::stopDoor(uint32_t nowMs) {
  // The intent, not what it will turn into: whether a stop needs an impulse
  // depends on the door still moving when the frame goes out, and between here
  // and there it can arrive - at which point the same impulse would start it.
  this->pending_ = HoermannCommand::STOP;
  this->pendingAt_ = nowMs;
  return true;
}

void Hcp1Codec::applyBroadcast(uint32_t nowMs, const uint8_t *data, uint8_t len) {
  if (len < 2)
    return;
  const uint8_t d0 = data[0];

  HoermannState::State next;
  if (d0 & HCP1_BC_MOVING)
    next = (d0 & HCP1_BC_CLOSING) ? HoermannState::CLOSING : HoermannState::OPENING;
  else if (d0 & HCP1_BC_VENTING)
    next = HoermannState::VENT;
  else if (d0 & HCP1_BC_OPEN)
    next = HoermannState::OPEN;
  else if (d0 & HCP1_BC_CLOSED)
    next = HoermannState::CLOSED;
  else
    next = HoermannState::STOPPED;

  if (next != this->state->state)
    this->events.push(HcpEvent::STATE_CHANGED, d0, 0, nowMs);
  this->state->setState(next);
  this->state->setLigthOn((d0 & HCP1_BC_LIGHT) != 0);
  this->state->setRelayOn((d0 & HCP1_BC_RELAY) != 0);
  this->state->setActuatorError((d0 & HCP1_BC_ERROR) != 0);
  // Nothing on this bus carries a position, so the only honest values are the
  // two ends. Anything in between would be invented.
  if (next == HoermannState::OPEN)
    this->state->setCurrentPosition(1.0f);
  else if (next == HoermannState::CLOSED)
    this->state->setCurrentPosition(0.0f);
  this->state->setValid(true);
}

size_t Hcp1Codec::answerScan(uint8_t counter, uint8_t *out) {
  const uint8_t data[] = {HCP1_TYPE_UAP1, HCP1_ADDR_SELF};
  return hcp1Build(out, HCP1_MAX_FRAME, HCP1_ADDR_MASTER, hcp1NextCounter(counter), data, 2);
}

size_t Hcp1Codec::answerStatus(uint32_t nowMs, uint8_t counter, uint8_t *out) {
  uint8_t doThis = 0;
  if (this->pending_ != HoermannCommand::NONE) {
    // A press only goes out while somebody could still be standing there.
    if (nowMs - this->pendingAt_ > CMD_STALE_MS) {
      this->events.push(HcpEvent::COMMAND_STALE, (uint16_t) this->pending_, 0, nowMs);
    } else {
      HoermannCommand cmd = this->pending_;
      // Resolved here, against the state as it stands now rather than as it
      // stood when somebody pressed the button. A standing door is stopped by
      // there being nothing to send.
      if (cmd == HoermannCommand::STOP) {
        const HoermannState::State now = this->state->state;
        const bool moving = now == HoermannState::OPENING || now == HoermannState::CLOSING ||
                            now == HoermannState::MOVE_HALF || now == HoermannState::MOVE_VENTING;
        cmd = moving ? HoermannCommand::IMPULSE : HoermannCommand::NONE;
      }
      doThis = this->commandByte(cmd);
      if (doThis != 0) {
        this->events.push(HcpEvent::COMMAND_SENT, (uint16_t) cmd, doThis, nowMs);
        this->lastSent_ = cmd;
        this->lastSentAt_ = nowMs;
      }
    }
    // Either way it is gone: the answer is built from nothing each time, so a
    // command not put back in is simply not there. Nothing re-sends it, because
    // nothing on this bus can tell us whether it arrived.
    this->pending_ = HoermannCommand::NONE;
  }

  const uint8_t data[] = {HCP1_CMD_STATUS_RESPONSE, doThis, HCP1_S0_HEALTHY};
  return hcp1Build(out, HCP1_MAX_FRAME, HCP1_ADDR_MASTER, hcp1NextCounter(counter), data, 3);
}

size_t Hcp1Codec::onFrame(uint32_t nowMs, const uint8_t *buf, size_t len, uint8_t *out) {
  Hcp1Frame f;
  if (!hcp1Parse(buf, len, &f))
    return 0;

  // Anything that parses is a sign the bus is alive, even if it is somebody
  // else's frame. Zero would read as "never", so it is nudged past it.
  this->lastFrameAt_ = nowMs == 0 ? 1 : nowMs;
  if (this->state->link == HcpLink::NEVER_SEEN)
    this->state->setLink(HcpLink::ENUMERATING);

  if (f.address == HCP1_ADDR_BROADCAST) {
    // Only once the drive has spoken to us directly. Before that we are
    // listening to a conversation we are not part of.
    if (this->addressed_)
      this->applyBroadcast(nowMs, f.data, f.length);
    return 0;  // a broadcast is addressed to everybody and answered by nobody
  }

  if (f.address != HCP1_ADDR_SELF || f.length < 1)
    return 0;  // somebody else's accessory

  switch (f.data[0]) {
    case HCP1_CMD_SCAN:
      this->addressed_ = true;
      this->events.push(HcpEvent::BUSSCAN, 0, 0, nowMs);
      return this->answerScan(f.counter, out);
    case HCP1_CMD_STATUS_REQUEST:
      if (!this->addressed_)
        return 0;  // not ours to answer until the drive has said so
      // Being polled by address is what registration means here: the drive only
      // does it after a scan it accepted an answer to.
      this->state->setLink(HcpLink::REGISTERED);
      return this->answerStatus(nowMs, f.counter, out);
    default:
      this->events.push(HcpEvent::UNKNOWN_TRANSFER_SUB, f.data[0], 0, nowMs);
      return 0;
  }
}

}  // namespace hcpbridge
}  // namespace esphome
