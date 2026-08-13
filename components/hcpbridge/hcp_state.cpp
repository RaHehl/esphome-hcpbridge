#include <cstring>

#include "hcp_state.h"

namespace esphome {
namespace hcpbridge {

const char *const TAG_HCI = "hcpbridge.bus";

const char *hoermannStateName(HoermannState::State state) {
  switch (state) {
    case HoermannState::OPENING:
      return "Opening";
    case HoermannState::MOVE_VENTING:
      return "Move venting";
    case HoermannState::MOVE_HALF:
      return "Move half";
    case HoermannState::CLOSING:
      return "Closing";
    case HoermannState::OPEN:
      return "Open";
    case HoermannState::CLOSED:
      return "Closed";
    case HoermannState::STOPPED:
      return "Stopped";
    case HoermannState::HALFOPEN:
      return "Half open";
    case HoermannState::VENT:
      return "Venting";
    default:
      return "Unknown";
  }
}

// Indexed by HoermannCommand; the order there and here has to stay in step.
// Light has one command per direction: a toggle can only mean "the other one",
// so a repeat undoes it and "on" is not something that can be asked for.
static const HoermannCommandValues COMMAND_VALUES[] = {
    {0x0000, 0x0000},  // NONE
    {0x0110, 0x0000},  // OPEN
    {0x0120, 0x0000},  // CLOSE
    {0x0140, 0x0000},  // IMPULSE
    {0x0100, 0x0400},  // HALF
    {0x0100, 0x4000},  // VENT
    {0x0880, 0x0000},  // LIGHT_ON
    {0x0800, 0x0100},  // LIGHT_OFF
    {0x0000, 0x0000},  // STOP: resolved before it is looked up, never sent
};
static const char *const COMMAND_NAMES[] = {
    "nothing", "open", "close", "impulse", "half", "vent", "light on", "light off", "stop",
};
static constexpr size_t COMMAND_COUNT = sizeof(COMMAND_VALUES) / sizeof(COMMAND_VALUES[0]);
static_assert(COMMAND_COUNT == (size_t) HoermannCommand::STOP + 1,
              "a command was added without the value it goes out as");
static_assert(sizeof(COMMAND_NAMES) / sizeof(COMMAND_NAMES[0]) == COMMAND_COUNT, "a command was added without a name");

const HoermannCommandValues &commandValues(HoermannCommand cmd) {
  const size_t i = (size_t) cmd;
  return COMMAND_VALUES[i < COMMAND_COUNT ? i : 0];
}

const char *hcpLinkName(HcpLink link) {
  switch (link) {
    case HcpLink::ENUMERATING:
      return "Enumerating";
    case HcpLink::REGISTERED:
      return "Registered";
    case HcpLink::SILENT:
      return "Silent";
    case HcpLink::PAUSED:
      return "Paused";
    default:
      return "Never seen";
  }
}

const char *commandName(HoermannCommand cmd) {
  const size_t i = (size_t) cmd;
  return COMMAND_NAMES[i < COMMAND_COUNT ? i : 0];
}

void HoermannState::setTargetPosition(float targetPosition) {
  this->targetPosition = targetPosition;
  this->changed = true;
}
void HoermannState::setGotoPosition(float setPosition) {
  this->gotoPosition = setPosition;
  this->changed = true;
}
void HoermannState::setCurrentPosition(float currentPosition) {
  this->currentPosition = currentPosition;
  this->changed = true;
}
void HoermannState::setLigthOn(bool lightOn) {
  this->lightOn = lightOn;
  this->changed = true;
}
void HoermannState::setRelayOn(bool relayOn) {
  this->relayOn = relayOn;
  this->changed = true;
}
void HoermannState::clearChanged() {
  // Exchange, not assign: a change raised by the bus task between the test and
  // this line would otherwise be dropped.
  this->changed.exchange(false);
}
static bool isMoving(HoermannState::State s) {
  return s == HoermannState::State::OPENING || s == HoermannState::State::CLOSING ||
         s == HoermannState::State::MOVE_HALF || s == HoermannState::State::MOVE_VENTING;
}

void HoermannState::setState(State state) {
  const bool was_moving = isMoving(this->state);
  this->state = state;
  this->changed = true;
  // A target must not survive a movement that ended for another reason, or the
  // next run in that direction stops where nobody asked.
  if (was_moving && !isMoving(state))
    this->gotoPosition = 0.0f;
}
void HoermannState::setActuatorError(bool actuatorError) {
  if (this->actuatorError == actuatorError)
    return;
  this->actuatorError = actuatorError;
  this->changed = true;
}

void HoermannState::setValid(bool isValid) {
  if (this->valid.exchange(isValid) == isValid)
    return;
  // Without this the connection sensor would only ever learn of the first
  // frame, never of the silence afterwards.
  this->changed = true;
}

void HoermannState::setLink(HcpLink link) {
  // Exchange rather than compare-then-write: this is the one field two tasks
  // set, the bus task on every answered poll and the loop when the bus has gone
  // quiet. Read first, a lost write also loses the changed flag, and the sensor
  // would sit on a stale link until something unrelated moved.
  if (this->link.exchange(link) == link)
    return;
  this->changed = true;
}

void HoermannState::setSerialNumber(const char *serialNumber) {
  if (strncmp(this->serialNumber, serialNumber, sizeof(this->serialNumber)) == 0)
    return;
  strncpy(this->serialNumber, serialNumber, sizeof(this->serialNumber) - 1);
  this->serialNumber[sizeof(this->serialNumber) - 1] = '\0';
  this->changed = true;
}

void HoermannState::setFirmwareVersion(const char *firmwareVersion) {
  if (strncmp(this->firmwareVersion, firmwareVersion, sizeof(this->firmwareVersion)) == 0)
    return;
  strncpy(this->firmwareVersion, firmwareVersion, sizeof(this->firmwareVersion) - 1);
  this->firmwareVersion[sizeof(this->firmwareVersion) - 1] = '\0';
  this->changed = true;
}

}  // namespace hcpbridge
}  // namespace esphome
