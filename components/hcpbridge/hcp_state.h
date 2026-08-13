// The door as this component understands it, and what can be asked of it.
//
// Neither belongs to one bus: both Hoermann protocols move the same door and
// answer to the same entities, and only the frames between them differ. Kept
// apart from either, so that a second protocol needs a second way of writing
// frames and not a second copy of everything above them.
#pragma once

#include <atomic>
#include <cstdint>

namespace esphome {
namespace hcpbridge {

/**
 * What can be asked of the drive.
 *
 * A value rather than a pointer to a static instance: the two are told apart by
 * what they are and not by where they live, which is what lets one travel
 * between tasks in a single atomic and be written down in a log or a test.
 */
enum class HoermannCommand : uint8_t {
  NONE = 0,
  OPEN,
  CLOSE,
  IMPULSE,
  HALF,
  VENT,
  LIGHT_ON,
  LIGHT_OFF,
  /**
     * Stop, which is not a thing the drive can be told directly.
     *
     * A moving door is stopped by an impulse; a standing one is started by the
     * same impulse. Which of the two applies has to be decided from the state
     * at the moment it goes on the wire, not from the one whoever pressed the
     * button was looking at - between those two the door can arrive, and then
     * a stop starts it.
     */
  STOP,
};

/** The two answer registers a command is carried in. */
struct HoermannCommandValues {
  uint16_t reg2;
  uint16_t reg3;
};

/**
 * Measured at a door. A command used to go out as two frames, a press and a
 * release, and only the second ever had an effect; these are those.
 */
const HoermannCommandValues &commandValues(HoermannCommand cmd);

/** For a log line or a test failure, never parsed. */
const char *commandName(HoermannCommand cmd);

// How long the drive's two identity strings are. Here rather than with the rest
// of the protocol constants because the state below has to hold them.
// Fourteen bytes then twelve, seven registers then six.
static constexpr uint32_t SERIAL_FIRST_REGS = 7;
static constexpr uint32_t SERIAL_SECOND_REGS = 6;
static constexpr uint8_t IDENT_SERIAL_LEN = (SERIAL_FIRST_REGS + SERIAL_SECOND_REGS) * 2;
static constexpr uint32_t IDENT_FIRMWARE_LEN = 12;

/**
 * Where the conversation with the drive stands.
 *
 * A single flag could only say whether it is going, and the two ways it can be
 * not going need opposite responses: a bus nothing has ever arrived on is a
 * wiring or enumeration problem, while one that has fallen quiet is a drive
 * that dropped us and wants power cycling. Told apart here rather than by
 * whoever is standing at the door.
 */
enum class HcpLink : uint8_t {
  NEVER_SEEN = 0,  // not one frame, ever
  ENUMERATING,     // the drive is talking, but has not accepted an answer yet
  REGISTERED,      // answering polls, which is the ordinary state
  SILENT,          // was registered, then nothing for a long time
  PAUSED,          // told the drive we are going quiet, and it agreed
};

/** For a log line, a sensor or a test failure. */
const char *hcpLinkName(HcpLink link);

class HoermannState {
 public:
  enum State { OPEN, OPENING, CLOSED, CLOSING, HALFOPEN, MOVE_VENTING, VENT, MOVE_HALF, STOPPED };
  // Written by the bus task, read by the ESPHome loop on the other core. Atomic
  // for the ordering, not for the width: every one of these fits a word, so a
  // torn read was never the risk. What the compiler is entitled to do with a
  // plain member it cannot see anyone else touch is.
  //
  // All position values are from 0 to 100
  std::atomic<float> targetPosition{0};
  std::atomic<float> currentPosition{0};
  std::atomic<bool> lightOn{false};
  std::atomic<bool> relayOn{false};
  std::atomic<bool> actuatorError{false};
  std::atomic<State> state{CLOSED};
  std::atomic<HcpLink> link{HcpLink::NEVER_SEEN};
  std::atomic<bool> changed{false};
  // Bus task only: armed when a command goes out, cleared when the door stops
  // or the state changes. The loop hands its wish over through the codec's
  // request slot rather than writing here.
  float gotoPosition = 0.0f;
  std::atomic<bool> valid{false};
  // Empty until the drive has answered the matching request. Fixed buffers:
  // both have a length the protocol fixes, and a state shared between tasks
  // is a poor place for something that reallocates.
  char serialNumber[IDENT_SERIAL_LEN + 1] = {0};
  char firmwareVersion[IDENT_FIRMWARE_LEN + 1] = {0};

  void setTargetPosition(float targetPosition);
  void setGotoPosition(float setPosition);
  void setCurrentPosition(float currentPosition);
  void setLigthOn(bool lightOn);
  void setRelayOn(bool relayOn);
  void setActuatorError(bool actuatorError);
  void clearChanged();
  void setState(State state);
  void setValid(bool isValid);
  void setLink(HcpLink link);
  void setSerialNumber(const char *serialNumber);
  void setFirmwareVersion(const char *firmwareVersion);
};

// A command is only done once the drive has acted on it. If nothing has changed
// by the first mark, send it once more; give up at the second and say so.
//
// The two are measured from different moments on purpose. The confirmation runs
// from the last time the command actually went out, because a repeat needs a
// chance to arrive before another is considered. The deadline runs from the
// first time, and no repeat may touch it - when both ran from the same moment,
// each repeat pushed the deadline out of reach and a drive that answered but
// never moved was asked again for as long as it stayed that way.
// A press the drive never fetched is not delayed, it is stale: nobody is
// standing there any more.
static constexpr uint32_t CMD_STALE_MS = 2000;

// The drive polls several times a second. Nothing at all for this long means the
// link is gone, not that the drive has nothing to say.
static constexpr uint32_t BUS_SILENCE_MS = 20000;

// Restarting mid frame leaves half a telegram on the wire, which is what makes
// a drive call an accessory faulty. Here rather than with the newer protocol:
// only the handshake belongs to that one, waiting for a gap belongs to anything
// with a wire.
static constexpr uint32_t PAUSE_SETTLE_MS = 200;
// A stretch this long with nothing arriving means no telegram is in flight.
static constexpr uint32_t PAUSE_QUIET_MS = 40;

static constexpr uint32_t CMD_CONFIRM_MS = 2000;
static constexpr uint32_t CMD_GIVEUP_MS = 5000;

// The drive polls several times a second, so this covers several polls. It is
// still a block on the loop task, called from an ota on_begin, so it is kept as
// short as the handshake allows rather than as long as it might ever need. Here
// rather than with the protocol that has the handshake, because the caller
// shutting down does not know which bus it got.
static constexpr uint32_t PAUSE_ACK_WAIT_MS = 800;

// One channel, whichever bus is underneath: at the log there is one door, and
// two tags spelling the same string is how they drift apart.
extern const char *const TAG_HCI;

/** The door's state as a word, for the log, the text sensor and the tests. */
const char *hoermannStateName(HoermannState::State state);

}  // namespace hcpbridge
}  // namespace esphome
