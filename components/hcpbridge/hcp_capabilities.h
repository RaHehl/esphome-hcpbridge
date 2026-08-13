// What a given bus protocol can and cannot do.
//
// The command lifecycle is the same shape on both Hoermann buses, but the
// limits inside it are not, and one of them is a safety question rather than a
// preference: HCP2 tells us whether an answer arrived, HCP1 does not. Re-sending
// a command on a bus with no delivery receipt is not a retry, it is a second key
// press. Writing that as a number here, read by the one place that decides, is
// what keeps it from being a rule somebody has to remember when a second
// protocol arrives.
#pragma once

#include <cstdint>

namespace esphome {
namespace hcpbridge {

struct HcpCapabilities {
  // How often a command may go out again after the bus has shown the answer
  // never arrived. Zero where the protocol cannot show that.
  uint8_t lostAnswerRepeats;
  // How often the drive may be asked again after it took the command and did
  // nothing with it.
  uint8_t unactedRepeats;
  // Whether the drive answers a request for its serial number and firmware.
  bool hasIdentity;
  // Whether the drive acknowledges an accessory going quiet.
  bool hasPause;
  // Whether the bus carries a door position rather than only open and shut.
  bool hasPosition;
};

}  // namespace hcpbridge
}  // namespace esphome
