#include "hcp_events.h"

namespace esphome {
namespace hcpbridge {

namespace {

struct Description {
  const char *format;
  bool trouble;
  HcpArgKind kind;
};

// Indexed by HcpEvent. Two slots, always in the order the event records them.
const Description DESCRIPTIONS[] = {
    {"nothing", false, HcpArgKind::NUMBERS},
    {"unanswerable frame: read %u registers, wrote %u", true, HcpArgKind::NUMBERS},
    {"frame names register block %04x/%04x, which is not ours", true, HcpArgKind::NUMBERS},
    {"broadcast names register block %04x, which is not ours", false, HcpArgKind::NUMBERS},
    {"transfer with sub code %02x, which nothing here answers", true, HcpArgKind::NUMBERS},
    {"command %s went out as %04x", false, HcpArgKind::COMMAND},
    {"repeating %s", false, HcpArgKind::COMMAND},
    {"dropping %s, the drive did not collect it in time", true, HcpArgKind::COMMAND},
    {"dropping a repeat the door has moved past", false, HcpArgKind::NUMBERS},
    {"the drive did not act on %s", true, HcpArgKind::COMMAND},
    {"the answer never arrived, sending %s again", false, HcpArgKind::COMMAND},
    {"the drive keeps repeating its counter, giving up", true, HcpArgKind::NUMBERS},
    {"registering on the bus", false, HcpArgKind::NUMBERS},
    {"a poll carrying no command", false, HcpArgKind::NUMBERS},
    {"the drive reports state %02x", false, HcpArgKind::NUMBERS},
    {"state %02x is one nothing here maps", true, HcpArgKind::NUMBERS},
    {"relay register now %04x", false, HcpArgKind::NUMBERS},
    {"the drive never answered identity request %02x", true, HcpArgKind::NUMBERS},
    {"the link went away, dropping a command nobody is waiting on", true, HcpArgKind::NUMBERS},
    {"%s not sent, the drive is not talking to us", true, HcpArgKind::COMMAND},
    {"%s dropped, the drive has not collected the last one yet", true, HcpArgKind::COMMAND},
    {"the drive's firmware version arrived", false, HcpArgKind::NUMBERS},
    {"the drive's serial number arrived", false, HcpArgKind::NUMBERS},
};

constexpr size_t COUNT = sizeof(DESCRIPTIONS) / sizeof(DESCRIPTIONS[0]);
static_assert(COUNT == (size_t) HcpEvent::IDENTITY_SERIAL + 1, "an event was added without a sentence to go with it");

const Description &describe(HcpEvent code) {
  const size_t i = (size_t) code;
  return DESCRIPTIONS[i < COUNT ? i : 0];
}

}  // namespace

const char *hcpEventFormat(HcpEvent code) { return describe(code).format; }

HcpArgKind hcpEventArgKind(HcpEvent code) { return describe(code).kind; }

bool hcpEventIsTrouble(HcpEvent code) { return describe(code).trouble; }

}  // namespace hcpbridge
}  // namespace esphome
