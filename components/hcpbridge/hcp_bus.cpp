#include <type_traits>

#include "hcp_bus.h"

namespace esphome {
namespace hcpbridge {

// Both, in every build. A codec that loses a method the component needs is a
// build error here, on the bus nobody happens to be building as well as on the
// one they are. This is the job hcp_contract.cpp used to do by hand.
template class HcpBusOf<Hcp2Transport>;
template class HcpBusOf<Hcp1Transport>;

// Instantiating an abstract class is allowed; only building one is not. Without
// this a missing override would compile here and fail later at the new in
// hcpbridge.cpp, which no desk check builds.
static_assert(!std::is_abstract<Hcp2Bus>::value, "the newer bus leaves something in HcpBus unanswered");
static_assert(!std::is_abstract<Hcp1Bus>::value, "the older bus leaves something in HcpBus unanswered");

}  // namespace hcpbridge
}  // namespace esphome
