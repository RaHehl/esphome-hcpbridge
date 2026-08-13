// Frames on the newer Hoermann bus, which is Modbus RTU underneath.
//
// The counterpart to hcp1_frame, and the reason both exist: a checksum belongs
// with whoever decides what the frame means, not with whoever collected the
// bytes. Kept here, the codec can be handed what actually came off the wire and
// asked whether it would answer it - checksum and all - without a port
// anywhere near the test.
#pragma once

#include <cstddef>
#include <cstdint>

namespace esphome {
namespace hcpbridge {

// The address this accessory answers to. A fact about the protocol, like the
// addresses on the older bus, not about the port it arrives on.
static constexpr uint8_t SLAVE_ID = 2;

// The longest anything on this bus can be. Both the port's buffers and the
// answers built against it are sized from here.
static constexpr size_t HCP2_MAX_FRAME = 256;

/** Poly 0xA001, the reversed form Modbus RTU uses. */
uint16_t hcp2Crc16(const uint8_t *data, size_t len);

/**
 * Checks a frame's checksum and hands back the length without it.
 *
 * Returns 0 for anything too short to hold one, or whose checksum disagrees.
 */
size_t hcp2Validate(const uint8_t *buf, size_t len);

/** Appends the checksum to a response and returns its full length. */
size_t hcp2Finish(uint8_t *buf, size_t len, size_t cap);

}  // namespace hcpbridge
}  // namespace esphome
