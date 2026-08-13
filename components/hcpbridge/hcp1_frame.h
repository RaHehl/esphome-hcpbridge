// Frames on the older Hoermann bus, the one a SupraMatic E3 speaks.
//
// It shares nothing with the newer bus below the door itself: different baud
// rate, different framing, a different checksum, and a length and counter
// packed into one byte. What it does share is what a door can be asked to do,
// which is why this is a second way of writing frames rather than a second
// component.
//
// Two sources agree on every value here: a disassembly of an unrelated
// accessory's firmware, and a published description written from a bus recorded
// at a real E3. They were derived independently and by different means, which
// is worth more than either alone - but no drive here speaks this, so what
// follows is checked against documented example frames and a simulator, and
// against no actual door.
#pragma once

#include <cstddef>
#include <cstdint>

namespace esphome {
namespace hcpbridge {

// Who is on the bus.
static constexpr uint8_t HCP1_ADDR_BROADCAST = 0x00;
static constexpr uint8_t HCP1_ADDR_SELF = 0x28;    // what a UAP1 answers to
static constexpr uint8_t HCP1_ADDR_MASTER = 0x80;  // the drive

// What the drive asks, and what we answer with.
static constexpr uint8_t HCP1_CMD_SCAN = 0x01;
static constexpr uint8_t HCP1_CMD_STATUS_REQUEST = 0x20;
static constexpr uint8_t HCP1_CMD_STATUS_RESPONSE = 0x29;
static constexpr uint8_t HCP1_TYPE_UAP1 = 0x14;

// The drive's state, in the first broadcast byte.
static constexpr uint8_t HCP1_BC_OPEN = 1 << 0;
static constexpr uint8_t HCP1_BC_CLOSED = 1 << 1;
static constexpr uint8_t HCP1_BC_RELAY = 1 << 2;
static constexpr uint8_t HCP1_BC_LIGHT = 1 << 3;
static constexpr uint8_t HCP1_BC_ERROR = 1 << 4;
static constexpr uint8_t HCP1_BC_CLOSING = 1 << 5;  // direction, only while moving
static constexpr uint8_t HCP1_BC_MOVING = 1 << 6;
static constexpr uint8_t HCP1_BC_VENTING = 1 << 7;
// Second broadcast byte. The pre-warning light is not acted on here; it is
// written down because reading the byte and not knowing what a bit means is
// worse than knowing and leaving it.
static constexpr uint8_t HCP1_BC2_PREWARN = 1 << 0;

// What we ask of the drive, all five in one byte.
static constexpr uint8_t HCP1_DO_OPEN = 1 << 0;
static constexpr uint8_t HCP1_DO_CLOSE = 1 << 1;
static constexpr uint8_t HCP1_DO_IMPULSE = 1 << 2;
static constexpr uint8_t HCP1_DO_LIGHT = 1 << 3;
static constexpr uint8_t HCP1_DO_VENT = 1 << 4;

// The emergency stop input a UAP1 has. It reads as healthy when set, and the
// drive halts with a fault on its display when it is not, so every answer
// carries it whether or not the input exists on this hardware.
static constexpr uint8_t HCP1_S0_HEALTHY = 1 << 4;

static constexpr size_t HCP1_MAX_FRAME = 16;

/** Poly 0x07, MSB first, starting at 0xF3 as the drive expects. */
uint8_t hcp1Crc8(const uint8_t *data, size_t len);

struct Hcp1Frame {
  uint8_t address;
  uint8_t counter;  // 0..15
  uint8_t length;   // data bytes, not counting address, header or checksum
  const uint8_t *data;
};

/**
 * Reads one frame, checksum included.
 *
 * The sync break that precedes it on the wire is the transport's business; what
 * arrives here starts at the address byte.
 */
bool hcp1Parse(const uint8_t *buf, size_t len, Hcp1Frame *out);

/** Builds a frame and returns its length, or 0 if it would not fit. */
size_t hcp1Build(uint8_t *buf, size_t cap, uint8_t address, uint8_t counter, const uint8_t *data, uint8_t length);

/** The counter to answer a frame with. */
inline uint8_t hcp1NextCounter(uint8_t received) { return (uint8_t) ((received + 1) & 0x0F); }

}  // namespace hcpbridge
}  // namespace esphome
