#pragma once
#include <cstdint>
namespace esphome {
uint32_t millis();
// The harness has no wall clock to wait against, and the only caller is a
// spacing delay between two frames that nothing here measures.
inline void delayMicroseconds(uint32_t) {}
}  // namespace esphome
using esphome::millis;
