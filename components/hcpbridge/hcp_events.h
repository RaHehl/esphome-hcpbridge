// What the protocol path has to say, recorded rather than printed.
//
// A log call between a frame and the answer to it costs milliseconds the drive
// is counting: at a low console baud rate a single line can outlast the window
// it answers in, and a drive that stops getting answers drops the accessory.
// That is not a rule anybody can be relied on to remember, so the path has no
// way to print at all - it records a number, and somebody else turns numbers
// into sentences later, off the deadline.
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

namespace esphome {
namespace hcpbridge {

enum class HcpEvent : uint16_t {
  NONE = 0,
  // Frames
  UNANSWERABLE_FRAME,     // a: read count, b: write count
  WRONG_REGISTER_BLOCK,   // a: read address, b: write address
  WRONG_BROADCAST_BLOCK,  // a: address
  UNKNOWN_TRANSFER_SUB,   // a: sub code
  // Commands
  COMMAND_SENT,      // a: command, b: the two registers it went out as
  COMMAND_REPEATED,  // a: command
  COMMAND_STALE,     // a: command
  COMMAND_REPEAT_DROPPED,
  COMMAND_UNANSWERED,  // a: command
  COMMAND_RESENT,      // a: command
  COMMAND_RESEND_GAVE_UP,
  // The drive
  BUSSCAN,
  EMPTY_COMMAND,
  STATE_CHANGED,        // a: the drive's state word
  UNKNOWN_STATE,        // a: the state word nothing maps
  RELAY_REGISTER,       // a: the register's value
  IDENTITY_UNANSWERED,  // a: which request
  // Refusals the caller is told about by the return value as well; recorded so
  // the reason survives past the moment somebody pressed something.
  LINK_LOST_COMMAND_DROPPED,
  COMMAND_REFUSED_NO_LINK,    // a: command
  COMMAND_REFUSED_SLOT_BUSY,  // a: command
  IDENTITY_FIRMWARE,
  IDENTITY_SERIAL,
};

struct HcpEventEntry {
  HcpEvent code;
  uint16_t a;
  uint32_t b;
  uint32_t atMs;
};

/**
 * Several writers, one reader, no waiting.
 *
 * The bus task writes from the answer path, and the ESPHome loop writes too:
 * a key press it had to refuse says here why, and only it knows whether the
 * link was down or the previous command had not been fetched yet. Both sides
 * really do run at once - on the dual-core parts the bus task is pinned to the
 * other core - so a single-writer ring would lose or tear an entry when a
 * press lands during a telegram.
 *
 * Nothing here waits, which is the whole point: a ring that could block would
 * be the same problem as the log call it replaces. When it fills up the oldest
 * entries are the ones kept and the losses are counted, because the entries
 * that explain how something started are worth more than the ones repeating
 * that it is still going.
 *
 * Each slot carries a sequence number, so a writer claims its place before it
 * fills it and only then hands it over. That is what lets two writers share the
 * ring without a lock: the reader takes a slot only once its sequence says the
 * entry is complete.
 */
template<size_t N> class HcpEventRing {
  static_assert((N & (N - 1)) == 0, "the capacity has to be a power of two");

 public:
  HcpEventRing() {
    for (size_t i = 0; i < N; i++)
      this->slots_[i].seq.store((uint32_t) i, std::memory_order_relaxed);
  }

  void push(HcpEvent code, uint16_t a = 0, uint32_t b = 0, uint32_t atMs = 0) {
    uint32_t pos = this->write_.load(std::memory_order_relaxed);
    Slot *slot;
    for (;;) {
      slot = &this->slots_[pos & (N - 1)];
      const uint32_t seq = slot->seq.load(std::memory_order_acquire);
      const int32_t free = (int32_t) (seq - pos);
      if (free == 0) {
        // Ours if nobody else takes it first.
        if (this->write_.compare_exchange_weak(pos, pos + 1, std::memory_order_relaxed, std::memory_order_relaxed))
          break;
      } else if (free < 0) {
        this->dropped_.fetch_add(1, std::memory_order_relaxed);
        return;
      } else {
        pos = this->write_.load(std::memory_order_relaxed);
      }
    }
    slot->entry = HcpEventEntry{code, a, b, atMs};
    // The entry has to be there before the reader is told it is.
    slot->seq.store(pos + 1, std::memory_order_release);
  }

  /** How many were taken; call until it returns less than `max`. */
  size_t drain(HcpEventEntry *out, size_t max) {
    size_t n = 0;
    while (n < max) {
      Slot *slot = &this->slots_[this->read_ & (N - 1)];
      const uint32_t seq = slot->seq.load(std::memory_order_acquire);
      // Written and handed over, rather than merely claimed.
      if ((int32_t) (seq - (this->read_ + 1)) != 0)
        break;
      out[n++] = slot->entry;
      slot->seq.store(this->read_ + (uint32_t) N, std::memory_order_release);
      this->read_++;
    }
    return n;
  }

  /** Entries that never fit, since the last time this was asked. */
  uint32_t takeDropped() { return this->dropped_.exchange(0, std::memory_order_relaxed); }

 private:
  struct Slot {
    std::atomic<uint32_t> seq;
    HcpEventEntry entry;
  };
  Slot slots_[N];
  std::atomic<uint32_t> write_{0};
  // The reader is alone, so this needs no atomic of its own.
  uint32_t read_{0};
  std::atomic<uint32_t> dropped_{0};
};

/** Whether the first slot holds a command or a plain number. */
enum class HcpArgKind : uint8_t {
  NUMBERS,
  COMMAND,
};

/** The sentence this event stands for, with two slots for its arguments. */
const char *hcpEventFormat(HcpEvent code);

/** Which of the two printf shapes the sentence expects. */
HcpArgKind hcpEventArgKind(HcpEvent code);

/** Whether it deserves a warning rather than a note. */
bool hcpEventIsTrouble(HcpEvent code);

}  // namespace hcpbridge
}  // namespace esphome
