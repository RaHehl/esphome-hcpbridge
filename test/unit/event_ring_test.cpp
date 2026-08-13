// The event ring under two real threads.
//
// It is the one place where the bus task and the loop task touch the same
// memory without a lock, so "it looked right" is not enough. Built with the
// thread sanitizer this fails on a race the eye would not find; built without
// it, it still catches entries arriving torn, out of order, or twice.
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <thread>
#include <vector>

#include "hcp_events.h"

using namespace esphome::hcpbridge;

namespace {

int failures = 0;

void check(bool ok, const char *what) {
  printf("  %-58s %s\n", what, ok ? "ok" : "FAILED");
  if (!ok)
    failures++;
}

// Each entry carries its own sequence number in both slots, so an entry that
// arrives half written is visible as the two disagreeing.
void producer(HcpEventRing<32> &ring, uint32_t count) {
  for (uint32_t i = 1; i <= count; i++)
    ring.push(HcpEvent::COMMAND_SENT, (uint16_t) (i & 0xFFFF), i, i);
}

}  // namespace

int main() {
  {
    HcpEventRing<32> ring;
    HcpEventEntry out[8];
    check(ring.drain(out, 8) == 0, "an empty ring hands out nothing");

    ring.push(HcpEvent::BUSSCAN, 7, 9, 11);
    const size_t n = ring.drain(out, 8);
    check(n == 1 && out[0].code == HcpEvent::BUSSCAN && out[0].a == 7 && out[0].b == 9 && out[0].atMs == 11,
          "what went in comes back unchanged");
    check(ring.drain(out, 8) == 0, "and only once");
  }

  {
    // More than it holds, with nobody draining: the count has to be honest.
    HcpEventRing<32> ring;
    for (int i = 0; i < 50; i++)
      ring.push(HcpEvent::BUSSCAN, (uint16_t) i);
    HcpEventEntry out[64];
    const size_t kept = ring.drain(out, 64);
    const uint32_t lost = ring.takeDropped();
    check(kept == 32 && lost == 18, "a full ring keeps 32 and counts the rest");
    check(out[0].a == 0 && out[31].a == 31, "the entries kept are the oldest, which is where an episode starts");
    check(ring.takeDropped() == 0, "the loss count resets once it is read");
  }

  {
    // The real arrangement: one thread appending as fast as it can, another
    // draining, neither waiting for the other.
    HcpEventRing<32> ring;
    const uint32_t total = 200000;
    std::atomic<bool> done{false};
    std::vector<HcpEventEntry> seen;
    seen.reserve(total);

    std::thread writer([&] {
      producer(ring, total);
      done.store(true, std::memory_order_release);
    });

    HcpEventEntry batch[16];
    while (!done.load(std::memory_order_acquire) || true) {
      const size_t n = ring.drain(batch, 16);
      for (size_t i = 0; i < n; i++)
        seen.push_back(batch[i]);
      if (n == 0 && done.load(std::memory_order_acquire))
        break;
    }
    writer.join();
    for (size_t n = ring.drain(batch, 16); n > 0; n = ring.drain(batch, 16))
      for (size_t i = 0; i < n; i++)
        seen.push_back(batch[i]);

    bool torn = false, backwards = false;
    uint32_t previous = 0;
    for (const HcpEventEntry &e : seen) {
      if (e.a != (uint16_t) (e.b & 0xFFFF) || e.atMs != e.b)
        torn = true;
      if (e.b <= previous)
        backwards = true;
      previous = e.b;
    }
    const uint32_t lost = ring.takeDropped();
    check(!torn, "no entry arrives half written");
    check(!backwards, "entries arrive in the order they were written");
    check(seen.size() + lost == total, "every entry is either delivered or counted");
    printf("  (%zu delivered, %u dropped while the reader was behind)\n", seen.size(), (unsigned) lost);
  }

  printf(failures ? "\nFAILED\n" : "\nall event ring checks passed\n");
  {
    // Two writers, which is what the device really does: the bus task writes
    // from the answer path while the ESPHome loop writes the reason a press
    // was refused. A single-writer ring passes every test above and still
    // loses or tears an entry here.
    HcpEventRing<32> ring;
    std::atomic<bool> go{false};
    constexpr uint32_t each = 20000;
    auto writer = [&ring, &go](uint32_t tag) {
      while (!go.load(std::memory_order_acquire)) {
      }
      for (uint32_t i = 1; i <= each; i++)
        ring.push(HcpEvent::COMMAND_SENT, (uint16_t) tag, i, tag);
    };
    std::thread a(writer, 1), b(writer, 2);
    std::vector<HcpEventEntry> seen;
    HcpEventEntry batch[16];
    go.store(true, std::memory_order_release);
    std::atomic<bool> writing{true};
    std::thread stopper([&a, &b, &writing]() {
      a.join();
      b.join();
      writing.store(false, std::memory_order_release);
    });
    for (;;) {
      const size_t n = ring.drain(batch, 16);
      for (size_t i = 0; i < n; i++)
        seen.push_back(batch[i]);
      if (n == 0 && !writing.load(std::memory_order_acquire))
        break;
    }
    stopper.join();
    for (size_t n = ring.drain(batch, 16); n > 0; n = ring.drain(batch, 16))
      for (size_t i = 0; i < n; i++)
        seen.push_back(batch[i]);

    bool torn = false;
    uint32_t last[3] = {0, 0, 0};
    bool backwards = false;
    for (const HcpEventEntry &e : seen) {
      if (e.a != (uint16_t) e.atMs || e.a < 1 || e.a > 2) {
        torn = true;
        continue;
      }
      if (e.b <= last[e.a])
        backwards = true;
      last[e.a] = e.b;
    }
    const uint32_t lost = ring.takeDropped();
    check(!torn, "with two writers, no entry arrives half written or mixed");
    check(!backwards, "each writer's entries keep their own order");
    check(seen.size() + lost == 2 * each, "with two writers, nothing vanishes uncounted");
    printf("  (%zu delivered, %u dropped, from two writers at once)\n", seen.size(), lost);
  }

  return failures ? 1 : 0;
}
