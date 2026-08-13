#pragma once
// Stub: the harness only needs the watchdog feed to be a no-op.
namespace esphome {
struct ApplicationStub {
  void feed_wdt() {}
};
static ApplicationStub App;
}  // namespace esphome
