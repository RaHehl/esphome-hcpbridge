// One bus, whichever protocol it speaks.
//
// The protocol used to be chosen when the image was built, which put a
// per-door option in charge of a whole-image decision: two doors could ask for
// different buses and quietly get the same one. It also meant the compiler saw
// only one codec, so the other could drift until somebody built for it - which
// is what hcp_contract.cpp existed to catch, and what the mixed-protocol
// refusal existed to catch, and what the #ifdef in hcpbridge.h existed to
// express. All three were the same workaround wearing different hats.
//
// Carrying both costs about 5 kB of a 1.8 MB partition, measured, and one
// indirect call per telegram - five a second. That buys an option that means
// what it says, a compiler that checks both codecs in every build, and two
// doors on one board that may speak to different drive generations.
#pragma once

#include <cstdint>

#include "hcp1_transport.h"
#include "hcp2_transport.h"
#include "hcp_events.h"
#include "hcp_state.h"

namespace esphome {
namespace hcpbridge {

/**
 * What the component and the entities are allowed to ask of a bus.
 *
 * Every door command answers whether it was taken, not whether the door moved:
 * a refusal means the drive was never told, and the caller says so at the door.
 */
class HcpBus {
 public:
  virtual ~HcpBus() = default;

  /** False when the port or the task could not be brought up. */
  virtual bool begin(int8_t rx, int8_t tx, int8_t rts, uint8_t uartNum) = 0;

  /**
   * Tells the drive we are about to go quiet and waits for it to acknowledge.
   * False if it stayed silent, or if this bus has no such handshake.
   */
  virtual bool announcePause(uint32_t timeoutMs) = 0;

  virtual bool openDoor(uint32_t nowMs) = 0;
  virtual bool closeDoor(uint32_t nowMs) = 0;
  virtual bool impulseDoor(uint32_t nowMs) = 0;
  virtual bool halfPositionDoor(uint32_t nowMs) = 0;
  virtual bool ventilationPositionDoor(uint32_t nowMs) = 0;
  virtual bool stopDoor(uint32_t nowMs) = 0;
  virtual bool turnLight(uint32_t nowMs, bool on) = 0;
  virtual bool setPosition(uint32_t nowMs, int percent) = 0;

  /** Both run on the component's own schedule, not on a frame. */
  virtual void checkBusSilence(uint32_t nowMs) = 0;
  virtual void publishIdentity() = 0;

  /**
   * The door as it stands, and the ring the protocol path writes into.
   *
   * Calls rather than members, so a bus cannot be built that forgot to point
   * them anywhere: leaving one out is a compile error, not a null at the first
   * telegram. They cost an indirect call on a path that runs twice a second.
   */
  virtual HoermannState *state() = 0;
  virtual HcpEventRing<32> *events() = 0;
};

/**
 * The bus a given transport makes.
 *
 * The transport owns the port and the task, the codec inside it decides what to
 * answer, and this is the seam that lets the component hold either one without
 * naming which. Instantiated for both transports in hcp_bus.cpp, so a method
 * missing from one codec is a build error everywhere rather than on the bus
 * nobody happens to build.
 */
template<typename Transport> class HcpBusOf : public HcpBus {
 public:
  HoermannState *state() override { return this->transport_.codec.state; }
  HcpEventRing<32> *events() override { return &this->transport_.codec.events; }

  bool begin(int8_t rx, int8_t tx, int8_t rts, uint8_t uartNum) override {
    return this->transport_.begin(rx, tx, rts, uartNum);
  }
  bool announcePause(uint32_t timeoutMs) override { return this->transport_.announcePause(timeoutMs); }

  bool openDoor(uint32_t nowMs) override { return this->transport_.codec.openDoor(nowMs); }
  bool closeDoor(uint32_t nowMs) override { return this->transport_.codec.closeDoor(nowMs); }
  bool impulseDoor(uint32_t nowMs) override { return this->transport_.codec.impulseDoor(nowMs); }
  bool halfPositionDoor(uint32_t nowMs) override { return this->transport_.codec.halfPositionDoor(nowMs); }
  bool ventilationPositionDoor(uint32_t nowMs) override {
    return this->transport_.codec.ventilationPositionDoor(nowMs);
  }
  bool stopDoor(uint32_t nowMs) override { return this->transport_.codec.stopDoor(nowMs); }
  bool turnLight(uint32_t nowMs, bool on) override { return this->transport_.codec.turnLight(nowMs, on); }
  bool setPosition(uint32_t nowMs, int percent) override { return this->transport_.codec.setPosition(nowMs, percent); }

  void checkBusSilence(uint32_t nowMs) override { this->transport_.codec.checkBusSilence(nowMs); }
  void publishIdentity() override { this->transport_.codec.publishIdentity(); }

 private:
  Transport transport_;
};

using Hcp2Bus = HcpBusOf<Hcp2Transport>;
using Hcp1Bus = HcpBusOf<Hcp1Transport>;

}  // namespace hcpbridge
}  // namespace esphome
