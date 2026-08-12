// Credits to https://github.com/Gifford47/HCPBridgeMqtt for the initial code base

#pragma once

#include <atomic>
#include <cstdint>
#include <string>

#include "modbus_rtu.h"

namespace esphome {
namespace hcpbridge {

// Constants, not macros: a macro called SLAVE_ID or PIN_TXD collides with
// anything else in the build that happens to use the same word.
static constexpr uint8_t SLAVE_ID = 2;
static constexpr uint32_t HCP_BAUD = 57600;
// Above the application, below the radio stack: the answer to a frame is short
// and time-critical, but starving Wi-Fi to deliver it would trade one problem
// for another. The task blocks on the driver's queue, so it only runs when a
// frame has actually arrived.
static constexpr unsigned HCP_TASK_PRIO = 19;

#ifdef CONFIG_IDF_TARGET_ESP32S3
static constexpr int8_t PIN_TXD = 17;
static constexpr int8_t PIN_RXD = 18;
#else
static constexpr int8_t PIN_TXD = 17;  // UART 2 TXD, G17
static constexpr int8_t PIN_RXD = 16;  // UART 2 RXD, G16
#endif

extern const char *const TAG_HCI;



class HoermannCommand
{
public:
    static const HoermannCommand STARTOPENDOOR;
    static const HoermannCommand STARTCLOSEDOOR;
    static const HoermannCommand STARTIMPULSE;
    static const HoermannCommand STARTOPENDOORHALF;
    static const HoermannCommand STARTVENTPOSITION;
    static const HoermannCommand LAMPON;
    static const HoermannCommand LAMPOFF;
    static const HoermannCommand WAITING;

public:
    uint16_t commandRegPlus2Value;
    uint16_t commandEndPlus2Value;
    uint16_t commandRegPlus3Value;
    uint16_t commandEndPlus3Value;

private:
    HoermannCommand(
        uint16_t commandRegPlus2Value,
        uint16_t commandEndPlus2Value,
        uint16_t commandRegPlus3Value,
        uint16_t commandEndPlus3Value)
    {
        this->commandRegPlus2Value = commandRegPlus2Value;
        this->commandEndPlus2Value = commandEndPlus2Value;
        this->commandRegPlus3Value = commandRegPlus3Value;
        this->commandEndPlus3Value = commandEndPlus3Value;
    }
};

class HoermannState
{
public:
    enum State
    {
        OPEN,
        OPENING,
        CLOSED,
        CLOSING,
        HALFOPEN,
        MOVE_VENTING,
        VENT,
        MOVE_HALF,
        STOPPED
    };
    // All position values are from 0 to 100
    float targetPosition = 0;
    float currentPosition = 0;
    bool lightOn = false;
    bool relayOn = false;
    bool actuatorError = false;
    State state = CLOSED;
    std::atomic<bool> changed{false};
    float gotoPosition = 0.0f;
    std::atomic<bool> valid{false};
    // Empty until the drive has answered the matching request.
    std::string serialNumber;
    std::string firmwareVersion;

    void setTargetPosition(float targetPosition);
    void setGotoPosition(float setPosition);
    void setCurrentPosition(float currentPosition);
    void setLigthOn(bool lightOn);
    void setRelayOn(bool relayOn);
    void setActuatorError(bool actuatorError);
    void clearChanged();
    void setState(State state);
    void setValid(bool isValid);
    void setSerialNumber(const std::string &serialNumber);
    void setFirmwareVersion(const std::string &firmwareVersion);

};

// A command is only done once the drive has acted on it. If nothing has changed
// by the first mark, send it once more; give up at the second and say so.
static constexpr uint32_t CMD_CONFIRM_MS = 2000;
static constexpr uint32_t CMD_GIVEUP_MS = 5000;
// A press the drive never fetched is not delayed, it is stale: nobody is
// standing there any more.
static constexpr uint32_t CMD_STALE_MS = 2000;
// How often one press may be re-sent after a lost answer.
static constexpr uint8_t MAX_LOST_REPEATS = 3;

// Hoermann bus register blocks: the drive writes commands to 0x9C41 and its
// state to 0x9D31, and reads our answer from 0x9CB9.
static constexpr uint16_t REG_CMD_BASE = 0x9C41;
// Nine, not three: the drive answers an identity request by writing its payload
// into this same block.
static constexpr uint32_t REG_CMD_COUNT = 9;
static constexpr uint16_t REG_BCAST_BASE = 0x9D31;
static constexpr uint32_t REG_BCAST_COUNT = 9;
static constexpr uint16_t REG_RESP_BASE = 0x9CB9;
static constexpr uint32_t REG_RESP_COUNT = 8;

// The largest register count a read may ask for; the response has to fit the
// transmit buffer.
static constexpr uint16_t MODBUS_MAX_WORDS = 0x007D;

// Identity exchange: which value rides along with a poll, what the drive sends
// back, and how long its answer is.
static constexpr uint16_t IDENT_REQ_SERIAL = 0x05;
static constexpr uint16_t IDENT_REQ_FIRMWARE = 0x06;
static constexpr uint16_t IDENT_SUB_SERIAL = 0x0C;
static constexpr uint16_t IDENT_SUB_FIRMWARE = 0x0D;
// Fourteen bytes then twelve, seven registers then six.
static constexpr uint32_t SERIAL_FIRST_REGS = 7;
static constexpr uint32_t SERIAL_SECOND_REGS = 6;
static constexpr uint8_t IDENT_SERIAL_LEN = (SERIAL_FIRST_REGS + SERIAL_SECOND_REGS) * 2;
static constexpr uint32_t IDENT_FIRMWARE_LEN = 12;
// A payload that outgrew the block would read past it instead of failing here.
static_assert(2 + SERIAL_FIRST_REGS <= REG_CMD_COUNT, "serial payload exceeds the command block");
static_assert(2 + IDENT_FIRMWARE_LEN / 2 <= REG_CMD_COUNT, "firmware payload exceeds the command block");
static constexpr uint32_t IDENT_RETRY_MS = 30000;
static constexpr uint32_t IDENT_MAX_ATTEMPTS = 3;
// Pausing: the drive is told on the next poll that we are about to go quiet,
// and confirms with its own sub code naming the address it is pausing.
static constexpr uint16_t IDENT_SUB_PAUSE_ACK = 0x19;
// The drive polls several times a second, so this covers several polls. It is
// still a block on the loop task, called from an ota on_begin, so it is kept as
// short as the handshake allows rather than as long as it might ever need.
static constexpr uint32_t PAUSE_ACK_WAIT_MS = 800;
// Restarting mid frame leaves half a telegram on the wire, which is what makes
// a drive call an accessory faulty.
static constexpr uint32_t PAUSE_SETTLE_MS = 200;
// A stretch this long with nothing arriving means no telegram is in flight.
static constexpr uint32_t PAUSE_QUIET_MS = 40;

// The drive polls several times a second. Nothing at all for this long means the
// link is gone, not that the drive has nothing to say.
static constexpr uint32_t BUS_SILENCE_MS = 20000;

// A frame shape we cannot answer properly repeats as fast as the drive polls,
// so the same one is only reported this often. A different shape is immediate.
static constexpr uint32_t UNKNOWN_SHAPE_REPEAT_MS = 10000;

// What the drive is asking for, in the low byte of the first written register.
static constexpr uint16_t CMD_CONNECT = 0x02;
static constexpr uint16_t CMD_STATUS = 0x03;
static constexpr uint16_t CMD_TRANSFER = 0x04;

// Answer codes we put in the low byte of the second answer register.
static constexpr uint16_t RESP_STATUS = 0x01;
static constexpr uint16_t RESP_REQUEST = 0x22;
static constexpr uint16_t RESP_PAUSE = 0x29;
static constexpr uint16_t RESP_ACK = 0xFD;
static constexpr uint16_t RESP_NAK = 0xFE;

class HoermannGarageEngine
{
public:
    HoermannState *state = new HoermannState();

    static HoermannGarageEngine& getInstance();

    /** False when the port or the bus task could not be brought up. */
    bool setup(int8_t rx, int8_t tx, int8_t rts, uint8_t uartNum);
    void handleModbus();

    // Answers one complete frame; returns the response length, or 0 to stay
    // silent.
    size_t onFrame(const uint8_t *req, size_t len, uint8_t *resp);

    void setCommandValuesToRead();
    void onDoorPositonChanged(uint16_t oldVal, uint16_t val);
    void onCurrentStateChanged(uint16_t oldVal, uint16_t val);
    void onRegSevenChanged(uint16_t oldVal, uint16_t val);

    // First register of 0x9C41: high byte counter, low byte command.
    void onCounterWrite(uint16_t val);
    void checkGotoTarget();
    bool gotoCheckPending = false;

    // After the whole write has landed, so a payload spread over several
    // registers can be read as one.
    void onWriteBlockComplete(uint16_t addr, uint16_t count);

    /** Ask the drive for its serial number, then its firmware version. */
    void requestDriveIdentity();

    /** False if the drive stayed silent. Carry on anyway: a restart must not
     *  hang on a bus that is already gone. */
    bool announcePause(uint32_t timeoutMs);

    /** Waits out any frame still on the wire before the caller restarts. */
    void settleBeforeRestart();


    /** Drops the connected state once the drive has gone quiet for too long. */
    void checkBusSilence();

    /** Main task only, so the string the sensors read has one writer. */
    void publishIdentity();

    // One map for all three blocks, so callbacks fire whichever code wrote.
    uint16_t *regPtr(uint16_t addr);
    bool regExists(uint16_t addr) { return this->regPtr(addr) != nullptr; }
    uint16_t regGet(uint16_t addr);
    bool regWrite(uint16_t addr, uint16_t val);
    bool regSetChecked(uint16_t addr, uint16_t val);
    void onRequestHook(uint8_t fc, uint16_t a1, uint16_t c1, uint16_t a2, uint16_t c2,
                       uint8_t command = 0);
    void reportUnknownShape(uint8_t fc, uint16_t a1, uint16_t c1, uint16_t a2, uint16_t c2);

    /** The counter is a delivery receipt: the drive holds its value until it
     *  has been answered, so one that fails to advance means ours was lost. */
    void syncCounter(uint8_t counterByte, uint8_t command);
    void advanceCounter();
    void rearmLostCommand();

    uint8_t txCounter = 0;      // goes into the answer
    uint8_t txCounterPrev = 0;  // the one to fall back on when an answer is lost
    bool txCounterValid = false;
    // What the last answer carried, so it can be put back if that answer was
    // lost. Mirrors the snapshot the count is checked against.
    const HoermannCommand *lastSentCommand = nullptr;

    // Last frame shape we had no branch for, so a repeat can be told from a new
    // one. Only ever touched from the bus task.
    bool unknownSeen = false;
    uint8_t unknownFc = 0;
    uint16_t unknownA1 = 0, unknownC1 = 0, unknownA2 = 0, unknownC2 = 0;
    uint32_t unknownLoggedOn = 0;

    /** False when the slot was occupied and the command was dropped. */
    bool setCommand(bool cond, const HoermannCommand *command);

    bool stopDoor();
    bool closeDoor();
    bool openDoor();
    bool impulseDoor();
    bool halfPositionDoor();
    bool ventilationPositionDoor();
    bool turnLight(bool on);
    bool setPosition(int setPosition);

private:
    HoermannGarageEngine(){};
    ModbusRtuServer mb;       // our own RTU server, no third-party library
    uint16_t regCmd[REG_CMD_COUNT] = {0};         // 0x9C41, written by the drive
    uint16_t regBcast[REG_BCAST_COUNT] = {0};     // 0x9D31, drive state
    uint16_t regResp[REG_RESP_COUNT] = {0};       // 0x9CB9, what we answer with
    std::atomic<const HoermannCommand *> nextCommand{nullptr};
    std::atomic<uint32_t> nextCommandOn{0};
    // Set by a stop on the main task, honoured by the bus task.
    std::atomic<bool> cancelWatch{false};  // shared with the bus task

    // Bus task only. What was sent and how the door looked then, so a missing
    // effect can be told apart from a delivered command.
    const HoermannCommand *awaitedCommand = nullptr;
    const HoermannCommand *repeatCommand = nullptr;
    uint32_t awaitedSince = 0;
    uint8_t awaitedRepeats = 0;
    // The drive's own state word, not our translation of it: a code we do not
    // translate must still count as the drive having reacted.
    uint8_t repeatsSent = 0;         // re-sends of the current command after a loss
    uint32_t stateWrites = 0;        // every change of the drive's state word
    uint32_t stateWritesWhenSent = 0;


    bool commandTookEffect() const;
    bool commandReached(const HoermannCommand *cmd) const;
    bool repeatStillMakesSense() const;
    void checkCommandEffect();

    // The drive only answers a request that rode along with a poll, and takes
    // it back out of the slot, so an unanswered one has to be repeated.
    uint8_t identityWanted = 0;      // 0 = nothing, else IDENT_REQ_*
    uint8_t identityAttempts = 0;
    uint32_t identityAskedOn = 0;
    // A separate flag, not identityAskedOn == 0: millis() really is 0 for the
    // first millisecond and wraps back through it every 49.7 days.
    bool identityAsked = false;
    bool identityStarted = false;

    // Handed from the bus task to the main task: buffer first, flag second.
    char identSerial[IDENT_SERIAL_LEN + 1] = {0};
    char identFirmware[IDENT_FIRMWARE_LEN + 1] = {0};
    std::atomic<bool> identSerialReady{false};
    std::atomic<bool> identFirmwareReady{false};
    uint8_t serialBuf[IDENT_SERIAL_LEN] = {0};
    bool serialFirstHalfSeen = false;

    // Set from the main task, read and answered by the bus task.
    std::atomic<bool> pauseRequested{false};
    std::atomic<bool> pauseConfirmed{false};
    // Written by the bus task on every frame, read by the main task.
    std::atomic<uint32_t> lastFrameOn{0};

    void copyRegsToBytes(uint8_t firstReg, uint8_t regCount, uint8_t *out);
    bool onIdentityData(uint8_t counterByte, uint8_t subCode, uint16_t count);
    void answerTransfer(uint8_t counterByte, uint8_t code);
    void armIdentityRequest(uint8_t request);
};

}  // namespace hcpbridge
}  // namespace esphome
