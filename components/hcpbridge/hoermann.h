// Credits to https://github.com/Gifford47/HCPBridgeMqtt for the initial code base

#ifndef HOERMANN_H_
#define HOERMANN_H_
#include <atomic>
#include <cstdint>
#include <string>

#include "modbus_rtu.h"

#define SLAVE_ID 2
#define HCP_BAUD 57600

#ifdef CONFIG_IDF_TARGET_ESP32S3
#define PIN_TXD 17
#define PIN_RXD 18
#else
#define PIN_TXD 17 // UART 2 TXT - G17
#define PIN_RXD 16 // UART 2 RXD - G16 
#endif

static const char *TAG_HCI = "HCI-BUS";



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
    bool actuatorFlag = false;
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
    void setActuatorFlag(bool actuatorFlag);
    void clearChanged();
    void setState(State state);
    void setValid(bool isValid);
    void setSerialNumber(const std::string &serialNumber);
    void setFirmwareVersion(const std::string &firmwareVersion);

};

// A command is only done once the drive has acted on it. If nothing has changed
// by the first mark, send it once more; give up at the second and say so.
#define CMD_CONFIRM_MS 2000
#define CMD_GIVEUP_MS 5000

// Hoermann bus register blocks: the drive writes commands to 0x9C41 and its
// state to 0x9D31, and reads our answer from 0x9CB9.
#define REG_CMD_BASE 0x9C41
// Nine, not three: normal traffic writes two or three registers here, but the
// drive answers an identity request by writing its payload into the same block,
// up to 0x9C49.
#define REG_CMD_COUNT 9
#define REG_BCAST_BASE 0x9D31
#define REG_BCAST_COUNT 9
#define REG_RESP_BASE 0x9CB9
#define REG_RESP_COUNT 8

// Same values the replaced library used, so malformed frames get the same
// answers as before.
#define MODBUS_MAX_WORDS 0x007D
#define MODBUS_MAX_BITS 0x07D0
#define EX_ILLEGAL_FUNCTION 0x01
#define EX_ILLEGAL_ADDRESS 0x02
#define EX_ILLEGAL_VALUE 0x03
#define EX_SLAVE_FAILURE 0x04

// Identity exchange: which value rides along with a poll, what the drive sends
// back, and how long its answer is.
#define IDENT_REQ_SERIAL 0x05
#define IDENT_REQ_FIRMWARE 0x06
#define IDENT_SUB_SERIAL 0x0C
#define IDENT_SUB_FIRMWARE 0x0D
// Fourteen bytes then twelve, seven registers then six.
#define SERIAL_FIRST_REGS 7
#define SERIAL_SECOND_REGS 6
#define IDENT_SERIAL_LEN ((SERIAL_FIRST_REGS + SERIAL_SECOND_REGS) * 2)
#define IDENT_FIRMWARE_LEN 12
// A payload that outgrew the block would read past it instead of failing here.
static_assert(2 + SERIAL_FIRST_REGS <= REG_CMD_COUNT, "serial payload exceeds the command block");
static_assert(2 + IDENT_FIRMWARE_LEN / 2 <= REG_CMD_COUNT, "firmware payload exceeds the command block");
#define IDENT_RETRY_MS 30000
#define IDENT_MAX_ATTEMPTS 3
// Pausing: the drive is told on the next poll that we are about to go quiet,
// and confirms with its own sub code naming the address it is pausing.
#define IDENT_SUB_PAUSE_ACK 0x19
#define PAUSE_ACK_WAIT_MS 3000
// After the pause is confirmed the bus task can still be in the middle of a
// frame. Restarting into that leaves half a telegram on the wire, which is
// exactly what makes a drive treat an accessory as faulty.
#define PAUSE_SETTLE_MS 1000
// A stretch this long with nothing arriving means no telegram is in flight.
#define PAUSE_QUIET_MS 40

// The drive polls several times a second. Nothing at all for this long means the
// link is gone, not that the drive has nothing to say.
#define BUS_SILENCE_MS 20000

// A frame shape we cannot answer properly repeats as fast as the drive polls,
// so the same one is only reported this often. A different shape is immediate.
#define UNKNOWN_SHAPE_REPEAT_MS 10000

// Answer codes we put in the low byte of the second answer register.
#define RESP_STATUS 0x01
#define RESP_REQUEST 0x22
#define RESP_PAUSE 0x29
#define RESP_ACK 0xFD
#define RESP_NAK 0xFE

class HoermannGarageEngine
{
public:
    HoermannState *state = new HoermannState();

    static HoermannGarageEngine& getInstance();

    /** False when the port or the bus task could not be brought up. */
    bool setup(int8_t rx, int8_t tx, int8_t rts);
    void handleModbus();

    // Answers one complete frame; returns the response length, or 0 to stay
    // silent.
    size_t onFrame(const uint8_t *req, size_t len, uint8_t *resp);

    void setCommandValuesToRead();
    void onDoorPositonChanged(uint16_t oldVal, uint16_t val);
    void onCurrentStateChanged(uint16_t oldVal, uint16_t val);
    void onRegSevenChanged(uint16_t oldVal, uint16_t val);

    /**
     * Write on 0x9C41 , byte1: counter, byte2: command
     */
    void onCounterWrite(uint16_t val);

    /**
     * Runs once all registers of a write have landed, so a payload spread over
     * several registers can be read as a whole.
     */
    void onWriteBlockComplete(uint16_t addr, uint16_t count);

    /** Ask the drive for its serial number, then its firmware version. */
    void requestDriveIdentity();

     /**
     * Tell the drive we are about to go quiet and wait for it to confirm.
     * Returns false if it stayed silent, in which case the caller should carry
     * on anyway; a restart must not hang on a bus that is already gone.
     */
    bool announcePause(uint32_t timeoutMs);

    /** Waits out any frame still on the wire before the caller restarts. */
    void settleBeforeRestart();

    /** Prints a finished counter run. Main task: formatting has no business
     *  inside the window the drive waits for an answer in. */
    void publishCounterProbe();

    /** Drops the connected state once the drive has gone quiet for too long. */
    void checkBusSilence();

    /**
     * Moves a finished identity answer into the state. Runs in the main task:
     * the bus task only ever fills a plain buffer, so the std::string that the
     * sensors read is never written from two places at once.
     */
    void publishIdentity();

    // One register map across all three blocks, as the replaced library had.
    // Every function code goes through it, so callbacks fire no matter which
    // one wrote.
    uint16_t *regPtr(uint16_t addr);
    bool regExists(uint16_t addr) { return this->regPtr(addr) != nullptr; }
    uint16_t regGet(uint16_t addr);
    bool regWrite(uint16_t addr, uint16_t val);       // wie Reg(addr,val)
    bool regSetChecked(uint16_t addr, uint16_t val);  // wie setMultipleWords je Register
    void onRequestHook(uint8_t fc, uint16_t a1, uint16_t c1, uint16_t a2, uint16_t c2);
    void reportUnknownShape(uint8_t fc, uint16_t a1, uint16_t c1, uint16_t a2, uint16_t c2);

    /**
     * The counter byte is a delivery receipt. The drive holds its value until
     * it has been answered, so a value that fails to advance says the previous
     * answer never arrived. Running the count ourselves is what makes that
     * visible; the byte we send is the same either way, because our count is
     * the drive's own value carried forward.
     */
    void syncCounter(uint8_t counterByte);
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

    /**
     * Helper to set next Command and *not* skip Current Command before end was
     * sent. Returns false when the slot was still occupied, i.e. the command
     * was dropped.
     */
    bool setCommand(bool cond, const HoermannCommand *command);

    /**
     * Control Functions
     */
    bool stopDoor();
    bool closeDoor();
    bool openDoor();
    bool impulseDoor();
    bool halfPositionDoor();
    bool ventilationPositionDoor();
    bool turnLight(bool on);
    bool toggleLight();
    bool setPosition(int setPosition);

private:
    HoermannGarageEngine(){};
    esphome::hcpbridge::ModbusRtuServer mb;       // eigener RTU-Server, keine Fremdbibliothek
    uint16_t regCmd[REG_CMD_COUNT] = {0};         // 0x9C41, written by the drive
    uint16_t regBcast[REG_BCAST_COUNT] = {0};     // 0x9D31, drive state
    uint16_t regResp[REG_RESP_COUNT] = {0};       // 0x9CB9, unsere Antwort
    std::atomic<const HoermannCommand *> nextCommand{nullptr};  // shared with the bus task

    // Bus task only. What was sent and what the door looked like at the time,
    // so the effect can be recognised and the command sent again if there was
    // none. A repeat needs no slot of its own now that a command is one frame.
    const HoermannCommand *awaitedCommand = nullptr;
    const HoermannCommand *repeatCommand = nullptr;
    uint32_t awaitedSince = 0;
    uint8_t awaitedRepeats = 0;
    HoermannState::State stateWhenSent = HoermannState::CLOSED;
    // The drive's own state word, not our translation of it: a code we do not
    // translate must still count as the drive having reacted.
    uint16_t rawStateWhenSent = 0;

    // Measurement only, not part of the protocol. Records how the counter byte
    // in the drive's requests moves from one frame to the next, which is the
    // one fact still missing before a lost answer can be recognised from it.
    static const uint8_t COUNTER_PROBE_LEN = 24;
    uint8_t counterProbe[COUNTER_PROBE_LEN] = {0};
    uint8_t counterProbeAt = 0;
    uint8_t counterProbeRuns = 0;
    std::atomic<bool> counterProbeReady{false};
    void probeCounter(uint8_t counterByte);

    bool commandTookEffect() const;
    void checkCommandEffect();

    // Identity exchange. The drive only ever answers a request that rode along
    // with a poll, and it takes the request out of the answer slot again, so a
    // request that goes unanswered has to be repeated.
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
#endif