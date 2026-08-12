// Credits to https://github.com/Gifford47/HCPBridgeMqtt for the initial code base

#include <cstring>

#include "hoermann.h"

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace esphome {
namespace hcpbridge {

const char *const TAG_HCI = "HCI-BUS";

// Only the second value of each pair is ever sent; see activeCommandValues.
const HoermannCommand HoermannCommand::STARTOPENDOOR = HoermannCommand(0x0210, 0x0110, 0x0000, 0x0000); // Typo 0201
const HoermannCommand HoermannCommand::STARTCLOSEDOOR = HoermannCommand(0x0220, 0x0120, 0x0000, 0x0000);
const HoermannCommand HoermannCommand::STARTIMPULSE = HoermannCommand(0x0240, 0x0140, 0x0000, 0x0000);
const HoermannCommand HoermannCommand::STARTOPENDOORHALF = HoermannCommand(0x0200, 0x0100, 0x0400, 0x0400);
const HoermannCommand HoermannCommand::STARTVENTPOSITION = HoermannCommand(0x0200, 0x0100, 0x4000, 0x4000);
// One command per direction. A toggle can only mean "the other one", so a
// repeat undoes it and "on" is not something that can be asked for.
const HoermannCommand HoermannCommand::LAMPON = HoermannCommand(0x0880, 0x0880, 0x0000, 0x0000);
const HoermannCommand HoermannCommand::LAMPOFF = HoermannCommand(0x0800, 0x0800, 0x0100, 0x0100);
const HoermannCommand HoermannCommand::WAITING = HoermannCommand(0x0000, 0x0000, 0x0000, 0x0000);

TaskHandle_t modBusTask;
void modbusServeTask(void *parameter);

void modbusServeTask(void *parameter)
{
  while (true)
  {
    HoermannGarageEngine::getInstance().handleModbus();
  }
  vTaskDelete(NULL);
}

HoermannGarageEngine &HoermannGarageEngine::getInstance()
{
  static HoermannGarageEngine instance;
  return instance;
}

bool HoermannGarageEngine::setup(int8_t rx, int8_t tx, int8_t rts)
{
  this->mb.set_handler([this](const uint8_t *req, size_t len, uint8_t *resp) -> size_t
                       { return this->onFrame(req, len, resp); });
  if (!this->mb.begin(UART_NUM_2, rx, tx, rts, HCP_BAUD, SLAVE_ID))
  {
    // No port, no task: at top priority it would spin, because poll() returns
    // immediately.
    ESP_LOGE(TAG_HCI, "serial setup failed, bus task not started");
    return false;
  }

  xTaskCreatePinnedToCore(
      modbusServeTask,          /* Function to implement the task */
      "ModBusTask",             /* Name of the task */
      8192,                     /* Stack in bytes; the protocol call chain needs the room */
      NULL,                     /* Task input parameter */
      configMAX_PRIORITIES - 1, /* Priority */
      &modBusTask,              /* Task handle */
      1);                       /* Core */
  if (modBusTask == nullptr)
  {
    ESP_LOGE(TAG_HCI, "bus task could not be created");
    return false;
  }
  return true;
}

void HoermannGarageEngine::handleModbus()
{
  this->mb.poll();
}

uint16_t *HoermannGarageEngine::regPtr(uint16_t addr)
{
  if (addr >= REG_CMD_BASE && addr < REG_CMD_BASE + REG_CMD_COUNT)
    return &this->regCmd[addr - REG_CMD_BASE];
  if (addr >= REG_BCAST_BASE && addr < REG_BCAST_BASE + REG_BCAST_COUNT)
    return &this->regBcast[addr - REG_BCAST_BASE];
  if (addr >= REG_RESP_BASE && addr < REG_RESP_BASE + REG_RESP_COUNT)
    return &this->regResp[addr - REG_RESP_BASE];
  return nullptr;
}

uint16_t HoermannGarageEngine::regGet(uint16_t addr)
{
  const uint16_t *p = this->regPtr(addr);
  return p != nullptr ? *p : 0x0000;  // absent registers read as 0
}

/** False when there is no such register. */
bool HoermannGarageEngine::regWrite(uint16_t addr, uint16_t val)
{
  uint16_t *p = this->regPtr(addr);
  if (p == nullptr)
    return false;
  const uint16_t old = *p;
  if (addr == REG_CMD_BASE + 0)
    this->onCounterWrite(val);
  else if (addr == REG_BCAST_BASE + 1)
    this->onDoorPositonChanged(old, val);
  else if (addr == REG_BCAST_BASE + 2)
  {
    if (old != val)
      this->stateWrites++;
    this->onCurrentStateChanged(old, val);
  }
  else if (addr == REG_BCAST_BASE + 6)
    this->onRegSevenChanged(old, val);
  *p = val;
  return true;
}

/**
 * Write one register and read it back. A register that does not exist accepts
 * nothing and reads as 0, so the write only counts as done when the value was
 * 0 to begin with.
 */
bool HoermannGarageEngine::regSetChecked(uint16_t addr, uint16_t val)
{
  this->regWrite(addr, val);
  return this->regGet(addr) == val;
}

/**
 * A frame we have no branch for. It is answered with a plain status and an
 * empty payload, so the shape is the only thing worth knowing about it: it is
 * what a missing branch would have to be written against. Repeats are
 * throttled, but a shape not seen before is always reported, because the
 * interesting ones arrive in bursts while the drive takes an accessory back
 * onto the bus.
 */
void HoermannGarageEngine::reportUnknownShape(uint8_t fc, uint16_t a1, uint16_t c1, uint16_t a2,
                                              uint16_t c2)
{
  const uint32_t now = esphome::millis();
  const bool sameShape = this->unknownSeen && this->unknownFc == fc && this->unknownA1 == a1 &&
                         this->unknownC1 == c1 && this->unknownA2 == a2 && this->unknownC2 == c2;
  // Subtract, never add: adding overflows when millis() wraps.
  if (sameShape && (now - this->unknownLoggedOn) < UNKNOWN_SHAPE_REPEAT_MS)
    return;
  this->unknownSeen = true;
  this->unknownFc = fc;
  this->unknownA1 = a1;
  this->unknownC1 = c1;
  this->unknownA2 = a2;
  this->unknownC2 = c2;
  this->unknownLoggedOn = now;
  ESP_LOGW(TAG_HCI, "unanswerable frame: read %u registers, wrote %u", (unsigned)c1,
           (unsigned)c2);
}

/**
 * The former onRequest callback. The library ran it for every function code it
 * knew, before any validation.
 */
void HoermannGarageEngine::onRequestHook(uint8_t fc, uint16_t a1, uint16_t c1, uint16_t a2,
                                        uint16_t c2, uint8_t command)
{
  // The command byte decides, not the register counts alone: a transfer written
  // as two registers used to land here and take a queued press with it, and the
  // answer that would have carried it is overwritten later anyway.
  if (fc == 0x17 && a2 == REG_CMD_BASE && c2 == 0x02 && a1 == REG_RESP_BASE && c1 == 0x08 &&
      command == CMD_STATUS)
  {
    this->regResp[0] = 0x0000;
    this->regResp[1] = RESP_STATUS;
    this->regResp[4] = 0x0000;
    this->regResp[5] = 0x0000;
    this->regResp[6] = 0x0000;
    this->regResp[7] = 0x0000;

    // Checked before the command slot is read, so a press that is still waiting
    // stays waiting instead of being consumed by a frame that does not carry it.
    if (this->pauseRequested.load())
    {
      this->regResp[1] = RESP_PAUSE;
      this->regResp[2] = SLAVE_ID;
      this->regResp[3] = 0x0000;
      // This task owns them. A repeat inside the settle window would start the
      // door a second before we disappear.
      this->awaitedCommand = nullptr;
      this->repeatCommand = nullptr;
      this->state->setValid(true);
      return;
    }
    setCommandValuesToRead();

    // The first answered poll means the drive is talking to us, so this is the
    // point to find out who it is.
    if (!this->identityStarted)
    {
      this->identityStarted = true;
      this->requestDriveIdentity();
    }
    // A request travels in the same registers a command would, so it has to
    // wait until nothing is being pressed.
    if (this->identityWanted != 0 && this->regResp[2] == 0x0000 && this->regResp[3] == 0x0000)
    {
      const uint32_t now = esphome::millis();
      const bool firstTry = !this->identityAsked;
      // Subtract, never add: adding overflows when millis() wraps.
      if (firstTry || (now - this->identityAskedOn) > IDENT_RETRY_MS)
      {
        if (this->identityAttempts >= IDENT_MAX_ATTEMPTS)
        {
          ESP_LOGW(TAG_HCI, "drive did not answer identity request %x",
                   this->identityWanted);
          this->identityWanted = 0;
        }
        else
        {
          this->identityAttempts++;
          this->identityAskedOn = now;
          this->identityAsked = true;
          this->regResp[1] = RESP_REQUEST;
          this->regResp[2] = (uint16_t)(this->identityWanted << 8);
          this->regResp[3] = 0x0000;
        }
      }
    }
  }
  else if (fc == 0x17 && a2 == REG_CMD_BASE && c2 == 0x02 && a1 == REG_RESP_BASE && c1 == 0x02)
  {
    this->regResp[0] = 0x0004;
    this->regResp[1] = 0x0000;
    ESP_LOGV(TAG_HCI, "executing empty command");
  }
  else if (fc == 0x17 && a2 == REG_CMD_BASE && c2 == 0x03 && a1 == REG_RESP_BASE && c1 == 0x05)
  {
    ESP_LOGV(TAG_HCI, "executing busscan");
    this->regResp[0] = 0x0000;
    this->regResp[1] = 0x0005;
    this->regResp[2] = 0x0430;
    this->regResp[3] = 0x10ff;
    this->regResp[4] = 0xa845;
  }
  else if (fc == 0x17 && a2 == REG_CMD_BASE && c2 >= 0x03 && a1 == REG_RESP_BASE)
  {
    // The whole block: it is read back in this same frame, so a stale command
    // would go out as a key press.
    for (uint8_t i = 0; i < REG_RESP_COUNT; i++)
      this->regResp[i] = 0x0000;
    // One we understand is answered later and overwrites this. This is one we
    // do not, and zero is not a defined answer code.
    this->regResp[1] = RESP_STATUS;
  }
  else if (fc == 0x10 && a1 == REG_BCAST_BASE)
  {
  }
  else
  {
    // Read back in the same frame, so a leftover command or signature would go
    // out a second time.
    for (uint8_t i = 0; i < REG_RESP_COUNT; i++)
      this->regResp[i] = 0x0000;
    // Zero is not a defined answer code. Plain status with an empty payload is
    // what this is.
    this->regResp[1] = RESP_STATUS;
    this->reportUnknownShape(fc, a1, c1, a2, c2);
  }
  this->state->setValid(true);
}

static inline uint16_t rd16(const uint8_t *p) { return (uint16_t)(p[0] << 8) | p[1]; }
static inline void wr16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)(v & 0xFF); }

/**
 * Answers one complete frame (CRC already stripped).
 *
 * Order follows the library: fill the response registers first, then apply the
 * drive's writes (which can still change 0x9CB9), then answer.
 */
size_t HoermannGarageEngine::onFrame(const uint8_t *req, size_t len, uint8_t *resp)
{
  // Anything addressed to us is a sign of life. millis() can be 0 for the
  // first millisecond, which would read as "never".
  const uint32_t now = esphome::millis();
  this->lastFrameOn.store(now == 0 ? 1 : now);

  const uint8_t fc = req[1];
  // A read/write frame to the broadcast address is never answered, so running
  // it would consume a queued press for nothing.
  if (req[0] == 0x00 && fc == 0x17)
    return 0;

  if (fc == 0x17)
  {
    if (len < 11)
      return 0;
    const uint16_t readAddr = rd16(req + 2);
    const uint16_t readCnt = rd16(req + 4);
    const uint16_t writeAddr = rd16(req + 6);
    const uint16_t writeCnt = rd16(req + 8);
    const uint8_t byteCnt = req[10];
    const uint8_t *wdata = req + 11;

    // All of it before anything is stored or answered. A frame that fails gets
    // no answer at all, there being no code for "your frame was wrong"; and
    // answering first let a malformed frame swallow a queued key press.
    if (readAddr != REG_RESP_BASE || writeAddr != REG_CMD_BASE)
    {
      // A wrong address shifts the payload by a register: counter read as
      // command, command as position.
      ESP_LOGW(TAG_HCI, "frame names read %04x write %04x, expected %04x and %04x", readAddr,
               writeAddr, REG_RESP_BASE, REG_CMD_BASE);
      return 0;
    }
    if (readCnt < 1 || readCnt > REG_RESP_COUNT || writeCnt < 1 ||
        (0xFFFF - readAddr) < readCnt || byteCnt != 2 * writeCnt)
      return 0;
    // One to nine registers; anything longer has no room in the block.
    if (byteCnt < 2 || byteCnt > 2 * REG_CMD_COUNT)
      return 0;
    if (len < (size_t)(11 + byteCnt))
      return 0;
    if (!this->regExists(readAddr))
      return 0;

    // Before the hook that reads the queue, because a lost answer puts a
    // command back into it. High byte counter, low byte command.
    this->syncCounter(wdata[0], wdata[1]);
    // Only the status branch refills it. Left standing, a lost busscan answer
    // re-sent a door command that had already been delivered.
    this->lastSentCommand = nullptr;

    this->onRequestHook(fc, readAddr, readCnt, writeAddr, writeCnt, wdata[1]);

    for (uint16_t i = 0; i < writeCnt; i++)
      this->regSetChecked((uint16_t)(writeAddr + i), rd16(wdata + 2 * i));
    this->onWriteBlockComplete(writeAddr, writeCnt);

    size_t n = 0;
    resp[n++] = req[0];
    resp[n++] = fc;
    resp[n++] = (uint8_t)(readCnt * 2);
    for (uint16_t i = 0; i < readCnt; i++)
    {
      wr16(resp + n, this->regGet((uint16_t)(readAddr + i)));
      n += 2;
    }
    // Only for an answer that is going out. The byte we send stays the drive's
    // own, mirrored; the count exists to notice a loss, not to be sent.
    this->advanceCounter();
    return n;
  }

  if (fc == 0x10)
  {
    if (len < 7)
      return 0;
    const uint16_t addr = rd16(req + 2);
    const uint16_t cnt = rd16(req + 4);
    const uint8_t byteCnt = req[6];
    const uint8_t *wdata = req + 7;

    // As above. A wrong address would land the door state where the position
    // is read from.
    if (addr != REG_BCAST_BASE)
    {
      ESP_LOGW(TAG_HCI, "broadcast names %04x, expected %04x", addr, REG_BCAST_BASE);
      return 0;
    }
    if (cnt < 1 || byteCnt != 2 * cnt || byteCnt < 2 || byteCnt > 2 * REG_BCAST_COUNT)
      return 0;
    if (len < (size_t)(7 + byteCnt))
      return 0;

    this->onRequestHook(fc, addr, cnt, 0, 0);

    for (uint16_t i = 0; i < cnt; i++)
      this->regSetChecked((uint16_t)(addr + i), rd16(wdata + 2 * i));
    this->checkGotoTarget();

    // Built for completeness; the frame layer never sends an answer to a
    // broadcast.
    size_t n = 0;
    resp[n++] = req[0];
    resp[n++] = fc;
    wr16(resp + n, addr); n += 2;
    wr16(resp + n, cnt); n += 2;
    return n;
  }

  // Two function codes exist here. The catalogue the old library served handed
  // our whole state to anyone asking and let anyone write into it.
  return 0;
}

// The drive acts on the second of the two values a command used to send over
// two frames, so that is the one it gets. Measured at a door.
static void activeCommandValues(const HoermannCommand *cmd, uint16_t *v2, uint16_t *v3)
{
  *v2 = cmd->commandEndPlus2Value;
  *v3 = cmd->commandEndPlus3Value;
}

void HoermannGarageEngine::setCommandValuesToRead()
{
  // Runs here, in the task that owns all of this, and it can arm a repeat,
  // which must not be decided from a half read state.
  this->checkCommandEffect();

  uint16_t regPlug2Value = 0x0000;
  uint16_t regPlug3Value = 0x0000;

  // Written once, then gone: the answer is rebuilt from nothing on every poll,
  // so a command that is not put back in is simply not there any more.
  // The watch is the bus task's, so a stop from the other side asks rather than
  // reaches in.
  if (this->cancelWatch.exchange(false))
  {
    this->awaitedCommand = nullptr;
    this->repeatCommand = nullptr;
  }

  const HoermannCommand *cmd = this->nextCommand.load();
  // A press only goes out while somebody could still be standing there. The
  // drive answers within one poll, so anything older than this waited for a
  // frame that never came: an enumeration burst, or a bus that was talking but
  // not to us.
  if (cmd != nullptr && (esphome::millis() - this->nextCommandOn.load()) > CMD_STALE_MS)
  {
    ESP_LOGW(TAG_HCI, "dropping a command the drive did not fetch in time");
    this->nextCommand.store(nullptr);
    cmd = nullptr;
  }
  if (cmd != nullptr)
  {
    // A press always wins over a repeat of our own.
    this->repeatCommand = nullptr;
    activeCommandValues(cmd, &regPlug2Value, &regPlug3Value);
    ESP_LOGI(TAG_HCI, "command %x %x", regPlug2Value, regPlug3Value);
    this->nextCommand.store(nullptr);
    // Start watching for the effect.
    this->awaitedCommand = cmd;
    this->awaitedSince = esphome::millis();
    this->awaitedRepeats = 0;
    this->stateWritesWhenSent = this->stateWrites;
    this->repeatsSent = 0;
  }
  else if (this->repeatCommand != nullptr && this->repeatStillMakesSense())
  {
    cmd = this->repeatCommand;
    activeCommandValues(cmd, &regPlug2Value, &regPlug3Value);
    ESP_LOGI(TAG_HCI, "repeating command %x %x", regPlug2Value, regPlug3Value);
    this->repeatCommand = nullptr;
    // The drive's grace period runs from the copy it actually received, not
    // from the one that was lost, or one press ends up on the wire three times.
    this->awaitedSince = esphome::millis();
    this->awaitedRepeats = 0;
  }
  else if (this->repeatCommand != nullptr)
  {
    ESP_LOGI(TAG_HCI, "dropping a repeat the door has moved past");
    this->repeatCommand = nullptr;
  }
  // So a lost answer can be reconstructed. Nothing carried, nothing to put
  // back.
  this->lastSentCommand = cmd;
  this->regResp[2] = regPlug2Value;
  this->regResp[3] = regPlug3Value;
}

// Did the door do what it was told? A direction that is already reached counts,
// otherwise pressing "open" on an open door would look like a failure.
/** Is the door already where this command would send it? */
bool HoermannGarageEngine::commandReached(const HoermannCommand *cmd) const
{
  const HoermannState::State now = this->state->state;
  if (cmd == &HoermannCommand::STARTOPENDOOR)
    return now == HoermannState::OPENING || now == HoermannState::OPEN;
  if (cmd == &HoermannCommand::STARTCLOSEDOOR)
    return now == HoermannState::CLOSING || now == HoermannState::CLOSED;
  if (cmd == &HoermannCommand::STARTOPENDOORHALF)
    return now == HoermannState::MOVE_HALF || now == HoermannState::HALFOPEN;
  if (cmd == &HoermannCommand::STARTVENTPOSITION)
    return now == HoermannState::MOVE_VENTING || now == HoermannState::VENT;
  if (cmd == &HoermannCommand::LAMPON)
    return this->state->lightOn;
  if (cmd == &HoermannCommand::LAMPOFF)
    return !this->state->lightOn;
  return false;
}

bool HoermannGarageEngine::commandTookEffect() const
{
  // Counted, not compared: a door that started and was stopped again between
  // two polls comes back to the same word, and a value test reads that as the
  // drive having done nothing.
  if (this->stateWrites != this->stateWritesWhenSent)
    return true;
  // Otherwise the destination counts, so asking an open door to open is not
  // taken for a failure.
  return this->commandReached(this->awaitedCommand);
}

/**
 * Between arming a repeat and sending it, the door can have moved on. An
 * impulse is the dangerous one: on a standing door it starts a run, so a repeat
 * armed while the door was travelling must not go out once it has stopped.
 */
bool HoermannGarageEngine::repeatStillMakesSense() const
{
  const HoermannCommand *const cmd = this->repeatCommand;
  if (cmd == nullptr)
    return false;
  if (cmd == &HoermannCommand::STARTIMPULSE)
    // An impulse has no destination to check against, so the question is
    // whether the drive did anything at all. It usually starts a standing door,
    // which is why "is it moving" was the wrong test: it threw away the repeat
    // for the ordinary toggle and told the caller the press had gone out.
    return this->stateWrites == this->stateWritesWhenSent;
  // Anything else names a destination, so it is still worth sending exactly
  // while the door is not already there.
  return !this->commandReached(cmd);
}

void HoermannGarageEngine::checkCommandEffect()
{
  // Once, so the checks below all see the same command.
  const HoermannCommand *const awaited = this->awaitedCommand;
  if (awaited == nullptr)
    return;
  if (this->commandTookEffect())
  {
    this->awaitedCommand = nullptr;
    // The repeat belongs to this command. After it took effect it is a second
    // key press.
    this->repeatCommand = nullptr;
    return;
  }
  // A press the caller queued in the meantime takes over; repeating an old one
  // on top of it would be a second press nobody asked for.
  if (this->nextCommand.load() != nullptr)
  {
    this->awaitedCommand = nullptr;
    this->repeatCommand = nullptr;
    return;
  }
  // Subtract, never add: adding overflows when millis() wraps.
  const uint32_t waited = esphome::millis() - this->awaitedSince;
  if (waited > CMD_GIVEUP_MS)
  {
    uint16_t shown2 = 0, shown3 = 0;
    activeCommandValues(awaited, &shown2, &shown3);
    ESP_LOGW(TAG_HCI, "drive did not act on command %x %x", shown2, shown3);
    this->awaitedCommand = nullptr;
    this->repeatCommand = nullptr;
    // A target outliving the command that created it hijacks the next run in
    // that direction, whoever started it.
    this->state->setGotoPosition(0.0f);
    return;
  }
  if (waited > CMD_CONFIRM_MS && this->awaitedRepeats == 0)
  {
    this->awaitedRepeats = 1;
    this->repeatCommand = awaited;
  }
}

void HoermannGarageEngine::onDoorPositonChanged(uint16_t oldVal, uint16_t val)
{
  // on First Byte changed (current)
  if ((oldVal & 0x00FF) != (val & 0x00FF))
  {
    this->state->setCurrentPosition((float)(val & 0x00FF) / 200.0f);
    // Decided after the whole telegram, not here. Registers are applied in
    // address order, so the state this would test is still the previous
    // telegram's: one that reports both "past the target" and "stopped" would
    // fire an impulse at a door somebody had just stopped.
    this->gotoCheckPending = true;
  }
  // on Second Byte changed (target)
  if ((oldVal & 0xFF00) != (val & 0xFF00))
  {
    this->state->setTargetPosition((float)((val & 0xFF00) >> 8) / 200.0f);
  }
}

void HoermannGarageEngine::onCurrentStateChanged(uint16_t oldVal, uint16_t val)
{
  // on First Byte changed
  if ((oldVal & 0xFF00) != (val & 0xFF00))
  {
    ESP_LOGI(TAG_HCI, "onCurrentStateChanged. address=%x, value=%x (actual: %x)", REG_BCAST_BASE + 2, val, (val & 0xFF00) >> 8);

    switch ((val & 0xFF00) >> 8)
    {
    case 0x1:
      this->state->setState(HoermannState::State::OPENING);
      break;
    case 0x2:
      this->state->setState(HoermannState::State::CLOSING);
      break;
    case 0x20:
      this->state->setState(HoermannState::State::OPEN);
      break;
    case 0x40:
      this->state->setState(HoermannState::State::CLOSED);
      break;
    case 0x80:
      this->state->setState(HoermannState::State::HALFOPEN);
      break;
    case 0x09:
      this->state->setState(HoermannState::State::MOVE_VENTING);
      break;
    case 0x05:
      this->state->setState(HoermannState::State::MOVE_HALF);
      break;
    case 0x0A:
      this->state->setState(HoermannState::State::VENT);
      break;
    case 0x00:
      // Additional check on the low byte when the high byte is 0x00
      if ((val & 0x00FF) == 0x61) {
        this->state->setState(HoermannState::State::VENT);
      } else {
        this->state->setState(HoermannState::State::STOPPED);
      } 
      break;
    default:
      ESP_LOGW(TAG_HCI, "unknown State %x", (val & 0xFF00) >> 8);
    }
  }
}

void HoermannGarageEngine::onRegSevenChanged(uint16_t oldVal, uint16_t val)
  //Observed Values, last bit 4 is assumed could not be tested as I have no UAP HCP.
  //0x00 0x00 Relay off - Light off
  //0x02 0x00 Relay on  - light off
  //0x02 0x10 Relay on  - light on
  //0x00 0x10 Relay off - light on
  //0x00 0x14 Relay on  - light on 
  //0x00 0x04 Relay on  - light off

{
  if ((oldVal & 0xFF00) != (val & 0xFF00)){
    // The drive's own fault indication, whatever the relay bit below means on
    // a given installation.
    this->state->setActuatorError((val & 0x3000) != 0);
    // 0x02 happen when relay menu 30 is set to 06, 07, 10 
    this->state->setRelayOn((val & 0xFF00) >> 8 == 0x02);
  }
  // On second byte changed
  if ((oldVal & 0x00FF) != (val & 0x00FF))
  {
    ESP_LOGI(TAG_HCI, "onRegSixChanged. address=%x, value=%x", REG_BCAST_BASE + 6, val);
    this->state->setLigthOn((val & 0x00FF) == 0x14 || (val & 0x00FF) == 0x10);
    this->state->setRelayOn((val & 0xFF00) >> 8 == 0x02 || (val & 0x00FF) == 0x14 || (val & 0x00FF) == 0x04);
  }
}

/**
 * Write on 0x9C41 , byte1: counter, byte2: command
 */
/** The go-to-position stop, once the whole broadcast has been applied. */
void HoermannGarageEngine::checkGotoTarget()
{
  if (!this->gotoCheckPending)
    return;
  this->gotoCheckPending = false;
  const float target = this->state->gotoPosition;
  if (target <= 0.0f)
    return;
  const HoermannState::State now = this->state->state;
  const bool arrived = (now == HoermannState::CLOSING && target >= this->state->currentPosition) ||
                       (now == HoermannState::OPENING && target <= this->state->currentPosition);
  if (!arrived)
    return;
  // Only a stop that was actually issued may clear the target; otherwise the
  // door runs to the end stop with nothing left to retry from.
  if (this->stopDoor())
    this->state->setGotoPosition(0.0f);
}

void HoermannGarageEngine::onCounterWrite(uint16_t val)
{
  // Top bit selects the half of a split payload; it is not part of the count.
  uint16_t counter = val & 0x7F00;
  uint16_t command = (val & 0x00FF) << 8;
  this->regResp[0] |= counter;
  this->regResp[1] |= command;
}

/**
 * Called with the drive's counter byte before the answer is built, because the
 * decision it makes has to happen before a command is taken out of the queue.
 *
 * The byte that ends up on the wire is not changed by any of this. In step the
 * count equals what the drive just sent, and after a loss the fallback value is
 * the one the drive is repeating, which is the same byte again. Its whole
 * purpose is to notice the loss.
 */
void HoermannGarageEngine::syncCounter(uint8_t counterByte, uint8_t command)
{
  // Top bit is the half selector of a split payload, not part of the count.
  const uint8_t rx = counterByte & 0x7F;

  if (!this->txCounterValid)
  {
    // First frame we ever see: adopt whatever the drive is counting.
    this->txCounter = rx;
    this->txCounterValid = true;
    return;
  }
  if (rx != this->txCounter)
  {
    // Only a repeat of the value we already answered is a loss, and only a
    // status poll carries a command.
    if (rx == this->txCounterPrev && command == CMD_STATUS)
      this->rearmLostCommand();
    // Anything else is the drive somewhere unexpected. Follow it: holding our
    // own value kept the mismatch, and the repeat, alive on every frame.
    this->txCounter = rx;
  }
}

void HoermannGarageEngine::advanceCounter()
{
  this->txCounterPrev = this->txCounter;
  this->txCounter = this->txCounter == 0x7F ? 1 : (uint8_t)(this->txCounter + 1);
}

/**
 * Put back what the lost answer was carrying. Only the repeat slot is used, so
 * a press made in the meantime still wins: that one is newer and says what the
 * user wants now.
 */
void HoermannGarageEngine::rearmLostCommand()
{
  if (this->lastSentCommand == nullptr)
    return;  // the lost answer carried no command, nothing to put back
  if (this->nextCommand.load() != nullptr)
    return;
  // While the drive repeats a counter our answer never arrived, so re-sending
  // is right and at most one copy can land. That rests on the drive holding its
  // counter until answered, which is a model of it, not a fact about it.
  // Capped, so a drive that behaves otherwise costs a bounded number of key
  // presses instead of one per poll for as long as it lasts.
  if (this->repeatsSent >= MAX_LOST_REPEATS)
  {
    ESP_LOGW(TAG_HCI, "drive keeps repeating its counter, giving up on the command");
    this->lastSentCommand = nullptr;
    return;
  }
  this->repeatsSent++;
  this->repeatCommand = this->lastSentCommand;
  // The drive told us the answer never arrived, so there is nothing left to
  // watch. Leaving the watch standing let the effect guess win over the
  // evidence: any unrelated change of the state word between the two polls
  // counted as "the drive acted" and threw the repeat away.
  this->awaitedCommand = nullptr;
  this->lastSentCommand = nullptr;
  ESP_LOGI(TAG_HCI, "answer did not reach the drive, sending the command again");
}

void HoermannGarageEngine::requestDriveIdentity()
{
  this->armIdentityRequest(IDENT_REQ_SERIAL);
}

void HoermannGarageEngine::armIdentityRequest(uint8_t request)
{
  this->identityWanted = request;
  this->identityAttempts = 0;
  this->identityAskedOn = 0;
  this->identityAsked = false;
  this->serialFirstHalfSeen = false;
}

// Registers hold two payload bytes each, high byte first.
void HoermannGarageEngine::copyRegsToBytes(uint8_t firstReg, uint8_t regCount, uint8_t *out)
{
  for (uint8_t i = 0; i < regCount; i++)
  {
    const uint16_t v = this->regCmd[firstReg + i];
    out[2 * i] = (uint8_t)(v >> 8);
    out[2 * i + 1] = (uint8_t)(v & 0x00FF);
  }
}

// Trailing padding varies, and anything unprintable would only confuse a text
// sensor, so cut at the first byte that is neither.
static void identityToText(const uint8_t *data, size_t len, char *out, size_t outSize)
{
  size_t at = 0;
  for (size_t i = 0; i < len && at + 1 < outSize; i++)
  {
    if (data[i] < 0x20 || data[i] > 0x7E)
      break;
    out[at++] = (char)data[i];
  }
  while (at > 0 && out[at - 1] == ' ')
    at--;
  out[at] = '\0';
}

// Silence would leave the code at zero, which the protocol does not define.
void HoermannGarageEngine::answerTransfer(uint8_t counterByte, uint8_t code)
{
  for (uint8_t i = 2; i < REG_RESP_COUNT; i++)
    this->regResp[i] = 0x0000;
  // Only the lower seven bits are the drive's running counter, the top bit
  // marks which half of a split payload this was and must not be echoed.
  this->regResp[0] = (uint16_t)((counterByte & 0x7F) << 8);
  this->regResp[1] = (uint16_t)(0x0400 | code);
}

void HoermannGarageEngine::onWriteBlockComplete(uint16_t addr, uint16_t count)
{
  if (addr != REG_CMD_BASE || count < 2)
    return;
  // Low byte of the first register is what the drive wants from us, high byte
  // its running counter. Only the payload transfer is of interest here.
  const uint8_t command = (uint8_t)(this->regCmd[0] & 0x00FF);
  if (command != 0x04)
    return;
  const uint8_t counterByte = (uint8_t)(this->regCmd[0] >> 8);
  const uint8_t subCode = (uint8_t)(this->regCmd[1] >> 8);

  if (subCode == IDENT_SUB_PAUSE_ACK)
  {
    // The drive names the address it is pausing. It sits astride two
    // registers: low byte of the sub code register, high byte of the next.
    if (count >= 3)
    {
      const uint16_t named =
          (uint16_t)(((this->regCmd[1] & 0x00FF) << 8) | (this->regCmd[2] >> 8));
      if (named == SLAVE_ID)
        this->pauseConfirmed.store(true);
    }
    // Answered either way: we understood the question, and leaving it open is
    // worse than answering for an address that was not ours.
    this->answerTransfer(counterByte, RESP_ACK);
    return;
  }

  if (subCode == IDENT_SUB_SERIAL || subCode == IDENT_SUB_FIRMWARE)
  {
    // Acknowledging a payload we threw away tells the drive it landed, so it
    // never sends it again and the value only returns via our own retry.
    this->answerTransfer(counterByte,
                         this->onIdentityData(counterByte, subCode, count) ? RESP_ACK : RESP_NAK);
    return;
  }

  // Something we do not know. Say so rather than leaving an undefined code.
  ESP_LOGW(TAG_HCI, "unknown transfer sub code %02x", subCode);
  this->answerTransfer(counterByte, RESP_NAK);
}

bool HoermannGarageEngine::announcePause(uint32_t timeoutMs)
{
  const uint32_t last = this->lastFrameOn.load();
  // Nobody to say it to: nothing ever arrived, or the bus has been quiet past
  // the point where we call it gone. A restart must not pay for that.
  if (last == 0 || (esphome::millis() - last) > BUS_SILENCE_MS)
    return false;

  // A waiting press is dropped rather than arriving after a restart with
  // nobody expecting it. Only the slot this task owns; the bus task clears its
  // own, because clearing them here races with the function that reads them.
  this->nextCommand.store(nullptr);

  this->pauseConfirmed.store(false);
  this->pauseRequested.store(true);
  const uint32_t started = esphome::millis();
  // Subtract, never add: adding overflows when millis() wraps.
  while ((esphome::millis() - started) < timeoutMs)
  {
    esphome::App.feed_wdt();
    if (this->pauseConfirmed.load())
    {
      // Going back to the ordinary status here would say "never mind" for two
      // seconds and then vanish anyway.
      this->settleBeforeRestart();
      this->pauseRequested.store(false);
      return true;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  // Back to answering normally: a restart that never happens must not leave the
  // bridge announcing a pause for ever.
  this->settleBeforeRestart();
  this->pauseRequested.store(false);
  return false;
}

// The restart has to land between telegrams, not inside one. The bus task
// keeps running, so a fixed sleep would prove nothing.
void HoermannGarageEngine::settleBeforeRestart()
{
  const uint32_t started = esphome::millis();
  while ((esphome::millis() - started) < PAUSE_SETTLE_MS)
  {
    esphome::App.feed_wdt();
    const uint32_t last = this->lastFrameOn.load();
    if (last != 0 && (esphome::millis() - last) > PAUSE_QUIET_MS)
      return;
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void HoermannGarageEngine::publishIdentity()
{
  if (this->identSerialReady.exchange(false))
    this->state->setSerialNumber(this->identSerial);
  if (this->identFirmwareReady.exchange(false))
    this->state->setFirmwareVersion(this->identFirmware);
}

void HoermannGarageEngine::checkBusSilence()
{
  const uint32_t last = this->lastFrameOn.load();
  if (last == 0)
    return;  // nothing ever arrived, the initial state already says so
  if ((esphome::millis() - last) > BUS_SILENCE_MS)
  {
    this->state->setValid(false);
    // The slot has no expiry, so a press made just before we called the link
    // dead would move the door whenever it returns.
    if (this->nextCommand.exchange(nullptr) != nullptr)
      ESP_LOGW(TAG_HCI, "dropping a command the drive never came back for");
  }
}

bool HoermannGarageEngine::onIdentityData(uint8_t counterByte, uint8_t subCode, uint16_t count)
{
  // A payload for a request that is no longer open is a late resend. Taking it
  // would restart the retry clock of whatever is outstanding now.
  const uint8_t answers = subCode == IDENT_SUB_SERIAL ? IDENT_REQ_SERIAL : IDENT_REQ_FIRMWARE;
  if (this->identityWanted != answers)
    return false;
  // The payload starts one register behind the sub code.
  const uint8_t firstPayloadReg = 2;
  const uint16_t payloadRegs = count > firstPayloadReg ? count - firstPayloadReg : 0;
  bool kept = false;

  if (subCode == IDENT_SUB_FIRMWARE && payloadRegs >= IDENT_FIRMWARE_LEN / 2)
  {
    uint8_t buf[IDENT_FIRMWARE_LEN];
    this->copyRegsToBytes(firstPayloadReg, IDENT_FIRMWARE_LEN / 2, buf);
    identityToText(buf, sizeof(buf), this->identFirmware, sizeof(this->identFirmware));
    this->identFirmwareReady.store(true);
    this->identityWanted = 0;
    ESP_LOGI(TAG_HCI, "drive firmware version %s", this->identFirmware);
    kept = true;
  }
  else if (subCode == IDENT_SUB_SERIAL)
  {
    // Top bit marks the first half. The halves differ in length, so each is
    // checked against its own; otherwise a short one pads with stale registers.
    const bool firstHalf = (counterByte & 0x80) != 0;
    if (firstHalf && payloadRegs >= SERIAL_FIRST_REGS)
    {
      this->copyRegsToBytes(firstPayloadReg, SERIAL_FIRST_REGS, this->serialBuf);
      this->serialFirstHalfSeen = true;
      // Half an answer is progress. The attempt count does not restart, or a
      // drive sending only the first half would be asked for ever.
      this->identityAskedOn = esphome::millis();
      this->identityAsked = true;
      kept = true;
    }
    else if (!firstHalf && this->serialFirstHalfSeen && payloadRegs >= SERIAL_SECOND_REGS)
    {
      this->copyRegsToBytes(firstPayloadReg, SERIAL_SECOND_REGS,
                            this->serialBuf + 2 * SERIAL_FIRST_REGS);
      this->serialFirstHalfSeen = false;
      identityToText(this->serialBuf, IDENT_SERIAL_LEN, this->identSerial, sizeof(this->identSerial));
      this->identSerialReady.store(true);
      ESP_LOGI(TAG_HCI, "drive serial number received");
      // Identity is only complete with the firmware version, so go straight on.
      this->armIdentityRequest(IDENT_REQ_FIRMWARE);
      kept = true;
    }
  }

  return kept;
}

/**
 * Helper to set next Command and *not* skip Current Command before end was sent
 */
bool HoermannGarageEngine::setCommand(bool cond, const HoermannCommand *command)
{
  if (cond)
  {
    // Queueing with nobody listening does not delay a press, it arms one: the
    // slot has no expiry, so the door moves whenever the link returns. Refusing
    // lets the caller say so while the person who pressed is still there.
    if (!this->state->valid)
    {
      ESP_LOGW(TAG_HCI, "no command sent, the drive is not talking to us");
      return false;
    }
    this->nextCommandOn.store(esphome::millis());
    // Set from the main loop, read by the bus task on core 1. The pointer
    // store is atomic on ESP32, the test-and-set below is not.
    const HoermannCommand *expected = nullptr;
    if (!this->nextCommand.compare_exchange_strong(expected, command))
    {
      ESP_LOGW(TAG_HCI, "Last Command was not yet fetched by modbus!");
      return false;
    }
  }
  return true;
}

/**
 * Control Functions
 */
bool HoermannGarageEngine::stopDoor()
{
  // A stop outranks whatever is queued. Competing for the slot meant the last
  // thing the user pressed could lose to the one before it: a stop while an
  // open was still waiting was reported as accepted and the door opened.
  const HoermannState::State now = this->state->state;
  const bool moving = now == HoermannState::CLOSING || now == HoermannState::OPENING ||
                      now == HoermannState::MOVE_HALF || now == HoermannState::MOVE_VENTING;
  if (!this->state->valid)
  {
    ESP_LOGW(TAG_HCI, "no stop sent, the drive is not talking to us");
    return false;
  }
  // A door that is not moving needs no impulse; cancelling what was queued is
  // the stop. One that is moving needs one, and it replaces the queue.
  this->nextCommand.store(moving ? &HoermannCommand::STARTIMPULSE : nullptr);
  this->nextCommandOn.store(esphome::millis());
  // The watch belongs to the bus task, so it is asked to drop it rather than
  // reached into from here.
  this->cancelWatch.store(true);
  return true;
}
bool HoermannGarageEngine::closeDoor()
{
  return setCommand(true, &HoermannCommand::STARTCLOSEDOOR);
}
bool HoermannGarageEngine::openDoor()
{
  return setCommand(true, &HoermannCommand::STARTOPENDOOR);
}
bool HoermannGarageEngine::impulseDoor()
{
  return setCommand(true, &HoermannCommand::STARTIMPULSE);
}
bool HoermannGarageEngine::halfPositionDoor()
{
  return setCommand(true, &HoermannCommand::STARTOPENDOORHALF);
}
bool HoermannGarageEngine::ventilationPositionDoor()
{
  return setCommand(true, &HoermannCommand::STARTVENTPOSITION);
}
bool HoermannGarageEngine::turnLight(bool on)
{
  return setCommand(on != this->state->lightOn,
                    on ? &HoermannCommand::LAMPON : &HoermannCommand::LAMPOFF);
}
bool HoermannGarageEngine::setPosition(int setPosition)
{
  // First and last movement segments seem a bit inconsistent on Promatic4, so it's better to leave it to fully open or close.
  if (setPosition <= 5)
    return closeDoor();
  if (setPosition >= 95)
    return openDoor();
  this->state->setGotoPosition(static_cast<float>(setPosition) / 100.0f);
  if (this->state->currentPosition < this->state->gotoPosition)
    return setCommand(true, &HoermannCommand::STARTOPENDOOR);
  if (this->state->currentPosition > this->state->gotoPosition)
    return setCommand(true, &HoermannCommand::STARTCLOSEDOOR);
  return true;  // already there
}

void HoermannState::setTargetPosition(float targetPosition)
{
  this->targetPosition = targetPosition;
  this->changed = true;
}
void HoermannState::setGotoPosition(float setPosition)
{
  this->gotoPosition = setPosition;
  this->changed = true;
}
void HoermannState::setCurrentPosition(float currentPosition)
{
  this->currentPosition = currentPosition;
  this->changed = true;
}
void HoermannState::setLigthOn(bool lightOn)
{
  this->lightOn = lightOn;
  this->changed = true;
}
void HoermannState::setRelayOn(bool relayOn)
{
  this->relayOn = relayOn;
  this->changed = true;
}
void HoermannState::clearChanged()
{
  // Exchange, not assign: a change raised by the bus task between the test and
  // this line would otherwise be dropped.
  this->changed.exchange(false);
}
static bool isMoving(HoermannState::State s)
{
  return s == HoermannState::State::OPENING || s == HoermannState::State::CLOSING ||
         s == HoermannState::State::MOVE_HALF || s == HoermannState::State::MOVE_VENTING;
}

void HoermannState::setState(State state)
{
  const bool was_moving = isMoving(this->state);
  this->state = state;
  this->changed = true;
  // A target must not survive a movement that ended for another reason, or the
  // next run in that direction stops where nobody asked.
  if (was_moving && !isMoving(state))
    this->gotoPosition = 0.0f;
}
void HoermannState::setActuatorError(bool actuatorError)
{
  if (this->actuatorError == actuatorError)
    return;
  this->actuatorError = actuatorError;
  this->changed = true;
}

void HoermannState::setValid(bool isValid)
{
  if (this->valid.exchange(isValid) == isValid)
    return;
  // Without this the connection sensor would only ever learn of the first
  // frame, never of the silence afterwards.
  this->changed = true;
}

void HoermannState::setSerialNumber(const std::string &serialNumber)
{
  if (this->serialNumber == serialNumber)
    return;
  this->serialNumber = serialNumber;
  this->changed = true;
}

void HoermannState::setFirmwareVersion(const std::string &firmwareVersion)
{
  if (this->firmwareVersion == firmwareVersion)
    return;
  this->firmwareVersion = firmwareVersion;
  this->changed = true;
}

}  // namespace hcpbridge
}  // namespace esphome
