// Credits to https://github.com/Gifford47/HCPBridgeMqtt for the initial code base

#include <cstring>

#include "hoermann.h"

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// arg1,arg4> command start value
// arg2,arg5> command end   value
const HoermannCommand HoermannCommand::STARTOPENDOOR = HoermannCommand(0x0210, 0x0110, 0x0000, 0x0000); // Typo 0201
const HoermannCommand HoermannCommand::STARTCLOSEDOOR = HoermannCommand(0x0220, 0x0120, 0x0000, 0x0000);
const HoermannCommand HoermannCommand::STARTIMPULSE = HoermannCommand(0x0240, 0x0140, 0x0000, 0x0000);
const HoermannCommand HoermannCommand::STARTOPENDOORHALF = HoermannCommand(0x0200, 0x0100, 0x0400, 0x0400);
const HoermannCommand HoermannCommand::STARTVENTPOSITION = HoermannCommand(0x0200, 0x0100, 0x4000, 0x4000);
const HoermannCommand HoermannCommand::STARTTOGGLELAMP = HoermannCommand(0x0100, 0x0800, 0x0200, 0x0200);
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

void HoermannGarageEngine::setup(int8_t rx, int8_t tx, int8_t rts)
{
  this->mb.set_handler([this](const uint8_t *req, size_t len, uint8_t *resp) -> size_t
                       { return this->onFrame(req, len, resp); });
  if (!this->mb.begin(UART_NUM_2, rx, tx, rts, HCP_BAUD, SLAVE_ID))
  {
    // No port, no task: at top priority it would spin, because poll() returns
    // immediately.
    ESP_LOGE(TAG_HCI, "serial setup failed, bus task not started");
    return;
  }

  xTaskCreatePinnedToCore(
      modbusServeTask,          /* Function to implement the task */
      "ModBusTask",             /* Name of the task */
      8192,                     /* Stapel in Byte (ESP-IDF), Protokollaufrufe brauchen Platz */
      NULL,                     /* Task input parameter */
      configMAX_PRIORITIES - 1, /* Priority */
      &modBusTask,              /* Task handle */
      1);                       /* Core */
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

/** Wie Reg(addr, val): false, wenn es das Register nicht gibt. */
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
    this->onCurrentStateChanged(old, val);
  else if (addr == REG_BCAST_BASE + 6)
    this->onRegSevenChanged(old, val);
  *p = val;
  return true;
}

/**
 * Wie setMultipleWords() je Register: schreiben und zurueckpruefen. Ein nicht
 * vorhandenes Register nimmt nichts an und liest sich als 0, der Schreibzugriff
 * gilt daher nur dann als erfolgreich, wenn der Wert ohnehin 0 war.
 */
bool HoermannGarageEngine::regSetChecked(uint16_t addr, uint16_t val)
{
  this->regWrite(addr, val);
  return this->regGet(addr) == val;
}

/**
 * The former onRequest callback. The library ran it for every function code it
 * knew, before any validation.
 */
void HoermannGarageEngine::onRequestHook(uint8_t fc, uint16_t a1, uint16_t c1, uint16_t a2, uint16_t c2)
{
  if (fc == 0x17 && a2 == REG_CMD_BASE && c2 == 0x02 && a1 == REG_RESP_BASE && c1 == 0x08)
  {
    this->regResp[0] = 0x0000;
    this->regResp[1] = 0x0001;
    setCommandValuesToRead();
    this->regResp[4] = 0x0000;
    this->regResp[5] = 0x0000;
    this->regResp[6] = 0x0000;
    this->regResp[7] = 0x0000;
  }
  else if (fc == 0x17 && a2 == REG_CMD_BASE && c2 == 0x02 && a1 == REG_RESP_BASE && c1 == 0x02)
  {
    this->regResp[0] = 0x0004;
    this->regResp[1] = 0x0000;
    ESP_LOGD(TAG_HCI, "executing empty command");
  }
  else if (fc == 0x17 && a2 == REG_CMD_BASE && c2 == 0x03 && a1 == REG_RESP_BASE && c1 == 0x05)
  {
    ESP_LOGD(TAG_HCI, "executing busscan");
    this->regResp[0] = 0x0000;
    this->regResp[1] = 0x0005;
    this->regResp[2] = 0x0430;
    this->regResp[3] = 0x10ff;
    this->regResp[4] = 0xa845;
  }
  else if (fc == 0x10 && a1 == REG_BCAST_BASE)
  {
    // Drive state broadcast, nothing to prepare
  }
  else
  {
    // The original built "unknown function code fc=" + fc here, which was
    // pointer arithmetic on the literal and never showed the code.
    ESP_LOGW(TAG_HCI, "unknown function code fc=%x", fc);
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
  const uint8_t fc = req[1];

  // The replaced library computed fn + 0x80, not fn | 0x80, which overflows
  // from function code 0x80 up. Kept so even nonsense requests get the same
  // bytes.
  auto except = [&](uint8_t code) -> size_t
  {
    resp[0] = req[0];
    resp[1] = (uint8_t)(fc + 0x80);
    resp[2] = code;
    return 3;
  };
  // REPLY_ECHO: the library sent the received PDU back untouched.
  auto echo = [&]() -> size_t
  {
    memcpy(resp, req, len);
    return len;
  };
  // The library read these fields without a length check; zero-fill instead of
  // reading past the frame.
  const uint16_t f1 = len >= 4 ? rd16(req + 2) : 0;
  const uint16_t f2 = len >= 6 ? rd16(req + 4) : 0;

  if (fc == 0x17)
  {
    if (len < 11)
    {
      this->onRequestHook(fc, f1, f2, 0, 0);
      return except(EX_ILLEGAL_VALUE);
    }
    const uint16_t readAddr = rd16(req + 2);
    const uint16_t readCnt = rd16(req + 4);
    const uint16_t writeAddr = rd16(req + 6);
    const uint16_t writeCnt = rd16(req + 8);
    const uint8_t byteCnt = req[10];
    const uint8_t *wdata = req + 11;

    this->onRequestHook(fc, readAddr, readCnt, writeAddr, writeCnt);

    // Exactly the library's check: it tested the read range twice and the
    // write range not at all. Reproduced on purpose.
    if (readCnt < 1 || readCnt > MODBUS_MAX_WORDS || writeCnt < 1 || writeCnt > MODBUS_MAX_WORDS ||
        (0xFFFF - readAddr) < readCnt || byteCnt != 2 * writeCnt)
      return except(EX_ILLEGAL_VALUE);
    // Added: the library read past the frame here.
    if (len < (size_t)(11 + byteCnt))
      return except(EX_ILLEGAL_VALUE);

    bool write_ok = true;
    for (uint16_t i = 0; i < writeCnt; i++)
      if (!this->regSetChecked((uint16_t)(writeAddr + i), rd16(wdata + 2 * i)))
        write_ok = false;
    if (!write_ok)
      return except(EX_SLAVE_FAILURE);

    // readWords without MODBUS_STRICT_REG: only the first register must exist.
    if (!this->regExists(readAddr))
      return except(EX_ILLEGAL_ADDRESS);
    size_t n = 0;
    resp[n++] = req[0];
    resp[n++] = fc;
    resp[n++] = (uint8_t)(readCnt * 2);
    for (uint16_t i = 0; i < readCnt; i++)
    {
      wr16(resp + n, this->regGet((uint16_t)(readAddr + i)));
      n += 2;
    }
    return n;
  }

  if (fc == 0x10)
  {
    if (len < 7)
    {
      this->onRequestHook(fc, f1, f2, 0, 0);
      return except(EX_ILLEGAL_VALUE);
    }
    const uint16_t addr = rd16(req + 2);
    const uint16_t cnt = rd16(req + 4);
    const uint8_t byteCnt = req[6];
    const uint8_t *wdata = req + 7;

    this->onRequestHook(fc, addr, cnt, 0, 0);

    if (cnt < 1 || cnt > MODBUS_MAX_WORDS || (0xFFFF - addr) < cnt || byteCnt != 2 * cnt)
      return except(EX_ILLEGAL_VALUE);
    for (uint16_t i = 0; i < cnt; i++)
      if (!this->regExists((uint16_t)(addr + i)))
        return except(EX_ILLEGAL_ADDRESS);
    // Added after the address check so the exception codes keep their order.
    if (len < (size_t)(7 + byteCnt))
      return except(EX_ILLEGAL_VALUE);

    bool write_ok = true;
    for (uint16_t i = 0; i < cnt; i++)
      if (!this->regSetChecked((uint16_t)(addr + i), rd16(wdata + 2 * i)))
        write_ok = false;
    if (!write_ok)
      return except(EX_SLAVE_FAILURE);

    size_t n = 0;
    resp[n++] = req[0];
    resp[n++] = fc;
    wr16(resp + n, addr); n += 2;
    wr16(resp + n, cnt); n += 2;
    return n;
  }

  if (fc == 0x03)  // Halteregister lesen
  {
    this->onRequestHook(fc, f1, f2, 0, 0);
    if (f2 < 1 || f2 > MODBUS_MAX_WORDS || (0xFFFF - f1) < f2)
      return except(EX_ILLEGAL_ADDRESS);
    if (!this->regExists(f1))
      return except(EX_ILLEGAL_ADDRESS);
    size_t n = 0;
    resp[n++] = req[0];
    resp[n++] = fc;
    resp[n++] = (uint8_t)(f2 * 2);
    for (uint16_t i = 0; i < f2; i++)
    {
      wr16(resp + n, this->regGet((uint16_t)(f1 + i)));
      n += 2;
    }
    return n;
  }

  if (fc == 0x06)  // einzelnes Halteregister schreiben
  {
    this->onRequestHook(fc, f1, f2, 0, 0);
    if (!this->regWrite(f1, f2))
      return except(EX_ILLEGAL_ADDRESS);
    if (this->regGet(f1) != f2)
      return except(EX_SLAVE_FAILURE);
    return echo();
  }

  if (fc == 0x16)  // Register mit Maske schreiben
  {
    const uint16_t f3 = len >= 8 ? rd16(req + 6) : 0;
    this->onRequestHook(fc, f1, f2, f3, 0);
    const uint16_t cur = this->regGet(f1);
    const uint16_t nv = (uint16_t)((cur & f2) | (f3 & (uint16_t)~f2));
    if (!this->regWrite(f1, nv))
      return except(EX_ILLEGAL_ADDRESS);
    if (this->regGet(f1) != nv)
      return except(EX_SLAVE_FAILURE);
    return echo();
  }

  if (fc == 0x14 || fc == 0x15)  // Dateisaetze lesen bzw. schreiben
  {
    // The only codes without the onRequest hook: the library jumped straight
    // into validation, so no setValid here either.
    const uint8_t n = len >= 3 ? req[2] : 0;  // Laengenbyte der Anfrage
    const uint8_t lo = fc == 0x14 ? 0x07 : 0x09;
    const uint8_t hi = fc == 0x14 ? 0xF5 : 0xFB;
    if (n < lo || n > hi)
      return except(EX_ILLEGAL_VALUE);
    // No file handler was ever registered, so the library always ended here
    // whatever the content - reproducible without copying its out-of-bounds
    // reads.
    return except(EX_ILLEGAL_ADDRESS);
  }

  // Coils, discrete inputs and input registers never existed here, so the
  // library served these codes but always failed the register lookup.
  if (fc == 0x01 || fc == 0x02 || fc == 0x04)
  {
    this->onRequestHook(fc, f1, f2, 0, 0);
    return except(EX_ILLEGAL_ADDRESS);
  }

  if (fc == 0x05)  // einzelne Spule schreiben
  {
    this->onRequestHook(fc, f1, f2, 0, 0);
    if (f2 != 0xFF00 && f2 != 0x0000)
      return except(EX_ILLEGAL_VALUE);
    return except(EX_ILLEGAL_ADDRESS);
  }

  if (fc == 0x0F)  // mehrere Spulen schreiben
  {
    this->onRequestHook(fc, f1, f2, 0, 0);
    uint16_t bytecount_calc = f2 / 8;
    if (f2 % 8)
      bytecount_calc++;
    const uint8_t bc = len >= 7 ? req[6] : 0;
    if (f2 < 1 || f2 > MODBUS_MAX_BITS || (0xFFFF - f1) < f2 || bc != bytecount_calc)
      return except(EX_ILLEGAL_VALUE);
    return except(EX_ILLEGAL_ADDRESS);
  }

  this->onRequestHook(fc, f1, f2, 0, 0);
  return except(EX_ILLEGAL_FUNCTION);
}

void HoermannGarageEngine::setCommandValuesToRead()
{
  uint16_t regPlug2Value = 0x0000;
  uint16_t regPlug3Value = 0x0000;

  // Command was set
  const HoermannCommand *cmd = this->nextCommand.load();
  if (cmd != nullptr)
  {
    // But not yet sent
    if (commandWrittenOn == 0)
    {
      // Send it
      regPlug2Value = cmd->commandRegPlus2Value;
      regPlug3Value = cmd->commandRegPlus3Value;
      ESP_LOGI(TAG_HCI, "command start %x %x", regPlug2Value, regPlug3Value);
      commandWrittenOn = esphome::millis();
      // It was written and it can be cleared
    }
    // Subtract instead of add: adding overflows when millis() wraps and the
    // command would stick forever.
    else if (commandWrittenOn != 0 && (esphome::millis() - commandWrittenOn) > SIMULATEKEYPRESSDELAYMS)
    {
      regPlug2Value = cmd->commandEndPlus2Value;
      regPlug3Value = cmd->commandEndPlus3Value;
      ESP_LOGI(TAG_HCI, "command dispose %x %x", regPlug2Value, regPlug3Value);
      // Reset Variables
      commandWrittenOn = 0;
      this->nextCommand.store(nullptr);
    }
  }
  this->regResp[2] = regPlug2Value;
  this->regResp[3] = regPlug3Value;
}

void HoermannGarageEngine::onDoorPositonChanged(uint16_t oldVal, uint16_t val)
{
  // on First Byte changed (current)
  if ((oldVal & 0x00FF) != (val & 0x00FF))
  {
    this->state->setCurrentPosition((float)(val & 0x00FF) / 200.0f);
    if ((this->state->gotoPosition > 0.0f && this->state->state == HoermannState::State::CLOSING && this->state->gotoPosition >= this->state->currentPosition) ||
        (this->state->gotoPosition > 0.0f && this->state->state == HoermannState::State::OPENING && this->state->gotoPosition <= this->state->currentPosition))
    {
      this->stopDoor();
      this->state->setGotoPosition(0.0f);
    }
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
void HoermannGarageEngine::onCounterWrite(uint16_t val)
{
  uint16_t counter = val & 0xFF00;
  uint16_t command = (val & 0x00FF) << 8;
  this->regResp[0] |= counter;
  this->regResp[1] |= command;
}

/**
 * Helper to set next Command and *not* skip Current Command before end was sent
 */
bool HoermannGarageEngine::setCommand(bool cond, const HoermannCommand *command)
{
  if (cond)
  {
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
  //only send impulse if door is in a moving state
  return setCommand( this->state->state == HoermannState::State::CLOSING || 
              this->state->state == HoermannState::State::OPENING ||
              this->state->state == HoermannState::State::MOVE_HALF ||
              this->state->state == HoermannState::State::MOVE_VENTING , &HoermannCommand::STARTIMPULSE);
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
  return setCommand((on && !this->state->lightOn) || (!on && this->state->lightOn), &HoermannCommand::STARTTOGGLELAMP);
}
bool HoermannGarageEngine::toggleLight()
{
  return setCommand(true, &HoermannCommand::STARTTOGGLELAMP);
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
  this->changed = false;
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
  // A go-to-position target must not survive a movement that ended for another
  // reason - obstacle, hand transmitter, stop button. It was only cleared on
  // reaching the target, so the next run in the same direction was stopped at
  // a position nobody asked for.
  if (was_moving && !isMoving(state))
    this->gotoPosition = 0.0f;
}
void HoermannState::setValid(bool isValid)
{
  this->valid = isValid;
}
