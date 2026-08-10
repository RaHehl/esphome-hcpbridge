// Credits to https://github.com/Gifford47/HCPBridgeMqtt for the initial code base

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

void DelayHandler(void)
{
  HoermannGarageEngine::getInstance().handleModbus();
}

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
  this->mb.begin(UART_NUM_2, rx, tx, rts, HCP_BAUD, SLAVE_ID);

  xTaskCreatePinnedToCore(
      modbusServeTask,          /* Function to implement the task */
      "ModBusTask",             /* Name of the task */
      4096,                     /* Stack size in words */
      NULL,                     /* Task input parameter */
      configMAX_PRIORITIES - 1, /* Priority */
      &modBusTask,              /* Task handle */
      1);                       /* Core */
}

void HoermannGarageEngine::handleModbus()
{
  this->mb.poll();
}

static inline uint16_t rd16(const uint8_t *p) { return (uint16_t)(p[0] << 8) | p[1]; }
static inline void wr16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)(v & 0xFF); }

/**
 * Beantwortet ein vollstaendiges Telegramm (ohne Pruefsumme).
 *
 * Reihenfolge wie in der frueheren Bibliothek: erst die Antwortregister mit
 * den Vorgabewerten fuellen, dann die Schreibzugriffe des Antriebs anwenden
 * (die veraendern 0x9CB9 noch), danach antworten.
 */
size_t HoermannGarageEngine::onFrame(const uint8_t *req, size_t len, uint8_t *resp)
{
  const uint8_t fc = req[1];
  this->state->recordModbusResponse();

  if (fc == 0x17 && len >= 11)
  {
    const uint16_t readAddr = rd16(req + 2);
    const uint16_t readCnt = rd16(req + 4);
    const uint16_t writeAddr = rd16(req + 6);
    const uint16_t writeCnt = rd16(req + 8);
    const uint8_t byteCnt = req[10];
    const uint8_t *wdata = req + 11;

    if (writeAddr == REG_CMD_BASE && writeCnt == 0x02 && readAddr == REG_RESP_BASE && readCnt == 0x08)
    {
      this->regResp[0] = 0x0000;
      this->regResp[1] = 0x0001;
      setCommandValuesToRead();
      this->regResp[4] = 0x0000;
      this->regResp[5] = 0x0000;
      this->regResp[6] = 0x0000;
      this->regResp[7] = 0x0000;
    }
    else if (writeAddr == REG_CMD_BASE && writeCnt == 0x02 && readAddr == REG_RESP_BASE && readCnt == 0x02)
    {
      this->regResp[0] = 0x0004;
      this->regResp[1] = 0x0000;
      ESP_LOGD(TAG_HCI, "executing empty command");
    }
    else if (writeAddr == REG_CMD_BASE && writeCnt == 0x03 && readAddr == REG_RESP_BASE && readCnt == 0x05)
    {
      ESP_LOGD(TAG_HCI, "executing busscan");
      this->regResp[0] = 0x0000;
      this->regResp[1] = 0x0005;
      this->regResp[2] = 0x0430;
      this->regResp[3] = 0x10ff;
      this->regResp[4] = 0xa845;
    }
    else
    {
      ESP_LOGW(TAG_HCI, "unexpected 0x17 read=%04x/%u write=%04x/%u", readAddr, readCnt, writeAddr, writeCnt);
    }

    // Schreibzugriffe uebernehmen
    for (uint16_t i = 0; i < writeCnt && (11 + 2 * i + 1) < len && 2 * i + 1 < byteCnt; i++)
    {
      const uint16_t val = rd16(wdata + 2 * i);
      const uint16_t idx = (uint16_t)(writeAddr + i - REG_CMD_BASE);
      if (idx < REG_CMD_COUNT)
      {
        if (idx == 0)
          this->onCounterWrite(val);
        this->regCmd[idx] = val;
      }
    }

    // Antwort zusammensetzen
    size_t n = 0;
    resp[n++] = req[0];
    resp[n++] = fc;
    resp[n++] = (uint8_t)(readCnt * 2);
    for (uint16_t i = 0; i < readCnt; i++)
    {
      const uint16_t idx = (uint16_t)(readAddr + i - REG_RESP_BASE);
      wr16(resp + n, idx < REG_RESP_COUNT ? this->regResp[idx] : 0x0000);
      n += 2;
    }
    this->state->setValid(true);
    return n;
  }

  if (fc == 0x10 && len >= 7)
  {
    const uint16_t addr = rd16(req + 2);
    const uint16_t cnt = rd16(req + 4);
    const uint8_t byteCnt = req[6];
    const uint8_t *wdata = req + 7;

    for (uint16_t i = 0; i < cnt && (7 + 2 * i + 1) < len && 2 * i + 1 < byteCnt; i++)
    {
      const uint16_t val = rd16(wdata + 2 * i);
      const uint16_t idx = (uint16_t)(addr + i - REG_BCAST_BASE);
      if (idx >= REG_BCAST_COUNT)
        continue;
      const uint16_t old = this->regBcast[idx];
      if (idx == 1)
        this->onDoorPositonChanged(old, val);
      else if (idx == 2)
        this->onCurrentStateChanged(old, val);
      else if (idx == 6)
        this->onRegSevenChanged(old, val);
      this->regBcast[idx] = val;
    }

    this->state->setValid(true);
    size_t n = 0;
    resp[n++] = req[0];
    resp[n++] = fc;
    wr16(resp + n, addr); n += 2;
    wr16(resp + n, cnt); n += 2;
    return n;
  }

  this->state->debugMessage = "unknown function code";
  this->state->debMessage = true;
  ESP_LOGW(TAG_HCI, "unknown function code fc=%x", fc);
  return 0;
}

void HoermannGarageEngine::setCommandValuesToRead()
{
  uint16_t regPlug2Value = 0x0000;
  uint16_t regPlug3Value = 0x0000;

  // Command was set
  if (nextCommand != nullptr)
  {
    // But not yet sent
    if (commandWrittenOn == 0)
    {
      // Send it
      regPlug2Value = nextCommand->commandRegPlus2Value;
      regPlug3Value = nextCommand->commandRegPlus3Value;
      ESP_LOGI(TAG_HCI, "command start %x %x", regPlug2Value, regPlug3Value);
      commandWrittenOn = esphome::millis();
      // It was written and it can be cleared
    }
    else if (commandWrittenOn != 0 && (commandWrittenOn + SIMULATEKEYPRESSDELAYMS) < esphome::millis())
    {
      regPlug2Value = nextCommand->commandEndPlus2Value;
      regPlug3Value = nextCommand->commandEndPlus3Value;
      ESP_LOGI(TAG_HCI, "command dispose %x %x", regPlug2Value, regPlug3Value);
      // Reset Variables
      commandWrittenOn = 0;
      nextCommand = nullptr;
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
void HoermannGarageEngine::setCommand(bool cond, const HoermannCommand *command)
{
  if (cond)
  {
    if (nextCommand != nullptr)
    {
      ESP_LOGW(TAG_HCI, "Last Command was not yet fetched by modbus!");
    }
    else
    {
      nextCommand = command;
    }
  }
}

/**
 * Control Functions
 */
void HoermannGarageEngine::stopDoor()
{
  //only send impulse if door is in a moving state
  setCommand( this->state->state == HoermannState::State::CLOSING || 
              this->state->state == HoermannState::State::OPENING ||
              this->state->state == HoermannState::State::MOVE_HALF ||
              this->state->state == HoermannState::State::MOVE_VENTING , &HoermannCommand::STARTIMPULSE);
}
void HoermannGarageEngine::closeDoor()
{
  setCommand(true, &HoermannCommand::STARTCLOSEDOOR);
}
void HoermannGarageEngine::openDoor()
{
  setCommand(true, &HoermannCommand::STARTOPENDOOR);
}
void HoermannGarageEngine::impulseDoor()
{
  setCommand(true, &HoermannCommand::STARTIMPULSE);
}
void HoermannGarageEngine::halfPositionDoor()
{
  setCommand(true, &HoermannCommand::STARTOPENDOORHALF);
}
void HoermannGarageEngine::ventilationPositionDoor()
{
  setCommand(true, &HoermannCommand::STARTVENTPOSITION);
}
void HoermannGarageEngine::turnLight(bool on)
{
  setCommand((on && !this->state->lightOn) || (!on && this->state->lightOn), &HoermannCommand::STARTTOGGLELAMP);
}
void HoermannGarageEngine::toggleLight()
{
  setCommand(true, &HoermannCommand::STARTTOGGLELAMP);
}
void HoermannGarageEngine::setPosition(int setPosition)
{
  // First and last movement segments seem a bit inconsistent on Promatic4, so it's better to leave it to fully open or close.
  if (setPosition <= 5)
    closeDoor();
  else if (setPosition >= 95)
    openDoor();
  else if ((setPosition > 5) && (setPosition < 95))
  {
    this->state->setGotoPosition(static_cast<float>(setPosition) / 100.0f);
    setCommand(this->state->currentPosition < this->state->gotoPosition, &HoermannCommand::STARTOPENDOOR);
    setCommand(this->state->currentPosition > this->state->gotoPosition, &HoermannCommand::STARTCLOSEDOOR);
  }
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
void HoermannState::recordModbusResponse()
{
  this->lastModbusRespone = esphome::millis();
}
void HoermannState::clearChanged()
{
  this->changed = false;
}
void HoermannState::clearDebug()
{
  this->debMessage = false;
  this->debugMessage = "Initial";
}
long HoermannState::responseAge()
{
  if (this->lastModbusRespone == 0)
  {
    return -1;
  }
  long diff = esphome::millis() - lastModbusRespone;
  if (diff < 0)
  {
    return -2;
  }
  return diff / 1000;
}
void HoermannState::setState(State state)
{
  this->state = state;
  this->changed = true;
}
void HoermannState::setValid(bool isValid)
{
  this->valid = isValid;
}
