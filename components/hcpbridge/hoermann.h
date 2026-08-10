// Credits to https://github.com/Gifford47/HCPBridgeMqtt for the initial code base

#ifndef HOERMANN_H_
#define HOERMANN_H_
#include <atomic>
#include <cstdint>

#include "modbus_rtu.h"

#define SLAVE_ID 2
#define HCP_BAUD 57600
#define SIMULATEKEYPRESSDELAYMS 100

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
    static const HoermannCommand STARTTOGGLELAMP;
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
    State state = CLOSED;
    const char *debugMessage = "initial";  // nur Literale, damit der Bus-Task keinen Heap anfasst
    unsigned long lastModbusRespone = 0;
    bool changed = false;
    bool debMessage = false;
    float gotoPosition = 0.0f;
    bool valid = false;

    void setTargetPosition(float targetPosition);
    void setGotoPosition(float setPosition);
    void setCurrentPosition(float currentPosition);
    void setLigthOn(bool lightOn);
    void setRelayOn(bool relayOn);
    void recordModbusResponse();
    void clearChanged();
    void clearDebug();
    void setState(State state);
    void setValid(bool isValid);

};

// Registerbereiche des Hoermann-Busses. Der Antrieb schreibt Befehle nach
// 0x9C41 und seinen Zustand nach 0x9D31, und liest unsere Antwort aus 0x9CB9.
#define REG_CMD_BASE 0x9C41
#define REG_CMD_COUNT 3
#define REG_BCAST_BASE 0x9D31
#define REG_BCAST_COUNT 9
#define REG_RESP_BASE 0x9CB9
#define REG_RESP_COUNT 8

// Werte wie in der frueheren Bibliothek, damit sich die Station bei
// fehlerhaften Telegrammen genauso verhaelt wie bisher.
#define MODBUS_MAX_WORDS 0x007D
#define EX_ILLEGAL_FUNCTION 0x01
#define EX_ILLEGAL_ADDRESS 0x02
#define EX_ILLEGAL_VALUE 0x03
#define EX_SLAVE_FAILURE 0x04

class HoermannGarageEngine
{
public:
    HoermannState *state = new HoermannState();

    static HoermannGarageEngine& getInstance();

    void setup(int8_t rx, int8_t tx, int8_t rts);
    void handleModbus();

    // Beantwortet ein vollstaendiges Telegramm; liefert die Antwortlaenge
    // oder 0, wenn geschwiegen werden soll.
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
     * Helper to set next Command and *not* skip Current Command before end was sent
     */
    void setCommand(bool cond, const HoermannCommand *command);

    /**
     * Control Functions
     */
    void stopDoor();
    void closeDoor();
    void openDoor();
    void impulseDoor();
    void halfPositionDoor();
    void ventilationPositionDoor();
    void turnLight(bool on);
    void toggleLight();
    void setPosition(int setPosition);

private:
    HoermannGarageEngine(){};
    esphome::hcpbridge::ModbusRtuServer mb;       // eigener RTU-Server, keine Fremdbibliothek
    uint16_t regBcast[REG_BCAST_COUNT] = {0};     // 0x9D31, Zustand des Antriebs
    uint16_t regResp[REG_RESP_COUNT] = {0};       // 0x9CB9, unsere Antwort
    std::atomic<const HoermannCommand *> nextCommand{nullptr}; // wird zwischen Hauptschleife und Bus-Task geteilt
    unsigned long commandWrittenOn = 0;           // When was last command written (wait 100ms before end of command is transmitted)
};
#endif