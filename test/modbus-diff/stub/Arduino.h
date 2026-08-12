#pragma once
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>
#include <cstdio>
typedef std::string String;
typedef uint8_t byte;
unsigned long millis();
inline void yield() {}
inline void delay(unsigned long) {}
unsigned long micros();
inline void delayMicroseconds(unsigned long) {}
inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
#define OUTPUT 1
#define HIGH 1
#define LOW 0
#define PROGMEM
#define pgm_read_word(a) (*(const uint16_t*)(a))
#define pgm_read_byte(a) (*(const uint8_t*)(a))
class __FlashStringHelper;
#define F(x) (x)
#define highByte(w) ((uint8_t)((w) >> 8))
#define lowByte(w)  ((uint8_t)((w) & 0xFF))
#include "Stream.h"
#define bitSet(v,b)   ((v) |= (1UL << (b)))
#define bitClear(v,b) ((v) &= ~(1UL << (b)))
#define bitRead(v,b)  (((v) >> (b)) & 0x01)
#define bitWrite(v,b,x) ((x) ? bitSet(v,b) : bitClear(v,b))
#define ESP_LOGD(t,...) do{}while(0)
#define ESP_LOGI(t,...) do{}while(0)
#define ESP_LOGW(t,...) do{}while(0)
#define ESP_LOGE(t,...) do{}while(0)
#define ESP_LOGV(t,...) do{}while(0)
#define SERIAL_8E1 0
inline void vTaskDelay(int) {}
inline void vTaskDelete(void*) {}
typedef void* TaskHandle_t;
#define configMAX_PRIORITIES 25
inline int xTaskCreatePinnedToCore(void(*)(void*),const char*,int,void*,int,TaskHandle_t*,int){return 1;}

