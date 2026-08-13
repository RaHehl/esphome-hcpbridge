#pragma once
#include "FreeRTOS.h"
inline void vTaskDelay(TickType_t) {}
inline void vTaskDelete(void *) {}
inline int xTaskCreatePinnedToCore(void (*)(void *), const char *, int, void *, int, TaskHandle_t *, int) { return 1; }
// The single-core branch; unreachable while the stub claims two cores, but it
// has to compile, or the variant that uses it breaks without the harness saying so.
inline int xTaskCreate(void (*)(void *), const char *, int, void *, int, TaskHandle_t *) { return 1; }
#define pdPASS 1
typedef int BaseType_t;
