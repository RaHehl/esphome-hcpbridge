#pragma once
#include "FreeRTOS.h"
inline void vTaskDelay(TickType_t){}
inline void vTaskDelete(void*){}
inline int xTaskCreatePinnedToCore(void(*)(void*),const char*,int,void*,int,TaskHandle_t*,int){return 1;}
