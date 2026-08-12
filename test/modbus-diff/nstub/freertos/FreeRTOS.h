#pragma once
#include <cstdint>
#include <cstddef>
typedef void* QueueHandle_t;
typedef void* TaskHandle_t;
typedef int BaseType_t;
typedef uint32_t TickType_t;
#define pdMS_TO_TICKS(x) (x)
#define pdTRUE 1
#define portMAX_DELAY 0xFFFFFFFF
#define configMAX_PRIORITIES 25
struct uart_ev_sim { int type; size_t size; };
inline int xQueueReceive(QueueHandle_t q,void*out,TickType_t);
inline void xQueueReset(QueueHandle_t){}
