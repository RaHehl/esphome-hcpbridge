#pragma once
#include <cstdint>
#include <cstddef>
#include "freertos/FreeRTOS.h"
#include "uartsim.h"
typedef int uart_port_t;
#define UART_NUM_2 2
#define UART_PIN_NO_CHANGE (-1)
typedef enum { UART_DATA, UART_BREAK, UART_BUFFER_FULL, UART_FIFO_OVF, UART_FRAME_ERR, UART_PARITY_ERR } uart_event_type_t;
typedef struct { uart_event_type_t type; size_t size; } uart_event_t;
typedef enum { UART_DATA_8_BITS } uart_word_length_t;
typedef enum { UART_PARITY_DISABLE, UART_PARITY_EVEN } uart_parity_t;
typedef enum { UART_STOP_BITS_1 } uart_stop_bits_t;
typedef enum { UART_HW_FLOWCTRL_DISABLE } uart_hw_flowcontrol_t;
typedef enum { UART_SCLK_DEFAULT } uart_sclk_t;
typedef enum { UART_MODE_UART, UART_MODE_RS485_HALF_DUPLEX } uart_mode_t;
typedef struct { int baud_rate; uart_word_length_t data_bits; uart_parity_t parity;
                 uart_stop_bits_t stop_bits; uart_hw_flowcontrol_t flow_ctrl;
                 uint8_t rx_flow_ctrl_thresh; uart_sclk_t source_clk; } uart_config_t;
typedef int esp_err_t;
#define ESP_OK 0
inline esp_err_t uart_param_config(uart_port_t,const uart_config_t*){return 0;}
inline esp_err_t uart_set_pin(uart_port_t,int,int,int,int){return 0;}
inline esp_err_t uart_driver_install(uart_port_t,int,int,int,QueueHandle_t*q,int){ if(q) *q=(QueueHandle_t)&g_uart; return 0;}
inline esp_err_t uart_set_mode(uart_port_t,uart_mode_t){return 0;}
inline esp_err_t uart_set_rx_timeout(uart_port_t,uint8_t){return 0;}
inline esp_err_t uart_set_rx_full_threshold(uart_port_t,int){return 0;}
inline esp_err_t uart_flush_input(uart_port_t){ g_uart.rx_pos=g_uart.rx_len; return 0;}
inline int uart_read_bytes(uart_port_t,uint8_t*b,size_t n,TickType_t){
  size_t avail = g_uart.rx_len - g_uart.rx_pos;
  size_t take = n < avail ? n : avail;
  memcpy(b, g_uart.rx + g_uart.rx_pos, take); g_uart.rx_pos += take; return (int)take; }
inline int uart_write_bytes(uart_port_t,const char*d,size_t n){
  memcpy(g_uart.tx + g_uart.tx_len, d, n); g_uart.tx_len += n; return (int)n; }
inline esp_err_t uart_wait_tx_done(uart_port_t,TickType_t){return 0;}
// muss nach der Ereignisstruktur stehen
inline int xQueueReceive(QueueHandle_t, void *out, TickType_t) {
  if (!g_uart.pending) return 0;
  g_uart.pending = 0;
  uart_event_t *e = (uart_event_t *)out;
  e->type = UART_DATA; e->size = g_uart.pending_size;
  return 1;  // pdTRUE
}
