#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef struct EspUart EspUart;

typedef struct {
  uint8_t uart_ch;
  uint8_t baudrate_mode;
  uint32_t baudrate;
} EspUartConfig;

typedef struct {
  uint32_t rx_cnt;
  uint32_t tx_cnt;
  uint32_t baudrate_cur;
} EspUartState;

EspUart esp_uart_enable(EspUartConfig* cfg);

void esp_uart_disable(EspUart* esp_uart);

void esp_uart_set_config(EspUart* esp_uart, EspUartConfig* cfg);

void esp_uart_get_config(EspUart* esp_part, EspUartConfig* cfg);

void esp_uart_get_state(EspUart* esp_uart, EspUartState* st);
