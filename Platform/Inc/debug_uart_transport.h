#ifndef DEBUG_UART_TRANSPORT_H
#define DEBUG_UART_TRANSPORT_H

#include "stm32h7xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void DebugUartTransport_Init(UART_HandleTypeDef *huart);
bool DebugUartTransport_Write(const char *text, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif
