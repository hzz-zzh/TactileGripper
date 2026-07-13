#ifndef DEBUG_UART_TRANSPORT_H
#define DEBUG_UART_TRANSPORT_H

#include "stm32h7xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void DebugUartTransport_Init(UART_HandleTypeDef *huart);
void DebugUartTransport_InitRs485(UART_HandleTypeDef *huart,
                                  GPIO_TypeDef *de_port,
                                  uint16_t de_pin);
bool DebugUartTransport_Write(const char *text, uint32_t timeout_ms);
bool DebugUartTransport_WriteBuffer(const uint8_t *data,
                                    uint16_t length,
                                    uint32_t timeout_ms);
bool DebugUartTransport_IsTxActive(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif
