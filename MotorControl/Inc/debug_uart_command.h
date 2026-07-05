#ifndef DEBUG_UART_COMMAND_H
#define DEBUG_UART_COMMAND_H

#include "stm32h7xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  DEBUG_UART_COMMAND_NONE = 0,
  DEBUG_UART_COMMAND_START,
  DEBUG_UART_COMMAND_STOP,
  DEBUG_UART_COMMAND_HOME,
  DEBUG_UART_COMMAND_CLEAR_FAULT,
  DEBUG_UART_COMMAND_STATUS,
  DEBUG_UART_COMMAND_ZERO,
  DEBUG_UART_COMMAND_TARGET_TURN,
  DEBUG_UART_COMMAND_TARGET_POSITION,
  DEBUG_UART_COMMAND_INVALID
} DebugUartCommandType_t;

typedef struct
{
  DebugUartCommandType_t type;
  int32_t target_turn_milli;
  int16_t target_position_permille;
} DebugUartCommand_t;

void DebugUartCommand_Init(UART_HandleTypeDef *huart);
bool DebugUartCommand_Poll(DebugUartCommand_t *command);
void DebugUartCommand_RxCpltCallback(UART_HandleTypeDef *huart);
void DebugUartCommand_ErrorCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif
