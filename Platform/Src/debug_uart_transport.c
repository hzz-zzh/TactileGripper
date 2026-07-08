#include "debug_uart_transport.h"

#include "cmsis_os2.h"

#include <string.h>

static UART_HandleTypeDef *debugUart;
static GPIO_TypeDef *debugRs485DirPort;
static uint16_t debugRs485DirPin;
static osMutexId_t debugUartMutex;

void DebugUartTransport_Init(UART_HandleTypeDef *huart)
{
  DebugUartTransport_InitRs485(huart, NULL, 0U);
}

void DebugUartTransport_InitRs485(UART_HandleTypeDef *huart,
                                  GPIO_TypeDef *dir_port,
                                  uint16_t dir_pin)
{
  const osMutexAttr_t mutexAttr = {
    .name = "debugUartTx"
  };

  debugUart = huart;
  debugRs485DirPort = dir_port;
  debugRs485DirPin = dir_pin;
  /* 首次调用发生在内核初始化前，仅登记句柄；内核就绪后再创建互斥量。 */
  if ((debugUartMutex == NULL) &&
      (osKernelGetState() != osKernelInactive))
  {
    debugUartMutex = osMutexNew(&mutexAttr);
  }
}

static void DebugUartTransport_SetRs485Tx(bool enabled)
{
  if ((debugRs485DirPort != NULL) && (debugRs485DirPin != 0U))
  {
    HAL_GPIO_WritePin(debugRs485DirPort, debugRs485DirPin,
                      enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }
}

bool DebugUartTransport_Write(const char *text, uint32_t timeout_ms)
{
  size_t length;
  HAL_StatusTypeDef result;
  bool mutexLocked = false;
  uint32_t mutexTimeoutTicks = timeout_ms;

  if ((debugUart == NULL) || (text == NULL))
  {
    return false;
  }
  length = strlen(text);
  if (length == 0U)
  {
    return true;
  }

  if ((debugUartMutex != NULL) &&
      (osKernelGetState() == osKernelRunning))
  {
    mutexTimeoutTicks =
      ((timeout_ms * osKernelGetTickFreq()) + 999U) / 1000U;
    if (osMutexAcquire(debugUartMutex, mutexTimeoutTicks) != osOK)
    {
      return false;
    }
    mutexLocked = true;
  }

  DebugUartTransport_SetRs485Tx(true);
  result = HAL_UART_Transmit(debugUart, (const uint8_t *)text,
                             (uint16_t)length, timeout_ms);
  if (result == HAL_OK)
  {
    uint32_t startTick = HAL_GetTick();
    while (__HAL_UART_GET_FLAG(debugUart, UART_FLAG_TC) == RESET)
    {
      if ((HAL_GetTick() - startTick) > timeout_ms)
      {
        result = HAL_TIMEOUT;
        break;
      }
    }
  }
  DebugUartTransport_SetRs485Tx(false);
  if (mutexLocked)
  {
    (void)osMutexRelease(debugUartMutex);
  }
  return result == HAL_OK;
}
