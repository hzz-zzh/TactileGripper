#include "debug_uart_transport.h"

#include "cmsis_os2.h"

#include <string.h>

static UART_HandleTypeDef *debugUart;
static osMutexId_t debugUartMutex;

void DebugUartTransport_Init(UART_HandleTypeDef *huart)
{
  const osMutexAttr_t mutexAttr = {
    .name = "debugUartTx"
  };

  debugUart = huart;
  /* 首次调用发生在内核初始化前，仅登记句柄；内核就绪后再创建互斥量。 */
  if ((debugUartMutex == NULL) &&
      (osKernelGetState() != osKernelInactive))
  {
    debugUartMutex = osMutexNew(&mutexAttr);
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

  result = HAL_UART_Transmit(debugUart, (const uint8_t *)text,
                             (uint16_t)length, timeout_ms);
  if (mutexLocked)
  {
    (void)osMutexRelease(debugUartMutex);
  }
  return result == HAL_OK;
}
