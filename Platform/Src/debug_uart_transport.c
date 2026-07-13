#include "debug_uart_transport.h"

#include "cmsis_os2.h"

#include <string.h>

static UART_HandleTypeDef *debugUart;
static osMutexId_t debugUartMutex;
static GPIO_TypeDef *debugRs485DePort;
static uint16_t debugRs485DePin;
static volatile uint8_t debugUartTxActive;
static volatile uint32_t debugUartTxQuietUntilTick;

static void DebugUartTransport_FlushRx(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    return;
  }

  __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF |
                               UART_CLEAR_FEF | UART_CLEAR_PEF);
  __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
}

void DebugUartTransport_Init(UART_HandleTypeDef *huart)
{
  DebugUartTransport_InitRs485(huart, NULL, 0U);
}

void DebugUartTransport_InitRs485(UART_HandleTypeDef *huart,
                                  GPIO_TypeDef *de_port,
                                  uint16_t de_pin)
{
  const osMutexAttr_t mutexAttr = {
    .name = "debugUartTx"
  };

  debugUart = huart;
  debugRs485DePort = de_port;
  debugRs485DePin = de_pin;
  debugUartTxActive = 0U;
  debugUartTxQuietUntilTick = 0U;
  if (debugRs485DePort != NULL)
  {
    /* 默认保持接收态，发送调试日志时再打开485驱动。 */
    HAL_GPIO_WritePin(debugRs485DePort, debugRs485DePin, GPIO_PIN_RESET);
  }

  /* 内核启动前只登记句柄，内核起来后再创建互斥量。 */
  if ((debugUartMutex == NULL) &&
      (osKernelGetState() != osKernelInactive))
  {
    debugUartMutex = osMutexNew(&mutexAttr);
  }
}

bool DebugUartTransport_Write(const char *text, uint32_t timeout_ms)
{
  size_t length;

  if (text == NULL)
  {
    return false;
  }
  length = strlen(text);
  if (length > UINT16_MAX)
  {
    return false;
  }
  return DebugUartTransport_WriteBuffer((const uint8_t *)text,
                                        (uint16_t)length,
                                        timeout_ms);
}

bool DebugUartTransport_WriteBuffer(const uint8_t *data,
                                    uint16_t length,
                                    uint32_t timeout_ms)
{
  HAL_StatusTypeDef result;
  bool mutexLocked = false;
  uint32_t mutexTimeoutTicks = timeout_ms;

  if ((debugUart == NULL) || (data == NULL))
  {
    return false;
  }
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

  if (debugRs485DePort != NULL)
  {
    HAL_GPIO_WritePin(debugRs485DePort, debugRs485DePin, GPIO_PIN_SET);
  }
  debugUartTxActive = 1U;
  result = HAL_UART_Transmit(debugUart, data, length, timeout_ms);
  debugUartTxQuietUntilTick = HAL_GetTick() + 2U;
  DebugUartTransport_FlushRx(debugUart);
  debugUartTxActive = 0U;
  if (debugRs485DePort != NULL)
  {
    HAL_GPIO_WritePin(debugRs485DePort, debugRs485DePin, GPIO_PIN_RESET);
  }

  if (mutexLocked)
  {
    (void)osMutexRelease(debugUartMutex);
  }
  return result == HAL_OK;
}

bool DebugUartTransport_IsTxActive(UART_HandleTypeDef *huart)
{
  if ((huart == NULL) || (huart != debugUart))
  {
    return false;
  }

  if (debugUartTxActive != 0U)
  {
    return true;
  }

  return ((int32_t)(debugUartTxQuietUntilTick - HAL_GetTick()) > 0);
}
