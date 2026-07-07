#include "debug_uart_transport.h"

#include "cmsis_os2.h"

#include <string.h>

static UART_HandleTypeDef *debugUart;
static osMutexId_t debugUartMutex;
static GPIO_TypeDef *debugDirectionPort;
static uint16_t debugDirectionPin;
static GPIO_PinState debugTxEnableState = GPIO_PIN_SET;

static GPIO_PinState DebugUartTransport_RxState(void)
{
  return (debugTxEnableState == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
}

static void DebugUartTransport_EnsureMutex(void)
{
  const osMutexAttr_t mutexAttr = {
    .name = "debugUartTx"
  };

  if ((debugUartMutex == NULL) &&
      (osKernelGetState() != osKernelInactive))
  {
    debugUartMutex = osMutexNew(&mutexAttr);
  }
}

void DebugUartTransport_Init(UART_HandleTypeDef *huart)
{
  DebugUartTransport_InitRs485(huart, NULL, 0U, GPIO_PIN_SET);
}

void DebugUartTransport_InitRs485(UART_HandleTypeDef *huart,
                                  GPIO_TypeDef *direction_port,
                                  uint16_t direction_pin,
                                  GPIO_PinState tx_enable_state)
{
  debugUart = huart;
  debugDirectionPort = direction_port;
  debugDirectionPin = direction_pin;
  debugTxEnableState = tx_enable_state;

  if (debugDirectionPort != NULL)
  {
    HAL_GPIO_WritePin(debugDirectionPort, debugDirectionPin,
                     DebugUartTransport_RxState());
  }

  /* 调试串口可能在内核启动前初始化，互斥量延后到内核运行后创建。 */
  DebugUartTransport_EnsureMutex();
}

bool DebugUartTransport_Write(const char *text, uint32_t timeout_ms)
{
  size_t length;
  HAL_StatusTypeDef result;
  bool mutexLocked = false;
  uint32_t mutexTimeoutTicks = timeout_ms;
  uint32_t txTimeoutMs;
  uint32_t baudRate;

  if ((debugUart == NULL) || (text == NULL))
  {
    return false;
  }

  length = strlen(text);
  if (length == 0U)
  {
    return true;
  }

  /*
   * RS485 调试口可能降到 460800。按 10bit/字节估算发送时间，
   * 再留出调度余量，避免长日志因为传入 timeout 偏小被截断。
   */
  baudRate = debugUart->Init.BaudRate;
  if (baudRate == 0U)
  {
    baudRate = 115200U;
  }
  txTimeoutMs = (uint32_t)(((length * 10U * 1000U) + baudRate - 1U) /
                           baudRate) + 5U;
  if (txTimeoutMs < timeout_ms)
  {
    txTimeoutMs = timeout_ms;
  }

  DebugUartTransport_EnsureMutex();
  if ((debugUartMutex != NULL) &&
      (osKernelGetState() == osKernelRunning))
  {
    mutexTimeoutTicks =
      ((txTimeoutMs * osKernelGetTickFreq()) + 999U) / 1000U;
    if (osMutexAcquire(debugUartMutex, mutexTimeoutTicks) != osOK)
    {
      return false;
    }
    mutexLocked = true;
  }

  if (debugDirectionPort != NULL)
  {
    HAL_GPIO_WritePin(debugDirectionPort, debugDirectionPin,
                     debugTxEnableState);
  }

  result = HAL_UART_Transmit(debugUart, (const uint8_t *)text,
                             (uint16_t)length, txTimeoutMs);

  if (debugDirectionPort != NULL)
  {
    uint32_t startTick = HAL_GetTick();
    while ((__HAL_UART_GET_FLAG(debugUart, UART_FLAG_TC) == RESET) &&
           ((HAL_GetTick() - startTick) <= txTimeoutMs))
    {
      /* 等待移位寄存器发送完成，再释放 RS485 总线。 */
    }
    HAL_GPIO_WritePin(debugDirectionPort, debugDirectionPin,
                     DebugUartTransport_RxState());
  }

  if (mutexLocked)
  {
    (void)osMutexRelease(debugUartMutex);
  }

  return result == HAL_OK;
}
