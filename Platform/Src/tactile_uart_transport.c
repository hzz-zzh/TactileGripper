#include "tactile_uart_transport.h"

#include "cmsis_os2.h"

#include <string.h>

static uint32_t TactileUartTransport_NowMs(void)
{
  return HAL_GetTick();
}

static void TactileUartTransport_YieldOneTick(void)
{
  if (osKernelGetState() == osKernelRunning)
  {
    (void)osDelay(1U);
  }
}

static void TactileUartTransport_InvalidateDCache(uint8_t *buffer,
                                                  uint16_t length)
{
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
  uintptr_t start = (uintptr_t)buffer & ~(uintptr_t)31U;
  uintptr_t end = ((uintptr_t)buffer + (uintptr_t)length + 31U) &
                  ~(uintptr_t)31U;

  SCB_InvalidateDCache_by_Addr((uint32_t *)start, (int32_t)(end - start));
#else
  (void)buffer;
  (void)length;
#endif
}

static void TactileUartTransport_ClearRx(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    return;
  }

  __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF |
                               UART_CLEAR_FEF | UART_CLEAR_PEF);
  __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
}

void TactileUartTransport_Init(TactileUartTransport_t *transport,
                               UART_HandleTypeDef *huart)
{
  if (transport == NULL)
  {
    return;
  }

  (void)memset(transport, 0, sizeof(*transport));
  transport->huart = huart;
}

bool TactileUartTransport_WriteRead(TactileUartTransport_t *transport,
                                    const uint8_t *tx_data,
                                    uint16_t tx_len,
                                    uint8_t *rx_data,
                                    uint16_t rx_capacity,
                                    uint16_t expected_rx_len,
                                    uint16_t *actual_rx_len,
                                    uint32_t timeout_ms)
{
  HAL_StatusTypeDef status;
  uint32_t startTick;
  uint16_t rxCount = 0U;
  uint16_t previewLength;

  if ((transport == NULL) || (transport->huart == NULL) ||
      (tx_data == NULL) || (tx_len == 0U) ||
      (rx_data == NULL) || (rx_capacity == 0U) ||
      (expected_rx_len == 0U) || (expected_rx_len > rx_capacity))
  {
    return false;
  }

  if (actual_rx_len != NULL)
  {
    *actual_rx_len = 0U;
  }
  transport->stats.last_rx_len = 0U;
  transport->stats.last_rx_capacity = rx_capacity;
  (void)memset(transport->stats.last_rx_preview, 0,
               sizeof(transport->stats.last_rx_preview));

  (void)HAL_UART_Abort(transport->huart);
  transport->huart->ErrorCode = HAL_UART_ERROR_NONE;
  TactileUartTransport_ClearRx(transport->huart);
  (void)memset(rx_data, 0, rx_capacity);
  TactileUartTransport_InvalidateDCache(rx_data, rx_capacity);

  /*
   * 先打开 DMA 接收再发送 2 字节读取命令，避免传感器快速回帧时丢首字节。
   * 读取逻辑必须等到完整 96 字节或超时，不能用短暂静默判断帧结束。
   */
  status = HAL_UART_Receive_DMA(transport->huart, rx_data, rx_capacity);
  transport->stats.last_hal_status = (uint32_t)status;
  transport->stats.last_hal_error = HAL_UART_GetError(transport->huart);
  if (status != HAL_OK)
  {
    TactileUartTransport_ClearRx(transport->huart);
    transport->stats.rx_error_count++;
    return false;
  }

  if (transport->huart->hdmarx != NULL)
  {
    __HAL_DMA_DISABLE_IT(transport->huart->hdmarx, DMA_IT_HT);
  }

  status = HAL_UART_Transmit(transport->huart, (uint8_t *)tx_data,
                             tx_len, timeout_ms);
  transport->stats.last_hal_status = (uint32_t)status;
  transport->stats.last_hal_error = HAL_UART_GetError(transport->huart);
  if (status != HAL_OK)
  {
    (void)HAL_UART_DMAStop(transport->huart);
    TactileUartTransport_ClearRx(transport->huart);
    transport->stats.tx_error_count++;
    return false;
  }

  startTick = TactileUartTransport_NowMs();
  while ((TactileUartTransport_NowMs() - startTick) <= timeout_ms)
  {
    if (transport->huart->hdmarx != NULL)
    {
      rxCount = (uint16_t)(rx_capacity -
                           __HAL_DMA_GET_COUNTER(transport->huart->hdmarx));
    }

    if (rxCount >= expected_rx_len)
    {
      break;
    }

    if (rxCount >= rx_capacity)
    {
      break;
    }

    TactileUartTransport_YieldOneTick();
  }

  (void)HAL_UART_DMAStop(transport->huart);
  TactileUartTransport_InvalidateDCache(rx_data, rx_capacity);
  transport->stats.last_hal_error = HAL_UART_GetError(transport->huart);
  transport->stats.last_rx_len = rxCount;
  if (actual_rx_len != NULL)
  {
    *actual_rx_len = rxCount;
  }

  previewLength = (rxCount < sizeof(transport->stats.last_rx_preview)) ?
                  rxCount : (uint16_t)sizeof(transport->stats.last_rx_preview);
  if (previewLength != 0U)
  {
    (void)memcpy(transport->stats.last_rx_preview, rx_data, previewLength);
  }

  if (rxCount < expected_rx_len)
  {
    TactileUartTransport_ClearRx(transport->huart);
    transport->stats.timeout_count++;
    return false;
  }

  if (transport->stats.last_hal_error != HAL_UART_ERROR_NONE)
  {
    TactileUartTransport_ClearRx(transport->huart);
  }

  return true;
}

void TactileUartTransport_GetStats(const TactileUartTransport_t *transport,
                                   TactileUartTransportStats_t *stats)
{
  if ((transport == NULL) || (stats == NULL))
  {
    return;
  }

  *stats = transport->stats;
}
