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
                               UART_CLEAR_FEF | UART_CLEAR_PEF |
                               UART_CLEAR_IDLEF);
  __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
}

static void TactileUartTransport_PrepareRx(TactileUartTransport_t *transport,
                                           const uint8_t *tx_data,
                                           uint16_t tx_len,
                                           uint8_t *rx_data,
                                           uint16_t rx_capacity)
{
  uint16_t previewLength;

  transport->stats.last_tx_len = tx_len;
  (void)memset(transport->stats.last_tx_preview, 0,
               sizeof(transport->stats.last_tx_preview));
  previewLength = (tx_len < sizeof(transport->stats.last_tx_preview)) ?
                  tx_len : (uint16_t)sizeof(transport->stats.last_tx_preview);
  if ((tx_data != NULL) && (previewLength != 0U))
  {
    (void)memcpy(transport->stats.last_tx_preview, tx_data, previewLength);
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
}

static void TactileUartTransport_SaveRxStats(TactileUartTransport_t *transport,
                                             const uint8_t *rx_data,
                                             uint16_t rx_count,
                                             uint16_t *actual_rx_len)
{
  uint16_t previewLength;

  transport->stats.last_hal_error = HAL_UART_GetError(transport->huart);
  transport->stats.last_rx_len = rx_count;
  if (actual_rx_len != NULL)
  {
    *actual_rx_len = rx_count;
  }

  previewLength = (rx_count < sizeof(transport->stats.last_rx_preview)) ?
                  rx_count : (uint16_t)sizeof(transport->stats.last_rx_preview);
  if (previewLength != 0U)
  {
    (void)memcpy(transport->stats.last_rx_preview, rx_data, previewLength);
  }
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
  TactileUartTransport_PrepareRx(transport, tx_data, tx_len,
                                 rx_data, rx_capacity);

  /*
   * 传感器收到 2 字节命令后很快回帧，先打开 DMA 再发送命令，避免丢首字节。
   * 本层只等待完整帧或超时，帧内容是否正确交给协议层判断。
   */
  /*
   * 传感器回包固定 96 字节，DMA 只收一帧长度，减少停止 DMA 前继续收尾部噪声的窗口。
   */
  status = HAL_UART_Receive_DMA(transport->huart, rx_data, expected_rx_len);
  transport->stats.last_hal_status = (uint32_t)status;
  transport->stats.last_hal_error = HAL_UART_GetError(transport->huart);
  if (status != HAL_OK)
  {
    TactileUartTransport_ClearRx(transport->huart);
    transport->stats.rx_error_count++;
    return false;
  }

  /*
   * DMA 已经在搬运数据时，NE/FE 这类错误中断会让 HAL 提前终止接收。
   * 调试阶段先屏蔽 UART 错误中断，优先观察固定 96 字节帧是否能完整进 DMA 缓冲区。
   */
  __HAL_UART_DISABLE_IT(transport->huart, UART_IT_PE);
  __HAL_UART_DISABLE_IT(transport->huart, UART_IT_ERR);

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
      rxCount = (uint16_t)(expected_rx_len -
                           __HAL_DMA_GET_COUNTER(transport->huart->hdmarx));
    }
    if (rxCount >= expected_rx_len)
    {
      break;
    }
    TactileUartTransport_YieldOneTick();
  }

  (void)HAL_UART_DMAStop(transport->huart);
  TactileUartTransport_InvalidateDCache(rx_data, rx_capacity);
  TactileUartTransport_SaveRxStats(transport, rx_data, rxCount,
                                   actual_rx_len);

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

bool TactileUartTransport_WriteReadPolling(TactileUartTransport_t *transport,
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
  TactileUartTransport_PrepareRx(transport, tx_data, tx_len,
                                 rx_data, rx_capacity);

  /*
   * 不用 DMA，但先打开 ReceiveToIdle 中断接收再发命令。
   * 这样可以避开发命令后传感器立即回包、阻塞接收启动太晚导致丢帧头的问题。
   */
  status = HAL_UART_Receive_IT(transport->huart,
                               rx_data,
                               expected_rx_len);
  transport->stats.last_hal_status = (uint32_t)status;
  transport->stats.last_hal_error = HAL_UART_GetError(transport->huart);
  if (status != HAL_OK)
  {
    TactileUartTransport_ClearRx(transport->huart);
    transport->stats.rx_error_count++;
    return false;
  }

  status = HAL_UART_Transmit(transport->huart, (uint8_t *)tx_data,
                             tx_len, timeout_ms);
  transport->stats.last_hal_status = (uint32_t)status;
  transport->stats.last_hal_error = HAL_UART_GetError(transport->huart);
  if (status != HAL_OK)
  {
    (void)HAL_UART_AbortReceive(transport->huart);
    TactileUartTransport_ClearRx(transport->huart);
    transport->stats.tx_error_count++;
    return false;
  }

  startTick = TactileUartTransport_NowMs();
  while ((TactileUartTransport_NowMs() - startTick) <= timeout_ms)
  {
    rxCount = (uint16_t)(expected_rx_len - transport->huart->RxXferCount);
    if (rxCount >= expected_rx_len)
    {
      break;
    }
    if ((transport->huart->RxState == HAL_UART_STATE_READY) ||
        (HAL_UART_GetError(transport->huart) != HAL_UART_ERROR_NONE))
    {
      break;
    }
    TactileUartTransport_YieldOneTick();
  }

  rxCount = (uint16_t)(expected_rx_len - transport->huart->RxXferCount);
  TactileUartTransport_SaveRxStats(transport, rx_data, rxCount,
                                   actual_rx_len);
  if (transport->huart->RxState != HAL_UART_STATE_READY)
  {
    (void)HAL_UART_AbortReceive(transport->huart);
  }

  if (rxCount < expected_rx_len)
  {
    TactileUartTransport_ClearRx(transport->huart);
    if ((TactileUartTransport_NowMs() - startTick) > timeout_ms)
    {
      transport->stats.last_hal_status = (uint32_t)HAL_TIMEOUT;
      transport->stats.timeout_count++;
    }
    else
    {
      transport->stats.rx_error_count++;
    }
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
