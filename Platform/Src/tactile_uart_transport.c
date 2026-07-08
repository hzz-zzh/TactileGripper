#include "tactile_uart_transport.h"

#include "cmsis_os2.h"

#include <string.h>

#define TACTILE_UART_FRAME_HEAD0       0x55U
#define TACTILE_UART_FRAME_HEAD1       0xAAU
#define TACTILE_UART_READ_RESPONSE_CMD 0x85U

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

static void TactileUartTransport_SaveTxStats(TactileUartTransport_t *transport,
                                             const uint8_t *tx_data,
                                             uint16_t tx_len,
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
}

static void TactileUartTransport_PrepareRx(TactileUartTransport_t *transport,
                                           const uint8_t *tx_data,
                                           uint16_t tx_len,
                                           uint8_t *rx_data,
                                           uint16_t rx_capacity)
{
  TactileUartTransport_SaveTxStats(transport, tx_data, tx_len, rx_capacity);

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

static uint16_t TactileUartTransport_RingNext(const TactileUartTransport_t *transport,
                                              uint16_t index)
{
  index++;
  return (index >= transport->ring_length) ? 0U : index;
}

static uint16_t TactileUartTransport_RingCount(const TactileUartTransport_t *transport)
{
  uint16_t head = transport->ring_head;
  uint16_t tail = transport->ring_tail;

  if (head >= tail)
  {
    return (uint16_t)(head - tail);
  }
  return (uint16_t)(transport->ring_length - tail + head);
}

static uint8_t TactileUartTransport_RingPeek(const TactileUartTransport_t *transport,
                                             uint16_t offset)
{
  uint16_t index = (uint16_t)(transport->ring_tail + offset);

  while (index >= transport->ring_length)
  {
    index = (uint16_t)(index - transport->ring_length);
  }
  return transport->ring_buffer[index];
}

static void TactileUartTransport_RingDrop(TactileUartTransport_t *transport,
                                          uint16_t count)
{
  while ((count != 0U) && (transport->ring_tail != transport->ring_head))
  {
    transport->ring_tail =
      TactileUartTransport_RingNext(transport, transport->ring_tail);
    count--;
  }
}

static void TactileUartTransport_RingPush(TactileUartTransport_t *transport,
                                          uint8_t value)
{
  uint16_t next = TactileUartTransport_RingNext(transport,
                                                transport->ring_head);

  if (next == transport->ring_tail)
  {
    transport->ring_tail =
      TactileUartTransport_RingNext(transport, transport->ring_tail);
    transport->stats.ring_overrun_count++;
  }

  transport->ring_buffer[transport->ring_head] = value;
  transport->ring_head = next;
}

static void TactileUartTransport_CopyDmaToRing(TactileUartTransport_t *transport,
                                               uint16_t start,
                                               uint16_t end)
{
  uint16_t index;

  for (index = start; index < end; index++)
  {
    TactileUartTransport_RingPush(transport,
                                  transport->dma_rx_buffer[index]);
  }
}

static void TactileUartTransport_ServiceCircularRx(TactileUartTransport_t *transport)
{
  uint16_t dmaPos;

  if ((transport == NULL) || (!transport->circular_rx_started) ||
      (transport->huart == NULL) || (transport->huart->hdmarx == NULL) ||
      (transport->dma_rx_buffer == NULL) || (transport->dma_rx_length == 0U))
  {
    return;
  }

  TactileUartTransport_InvalidateDCache(transport->dma_rx_buffer,
                                        transport->dma_rx_length);

  dmaPos = (uint16_t)(transport->dma_rx_length -
                      __HAL_DMA_GET_COUNTER(transport->huart->hdmarx));
  if (dmaPos >= transport->dma_rx_length)
  {
    dmaPos = 0U;
  }

  if (dmaPos == transport->dma_last_pos)
  {
    return;
  }

  if (dmaPos > transport->dma_last_pos)
  {
    TactileUartTransport_CopyDmaToRing(transport,
                                       transport->dma_last_pos,
                                       dmaPos);
  }
  else
  {
    TactileUartTransport_CopyDmaToRing(transport,
                                       transport->dma_last_pos,
                                       transport->dma_rx_length);
    TactileUartTransport_CopyDmaToRing(transport, 0U, dmaPos);
  }

  transport->dma_last_pos = dmaPos;
}

static bool TactileUartTransport_TryReadFrame(TactileUartTransport_t *transport,
                                              uint8_t expected_address,
                                              uint8_t *rx_data,
                                              uint16_t expected_rx_len,
                                              uint16_t *actual_rx_len)
{
  uint16_t available;
  uint16_t index;

  if ((transport == NULL) || (rx_data == NULL) || (expected_rx_len < 4U))
  {
    return false;
  }

  for (;;)
  {
    available = TactileUartTransport_RingCount(transport);
    if (available < 4U)
    {
      return false;
    }

    if ((TactileUartTransport_RingPeek(transport, 0U) == TACTILE_UART_FRAME_HEAD0) &&
        (TactileUartTransport_RingPeek(transport, 1U) == TACTILE_UART_FRAME_HEAD1) &&
        (TactileUartTransport_RingPeek(transport, 2U) == expected_address) &&
        (TactileUartTransport_RingPeek(transport, 3U) == TACTILE_UART_READ_RESPONSE_CMD))
    {
      break;
    }

    TactileUartTransport_RingDrop(transport, 1U);
  }

  if (available < expected_rx_len)
  {
    return false;
  }

  for (index = 0U; index < expected_rx_len; index++)
  {
    rx_data[index] = TactileUartTransport_RingPeek(transport, index);
  }
  TactileUartTransport_RingDrop(transport, expected_rx_len);
  TactileUartTransport_SaveRxStats(transport, rx_data, expected_rx_len,
                                   actual_rx_len);
  return true;
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

bool TactileUartTransport_StartCircularRx(TactileUartTransport_t *transport,
                                          uint8_t *dma_rx_buffer,
                                          uint16_t dma_rx_length,
                                          uint8_t *ring_buffer,
                                          uint16_t ring_length)
{
  HAL_StatusTypeDef status;

  if ((transport == NULL) || (transport->huart == NULL) ||
      (dma_rx_buffer == NULL) || (dma_rx_length == 0U) ||
      (ring_buffer == NULL) || (ring_length < dma_rx_length))
  {
    if (transport != NULL)
    {
      transport->stats.last_result = TACTILE_UART_RESULT_BAD_ARGUMENT;
    }
    return false;
  }

  (void)HAL_UART_Abort(transport->huart);
  transport->huart->ErrorCode = HAL_UART_ERROR_NONE;
  TactileUartTransport_ClearRx(transport->huart);

  (void)memset(dma_rx_buffer, 0, dma_rx_length);
  (void)memset(ring_buffer, 0, ring_length);
  TactileUartTransport_InvalidateDCache(dma_rx_buffer, dma_rx_length);

  transport->dma_rx_buffer = dma_rx_buffer;
  transport->dma_rx_length = dma_rx_length;
  transport->dma_last_pos = 0U;
  transport->ring_buffer = ring_buffer;
  transport->ring_length = ring_length;
  transport->ring_head = 0U;
  transport->ring_tail = 0U;
  transport->circular_rx_started = false;

  status = HAL_UART_Receive_DMA(transport->huart,
                                dma_rx_buffer,
                                dma_rx_length);
  transport->stats.last_hal_status = (uint32_t)status;
  transport->stats.last_hal_error = HAL_UART_GetError(transport->huart);
  if (status != HAL_OK)
  {
    transport->stats.last_result = TACTILE_UART_RESULT_RX_START_ERROR;
    transport->stats.rx_error_count++;
    return false;
  }

  /*
   * USART2 接收由 DMA 循环搬运，任务按 DMA 写指针推进 ring。
   * 关闭 DMA HT/TC 中断，避免 460800bps 下产生无意义的半满/满中断。
   */
  if (transport->huart->hdmarx != NULL)
  {
    __HAL_DMA_DISABLE_IT(transport->huart->hdmarx, DMA_IT_HT | DMA_IT_TC);
  }

  __HAL_UART_DISABLE_IT(transport->huart, UART_IT_PE);
  __HAL_UART_DISABLE_IT(transport->huart, UART_IT_ERR);
  __HAL_UART_CLEAR_FLAG(transport->huart, UART_CLEAR_IDLEF);
  __HAL_UART_ENABLE_IT(transport->huart, UART_IT_IDLE);

  transport->circular_rx_started = true;
  transport->stats.last_result = TACTILE_UART_RESULT_OK;
  return true;
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
    if (transport != NULL)
    {
      transport->stats.last_result = TACTILE_UART_RESULT_BAD_ARGUMENT;
    }
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
    transport->stats.last_result = TACTILE_UART_RESULT_RX_START_ERROR;
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
    transport->stats.last_result = TACTILE_UART_RESULT_TX_ERROR;
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
    transport->stats.last_result = TACTILE_UART_RESULT_RX_TIMEOUT;
    transport->stats.timeout_count++;
    return false;
  }

  if (transport->stats.last_hal_error != HAL_UART_ERROR_NONE)
  {
    TactileUartTransport_ClearRx(transport->huart);
  }

  transport->stats.last_result = TACTILE_UART_RESULT_OK;
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
    if (transport != NULL)
    {
      transport->stats.last_result = TACTILE_UART_RESULT_BAD_ARGUMENT;
    }
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
    transport->stats.last_result = TACTILE_UART_RESULT_RX_START_ERROR;
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
    transport->stats.last_result = TACTILE_UART_RESULT_TX_ERROR;
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
      transport->stats.last_result = TACTILE_UART_RESULT_RX_TIMEOUT;
      transport->stats.timeout_count++;
    }
    else
    {
      transport->stats.last_result = TACTILE_UART_RESULT_RX_INCOMPLETE;
      transport->stats.rx_error_count++;
    }
    return false;
  }

  if (transport->stats.last_hal_error != HAL_UART_ERROR_NONE)
  {
    TactileUartTransport_ClearRx(transport->huart);
  }

  transport->stats.last_result = TACTILE_UART_RESULT_OK;
  return true;
}

bool TactileUartTransport_WriteReadCircular(TactileUartTransport_t *transport,
                                            const uint8_t *tx_data,
                                            uint16_t tx_len,
                                            uint8_t expected_address,
                                            uint8_t *rx_data,
                                            uint16_t rx_capacity,
                                            uint16_t expected_rx_len,
                                            uint16_t *actual_rx_len,
                                            uint32_t timeout_ms)
{
  HAL_StatusTypeDef status;
  uint32_t startTick;
  uint16_t rxCount;
  uint16_t index;

  if ((transport == NULL) || (transport->huart == NULL) ||
      (!transport->circular_rx_started) ||
      (tx_data == NULL) || (tx_len == 0U) ||
      (rx_data == NULL) || (rx_capacity == 0U) ||
      (expected_rx_len == 0U) || (expected_rx_len > rx_capacity))
  {
    if (transport != NULL)
    {
      transport->stats.last_result = TACTILE_UART_RESULT_BAD_ARGUMENT;
    }
    return false;
  }

  if (actual_rx_len != NULL)
  {
    *actual_rx_len = 0U;
  }

  TactileUartTransport_SaveTxStats(transport, tx_data, tx_len, rx_capacity);
  (void)memset(rx_data, 0, rx_capacity);

  /*
   * 发新命令前丢弃 ring 里残留的旧帧，避免上一次超时留下的数据被误判为本次响应。
   * DMA 保持运行，只同步当前位置并清空软件 ring，不会关闭接收通道。
   */
  TactileUartTransport_ServiceCircularRx(transport);
  transport->ring_tail = transport->ring_head;
  transport->stats.last_hal_error = HAL_UART_GetError(transport->huart);
  __HAL_UART_CLEAR_FLAG(transport->huart, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                       UART_CLEAR_FEF | UART_CLEAR_PEF |
                                       UART_CLEAR_IDLEF);

  status = HAL_UART_Transmit(transport->huart, (uint8_t *)tx_data,
                             tx_len, timeout_ms);
  transport->stats.last_hal_status = (uint32_t)status;
  transport->stats.last_hal_error = HAL_UART_GetError(transport->huart);
  if (status != HAL_OK)
  {
    transport->stats.last_result = TACTILE_UART_RESULT_TX_ERROR;
    transport->stats.tx_error_count++;
    return false;
  }

  startTick = TactileUartTransport_NowMs();
  while ((TactileUartTransport_NowMs() - startTick) <= timeout_ms)
  {
    TactileUartTransport_ServiceCircularRx(transport);
    if (TactileUartTransport_TryReadFrame(transport,
                                          expected_address,
                                          rx_data,
                                          expected_rx_len,
                                          actual_rx_len))
    {
      transport->stats.last_hal_status = (uint32_t)HAL_OK;
      transport->stats.last_hal_error = HAL_UART_GetError(transport->huart);
      transport->stats.last_result = TACTILE_UART_RESULT_OK;
      return true;
    }
    TactileUartTransport_YieldOneTick();
  }

  TactileUartTransport_ServiceCircularRx(transport);
  if (TactileUartTransport_TryReadFrame(transport,
                                        expected_address,
                                        rx_data,
                                        expected_rx_len,
                                        actual_rx_len))
  {
    transport->stats.last_hal_status = (uint32_t)HAL_OK;
    transport->stats.last_hal_error = HAL_UART_GetError(transport->huart);
    transport->stats.last_result = TACTILE_UART_RESULT_OK;
    return true;
  }

  rxCount = TactileUartTransport_RingCount(transport);
  if (rxCount > rx_capacity)
  {
    rxCount = rx_capacity;
  }
  for (index = 0U; index < rxCount; index++)
  {
    rx_data[index] = TactileUartTransport_RingPeek(transport, index);
  }
  TactileUartTransport_SaveRxStats(transport, rx_data, rxCount,
                                   actual_rx_len);

  transport->stats.last_hal_status = (uint32_t)HAL_TIMEOUT;
  transport->stats.last_hal_error = HAL_UART_GetError(transport->huart);
  transport->stats.last_result = TACTILE_UART_RESULT_RX_TIMEOUT;
  transport->stats.timeout_count++;
  return false;
}

void TactileUartTransport_HandleIdleIrq(TactileUartTransport_t *transport)
{
  if ((transport == NULL) || (transport->huart == NULL) ||
      (!transport->circular_rx_started))
  {
    return;
  }

  if ((__HAL_UART_GET_FLAG(transport->huart, UART_FLAG_IDLE) != RESET) &&
      (__HAL_UART_GET_IT_SOURCE(transport->huart, UART_IT_IDLE) != RESET))
  {
    /*
     * IDLE 只作为一帧结束的硬件提示，具体搬运仍由任务读取 DMA NDTR 完成。
     * 中断里不解析帧，避免占用过长时间影响电机控制中断。
     */
    __HAL_UART_CLEAR_FLAG(transport->huart, UART_CLEAR_IDLEF);
    transport->stats.idle_event_count++;
  }
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
