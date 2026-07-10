#include "tactile_sensor.h"

#include "cmsis_os2.h"
#include "debug_uart_transport.h"
#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>
#include <string.h>

#define TACTILE_SENSOR_ADDRESS              0x36U
#define TACTILE_SENSOR_REQUEST_CHECK        0xC9U
#define TACTILE_HEADER0                     0x55U
#define TACTILE_HEADER1                     0xAAU
#define TACTILE_CHECKSUM_SIZE               2U
#define TACTILE_SAMPLE_PERIOD_MS            20U
#define TACTILE_STATS_PERIOD_MS             2000U
#define TACTILE_TX_TIMEOUT_MS               2U
#define TACTILE_DEBUG_TIMEOUT_MS            10U
#define TACTILE_DMA_BUFFER_SIZE             512U
#define TACTILE_DMA_BUFFER_ADDR             0x24000200UL
#define TACTILE_UART_CLEAR_FLAGS            (UART_CLEAR_OREF | UART_CLEAR_NEF | \
                                              UART_CLEAR_FEF | UART_CLEAR_PEF | \
                                              UART_CLEAR_RTOF | UART_CLEAR_IDLEF)

typedef struct
{
  UART_HandleTypeDef *uart;
  uint16_t dma_read_position;
  uint16_t assemble_length;
  uint32_t current_response_length;
  uint8_t assemble_buffer[TACTILE_SENSOR_FRAME_SIZE];
  uint8_t latest_frame[TACTILE_SENSOR_FRAME_SIZE];
  volatile uint32_t frame_count;
  volatile uint32_t valid_frame_count;
  volatile uint32_t rx_event_count;
  volatile uint32_t rx_byte_count;
  volatile uint32_t trigger_count;
  volatile uint32_t checksum_error_count;
  volatile uint32_t header_error_count;
  volatile uint32_t sync_drop_count;
  volatile uint32_t short_response_count;
  volatile uint32_t noise_flag_count;
  volatile uint32_t framing_flag_count;
  volatile uint32_t overrun_flag_count;
  volatile uint32_t dma_position_error_count;
  volatile uint32_t uart_error_count;
  volatile uint32_t tx_error_count;
  volatile uint32_t assembler_overflow_count;
  volatile uint32_t restart_count;
  volatile uint8_t restart_pending;
  volatile uint16_t last_response_length;
  volatile uint16_t last_short_response_length;
} TactileSensorContext_t;

static osThreadId_t tactileTaskHandle;
static TactileSensorContext_t tactileSensor;

/*
 * DMA缓冲区独占0x24000200-0x240003FF，
 * Keil IRAM2从0x24000400开始，避免链接器把普通变量放入该区域。
 */
#if defined(__CC_ARM)
__attribute__((at(TACTILE_DMA_BUFFER_ADDR), aligned(32)))
static uint8_t tactileDmaBuffer[TACTILE_DMA_BUFFER_SIZE];
#elif defined(__ARMCC_VERSION)
__attribute__((section(".ARM.__at_0x24000200"), aligned(32)))
static uint8_t tactileDmaBuffer[TACTILE_DMA_BUFFER_SIZE];
#else
__attribute__((section(".dma_buffer_tactile"), aligned(32)))
static uint8_t tactileDmaBuffer[TACTILE_DMA_BUFFER_SIZE];
#endif

static void TactileSensor_Write(const char *text)
{
  (void)DebugUartTransport_Write(text, TACTILE_DEBUG_TIMEOUT_MS);
}

static uint16_t TactileSensor_Checksum16(const uint8_t *data,
                                         uint16_t length)
{
  uint32_t sum = 0U;
  uint16_t index;

  for (index = 0U; index < length; index++)
  {
    sum += data[index];
  }
  return (uint16_t)(sum & 0xFFFFU);
}

static bool TactileSensor_ValidateFrame(const uint8_t *frame)
{
  uint16_t checksum_calc;
  uint16_t checksum_recv;

  if ((frame[0] != TACTILE_HEADER0) ||
      (frame[1] != TACTILE_HEADER1) ||
      (frame[2] != TACTILE_SENSOR_ADDRESS))
  {
    tactileSensor.header_error_count++;
    return false;
  }

  checksum_calc = TactileSensor_Checksum16(
    frame, TACTILE_SENSOR_FRAME_SIZE - TACTILE_CHECKSUM_SIZE);
  checksum_recv =
    (uint16_t)frame[TACTILE_SENSOR_FRAME_SIZE - 2U] |
    (uint16_t)((uint16_t)frame[TACTILE_SENSOR_FRAME_SIZE - 1U] << 8U);
  if (checksum_calc != checksum_recv)
  {
    tactileSensor.checksum_error_count++;
    return false;
  }
  return true;
}

static void TactileSensor_ResyncAssembler(void)
{
  uint16_t keep_start = TACTILE_SENSOR_FRAME_SIZE;
  uint16_t index;

  for (index = 1U; index < (TACTILE_SENSOR_FRAME_SIZE - 2U); index++)
  {
    if ((tactileSensor.assemble_buffer[index] == TACTILE_HEADER0) &&
        (tactileSensor.assemble_buffer[index + 1U] == TACTILE_HEADER1) &&
        (tactileSensor.assemble_buffer[index + 2U] ==
         TACTILE_SENSOR_ADDRESS))
    {
      keep_start = index;
      break;
    }
  }

  if (keep_start == TACTILE_SENSOR_FRAME_SIZE)
  {
    if ((tactileSensor.assemble_buffer[TACTILE_SENSOR_FRAME_SIZE - 2U] ==
         TACTILE_HEADER0) &&
        (tactileSensor.assemble_buffer[TACTILE_SENSOR_FRAME_SIZE - 1U] ==
         TACTILE_HEADER1))
    {
      keep_start = TACTILE_SENSOR_FRAME_SIZE - 2U;
    }
    else if (tactileSensor.assemble_buffer[TACTILE_SENSOR_FRAME_SIZE - 1U] ==
             TACTILE_HEADER0)
    {
      keep_start = TACTILE_SENSOR_FRAME_SIZE - 1U;
    }
  }

  tactileSensor.sync_drop_count += keep_start;
  tactileSensor.assemble_length = TACTILE_SENSOR_FRAME_SIZE - keep_start;
  if (tactileSensor.assemble_length > 0U)
  {
    memmove(tactileSensor.assemble_buffer,
            &tactileSensor.assemble_buffer[keep_start],
            tactileSensor.assemble_length);
  }
}

static void TactileSensor_ProcessByte(uint8_t data)
{
  if (tactileSensor.assemble_length == 0U)
  {
    if (data == TACTILE_HEADER0)
    {
      tactileSensor.assemble_buffer[0] = data;
      tactileSensor.assemble_length = 1U;
    }
    else
    {
      tactileSensor.sync_drop_count++;
    }
    return;
  }

  if (tactileSensor.assemble_length == 1U)
  {
    if (data == TACTILE_HEADER1)
    {
      tactileSensor.assemble_buffer[1] = data;
      tactileSensor.assemble_length = 2U;
    }
    else if (data == TACTILE_HEADER0)
    {
      tactileSensor.sync_drop_count++;
    }
    else
    {
      tactileSensor.sync_drop_count += 2U;
      tactileSensor.assemble_length = 0U;
    }
    return;
  }

  if (tactileSensor.assemble_length == 2U)
  {
    if (data == TACTILE_SENSOR_ADDRESS)
    {
      tactileSensor.assemble_buffer[2] = data;
      tactileSensor.assemble_length = 3U;
    }
    else
    {
      tactileSensor.header_error_count++;
      if (data == TACTILE_HEADER0)
      {
        tactileSensor.sync_drop_count += 2U;
        tactileSensor.assemble_buffer[0] = data;
        tactileSensor.assemble_length = 1U;
      }
      else
      {
        tactileSensor.sync_drop_count += 3U;
        tactileSensor.assemble_length = 0U;
      }
    }
    return;
  }

  if (tactileSensor.assemble_length >= TACTILE_SENSOR_FRAME_SIZE)
  {
    tactileSensor.assembler_overflow_count++;
    tactileSensor.assemble_length = 0U;
    return;
  }

  tactileSensor.assemble_buffer[tactileSensor.assemble_length++] = data;
  if (tactileSensor.assemble_length < TACTILE_SENSOR_FRAME_SIZE)
  {
    return;
  }

  tactileSensor.frame_count++;
  if (TactileSensor_ValidateFrame(tactileSensor.assemble_buffer))
  {
    memcpy(tactileSensor.latest_frame,
           tactileSensor.assemble_buffer,
           TACTILE_SENSOR_FRAME_SIZE);
    /* 数据复制完成后再更新计数，读取方看到新序号时帧内容已经完整。 */
    tactileSensor.valid_frame_count++;
    tactileSensor.assemble_length = 0U;
  }
  else
  {
    TactileSensor_ResyncAssembler();
  }
}

static void TactileSensor_ProcessBytes(const uint8_t *data,
                                       uint16_t length)
{
  uint16_t index;

  for (index = 0U; index < length; index++)
  {
    TactileSensor_ProcessByte(data[index]);
  }
}

static void TactileSensor_RecordAndClearRxFlags(void)
{
  uint32_t isr = tactileSensor.uart->Instance->ISR;
  uint32_t clear_flags = 0U;

  if ((isr & USART_ISR_NE) != 0U)
  {
    tactileSensor.noise_flag_count++;
    clear_flags |= UART_CLEAR_NEF;
  }
  if ((isr & USART_ISR_FE) != 0U)
  {
    tactileSensor.framing_flag_count++;
    clear_flags |= UART_CLEAR_FEF;
  }
  if ((isr & USART_ISR_ORE) != 0U)
  {
    tactileSensor.overrun_flag_count++;
    clear_flags |= UART_CLEAR_OREF;
  }
  if (clear_flags != 0U)
  {
    __HAL_UART_CLEAR_FLAG(tactileSensor.uart, clear_flags);
  }
}

static void TactileSensor_FlushRx(void)
{
  __HAL_UART_CLEAR_FLAG(tactileSensor.uart, TACTILE_UART_CLEAR_FLAGS);
  __HAL_UART_SEND_REQ(tactileSensor.uart, UART_RXDATA_FLUSH_REQUEST);
}

static bool TactileSensor_StartReceive(void)
{
  HAL_StatusTypeDef status;

  if ((tactileSensor.uart == NULL) ||
      (tactileSensor.uart->RxState != HAL_UART_STATE_READY))
  {
    return false;
  }

  tactileSensor.dma_read_position = 0U;
  status = HAL_UARTEx_ReceiveToIdle_DMA(tactileSensor.uart,
                                        tactileDmaBuffer,
                                        TACTILE_DMA_BUFFER_SIZE);
  if ((status == HAL_OK) && (tactileSensor.uart->hdmarx != NULL))
  {
    __HAL_DMA_DISABLE_IT(tactileSensor.uart->hdmarx, DMA_IT_HT);
    /* 帧校验负责最终判定，NE/FE不再中止循环DMA。 */
    CLEAR_BIT(tactileSensor.uart->Instance->CR3, USART_CR3_EIE);
  }
  return status == HAL_OK;
}

static bool TactileSensor_RestartReceive(void)
{
  if (tactileSensor.uart->RxState != HAL_UART_STATE_READY)
  {
    (void)HAL_UART_AbortReceive(tactileSensor.uart);
  }
  tactileSensor.dma_read_position = 0U;
  tactileSensor.assemble_length = 0U;
  tactileSensor.current_response_length = 0U;
  TactileSensor_FlushRx();
  return TactileSensor_StartReceive();
}

static void TactileSensor_SendRequest(void)
{
  uint8_t request[2] = {
    TACTILE_SENSOR_ADDRESS,
    TACTILE_SENSOR_REQUEST_CHECK
  };
  HAL_StatusTypeDef status;

  status = HAL_UART_Transmit(tactileSensor.uart,
                             request,
                             sizeof(request),
                             TACTILE_TX_TIMEOUT_MS);
  tactileSensor.trigger_count++;
  if (status != HAL_OK)
  {
    tactileSensor.tx_error_count++;
  }
}

static uint32_t TactileSensor_Rate10(uint32_t delta,
                                     uint32_t elapsed,
                                     uint32_t tick_frequency)
{
  if (elapsed == 0U)
  {
    return 0U;
  }
  return (uint32_t)(((uint64_t)delta * tick_frequency * 10ULL) / elapsed);
}

static void TactileSensor_DumpStats(uint32_t elapsed,
                                    uint32_t tick_frequency,
                                    uint32_t *last_frames,
                                    uint32_t *last_valid,
                                    uint32_t *last_triggers,
                                    uint32_t *last_bytes)
{
  uint32_t frame_count = tactileSensor.frame_count;
  uint32_t valid_count = tactileSensor.valid_frame_count;
  uint32_t trigger_count = tactileSensor.trigger_count;
  uint32_t byte_count = tactileSensor.rx_byte_count;
  uint32_t rx_rate10 = TactileSensor_Rate10(
    frame_count - *last_frames, elapsed, tick_frequency);
  uint32_t ok_rate10 = TactileSensor_Rate10(
    valid_count - *last_valid, elapsed, tick_frequency);
  uint32_t tx_rate10 = TactileSensor_Rate10(
    trigger_count - *last_triggers, elapsed, tick_frequency);
  uint32_t bytes_per_second =
    (uint32_t)(((uint64_t)(byte_count - *last_bytes) * tick_frequency) /
               elapsed);
  char line[384];

  *last_frames = frame_count;
  *last_valid = valid_count;
  *last_triggers = trigger_count;
  *last_bytes = byte_count;

  (void)snprintf(
    line, sizeof(line),
    "TACTILE,STAT,rx=%lu.%lu,ok=%lu.%lu,tx=%lu.%lu,bps=%lu,total=%lu,valid=%lu,trig=%lu,evt=%lu,len=%u,resp=%u,short=%lu,lastshort=%u,ckerr=%lu,hdrerr=%lu,drop=%lu,ne=%lu,fe=%lu,ore=%lu,dmaerr=%lu,uart_err=%lu,txerr=%lu,ovf=%lu\r\n",
    (unsigned long)(rx_rate10 / 10U),
    (unsigned long)(rx_rate10 % 10U),
    (unsigned long)(ok_rate10 / 10U),
    (unsigned long)(ok_rate10 % 10U),
    (unsigned long)(tx_rate10 / 10U),
    (unsigned long)(tx_rate10 % 10U),
    (unsigned long)bytes_per_second,
    (unsigned long)frame_count,
    (unsigned long)valid_count,
    (unsigned long)trigger_count,
    (unsigned long)tactileSensor.rx_event_count,
    (unsigned int)TACTILE_SENSOR_FRAME_SIZE,
    (unsigned int)tactileSensor.last_response_length,
    (unsigned long)tactileSensor.short_response_count,
    (unsigned int)tactileSensor.last_short_response_length,
    (unsigned long)tactileSensor.checksum_error_count,
    (unsigned long)tactileSensor.header_error_count,
    (unsigned long)tactileSensor.sync_drop_count,
    (unsigned long)tactileSensor.noise_flag_count,
    (unsigned long)tactileSensor.framing_flag_count,
    (unsigned long)tactileSensor.overrun_flag_count,
    (unsigned long)tactileSensor.dma_position_error_count,
    (unsigned long)tactileSensor.uart_error_count,
    (unsigned long)tactileSensor.tx_error_count,
    (unsigned long)tactileSensor.assembler_overflow_count);
  TactileSensor_Write(line);
}

void TactileSensor_Init(UART_HandleTypeDef *uart)
{
  memset(&tactileSensor, 0, sizeof(tactileSensor));
  tactileSensor.uart = uart;
}

bool TactileSensor_GetLatestRaw(uint8_t *buffer,
                                uint16_t capacity,
                                uint32_t *frame_count)
{
  uint32_t count;

  if ((buffer == NULL) || (capacity < TACTILE_SENSOR_FRAME_SIZE))
  {
    return false;
  }

  taskENTER_CRITICAL();
  memcpy(buffer, tactileSensor.latest_frame, TACTILE_SENSOR_FRAME_SIZE);
  count = tactileSensor.valid_frame_count;
  taskEXIT_CRITICAL();
  if (frame_count != NULL)
  {
    *frame_count = count;
  }
  return count != 0U;
}

void TactileSensor_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
  HAL_UART_RxEventTypeTypeDef event_type;
  uint16_t previous_position;
  uint16_t current_position;
  uint16_t chunk_length = 0U;

  if ((tactileSensor.uart == NULL) || (huart != tactileSensor.uart))
  {
    return;
  }
  if (size > TACTILE_DMA_BUFFER_SIZE)
  {
    tactileSensor.dma_position_error_count++;
    return;
  }

  event_type = HAL_UARTEx_GetRxEventType(huart);
  previous_position = tactileSensor.dma_read_position;
  current_position = size;
  if (current_position == TACTILE_DMA_BUFFER_SIZE)
  {
    current_position = 0U;
    /* IDLE和DMA TC可能报告同一位置，读指针已归零时不重复处理。 */
    if (previous_position != 0U)
    {
      chunk_length = TACTILE_DMA_BUFFER_SIZE - previous_position;
      TactileSensor_ProcessBytes(&tactileDmaBuffer[previous_position],
                                 chunk_length);
    }
  }
  else if (current_position > previous_position)
  {
    chunk_length = current_position - previous_position;
    TactileSensor_ProcessBytes(&tactileDmaBuffer[previous_position],
                               chunk_length);
  }
  else if (current_position < previous_position)
  {
    uint16_t tail_length = TACTILE_DMA_BUFFER_SIZE - previous_position;

    chunk_length = tail_length + current_position;
    TactileSensor_ProcessBytes(&tactileDmaBuffer[previous_position],
                               tail_length);
    if (current_position > 0U)
    {
      TactileSensor_ProcessBytes(tactileDmaBuffer, current_position);
    }
  }

  tactileSensor.dma_read_position = current_position;
  if (chunk_length > 0U)
  {
    tactileSensor.rx_byte_count += chunk_length;
    tactileSensor.rx_event_count++;
    tactileSensor.current_response_length += chunk_length;
  }
  TactileSensor_RecordAndClearRxFlags();

  if ((event_type == HAL_UART_RXEVENT_IDLE) &&
      (tactileSensor.current_response_length > 0U))
  {
    uint32_t response_length = tactileSensor.current_response_length;

    if (response_length > 0xFFFFU)
    {
      response_length = 0xFFFFU;
    }
    tactileSensor.last_response_length = (uint16_t)response_length;
    if (response_length < TACTILE_SENSOR_FRAME_SIZE)
    {
      tactileSensor.short_response_count++;
      tactileSensor.last_short_response_length = (uint16_t)response_length;
    }
    tactileSensor.current_response_length = 0U;
  }
}

void TactileSensor_ErrorCallback(UART_HandleTypeDef *huart)
{
  if ((tactileSensor.uart != NULL) && (huart == tactileSensor.uart))
  {
    tactileSensor.uart_error_count++;
    tactileSensor.restart_pending = 1U;
  }
}

static void TactileSensor_Task(void *argument)
{
  uint32_t tick_frequency = osKernelGetTickFreq();
  uint32_t period_ticks =
    (TACTILE_SAMPLE_PERIOD_MS * tick_frequency) / 1000U;
  uint32_t stats_ticks =
    (TACTILE_STATS_PERIOD_MS * tick_frequency) / 1000U;
  uint32_t next = osKernelGetTickCount();
  uint32_t stats_tick = next;
  uint32_t stats_last_frames = 0U;
  uint32_t stats_last_valid = 0U;
  uint32_t stats_last_triggers = 0U;
  uint32_t stats_last_bytes = 0U;
  (void)argument;

  if (period_ticks == 0U)
  {
    period_ticks = 1U;
  }
  if (stats_ticks == 0U)
  {
    stats_ticks = 1U;
  }

  if (!TactileSensor_RestartReceive())
  {
    tactileSensor.restart_pending = 1U;
  }
  TactileSensor_Write("TACTILE,TASK,start,uart=2,addr=0x36,rate=50Hz\r\n");

  for (;;)
  {
    uint32_t now;

    if (tactileSensor.restart_pending != 0U)
    {
      tactileSensor.restart_pending = 0U;
      if (TactileSensor_RestartReceive())
      {
        tactileSensor.restart_count++;
      }
      else
      {
        tactileSensor.restart_pending = 1U;
      }
    }

    TactileSensor_SendRequest();
    now = osKernelGetTickCount();
    if ((now - stats_tick) >= stats_ticks)
    {
      uint32_t elapsed = now - stats_tick;

      TactileSensor_DumpStats(elapsed, tick_frequency,
                              &stats_last_frames,
                              &stats_last_valid,
                              &stats_last_triggers,
                              &stats_last_bytes);
      stats_tick = now;
    }

    next += period_ticks;
    now = osKernelGetTickCount();
    if ((int32_t)(now - next) >= 0)
    {
      next = now + period_ticks;
    }
    (void)osDelayUntil(next);
  }
}

void TactileSensor_CreateTask(void)
{
  const osThreadAttr_t attributes = {
    .name = "tactileSensor",
    .stack_size = 1024U * 4U,
    .priority = osPriorityBelowNormal
  };

  tactileTaskHandle = osThreadNew(TactileSensor_Task, NULL, &attributes);
  if (tactileTaskHandle == NULL)
  {
    TactileSensor_Write("TACTILE,TASK,create_failed\r\n");
  }
  else
  {
    TactileSensor_Write("TACTILE,TASK,created\r\n");
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
  TactileSensor_RxEventCallback(huart, size);
}
