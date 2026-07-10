#include "tactile_sensor.h"

#include "cmsis_os2.h"
#include "debug_uart_transport.h"
#include "FreeRTOS.h"
#include "tactile_data_store.h"
#include "tactile_protocol.h"
#include "task.h"

#include <stdio.h>
#include <string.h>

#define TACTILE_SENSOR_PORT_COUNT           2U
#define TACTILE_SENSOR_ADDRESS_COUNT        2U
#define TACTILE_SAMPLE_PERIOD_MS            25U
#define TACTILE_RESPONSE_TIMEOUT_MS         5U
#define TACTILE_STATS_PERIOD_MS             2000U
#define TACTILE_TX_TIMEOUT_MS               2U
#define TACTILE_DEBUG_TIMEOUT_MS            10U
#define TACTILE_DMA_BUFFER_SIZE             512U
#define TACTILE_UART1_DMA_BUFFER_ADDR       0x24000100UL
#define TACTILE_UART2_DMA_BUFFER_ADDR       0x24000300UL
#define TACTILE_UART_CLEAR_FLAGS            (UART_CLEAR_OREF | UART_CLEAR_NEF | \
                                              UART_CLEAR_FEF | UART_CLEAR_PEF | \
                                              UART_CLEAR_RTOF | UART_CLEAR_IDLEF)

#if (TACTILE_SENSOR_FRAME_SIZE != TACTILE_PROTOCOL_FRAME_SIZE) || \
    (TACTILE_SENSOR_ADDRESS_36 != TACTILE_PROTOCOL_ADDRESS_36) || \
    (TACTILE_SENSOR_ADDRESS_37 != TACTILE_PROTOCOL_ADDRESS_37)
#error "Tactile sensor API and protocol constants must match"
#endif

/* 每个UART拥有独立的DMA位置、响应缓存和错误计数。 */
typedef struct
{
  UART_HandleTypeDef *uart;
  uint8_t *dma_buffer;
  uint16_t dma_read_position;
  uint32_t response_length;
  uint8_t pending_address;
  uint8_t response_buffer[TACTILE_SENSOR_FRAME_SIZE];
  uint8_t latest_raw[TACTILE_SENSOR_ADDRESS_COUNT]
                    [TACTILE_SENSOR_FRAME_SIZE];
  volatile uint32_t valid_frame_count[TACTILE_SENSOR_ADDRESS_COUNT];
  volatile uint32_t rx_event_count;
  volatile uint32_t response_count;
  volatile uint32_t response_timeout_count;
  volatile uint32_t rx_byte_count;
  volatile uint32_t trigger_count[TACTILE_SENSOR_ADDRESS_COUNT];
  volatile uint32_t checksum_error_count;
  volatile uint32_t format_error_count;
  volatile uint32_t short_response_count;
  volatile uint32_t noise_flag_count;
  volatile uint32_t framing_flag_count;
  volatile uint32_t overrun_flag_count;
  volatile uint32_t dma_position_error_count;
  volatile uint32_t uart_error_count;
  volatile uint32_t tx_error_count;
  volatile uint32_t response_overflow_count;
  volatile uint32_t restart_count;
  volatile uint8_t restart_pending;
  volatile uint16_t last_response_length;
  volatile uint16_t last_short_response_length;
} TactilePortContext_t;

static osThreadId_t tactileTaskHandle;
static TactilePortContext_t tactilePorts[TACTILE_SENSOR_PORT_COUNT];

/*
 * USART1和USART2各使用512字节DMA缓冲区，
 * Keil IRAM2从0x24000500开始，避免普通变量覆盖DMA区域。
 */
#if defined(__CC_ARM)
__attribute__((at(TACTILE_UART1_DMA_BUFFER_ADDR), aligned(32)))
static uint8_t tactileUart1DmaBuffer[TACTILE_DMA_BUFFER_SIZE];
__attribute__((at(TACTILE_UART2_DMA_BUFFER_ADDR), aligned(32)))
static uint8_t tactileUart2DmaBuffer[TACTILE_DMA_BUFFER_SIZE];
#elif defined(__ARMCC_VERSION)
__attribute__((section(".ARM.__at_0x24000100"), aligned(32)))
static uint8_t tactileUart1DmaBuffer[TACTILE_DMA_BUFFER_SIZE];
__attribute__((section(".ARM.__at_0x24000300"), aligned(32)))
static uint8_t tactileUart2DmaBuffer[TACTILE_DMA_BUFFER_SIZE];
#else
__attribute__((section(".dma_buffer_tactile_uart1"), aligned(32)))
static uint8_t tactileUart1DmaBuffer[TACTILE_DMA_BUFFER_SIZE];
__attribute__((section(".dma_buffer_tactile_uart2"), aligned(32)))
static uint8_t tactileUart2DmaBuffer[TACTILE_DMA_BUFFER_SIZE];
#endif

static void TactileSensor_Write(const char *text)
{
  (void)DebugUartTransport_Write(text, TACTILE_DEBUG_TIMEOUT_MS);
}

static uint32_t TactileSensor_AddressIndex(uint8_t address)
{
  return (address == TACTILE_SENSOR_ADDRESS_37) ? 1U : 0U;
}

static tactile_finger_id_t TactileSensor_PortFinger(uint32_t port_index)
{
  return (port_index == TACTILE_SENSOR_PORT_USART1) ?
         TACTILE_FINGER_LEFT : TACTILE_FINGER_RIGHT;
}

static tactile_unit_id_t TactileSensor_AddressUnit(uint8_t address)
{
  return (address == TACTILE_SENSOR_ADDRESS_37) ?
         TACTILE_UNIT_UPPER : TACTILE_UNIT_LOWER;
}

static TactilePortContext_t *TactileSensor_FindPort(
  UART_HandleTypeDef *uart)
{
  uint32_t port_index;

  for (port_index = 0U;
       port_index < TACTILE_SENSOR_PORT_COUNT;
       port_index++)
  {
    if (tactilePorts[port_index].uart == uart)
    {
      return &tactilePorts[port_index];
    }
  }
  return NULL;
}

static uint32_t TactileSensor_PortIndex(const TactilePortContext_t *port)
{
  return (port == &tactilePorts[TACTILE_SENSOR_PORT_USART1]) ?
         TACTILE_SENSOR_PORT_USART1 : TACTILE_SENSOR_PORT_USART2;
}

static void TactileSensor_RecordProtocolError(
  TactilePortContext_t *port,
  tactile_protocol_result_t result)
{
  if (result == TACTILE_PROTOCOL_BAD_CHECKSUM)
  {
    port->checksum_error_count++;
  }
  else
  {
    port->format_error_count++;
  }
}

static void TactileSensor_FinalizeResponse(TactilePortContext_t *port)
{
  tactile_unit_data_t unit_data;
  tactile_protocol_result_t result;
  uint32_t address_index;
  uint32_t port_index;

  /* IDLE表示本次传感器响应结束，长度不是96时直接丢弃，不跨请求拼帧。 */
  port->last_response_length = (port->response_length > 0xFFFFU) ?
    0xFFFFU : (uint16_t)port->response_length;
  if (port->response_length != TACTILE_SENSOR_FRAME_SIZE)
  {
    if (port->response_length < TACTILE_SENSOR_FRAME_SIZE)
    {
      port->short_response_count++;
      port->last_short_response_length = (uint16_t)port->response_length;
    }
    else
    {
      port->response_overflow_count++;
    }
    port->response_length = 0U;
    port->response_count++;
    return;
  }

  result = TactileProtocol_DecodeFrame(port->pending_address,
                                       port->response_buffer,
                                       TACTILE_SENSOR_FRAME_SIZE,
                                       &unit_data);
  if (result != TACTILE_PROTOCOL_OK)
  {
    TactileSensor_RecordProtocolError(port, result);
    port->response_length = 0U;
    port->response_count++;
    return;
  }

  address_index = TactileSensor_AddressIndex(port->pending_address);
  memcpy(port->latest_raw[address_index],
         port->response_buffer,
         TACTILE_SENSOR_FRAME_SIZE);
  port->valid_frame_count[address_index]++;

  /* USART1映射左指、USART2映射右指；0x37为上单元、0x36为下单元。 */
  port_index = TactileSensor_PortIndex(port);
  TactileDataStore_UpdateUnit(
    TactileSensor_PortFinger(port_index),
    TactileSensor_AddressUnit(port->pending_address),
    &unit_data);

  port->response_length = 0U;
  port->response_count++;
}

static void TactileSensor_AppendRxBytes(TactilePortContext_t *port,
                                        const uint8_t *data,
                                        uint16_t length)
{
  uint16_t free_length;
  uint16_t copy_length;

  if ((data == NULL) || (length == 0U))
  {
    return;
  }

  /* DMA TC和IDLE可能分多次上报同一响应，这里只负责顺序追加。 */
  free_length = (port->response_length < TACTILE_SENSOR_FRAME_SIZE) ?
    (uint16_t)(TACTILE_SENSOR_FRAME_SIZE - port->response_length) : 0U;
  copy_length = (length <= free_length) ? length : free_length;
  if (copy_length > 0U)
  {
    memcpy(&port->response_buffer[(uint16_t)port->response_length],
           data,
           copy_length);
  }
  port->response_length += length;
}

static void TactileSensor_RecordAndClearRxFlags(
  TactilePortContext_t *port)
{
  uint32_t isr = port->uart->Instance->ISR;
  uint32_t clear_flags = 0U;

  if ((isr & USART_ISR_NE) != 0U)
  {
    port->noise_flag_count++;
    clear_flags |= UART_CLEAR_NEF;
  }
  if ((isr & USART_ISR_FE) != 0U)
  {
    port->framing_flag_count++;
    clear_flags |= UART_CLEAR_FEF;
  }
  if ((isr & USART_ISR_ORE) != 0U)
  {
    port->overrun_flag_count++;
    clear_flags |= UART_CLEAR_OREF;
  }
  if (clear_flags != 0U)
  {
    __HAL_UART_CLEAR_FLAG(port->uart, clear_flags);
  }
}

static void TactileSensor_FlushRx(TactilePortContext_t *port)
{
  __HAL_UART_CLEAR_FLAG(port->uart, TACTILE_UART_CLEAR_FLAGS);
  __HAL_UART_SEND_REQ(port->uart, UART_RXDATA_FLUSH_REQUEST);
}

static bool TactileSensor_StartReceive(TactilePortContext_t *port)
{
  HAL_StatusTypeDef status;

  if ((port->uart == NULL) ||
      (port->uart->RxState != HAL_UART_STATE_READY))
  {
    return false;
  }

  port->dma_read_position = 0U;
  status = HAL_UARTEx_ReceiveToIdle_DMA(port->uart,
                                        port->dma_buffer,
                                        TACTILE_DMA_BUFFER_SIZE);
  if ((status == HAL_OK) && (port->uart->hdmarx != NULL))
  {
    __HAL_DMA_DISABLE_IT(port->uart->hdmarx, DMA_IT_HT);
    /* 协议校验负责最终判定，NE/FE不再中止循环DMA。 */
    CLEAR_BIT(port->uart->Instance->CR3, USART_CR3_EIE);
  }
  return status == HAL_OK;
}

static bool TactileSensor_RestartReceive(TactilePortContext_t *port)
{
  if (port->uart->RxState != HAL_UART_STATE_READY)
  {
    (void)HAL_UART_AbortReceive(port->uart);
  }
  port->dma_read_position = 0U;
  port->response_length = 0U;
  TactileSensor_FlushRx(port);
  return TactileSensor_StartReceive(port);
}

static void TactileSensor_SendRequest(TactilePortContext_t *port,
                                      uint8_t address)
{
  uint8_t command[2];
  uint32_t address_index = TactileSensor_AddressIndex(address);
  HAL_StatusTypeDef status;

  TactileProtocol_BuildReadCommand(address, command);
  port->pending_address = address;
  status = HAL_UART_Transmit(port->uart,
                             command,
                             sizeof(command),
                             TACTILE_TX_TIMEOUT_MS);
  port->trigger_count[address_index]++;
  if (status != HAL_OK)
  {
    port->tx_error_count++;
  }
}

static void TactileSensor_RequestAddress(uint8_t address,
                                         uint32_t timeout_ticks)
{
  uint32_t previous_count[TACTILE_SENSOR_PORT_COUNT];
  uint32_t start;
  uint32_t port_index;

  /* 两个UART先并行发出同一地址请求，再共同等待各自的IDLE响应。 */
  for (port_index = 0U;
       port_index < TACTILE_SENSOR_PORT_COUNT;
       port_index++)
  {
    previous_count[port_index] = tactilePorts[port_index].response_count;
    TactileSensor_SendRequest(&tactilePorts[port_index], address);
  }

  start = osKernelGetTickCount();
  for (;;)
  {
    bool all_received = true;

    for (port_index = 0U;
         port_index < TACTILE_SENSOR_PORT_COUNT;
         port_index++)
    {
      if (tactilePorts[port_index].response_count ==
          previous_count[port_index])
      {
        all_received = false;
      }
    }
    if (all_received)
    {
      return;
    }
    if ((osKernelGetTickCount() - start) >= timeout_ticks)
    {
      for (port_index = 0U;
           port_index < TACTILE_SENSOR_PORT_COUNT;
           port_index++)
      {
        if (tactilePorts[port_index].response_count ==
            previous_count[port_index])
        {
          tactilePorts[port_index].response_timeout_count++;
          tactilePorts[port_index].response_length = 0U;
        }
      }
      return;
    }
    (void)osDelay(1U);
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

static void TactileSensor_FormatForce(float force_n,
                                      char *text,
                                      uint16_t capacity)
{
  int32_t milli_newton;
  uint32_t magnitude;
  const char *sign;

  if ((text == NULL) || (capacity == 0U))
  {
    return;
  }

  milli_newton = (force_n >= 0.0f) ?
    (int32_t)(force_n * 1000.0f + 0.5f) :
    (int32_t)(force_n * 1000.0f - 0.5f);
  sign = (milli_newton < 0) ? "-" : "";
  magnitude = (milli_newton < 0) ?
    (uint32_t)(-(int64_t)milli_newton) : (uint32_t)milli_newton;
  (void)snprintf(text, capacity, "%s%lu.%03lu",
                 sign,
                 (unsigned long)(magnitude / 1000U),
                 (unsigned long)(magnitude % 1000U));
}

static void TactileSensor_DumpForce(void)
{
  gripper_tactile_data_t data;
  char force_text[8][16];
  char line[320];

  if (!TactileDataStore_GetLatest(&data))
  {
    return;
  }

  /* 输出顺序与数据结构映射一致：左/右指内均先0x36、后0x37。 */
  TactileSensor_FormatForce(
    data.finger[TACTILE_FINGER_LEFT]
        .unit[TACTILE_UNIT_LOWER].normal_force_n,
    force_text[0], sizeof(force_text[0]));
  TactileSensor_FormatForce(
    data.finger[TACTILE_FINGER_LEFT]
        .unit[TACTILE_UNIT_LOWER].tangential_force_n,
    force_text[1], sizeof(force_text[1]));
  TactileSensor_FormatForce(
    data.finger[TACTILE_FINGER_LEFT]
        .unit[TACTILE_UNIT_UPPER].normal_force_n,
    force_text[2], sizeof(force_text[2]));
  TactileSensor_FormatForce(
    data.finger[TACTILE_FINGER_LEFT]
        .unit[TACTILE_UNIT_UPPER].tangential_force_n,
    force_text[3], sizeof(force_text[3]));
  TactileSensor_FormatForce(
    data.finger[TACTILE_FINGER_RIGHT]
        .unit[TACTILE_UNIT_LOWER].normal_force_n,
    force_text[4], sizeof(force_text[4]));
  TactileSensor_FormatForce(
    data.finger[TACTILE_FINGER_RIGHT]
        .unit[TACTILE_UNIT_LOWER].tangential_force_n,
    force_text[5], sizeof(force_text[5]));
  TactileSensor_FormatForce(
    data.finger[TACTILE_FINGER_RIGHT]
        .unit[TACTILE_UNIT_UPPER].normal_force_n,
    force_text[6], sizeof(force_text[6]));
  TactileSensor_FormatForce(
    data.finger[TACTILE_FINGER_RIGHT]
        .unit[TACTILE_UNIT_UPPER].tangential_force_n,
    force_text[7], sizeof(force_text[7]));

  (void)snprintf(
    line, sizeof(line),
    "TACTILE,FORCE,seq=%lu,L36,N=%s,T=%s,L37,N=%s,T=%s,R36,N=%s,T=%s,R37,N=%s,T=%s\r\n",
    (unsigned long)data.sequence,
    force_text[0], force_text[1],
    force_text[2], force_text[3],
    force_text[4], force_text[5],
    force_text[6], force_text[7]);
  TactileSensor_Write(line);
}

static void TactileSensor_DumpStats(
  uint32_t elapsed,
  uint32_t tick_frequency,
  uint32_t last_valid[TACTILE_SENSOR_PORT_COUNT]
                     [TACTILE_SENSOR_ADDRESS_COUNT],
  uint32_t last_triggers[TACTILE_SENSOR_PORT_COUNT]
                        [TACTILE_SENSOR_ADDRESS_COUNT],
  uint32_t last_bytes[TACTILE_SENSOR_PORT_COUNT],
  uint32_t *last_snapshot)
{
  gripper_tactile_data_t data;
  uint32_t rates[TACTILE_SENSOR_PORT_COUNT]
                [TACTILE_SENSOR_ADDRESS_COUNT];
  uint32_t tx_rates[TACTILE_SENSOR_PORT_COUNT];
  uint32_t bytes_per_second[TACTILE_SENSOR_PORT_COUNT];
  uint32_t snapshot_count = 0U;
  uint32_t snapshot_rate;
  uint32_t port_index;
  uint32_t address_index;
  char line[512];

  if (TactileDataStore_GetLatest(&data))
  {
    snapshot_count = data.sequence;
  }
  snapshot_rate = TactileSensor_Rate10(
    snapshot_count - *last_snapshot, elapsed, tick_frequency);
  *last_snapshot = snapshot_count;

  for (port_index = 0U;
       port_index < TACTILE_SENSOR_PORT_COUNT;
       port_index++)
  {
    TactilePortContext_t *port = &tactilePorts[port_index];

    for (address_index = 0U;
         address_index < TACTILE_SENSOR_ADDRESS_COUNT;
         address_index++)
    {
      uint32_t current = port->valid_frame_count[address_index];

      rates[port_index][address_index] = TactileSensor_Rate10(
        current - last_valid[port_index][address_index],
        elapsed, tick_frequency);
      last_valid[port_index][address_index] = current;
    }
    tx_rates[port_index] = TactileSensor_Rate10(
      port->trigger_count[0] - last_triggers[port_index][0],
      elapsed, tick_frequency);
    last_triggers[port_index][0] = port->trigger_count[0];
    last_triggers[port_index][1] = port->trigger_count[1];
    bytes_per_second[port_index] =
      (uint32_t)(((uint64_t)(port->rx_byte_count - last_bytes[port_index]) *
                  tick_frequency) / elapsed);
    last_bytes[port_index] = port->rx_byte_count;
  }

  (void)snprintf(
    line, sizeof(line),
    "TACTILE,STAT,snap=%lu.%lu,U1_36=%lu.%lu,U1_37=%lu.%lu,U2_36=%lu.%lu,U2_37=%lu.%lu,tx1=%lu.%lu,tx2=%lu.%lu,bps1=%lu,bps2=%lu,ck1=%lu,ck2=%lu,fmt1=%lu,fmt2=%lu,short1=%lu/%u,short2=%lu/%u,fe1=%lu,fe2=%lu,ne1=%lu,ne2=%lu,ore1=%lu,ore2=%lu,to1=%lu,to2=%lu,dma1=%lu,dma2=%lu,uart1=%lu,uart2=%lu,txerr1=%lu,txerr2=%lu,ovf1=%lu,ovf2=%lu\r\n",
    (unsigned long)(snapshot_rate / 10U),
    (unsigned long)(snapshot_rate % 10U),
    (unsigned long)(rates[TACTILE_SENSOR_PORT_USART1][0] / 10U),
    (unsigned long)(rates[TACTILE_SENSOR_PORT_USART1][0] % 10U),
    (unsigned long)(rates[TACTILE_SENSOR_PORT_USART1][1] / 10U),
    (unsigned long)(rates[TACTILE_SENSOR_PORT_USART1][1] % 10U),
    (unsigned long)(rates[TACTILE_SENSOR_PORT_USART2][0] / 10U),
    (unsigned long)(rates[TACTILE_SENSOR_PORT_USART2][0] % 10U),
    (unsigned long)(rates[TACTILE_SENSOR_PORT_USART2][1] / 10U),
    (unsigned long)(rates[TACTILE_SENSOR_PORT_USART2][1] % 10U),
    (unsigned long)(tx_rates[TACTILE_SENSOR_PORT_USART1] / 10U),
    (unsigned long)(tx_rates[TACTILE_SENSOR_PORT_USART1] % 10U),
    (unsigned long)(tx_rates[TACTILE_SENSOR_PORT_USART2] / 10U),
    (unsigned long)(tx_rates[TACTILE_SENSOR_PORT_USART2] % 10U),
    (unsigned long)bytes_per_second[TACTILE_SENSOR_PORT_USART1],
    (unsigned long)bytes_per_second[TACTILE_SENSOR_PORT_USART2],
    (unsigned long)tactilePorts[TACTILE_SENSOR_PORT_USART1]
                  .checksum_error_count,
    (unsigned long)tactilePorts[TACTILE_SENSOR_PORT_USART2]
                  .checksum_error_count,
    (unsigned long)tactilePorts[TACTILE_SENSOR_PORT_USART1]
                  .format_error_count,
    (unsigned long)tactilePorts[TACTILE_SENSOR_PORT_USART2]
                  .format_error_count,
    (unsigned long)tactilePorts[TACTILE_SENSOR_PORT_USART1]
                  .short_response_count,
    (unsigned int)tactilePorts[TACTILE_SENSOR_PORT_USART1]
                 .last_short_response_length,
    (unsigned long)tactilePorts[TACTILE_SENSOR_PORT_USART2]
                  .short_response_count,
    (unsigned int)tactilePorts[TACTILE_SENSOR_PORT_USART2]
                 .last_short_response_length,
    (unsigned long)tactilePorts[TACTILE_SENSOR_PORT_USART1]
                  .framing_flag_count,
    (unsigned long)tactilePorts[TACTILE_SENSOR_PORT_USART2]
                  .framing_flag_count,
    (unsigned long)tactilePorts[TACTILE_SENSOR_PORT_USART1]
                  .noise_flag_count,
    (unsigned long)tactilePorts[TACTILE_SENSOR_PORT_USART2]
                  .noise_flag_count,
    (unsigned long)tactilePorts[TACTILE_SENSOR_PORT_USART1]
                  .overrun_flag_count,
    (unsigned long)tactilePorts[TACTILE_SENSOR_PORT_USART2]
                  .overrun_flag_count,
    (unsigned long)tactilePorts[TACTILE_SENSOR_PORT_USART1]
                  .response_timeout_count,
    (unsigned long)tactilePorts[TACTILE_SENSOR_PORT_USART2]
                  .response_timeout_count,
    (unsigned long)tactilePorts[TACTILE_SENSOR_PORT_USART1]
                  .dma_position_error_count,
    (unsigned long)tactilePorts[TACTILE_SENSOR_PORT_USART2]
                  .dma_position_error_count,
    (unsigned long)tactilePorts[TACTILE_SENSOR_PORT_USART1]
                  .uart_error_count,
    (unsigned long)tactilePorts[TACTILE_SENSOR_PORT_USART2]
                  .uart_error_count,
    (unsigned long)tactilePorts[TACTILE_SENSOR_PORT_USART1]
                  .tx_error_count,
    (unsigned long)tactilePorts[TACTILE_SENSOR_PORT_USART2]
                  .tx_error_count,
    (unsigned long)tactilePorts[TACTILE_SENSOR_PORT_USART1]
                  .response_overflow_count,
    (unsigned long)tactilePorts[TACTILE_SENSOR_PORT_USART2]
                  .response_overflow_count);
  TactileSensor_Write(line);
}

void TactileSensor_Init(UART_HandleTypeDef *uart1,
                        UART_HandleTypeDef *uart2)
{
  memset(tactilePorts, 0, sizeof(tactilePorts));
  tactilePorts[TACTILE_SENSOR_PORT_USART1].uart = uart1;
  tactilePorts[TACTILE_SENSOR_PORT_USART1].dma_buffer =
    tactileUart1DmaBuffer;
  tactilePorts[TACTILE_SENSOR_PORT_USART2].uart = uart2;
  tactilePorts[TACTILE_SENSOR_PORT_USART2].dma_buffer =
    tactileUart2DmaBuffer;
  TactileDataStore_Init();
}

bool TactileSensor_GetLatestData(gripper_tactile_data_t *data)
{
  return TactileDataStore_GetLatest(data);
}

bool TactileSensor_GetLatestRaw(uint8_t *buffer,
                                uint16_t capacity,
                                uint32_t *frame_count)
{
  return TactileSensor_GetLatestRawByPort(
    TACTILE_SENSOR_PORT_USART2,
    TACTILE_SENSOR_ADDRESS_36,
    buffer, capacity, frame_count);
}

bool TactileSensor_GetLatestRawByAddress(uint8_t address,
                                         uint8_t *buffer,
                                         uint16_t capacity,
                                         uint32_t *frame_count)
{
  return TactileSensor_GetLatestRawByPort(
    TACTILE_SENSOR_PORT_USART2,
    address,
    buffer, capacity, frame_count);
}

bool TactileSensor_GetLatestRawByPort(uint8_t port_index,
                                      uint8_t address,
                                      uint8_t *buffer,
                                      uint16_t capacity,
                                      uint32_t *frame_count)
{
  TactilePortContext_t *port;
  uint32_t address_index;
  uint32_t count;

  if ((port_index >= TACTILE_SENSOR_PORT_COUNT) ||
      !TactileProtocol_IsAddress(address) ||
      (buffer == NULL) ||
      (capacity < TACTILE_SENSOR_FRAME_SIZE))
  {
    return false;
  }
  port = &tactilePorts[port_index];
  address_index = TactileSensor_AddressIndex(address);

  taskENTER_CRITICAL();
  memcpy(buffer,
         port->latest_raw[address_index],
         TACTILE_SENSOR_FRAME_SIZE);
  count = port->valid_frame_count[address_index];
  taskEXIT_CRITICAL();
  if (frame_count != NULL)
  {
    *frame_count = count;
  }
  return count != 0U;
}

void TactileSensor_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size)
{
  TactilePortContext_t *port = TactileSensor_FindPort(uart);
  HAL_UART_RxEventTypeTypeDef event_type;
  uint16_t previous_position;
  uint16_t current_position;
  uint16_t chunk_length = 0U;

  if (port == NULL)
  {
    return;
  }
  if (size > TACTILE_DMA_BUFFER_SIZE)
  {
    port->dma_position_error_count++;
    return;
  }

  event_type = HAL_UARTEx_GetRxEventType(uart);
  previous_position = port->dma_read_position;
  current_position = size;
  /* HAL回调给出循环DMA当前写位置，需要分别处理尾部、前部和整圈边界。 */
  if (current_position == TACTILE_DMA_BUFFER_SIZE)
  {
    current_position = 0U;
    /* IDLE和DMA TC可能报告同一位置，读指针已归零时不重复处理。 */
    if (previous_position != 0U)
    {
      chunk_length = TACTILE_DMA_BUFFER_SIZE - previous_position;
      TactileSensor_AppendRxBytes(
        port, &port->dma_buffer[previous_position], chunk_length);
    }
  }
  else if (current_position > previous_position)
  {
    chunk_length = current_position - previous_position;
    TactileSensor_AppendRxBytes(
      port, &port->dma_buffer[previous_position], chunk_length);
  }
  else if (current_position < previous_position)
  {
    uint16_t tail_length = TACTILE_DMA_BUFFER_SIZE - previous_position;

    chunk_length = tail_length + current_position;
    TactileSensor_AppendRxBytes(
      port, &port->dma_buffer[previous_position], tail_length);
    if (current_position > 0U)
    {
      TactileSensor_AppendRxBytes(port,
                                  port->dma_buffer,
                                  current_position);
    }
  }

  port->dma_read_position = current_position;
  if (chunk_length > 0U)
  {
    port->rx_byte_count += chunk_length;
    port->rx_event_count++;
  }
  TactileSensor_RecordAndClearRxFlags(port);

  if ((event_type == HAL_UART_RXEVENT_IDLE) &&
      (port->response_length > 0U))
  {
    TactileSensor_FinalizeResponse(port);
  }
}

void TactileSensor_ErrorCallback(UART_HandleTypeDef *uart)
{
  TactilePortContext_t *port = TactileSensor_FindPort(uart);

  if (port != NULL)
  {
    port->uart_error_count++;
    port->restart_pending = 1U;
  }
}

static void TactileSensor_Task(void *argument)
{
  uint32_t tick_frequency = osKernelGetTickFreq();
  uint32_t period_ticks =
    (TACTILE_SAMPLE_PERIOD_MS * tick_frequency) / 1000U;
  uint32_t timeout_ticks =
    (TACTILE_RESPONSE_TIMEOUT_MS * tick_frequency) / 1000U;
  uint32_t stats_ticks =
    (TACTILE_STATS_PERIOD_MS * tick_frequency) / 1000U;
  uint32_t next = osKernelGetTickCount();
  uint32_t stats_tick = next;
  uint32_t last_valid[TACTILE_SENSOR_PORT_COUNT]
                     [TACTILE_SENSOR_ADDRESS_COUNT] = {{0U}};
  uint32_t last_triggers[TACTILE_SENSOR_PORT_COUNT]
                        [TACTILE_SENSOR_ADDRESS_COUNT] = {{0U}};
  uint32_t last_bytes[TACTILE_SENSOR_PORT_COUNT] = {0U};
  uint32_t last_snapshot = 0U;
  uint32_t port_index;
  (void)argument;

  if (period_ticks == 0U)
  {
    period_ticks = 1U;
  }
  if (timeout_ticks == 0U)
  {
    timeout_ticks = 1U;
  }
  if (stats_ticks == 0U)
  {
    stats_ticks = 1U;
  }

  for (port_index = 0U;
       port_index < TACTILE_SENSOR_PORT_COUNT;
       port_index++)
  {
    if (!TactileSensor_RestartReceive(&tactilePorts[port_index]))
    {
      tactilePorts[port_index].restart_pending = 1U;
    }
  }
  TactileSensor_Write(
    "TACTILE,TASK,start,uart=1/2,addr=0x36/0x37,rate=40Hz\r\n");

  for (;;)
  {
    uint32_t now;

    for (port_index = 0U;
         port_index < TACTILE_SENSOR_PORT_COUNT;
         port_index++)
    {
      TactilePortContext_t *port = &tactilePorts[port_index];

      if (port->restart_pending != 0U)
      {
        port->restart_pending = 0U;
        if (TactileSensor_RestartReceive(port))
        {
          port->restart_count++;
        }
        else
        {
          port->restart_pending = 1U;
        }
      }
    }

    /* 每25ms启动一轮四单元采样，只有四帧均有效才生成新快照。 */
    TactileDataStore_BeginCycle();
    TactileSensor_RequestAddress(TACTILE_SENSOR_ADDRESS_36, timeout_ticks);
    TactileSensor_RequestAddress(TACTILE_SENSOR_ADDRESS_37, timeout_ticks);

    now = osKernelGetTickCount();
    if ((now - stats_tick) >= stats_ticks)
    {
      uint32_t elapsed = now - stats_tick;

      TactileSensor_DumpStats(elapsed, tick_frequency,
                              last_valid, last_triggers, last_bytes,
                              &last_snapshot);
      TactileSensor_DumpForce();
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
    .stack_size = 1536U * 4U,
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

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size)
{
  TactileSensor_RxEventCallback(uart, size);
}
