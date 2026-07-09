#include "tactile_service.h"

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "debug_uart_transport.h"
#include "tactile_data.h"
#include "tactile_protocol.h"
#include "tactile_uart_transport.h"

#include <stdio.h>
#include <string.h>

#define TACTILE_SERVICE_SAMPLE_RATE_HZ       40U
#define TACTILE_SERVICE_PERIOD_MS            (1000U / TACTILE_SERVICE_SAMPLE_RATE_HZ)
#define TACTILE_SERVICE_UNIT_TIMEOUT_MS      8U
#define TACTILE_SERVICE_INTER_UNIT_GAP_MS    2U
#define TACTILE_SERVICE_RETRY_DELAY_MS       1U
#define TACTILE_SERVICE_MAX_READ_ATTEMPTS    1U
#define TACTILE_SERVICE_RING_DMA_READ_ATTEMPTS 1U
#define TACTILE_SERVICE_RX_BUFFER_LENGTH     128U
#define TACTILE_SERVICE_DEBUG_PRINT_MS       250U
#define TACTILE_SERVICE_FPS_PRINT_MS         1000U
#define TACTILE_SERVICE_LEGACY_TOUCH_LOG_ENABLE 0U
#define TACTILE_SERVICE_ACQ_STACK_WORDS      384U
#define TACTILE_SERVICE_DEBUG_STACK_WORDS    768U
#define TACTILE_USART1_DMA_RX_ADDRESS        0x2404FE00UL
#define TACTILE_USART2_DMA_RX_ADDRESS        0x2404FF00UL
#define TACTILE_USART2_DMA_RING_LENGTH       256U
#define TACTILE_USART2_SW_RING_LENGTH        512U
#if !TACTILE_UART_DMA_ENABLE
#define TACTILE_SERVICE_USE_DMA              false
#define TACTILE_SERVICE_UART1_BOOT_MODE      "no_dma"
#else
#define TACTILE_SERVICE_USE_DMA              true
#define TACTILE_SERVICE_UART1_BOOT_MODE      "dma"
#endif
#if TACTILE_USART2_RING_DMA_ENABLE
#define TACTILE_SERVICE_UART2_BOOT_MODE      "ringdma"
#else
#define TACTILE_SERVICE_UART2_BOOT_MODE      TACTILE_SERVICE_UART1_BOOT_MODE
#endif

typedef struct
{
  uint32_t attempts;
  uint32_t rx_frame_count;
  uint32_t ok_count;
  uint32_t tx_error_count;
  uint32_t rx_error_count;
  uint32_t timeout_count;
  uint32_t bad_length_count;
  uint32_t bad_header_count;
  uint32_t checksum_error_count;
  uint32_t bad_channel_count;
  uint32_t ring_overrun_count;
  uint32_t idle_event_count;
  uint32_t last_result;
  uint32_t last_hal_status;
  uint16_t last_rx_len;
  uint16_t last_tx_len;
  uint32_t last_hal_error;
  TactileProtocolResult_t last_protocol;
  uint8_t last_tx_preview[2];
  uint8_t last_rx_preview[8];
} TactileUnitRuntimeStat_t;

typedef struct
{
  const char *name;
  uint8_t sensor_index;
  UART_HandleTypeDef *huart;
  TactileUartTransport_t transport;
  uint8_t *rx_buffer;
  uint8_t *dma_ring_buffer;
  uint16_t dma_ring_length;
  uint8_t *sw_ring_buffer;
  uint16_t sw_ring_length;
  osThreadId_t task_handle;
  uint32_t start_delay_ms;
  bool use_dma;
  bool use_ring_dma;
} TactileSensorContext_t;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

#if !TACTILE_UART_DMA_ENABLE
/* 非 DMA 调试时使用普通静态缓冲区，避免绝对地址段影响问题定位。 */
static uint8_t tactileUsart1RxBuffer[TACTILE_SERVICE_RX_BUFFER_LENGTH];
static uint8_t tactileUsart2RxBuffer[TACTILE_SERVICE_RX_BUFFER_LENGTH];
#elif defined(__CC_ARM)
__attribute__((at(TACTILE_USART1_DMA_RX_ADDRESS), aligned(32)))
static uint8_t tactileUsart1RxBuffer[TACTILE_SERVICE_RX_BUFFER_LENGTH];
__attribute__((at(TACTILE_USART2_DMA_RX_ADDRESS), aligned(32)))
static uint8_t tactileUsart2RxBuffer[TACTILE_SERVICE_RX_BUFFER_LENGTH];
#elif defined(__ARMCC_VERSION)
__attribute__((section(".ARM.__at_0x2404FE00"), aligned(32)))
static uint8_t tactileUsart1RxBuffer[TACTILE_SERVICE_RX_BUFFER_LENGTH];
__attribute__((section(".ARM.__at_0x2404FF00"), aligned(32)))
static uint8_t tactileUsart2RxBuffer[TACTILE_SERVICE_RX_BUFFER_LENGTH];
#else
__attribute__((section(".dma_buffer"), aligned(32)))
static uint8_t tactileUsart1RxBuffer[TACTILE_SERVICE_RX_BUFFER_LENGTH];
__attribute__((section(".dma_buffer"), aligned(32)))
static uint8_t tactileUsart2RxBuffer[TACTILE_SERVICE_RX_BUFFER_LENGTH];
#endif

#if TACTILE_USART2_RING_DMA_ENABLE
#if defined(__CC_ARM)
__align(32)
static uint8_t tactileUsart2DmaRingBuffer[TACTILE_USART2_DMA_RING_LENGTH];
#elif defined(__ARMCC_VERSION) || defined(__GNUC__)
__attribute__((aligned(32)))
static uint8_t tactileUsart2DmaRingBuffer[TACTILE_USART2_DMA_RING_LENGTH];
#else
static uint8_t tactileUsart2DmaRingBuffer[TACTILE_USART2_DMA_RING_LENGTH];
#endif
static uint8_t tactileUsart2SwRingBuffer[TACTILE_USART2_SW_RING_LENGTH];
#endif

static volatile bool tactileServiceEnabled = true;
static TactileSnapshot_t serviceSnapshot;
static TactileStats_t serviceStats;
static uint32_t fpsWindowCounter[TACTILE_SENSOR_COUNT][TACTILE_UNIT_COUNT];
static TactileUnitRuntimeStat_t runtimeStats[TACTILE_SENSOR_COUNT][TACTILE_UNIT_COUNT];
static StaticTask_t tactileUsart1TaskControlBlock;
static StaticTask_t tactileUsart2TaskControlBlock;
static StaticTask_t tactileDebugTaskControlBlock;
static StackType_t tactileUsart1TaskStack[TACTILE_SERVICE_ACQ_STACK_WORDS];
static StackType_t tactileUsart2TaskStack[TACTILE_SERVICE_ACQ_STACK_WORDS];
static StackType_t tactileDebugTaskStack[TACTILE_SERVICE_DEBUG_STACK_WORDS];
static osThreadId_t tactileDebugTaskHandle;

static TactileSensorContext_t sensorContexts[TACTILE_SENSOR_COUNT] = {
  {
    .name = "usart1",
    .sensor_index = TACTILE_SENSOR_USART1,
    .huart = &huart1,
    .rx_buffer = tactileUsart1RxBuffer,
    .dma_ring_buffer = NULL,
    .dma_ring_length = 0U,
    .sw_ring_buffer = NULL,
    .sw_ring_length = 0U,
    .task_handle = NULL,
    .start_delay_ms = 0U,
    .use_dma = TACTILE_SERVICE_USE_DMA,
    .use_ring_dma = false
  },
  {
    .name = "usart2",
    .sensor_index = TACTILE_SENSOR_USART2,
    .huart = &huart2,
    .rx_buffer = tactileUsart2RxBuffer,
#if TACTILE_USART2_RING_DMA_ENABLE
    .dma_ring_buffer = tactileUsart2DmaRingBuffer,
    .dma_ring_length = TACTILE_USART2_DMA_RING_LENGTH,
    .sw_ring_buffer = tactileUsart2SwRingBuffer,
    .sw_ring_length = TACTILE_USART2_SW_RING_LENGTH,
#else
    .dma_ring_buffer = NULL,
    .dma_ring_length = 0U,
    .sw_ring_buffer = NULL,
    .sw_ring_length = 0U,
#endif
    .task_handle = NULL,
    .start_delay_ms = TACTILE_SERVICE_PERIOD_MS / 2U,
#if TACTILE_USART2_RING_DMA_ENABLE
    .use_dma = false,
    .use_ring_dma = true,
#else
    .use_dma = TACTILE_SERVICE_USE_DMA,
    .use_ring_dma = false,
#endif
  }
};

static uint32_t TactileService_MsToTicks(uint32_t milliseconds)
{
  uint32_t ticks = ((milliseconds * osKernelGetTickFreq()) + 999U) / 1000U;
  return (ticks == 0U) ? 1U : ticks;
}

static uint8_t TactileService_UnitAddress(uint8_t unitIndex)
{
  return (unitIndex == TACTILE_UNIT_UPPER) ?
         TACTILE_UPPER_ADDRESS : TACTILE_LOWER_ADDRESS;
}

static uint8_t TactileService_LocalUnitMask(uint8_t unitIndex)
{
  return (unitIndex == TACTILE_UNIT_UPPER) ?
         TACTILE_MASK_UPPER : TACTILE_MASK_LOWER;
}

static uint8_t TactileService_GlobalUnitMask(uint8_t sensorIndex,
                                             uint8_t unitIndex)
{
  return (uint8_t)(1U << ((sensorIndex * TACTILE_UNIT_COUNT) + unitIndex));
}

#if TACTILE_SERVICE_LEGACY_TOUCH_LOG_ENABLE
static int16_t TactileService_ForceOrZero(const TactileUnitData_t *unit,
                                          bool tangent)
{
  if ((unit == NULL) || (!unit->valid))
  {
    return 0;
  }
  return tangent ? unit->tangent_force : unit->normal_force;
}

static uint32_t TactileService_ProximityOrZero(const TactileUnitData_t *unit)
{
  return ((unit != NULL) && unit->valid) ? unit->proximity : 0U;
}

static uint8_t TactileService_StatusOrZero(const TactileUnitData_t *unit)
{
  return ((unit != NULL) && unit->valid) ? unit->status : 0U;
}
#endif

static void TactileService_UpdateRuntimeStat(
  TactileSensorContext_t *context,
  uint8_t unitIndex,
  bool transportOk,
  TactileProtocolResult_t protocolResult)
{
  TactileUartTransportStats_t transportStats;
  TactileUnitRuntimeStat_t *stat;
  uint8_t sensorIndex;

  if ((context == NULL) || (unitIndex >= TACTILE_UNIT_COUNT))
  {
    return;
  }

  sensorIndex = context->sensor_index;
  TactileUartTransport_GetStats(&context->transport, &transportStats);

  taskENTER_CRITICAL();
  stat = &runtimeStats[sensorIndex][unitIndex];
  stat->attempts++;
  stat->last_result = transportStats.last_result;
  stat->last_hal_status = transportStats.last_hal_status;
  stat->last_rx_len = transportStats.last_rx_len;
  stat->last_tx_len = transportStats.last_tx_len;
  stat->last_hal_error = transportStats.last_hal_error;
  stat->last_protocol = protocolResult;
  stat->ring_overrun_count = transportStats.ring_overrun_count;
  stat->idle_event_count = transportStats.idle_event_count;

  if (transportOk)
  {
    stat->rx_frame_count++;
    switch (protocolResult)
    {
      case TACTILE_PROTOCOL_OK:
        stat->ok_count++;
        break;
      case TACTILE_PROTOCOL_BAD_LENGTH:
        stat->bad_length_count++;
        break;
      case TACTILE_PROTOCOL_BAD_HEADER:
        stat->bad_header_count++;
        break;
      case TACTILE_PROTOCOL_BAD_CHECKSUM:
        stat->checksum_error_count++;
        break;
      case TACTILE_PROTOCOL_BAD_CHANNEL_COUNT:
        stat->bad_channel_count++;
        break;
      default:
        stat->rx_error_count++;
        break;
    }
  }
  else
  {
    switch ((TactileUartTransportResult_t)transportStats.last_result)
    {
      case TACTILE_UART_RESULT_TX_ERROR:
        stat->tx_error_count++;
        break;
      case TACTILE_UART_RESULT_RX_TIMEOUT:
        stat->timeout_count++;
        break;
      case TACTILE_UART_RESULT_RX_START_ERROR:
      case TACTILE_UART_RESULT_RX_INCOMPLETE:
        stat->rx_error_count++;
        break;
      default:
        stat->rx_error_count++;
        break;
    }
  }
  (void)memcpy(stat->last_tx_preview,
               transportStats.last_tx_preview,
               sizeof(stat->last_tx_preview));
  (void)memcpy(stat->last_rx_preview,
               transportStats.last_rx_preview,
               sizeof(stat->last_rx_preview));
  taskEXIT_CRITICAL();
}

static void TactileService_RecordError(TactileSensorContext_t *context,
                                       uint8_t unitIndex,
                                       bool transportOk,
                                       TactileProtocolResult_t protocolResult)
{
  TactileSensorData_t *sensor;
  TactileUnitData_t *unit;
  uint8_t sensorIndex;

  if ((context == NULL) || (unitIndex >= TACTILE_UNIT_COUNT))
  {
    return;
  }

  sensorIndex = context->sensor_index;

  taskENTER_CRITICAL();
  sensor = &serviceSnapshot.sensor[sensorIndex];
  unit = &sensor->unit[unitIndex];
  unit->address = TactileService_UnitAddress(unitIndex);
  unit->sensor_index = sensorIndex;
  unit->unit_index = unitIndex;
  unit->timestamp_ms = HAL_GetTick();
  unit->valid = false;

  if (!transportOk)
  {
    unit->timeout_count++;
    sensor->timeout_count++;
    serviceStats.timeout_count++;
  }
  else if (protocolResult == TACTILE_PROTOCOL_BAD_CHECKSUM)
  {
    unit->checksum_error_count++;
    sensor->checksum_error_count++;
    serviceStats.checksum_error_count++;
  }
  else
  {
    unit->frame_error_count++;
    sensor->frame_error_count++;
    serviceStats.frame_error_count++;
  }
  taskEXIT_CRITICAL();
}

static bool TactileService_ReadUnit(TactileSensorContext_t *context,
                                    uint8_t unitIndex)
{
  uint8_t command[2];
  uint16_t rxLength = 0U;
  uint8_t address;
  uint8_t sensorIndex;
  TactileProtocolResult_t protocolResult = TACTILE_PROTOCOL_BAD_LENGTH;
  TactileUnitData_t decoded;
  bool transportOk = false;
  uint8_t maxAttempts;
  uint8_t attempt;

  if ((context == NULL) || (unitIndex >= TACTILE_UNIT_COUNT))
  {
    return false;
  }

  sensorIndex = context->sensor_index;
  address = TactileService_UnitAddress(unitIndex);
  TactileProtocol_BuildReadCommand(address, command);
  maxAttempts = context->use_ring_dma ?
                TACTILE_SERVICE_RING_DMA_READ_ATTEMPTS :
                TACTILE_SERVICE_MAX_READ_ATTEMPTS;

  for (attempt = 0U; attempt < maxAttempts; attempt++)
  {
    rxLength = 0U;
    protocolResult = TACTILE_PROTOCOL_BAD_LENGTH;

    if (context->use_ring_dma)
    {
      transportOk = TactileUartTransport_WriteReadCircular(
        &context->transport,
        command, sizeof(command),
        address,
        context->rx_buffer,
        TACTILE_SERVICE_RX_BUFFER_LENGTH,
        TACTILE_FRAME_LENGTH,
        &rxLength,
        TACTILE_SERVICE_UNIT_TIMEOUT_MS);
    }
    else if (context->use_dma)
    {
      transportOk = TactileUartTransport_WriteRead(
        &context->transport,
        command, sizeof(command),
        context->rx_buffer,
        TACTILE_SERVICE_RX_BUFFER_LENGTH,
        TACTILE_FRAME_LENGTH,
        &rxLength,
        TACTILE_SERVICE_UNIT_TIMEOUT_MS);
    }
    else
    {
      transportOk = TactileUartTransport_WriteReadPolling(
        &context->transport,
        command, sizeof(command),
        context->rx_buffer,
        TACTILE_SERVICE_RX_BUFFER_LENGTH,
        TACTILE_FRAME_LENGTH,
        &rxLength,
        TACTILE_SERVICE_UNIT_TIMEOUT_MS);
    }

    if (transportOk)
    {
      taskENTER_CRITICAL();
      decoded = serviceSnapshot.sensor[sensorIndex].unit[unitIndex];
      taskEXIT_CRITICAL();

      protocolResult = TactileProtocol_DecodeFrame(address,
                                                  context->rx_buffer,
                                                  rxLength,
                                                  &decoded);
      if (protocolResult == TACTILE_PROTOCOL_OK)
      {
        decoded.address = address;
        decoded.sensor_index = sensorIndex;
        decoded.unit_index = unitIndex;
        decoded.timestamp_ms = HAL_GetTick();
        decoded.valid = true;

        taskENTER_CRITICAL();
        serviceSnapshot.sensor[sensorIndex].unit[unitIndex] = decoded;
        fpsWindowCounter[sensorIndex][unitIndex]++;
        taskEXIT_CRITICAL();

        TactileService_UpdateRuntimeStat(context, unitIndex,
                                         transportOk, protocolResult);
        return true;
      }
    }

    TactileService_UpdateRuntimeStat(context, unitIndex,
                                     transportOk, protocolResult);
    if ((attempt + 1U) < maxAttempts)
    {
      /*
       * 失败后给传感器和 UART 状态机留一个很短的恢复间隔，再重发同一地址。
       * 40Hz 周期有足够余量，优先保证本周期拿到有效帧。
       */
      (void)osDelay(TactileService_MsToTicks(TACTILE_SERVICE_RETRY_DELAY_MS));
    }
  }

  TactileService_RecordError(context, unitIndex, transportOk, protocolResult);
  return false;
}

static void TactileService_RebuildMasks(void)
{
  uint8_t sensorIndex;
  uint8_t unitIndex;
  uint8_t globalMask = 0U;
  uint8_t localMask;

  for (sensorIndex = 0U; sensorIndex < TACTILE_SENSOR_COUNT; sensorIndex++)
  {
    localMask = 0U;
    for (unitIndex = 0U; unitIndex < TACTILE_UNIT_COUNT; unitIndex++)
    {
      if (serviceSnapshot.sensor[sensorIndex].unit[unitIndex].valid)
      {
        localMask |= TactileService_LocalUnitMask(unitIndex);
        globalMask |= TactileService_GlobalUnitMask(sensorIndex, unitIndex);
      }
    }
    serviceSnapshot.sensor[sensorIndex].valid_mask = localMask;
    serviceSnapshot.sensor[sensorIndex].complete =
      (localMask == (TACTILE_MASK_UPPER | TACTILE_MASK_LOWER));
  }

  serviceSnapshot.valid_mask = globalMask;
  serviceSnapshot.complete = (globalMask == TACTILE_MASK_ALL);
}

static void TactileService_PublishSnapshot(void)
{
  TactileSnapshot_t localSnapshot;

  taskENTER_CRITICAL();
  TactileService_RebuildMasks();
  serviceSnapshot.sample_id++;
  serviceSnapshot.timestamp_ms = HAL_GetTick();

  serviceStats.sample_count++;
  serviceStats.last_valid_mask = serviceSnapshot.valid_mask;
  if (serviceSnapshot.complete)
  {
    serviceStats.complete_count++;
  }
  else
  {
    serviceStats.partial_count++;
  }

  localSnapshot = serviceSnapshot;
  taskEXIT_CRITICAL();

  TactileData_Publish(&localSnapshot);
}

#if TACTILE_SERVICE_LEGACY_TOUCH_LOG_ENABLE
static void TactileService_PrintSummary(void)
{
  TactileSnapshot_t snapshot;
  const TactileUnitData_t *u1Upper;
  const TactileUnitData_t *u1Lower;
  const TactileUnitData_t *u2Upper;
  const TactileUnitData_t *u2Lower;
  char line[256];

  if (!TactileData_GetSnapshot(&snapshot, HAL_GetTick()))
  {
    return;
  }

  u1Upper = &snapshot.sensor[TACTILE_SENSOR_USART1].unit[TACTILE_UNIT_UPPER];
  u1Lower = &snapshot.sensor[TACTILE_SENSOR_USART1].unit[TACTILE_UNIT_LOWER];
  u2Upper = &snapshot.sensor[TACTILE_SENSOR_USART2].unit[TACTILE_UNIT_UPPER];
  u2Lower = &snapshot.sensor[TACTILE_SENSOR_USART2].unit[TACTILE_UNIT_LOWER];

  (void)snprintf(line, sizeof(line),
                 "touch: %lu,%u,%d,%d,%d,%d,%d,%d,%d,%d,%lu,%lu,%lu,%lu,%u,%u,%u,%u\r\n",
                 (unsigned long)snapshot.sample_id,
                 snapshot.valid_mask,
                 TactileService_ForceOrZero(u1Upper, false),
                 TactileService_ForceOrZero(u1Upper, true),
                 TactileService_ForceOrZero(u1Lower, false),
                 TactileService_ForceOrZero(u1Lower, true),
                 TactileService_ForceOrZero(u2Upper, false),
                 TactileService_ForceOrZero(u2Upper, true),
                 TactileService_ForceOrZero(u2Lower, false),
                 TactileService_ForceOrZero(u2Lower, true),
                 (unsigned long)TactileService_ProximityOrZero(u1Upper),
                 (unsigned long)TactileService_ProximityOrZero(u1Lower),
                 (unsigned long)TactileService_ProximityOrZero(u2Upper),
                 (unsigned long)TactileService_ProximityOrZero(u2Lower),
                 TactileService_StatusOrZero(u1Upper),
                 TactileService_StatusOrZero(u1Lower),
                 TactileService_StatusOrZero(u2Upper),
                 TactileService_StatusOrZero(u2Lower));
  (void)DebugUartTransport_Write(line, 10U);
}

static void TactileService_PrintDiagPair(const char *prefix,
                                         const TactileUnitRuntimeStat_t *upper,
                                         const TactileUnitRuntimeStat_t *lower)
{
  char line[256];

  if ((prefix == NULL) || (upper == NULL) || (lower == NULL))
  {
    return;
  }

  (void)snprintf(line, sizeof(line),
                 "%s: %lu,%lu,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,"
                 "%lu,%lu,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X\r\n",
                 prefix,
                 (unsigned long)upper->last_hal_status,
                 (unsigned long)upper->last_hal_error,
                 upper->last_rx_preview[0],
                 upper->last_rx_preview[1],
                 upper->last_rx_preview[2],
                 upper->last_rx_preview[3],
                 upper->last_rx_preview[4],
                 upper->last_rx_preview[5],
                 upper->last_rx_preview[6],
                 upper->last_rx_preview[7],
                 (unsigned long)lower->last_hal_status,
                 (unsigned long)lower->last_hal_error,
                 lower->last_rx_preview[0],
                 lower->last_rx_preview[1],
                 lower->last_rx_preview[2],
                 lower->last_rx_preview[3],
                 lower->last_rx_preview[4],
                 lower->last_rx_preview[5],
                 lower->last_rx_preview[6],
                 lower->last_rx_preview[7]);
  (void)DebugUartTransport_Write(line, 10U);
}

static void TactileService_PrintFrameRate(void)
{
  uint32_t fps[TACTILE_SENSOR_COUNT][TACTILE_UNIT_COUNT];
  TactileUnitRuntimeStat_t stat[TACTILE_SENSOR_COUNT][TACTILE_UNIT_COUNT];
  char line[256];

  taskENTER_CRITICAL();
  (void)memcpy(fps, fpsWindowCounter, sizeof(fps));
  (void)memset(fpsWindowCounter, 0, sizeof(fpsWindowCounter));
  (void)memcpy(serviceStats.unit_fps, fps, sizeof(serviceStats.unit_fps));
  (void)memcpy(stat, runtimeStats, sizeof(stat));
  (void)memset(runtimeStats, 0, sizeof(runtimeStats));
  taskEXIT_CRITICAL();

  (void)snprintf(line, sizeof(line),
                 "touchfps: %lu,%lu,%lu,%lu\r\n",
                 (unsigned long)fps[TACTILE_SENSOR_USART1][TACTILE_UNIT_UPPER],
                 (unsigned long)fps[TACTILE_SENSOR_USART1][TACTILE_UNIT_LOWER],
                 (unsigned long)fps[TACTILE_SENSOR_USART2][TACTILE_UNIT_UPPER],
                 (unsigned long)fps[TACTILE_SENSOR_USART2][TACTILE_UNIT_LOWER]);
  (void)DebugUartTransport_Write(line, 10U);

  /*
   * touchstat 每组含义：try,last_rx_len,last_protocol,last_hal_error。
   * 组顺序固定为 USART1上、USART1下、USART2上、USART2下，便于上位机解析。
   */
  (void)snprintf(line, sizeof(line),
                 "touchstat: %lu,%u,%d,%lu,%lu,%u,%d,%lu,%lu,%u,%d,%lu,%lu,%u,%d,%lu\r\n",
                 (unsigned long)stat[TACTILE_SENSOR_USART1][TACTILE_UNIT_UPPER].attempts,
                 stat[TACTILE_SENSOR_USART1][TACTILE_UNIT_UPPER].last_rx_len,
                 (int)stat[TACTILE_SENSOR_USART1][TACTILE_UNIT_UPPER].last_protocol,
                 (unsigned long)stat[TACTILE_SENSOR_USART1][TACTILE_UNIT_UPPER].last_hal_error,
                 (unsigned long)stat[TACTILE_SENSOR_USART1][TACTILE_UNIT_LOWER].attempts,
                 stat[TACTILE_SENSOR_USART1][TACTILE_UNIT_LOWER].last_rx_len,
                 (int)stat[TACTILE_SENSOR_USART1][TACTILE_UNIT_LOWER].last_protocol,
                 (unsigned long)stat[TACTILE_SENSOR_USART1][TACTILE_UNIT_LOWER].last_hal_error,
                 (unsigned long)stat[TACTILE_SENSOR_USART2][TACTILE_UNIT_UPPER].attempts,
                 stat[TACTILE_SENSOR_USART2][TACTILE_UNIT_UPPER].last_rx_len,
                 (int)stat[TACTILE_SENSOR_USART2][TACTILE_UNIT_UPPER].last_protocol,
                 (unsigned long)stat[TACTILE_SENSOR_USART2][TACTILE_UNIT_UPPER].last_hal_error,
                 (unsigned long)stat[TACTILE_SENSOR_USART2][TACTILE_UNIT_LOWER].attempts,
                 stat[TACTILE_SENSOR_USART2][TACTILE_UNIT_LOWER].last_rx_len,
                 (int)stat[TACTILE_SENSOR_USART2][TACTILE_UNIT_LOWER].last_protocol,
                 (unsigned long)stat[TACTILE_SENSOR_USART2][TACTILE_UNIT_LOWER].last_hal_error);
  (void)DebugUartTransport_Write(line, 10U);

  /*
   * touchcmd 每两字节是一组最近查询命令，组顺序同 touchstat。
   * 正常应为 6D,92,36,C9,6D,92,36,C9。
   */
  (void)snprintf(line, sizeof(line),
                 "touchcmd: %02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X\r\n",
                 stat[TACTILE_SENSOR_USART1][TACTILE_UNIT_UPPER].last_tx_preview[0],
                 stat[TACTILE_SENSOR_USART1][TACTILE_UNIT_UPPER].last_tx_preview[1],
                 stat[TACTILE_SENSOR_USART1][TACTILE_UNIT_LOWER].last_tx_preview[0],
                 stat[TACTILE_SENSOR_USART1][TACTILE_UNIT_LOWER].last_tx_preview[1],
                 stat[TACTILE_SENSOR_USART2][TACTILE_UNIT_UPPER].last_tx_preview[0],
                 stat[TACTILE_SENSOR_USART2][TACTILE_UNIT_UPPER].last_tx_preview[1],
                 stat[TACTILE_SENSOR_USART2][TACTILE_UNIT_LOWER].last_tx_preview[0],
                 stat[TACTILE_SENSOR_USART2][TACTILE_UNIT_LOWER].last_tx_preview[1]);
  (void)DebugUartTransport_Write(line, 10U);

  TactileService_PrintDiagPair("touchdiag1",
                               &stat[TACTILE_SENSOR_USART1][TACTILE_UNIT_UPPER],
                               &stat[TACTILE_SENSOR_USART1][TACTILE_UNIT_LOWER]);
  TactileService_PrintDiagPair("touchdiag2",
                               &stat[TACTILE_SENSOR_USART2][TACTILE_UNIT_UPPER],
                               &stat[TACTILE_SENSOR_USART2][TACTILE_UNIT_LOWER]);
}
#endif

static void TactileService_PrintUnitDiag(const char *name,
                                         const TactileUnitRuntimeStat_t *stat)
{
  char line[320];

  if ((name == NULL) || (stat == NULL))
  {
    return;
  }

  (void)snprintf(line, sizeof(line),
                 "tdiag: %s,q=%lu,rx=%lu,ok=%lu,to=%lu,txe=%lu,rxe=%lu,"
                 "len=%lu,hdr=%lu,chk=%lu,ch=%lu,ring=%lu,idle=%lu,"
                 "last=%lu/%lu/%lu/%u,cmd=%02X%02X,rx=%02X%02X%02X%02X%02X%02X%02X%02X\r\n",
                 name,
                 (unsigned long)stat->attempts,
                 (unsigned long)stat->rx_frame_count,
                 (unsigned long)stat->ok_count,
                 (unsigned long)stat->timeout_count,
                 (unsigned long)stat->tx_error_count,
                 (unsigned long)stat->rx_error_count,
                 (unsigned long)stat->bad_length_count,
                 (unsigned long)stat->bad_header_count,
                 (unsigned long)stat->checksum_error_count,
                 (unsigned long)stat->bad_channel_count,
                 (unsigned long)stat->ring_overrun_count,
                 (unsigned long)stat->idle_event_count,
                 (unsigned long)stat->last_result,
                 (unsigned long)stat->last_hal_status,
                 (unsigned long)stat->last_hal_error,
                 stat->last_rx_len,
                 stat->last_tx_preview[0],
                 stat->last_tx_preview[1],
                 stat->last_rx_preview[0],
                 stat->last_rx_preview[1],
                 stat->last_rx_preview[2],
                 stat->last_rx_preview[3],
                 stat->last_rx_preview[4],
                 stat->last_rx_preview[5],
                 stat->last_rx_preview[6],
                 stat->last_rx_preview[7]);
  (void)DebugUartTransport_Write(line, 10U);
}

static void TactileService_PrintDiagnostics(void)
{
  uint32_t fps[TACTILE_SENSOR_COUNT][TACTILE_UNIT_COUNT];
  TactileUnitRuntimeStat_t stat[TACTILE_SENSOR_COUNT][TACTILE_UNIT_COUNT];

  taskENTER_CRITICAL();
  (void)memcpy(fps, fpsWindowCounter, sizeof(fps));
  (void)memset(fpsWindowCounter, 0, sizeof(fpsWindowCounter));
  (void)memcpy(serviceStats.unit_fps, fps, sizeof(serviceStats.unit_fps));
  (void)memcpy(stat, runtimeStats, sizeof(stat));
  (void)memset(runtimeStats, 0, sizeof(runtimeStats));
  taskEXIT_CRITICAL();

  /*
   * q/rx/ok 分别表示发起查询、收到完整帧、协议解析成功次数。
   * last 字段依次为 transport_result/HAL_Status/HAL_Error/last_rx_len。
   */
  TactileService_PrintUnitDiag("u1u",
                               &stat[TACTILE_SENSOR_USART1][TACTILE_UNIT_UPPER]);
  TactileService_PrintUnitDiag("u1d",
                               &stat[TACTILE_SENSOR_USART1][TACTILE_UNIT_LOWER]);
  TactileService_PrintUnitDiag("u2u",
                               &stat[TACTILE_SENSOR_USART2][TACTILE_UNIT_UPPER]);
  TactileService_PrintUnitDiag("u2d",
                               &stat[TACTILE_SENSOR_USART2][TACTILE_UNIT_LOWER]);
}

static void TactileService_WaitNextPeriod(uint32_t *nextTick,
                                          uint32_t periodTicks)
{
  if (nextTick == NULL)
  {
    return;
  }

  *nextTick += periodTicks;
  if (osDelayUntil(*nextTick) != osOK)
  {
    /*
     * 周期任务偶发超时后不追赶补跑，直接重新对齐到下一个 25ms 周期。
     * 这样 USART2 查询节拍保持稳定，不会出现短时间内连续发命令的情况。
     */
    *nextTick = osKernelGetTickCount() + periodTicks;
    (void)osDelayUntil(*nextTick);
  }
}

static void TactileService_Task(void *argument)
{
  TactileSensorContext_t *context = (TactileSensorContext_t *)argument;
  uint32_t nextTick;
  const uint32_t periodTicks =
    TactileService_MsToTicks(TACTILE_SERVICE_PERIOD_MS);
  const uint32_t interUnitGapTicks =
    TactileService_MsToTicks(TACTILE_SERVICE_INTER_UNIT_GAP_MS);

  if (context == NULL)
  {
    for (;;)
    {
      (void)osDelay(1000U);
    }
  }

  TactileUartTransport_Init(&context->transport, context->huart);
  if (context->use_ring_dma)
  {
    /*
     * USART2 常驻 DMA 循环接收，后续每次查询只发命令并从软件 ring 中筛选完整帧。
     * 这样不会再产生逐字节 UART RX 中断，也能减少发命令与开接收之间的丢头风险。
     */
    (void)TactileUartTransport_StartCircularRx(&context->transport,
                                               context->dma_ring_buffer,
                                               context->dma_ring_length,
                                               context->sw_ring_buffer,
                                               context->sw_ring_length);
  }
  if (context->start_delay_ms != 0U)
  {
    (void)osDelay(TactileService_MsToTicks(context->start_delay_ms));
  }
  nextTick = osKernelGetTickCount();

  for (;;)
  {
    if (tactileServiceEnabled)
    {
      /*
       * 每个串口以 40Hz 轮询同一个传感器的上下两部分。
       * 上下半区之间留出短间隔，避免连续命令让传感器响应或 UART 状态残留互相影响。
       */
      (void)TactileService_ReadUnit(context, TACTILE_UNIT_UPPER);
      (void)osDelay(interUnitGapTicks);
      (void)TactileService_ReadUnit(context, TACTILE_UNIT_LOWER);
      TactileService_PublishSnapshot();
    }

    TactileService_WaitNextPeriod(&nextTick, periodTicks);
  }
}

static void TactileService_DebugTask(void *argument)
{
  uint32_t lastDiagMs = HAL_GetTick();
  uint32_t nowMs;

  (void)argument;

  for (;;)
  {
    nowMs = HAL_GetTick();
    if ((nowMs - lastDiagMs) >= TACTILE_SERVICE_FPS_PRINT_MS)
    {
      lastDiagMs = nowMs;
      TactileService_PrintDiagnostics();
    }

    /*
     * 调试输出独立低优先级运行，避免 RS485 阻塞写影响 USART2 的 25ms 采集节拍。
     */
    (void)osDelay(TactileService_MsToTicks(10U));
  }
}

void TactileService_CreateTask(void)
{
  char line[192];
  const osThreadAttr_t usart1TaskAttr = {
    .name = "touchU1",
    .cb_mem = &tactileUsart1TaskControlBlock,
    .cb_size = sizeof(tactileUsart1TaskControlBlock),
    .stack_mem = tactileUsart1TaskStack,
    .stack_size = sizeof(tactileUsart1TaskStack),
    .priority = osPriorityNormal
  };
  const osThreadAttr_t usart2TaskAttr = {
    .name = "touchU2",
    .cb_mem = &tactileUsart2TaskControlBlock,
    .cb_size = sizeof(tactileUsart2TaskControlBlock),
    .stack_mem = tactileUsart2TaskStack,
    .stack_size = sizeof(tactileUsart2TaskStack),
    .priority = osPriorityAboveNormal
  };
  const osThreadAttr_t debugTaskAttr = {
    .name = "touchDbg",
    .cb_mem = &tactileDebugTaskControlBlock,
    .cb_size = sizeof(tactileDebugTaskControlBlock),
    .stack_mem = tactileDebugTaskStack,
    .stack_size = sizeof(tactileDebugTaskStack),
    .priority = osPriorityLow
  };

  TactileData_Init();
  (void)memset(&serviceSnapshot, 0, sizeof(serviceSnapshot));
  (void)memset(&serviceStats, 0, sizeof(serviceStats));
  (void)memset(fpsWindowCounter, 0, sizeof(fpsWindowCounter));
  (void)memset(runtimeStats, 0, sizeof(runtimeStats));

  sensorContexts[TACTILE_SENSOR_USART1].task_handle =
    osThreadNew(TactileService_Task,
                &sensorContexts[TACTILE_SENSOR_USART1],
                &usart1TaskAttr);
  sensorContexts[TACTILE_SENSOR_USART2].task_handle =
    osThreadNew(TactileService_Task,
                &sensorContexts[TACTILE_SENSOR_USART2],
                &usart2TaskAttr);
  tactileDebugTaskHandle = osThreadNew(TactileService_DebugTask,
                                       NULL,
                                       &debugTaskAttr);

  if ((sensorContexts[TACTILE_SENSOR_USART1].task_handle == NULL) ||
      (sensorContexts[TACTILE_SENSOR_USART2].task_handle == NULL) ||
      (tactileDebugTaskHandle == NULL))
  {
    (void)snprintf(line, sizeof(line),
                   "touchboot: create_failed,u1=%u,u2=%u,dbg=%u,heap=%lu\r\n",
                   (sensorContexts[TACTILE_SENSOR_USART1].task_handle != NULL) ? 1U : 0U,
                   (sensorContexts[TACTILE_SENSOR_USART2].task_handle != NULL) ? 1U : 0U,
                   (tactileDebugTaskHandle != NULL) ? 1U : 0U,
                   (unsigned long)xPortGetFreeHeapSize());
    (void)DebugUartTransport_Write(line, 10U);
  }
  else
  {
    (void)snprintf(line, sizeof(line),
                   "touchboot: ok,u1=%s,u2=%s,rate=%u,gap=%u,retry=%u,u2try=%u,u1stk=%lu,u2stk=%lu,dbgstk=%lu,heap=%lu\r\n",
                   TACTILE_SERVICE_UART1_BOOT_MODE,
                   TACTILE_SERVICE_UART2_BOOT_MODE,
                   TACTILE_SERVICE_SAMPLE_RATE_HZ,
                   TACTILE_SERVICE_INTER_UNIT_GAP_MS,
                   TACTILE_SERVICE_MAX_READ_ATTEMPTS,
                   TACTILE_SERVICE_RING_DMA_READ_ATTEMPTS,
                   (unsigned long)sizeof(tactileUsart1TaskStack),
                   (unsigned long)sizeof(tactileUsart2TaskStack),
                   (unsigned long)sizeof(tactileDebugTaskStack),
                   (unsigned long)xPortGetFreeHeapSize());
    (void)DebugUartTransport_Write(line, 10U);
  }
}

bool TactileService_GetSnapshot(TactileSnapshot_t *snapshot)
{
  return TactileData_GetSnapshot(snapshot, HAL_GetTick());
}

void TactileService_GetStats(TactileStats_t *stats)
{
  if (stats == NULL)
  {
    return;
  }

  taskENTER_CRITICAL();
  *stats = serviceStats;
  taskEXIT_CRITICAL();
}

void TactileService_SetEnabled(bool enabled)
{
  tactileServiceEnabled = enabled;
}

void TactileService_USART2IdleIRQHandler(void)
{
#if TACTILE_USART2_RING_DMA_ENABLE
  TactileUartTransport_HandleIdleIrq(
    &sensorContexts[TACTILE_SENSOR_USART2].transport);
#endif
}
