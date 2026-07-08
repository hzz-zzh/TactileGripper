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

#define TACTILE_SERVICE_SAMPLE_RATE_HZ       50U
#define TACTILE_SERVICE_PERIOD_MS            (1000U / TACTILE_SERVICE_SAMPLE_RATE_HZ)
#define TACTILE_SERVICE_UNIT_TIMEOUT_MS      10U
#define TACTILE_SERVICE_RX_BUFFER_LENGTH     128U
#define TACTILE_SERVICE_DEBUG_PRINT_MS       100U
#define TACTILE_SERVICE_FPS_PRINT_MS         1000U
#define TACTILE_SERVICE_ACQ_STACK_WORDS      384U
#define TACTILE_SERVICE_LOG_STACK_WORDS      768U
#define TACTILE_USART1_DMA_RX_ADDRESS        0x2404FE00UL
#define TACTILE_USART2_DMA_RX_ADDRESS        0x2404FF00UL
#if TACTILE_SENSOR_ONLY_MODE
#define TACTILE_SERVICE_USE_DMA              false
#define TACTILE_SERVICE_BOOT_MODE            "no_dma"
#else
#define TACTILE_SERVICE_USE_DMA              true
#define TACTILE_SERVICE_BOOT_MODE            "dma"
#endif

typedef struct
{
  uint32_t attempts;
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
  osThreadId_t task_handle;
  bool use_dma;
  bool print_debug;
} TactileSensorContext_t;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

#if TACTILE_SENSOR_ONLY_MODE
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

static volatile bool tactileServiceEnabled = true;
static TactileSnapshot_t serviceSnapshot;
static TactileStats_t serviceStats;
static uint32_t fpsWindowCounter[TACTILE_SENSOR_COUNT][TACTILE_UNIT_COUNT];
static TactileUnitRuntimeStat_t runtimeStats[TACTILE_SENSOR_COUNT][TACTILE_UNIT_COUNT];
static StaticTask_t tactileUsart1TaskControlBlock;
static StaticTask_t tactileUsart2TaskControlBlock;
static StackType_t tactileUsart1TaskStack[TACTILE_SERVICE_ACQ_STACK_WORDS];
static StackType_t tactileUsart2TaskStack[TACTILE_SERVICE_LOG_STACK_WORDS];

static TactileSensorContext_t sensorContexts[TACTILE_SENSOR_COUNT] = {
  {
    .name = "usart1",
    .sensor_index = TACTILE_SENSOR_USART1,
    .huart = &huart1,
    .rx_buffer = tactileUsart1RxBuffer,
    .task_handle = NULL,
    .use_dma = TACTILE_SERVICE_USE_DMA,
    .print_debug = false
  },
  {
    .name = "usart2",
    .sensor_index = TACTILE_SENSOR_USART2,
    .huart = &huart2,
    .rx_buffer = tactileUsart2RxBuffer,
    .task_handle = NULL,
    .use_dma = TACTILE_SERVICE_USE_DMA,
    .print_debug = true
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

static void TactileService_UpdateRuntimeStat(
  TactileSensorContext_t *context,
  uint8_t unitIndex,
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
  stat->last_hal_status = transportStats.last_hal_status;
  stat->last_rx_len = transportStats.last_rx_len;
  stat->last_tx_len = transportStats.last_tx_len;
  stat->last_hal_error = transportStats.last_hal_error;
  stat->last_protocol = protocolResult;
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
  bool transportOk;

  if ((context == NULL) || (unitIndex >= TACTILE_UNIT_COUNT))
  {
    return false;
  }

  sensorIndex = context->sensor_index;
  address = TactileService_UnitAddress(unitIndex);
  TactileProtocol_BuildReadCommand(address, command);

  if (context->use_dma)
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

      TactileService_UpdateRuntimeStat(context, unitIndex, protocolResult);
      return true;
    }
  }

  TactileService_UpdateRuntimeStat(context, unitIndex, protocolResult);
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

static void TactileService_Task(void *argument)
{
  TactileSensorContext_t *context = (TactileSensorContext_t *)argument;
  uint32_t nextTick = osKernelGetTickCount();
  const uint32_t periodTicks =
    TactileService_MsToTicks(TACTILE_SERVICE_PERIOD_MS);
  uint32_t lastSummaryMs = HAL_GetTick();
  uint32_t lastFpsMs = HAL_GetTick();
  uint32_t nowMs;

  if (context == NULL)
  {
    for (;;)
    {
      (void)osDelay(1000U);
    }
  }

  TactileUartTransport_Init(&context->transport, context->huart);

  for (;;)
  {
    if (tactileServiceEnabled)
    {
      /*
       * 每个串口独立以 50Hz 轮询同一个传感器的上下两部分。
       * 上层只消费统一快照，不需要关心数据来自 USART1 还是 USART2。
       */
      (void)TactileService_ReadUnit(context, TACTILE_UNIT_UPPER);
      (void)TactileService_ReadUnit(context, TACTILE_UNIT_LOWER);
      TactileService_PublishSnapshot();
    }

    if (context->print_debug)
    {
      nowMs = HAL_GetTick();
      if ((nowMs - lastSummaryMs) >= TACTILE_SERVICE_DEBUG_PRINT_MS)
      {
        lastSummaryMs = nowMs;
        TactileService_PrintSummary();
      }
      if ((nowMs - lastFpsMs) >= TACTILE_SERVICE_FPS_PRINT_MS)
      {
        lastFpsMs = nowMs;
        TactileService_PrintFrameRate();
      }
    }

    nextTick += periodTicks;
    (void)osDelayUntil(nextTick);
  }
}

void TactileService_CreateTask(void)
{
  char line[96];
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
    .priority = osPriorityNormal
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

  if ((sensorContexts[TACTILE_SENSOR_USART1].task_handle == NULL) ||
      (sensorContexts[TACTILE_SENSOR_USART2].task_handle == NULL))
  {
    (void)snprintf(line, sizeof(line),
                   "touchboot: create_failed,u1=%u,u2=%u,heap=%lu\r\n",
                   (sensorContexts[TACTILE_SENSOR_USART1].task_handle != NULL) ? 1U : 0U,
                   (sensorContexts[TACTILE_SENSOR_USART2].task_handle != NULL) ? 1U : 0U,
                   (unsigned long)xPortGetFreeHeapSize());
    (void)DebugUartTransport_Write(line, 10U);
  }
  else
  {
    (void)snprintf(line, sizeof(line),
                   "touchboot: ok,u1=%s,u2=%s,u1stk=%lu,u2stk=%lu,heap=%lu\r\n",
                   TACTILE_SERVICE_BOOT_MODE,
                   TACTILE_SERVICE_BOOT_MODE,
                   (unsigned long)sizeof(tactileUsart1TaskStack),
                   (unsigned long)sizeof(tactileUsart2TaskStack),
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
