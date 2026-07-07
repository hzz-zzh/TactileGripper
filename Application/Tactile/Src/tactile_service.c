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

#define TACTILE_SERVICE_UNIT_RATE_HZ         10U
#define TACTILE_SERVICE_SLOT_RATE_HZ         (TACTILE_SERVICE_UNIT_RATE_HZ * TACTILE_UNITS_PER_SENSOR)
#define TACTILE_SERVICE_PERIOD_MS            (1000U / TACTILE_SERVICE_SLOT_RATE_HZ)
#define TACTILE_SERVICE_UNIT_TIMEOUT_MS      20U
#define TACTILE_SERVICE_MAX_READ_ATTEMPTS    2U
#define TACTILE_SERVICE_RETRY_DELAY_MS       2U
#define TACTILE_SERVICE_RX_BUFFER_LENGTH     128U
#define TACTILE_SERVICE_DEBUG_PRINT_MS       100U
#define TACTILE_SERVICE_FPS_PRINT_MS         1000U
#define TACTILE_SERVICE_TASK_STACK_WORDS     512U
#define TACTILE_SERVICE_DIAG_ENABLE          0U
#define TACTILE_SERVICE_DIAG_PRINT_MS        1000U
/* 当前优先单独验证右侧 USART1 链路，左侧 USART2 暂时关闭以减少干扰。 */
#define TACTILE_SERVICE_ENABLE_RIGHT_SENSOR  1U
#define TACTILE_SERVICE_ENABLE_LEFT_SENSOR   1U

#define TACTILE_DMA_RIGHT_RX_ADDRESS         0x24000100UL
#define TACTILE_DMA_LEFT_RX_ADDRESS          0x24000200UL

typedef struct
{
  const char *name;
  uint8_t sensor_index;
  UART_HandleTypeDef *huart;
  TactileUartTransport_t transport;
  TactileSensorData_t sensor_data;
  uint8_t first_unit;
  uint32_t last_diag_ms[TACTILE_UNITS_PER_SENSOR];
  osThreadId_t task_handle;
} TactileSensorContext_t;

typedef struct
{
  uint32_t attempts;
  uint16_t last_rx_len;
  uint32_t last_hal_error;
  TactileProtocolResult_t last_protocol;
} TactileUnitRuntimeStat_t;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

#if defined(__CC_ARM)
__attribute__((at(TACTILE_DMA_RIGHT_RX_ADDRESS), aligned(32)))
static uint8_t tactileRightRxBuffer[TACTILE_SERVICE_RX_BUFFER_LENGTH];
__attribute__((at(TACTILE_DMA_LEFT_RX_ADDRESS), aligned(32)))
static uint8_t tactileLeftRxBuffer[TACTILE_SERVICE_RX_BUFFER_LENGTH];
#elif defined(__ARMCC_VERSION)
__attribute__((section(".ARM.__at_0x24000100"), aligned(32)))
static uint8_t tactileRightRxBuffer[TACTILE_SERVICE_RX_BUFFER_LENGTH];
__attribute__((section(".ARM.__at_0x24000200"), aligned(32)))
static uint8_t tactileLeftRxBuffer[TACTILE_SERVICE_RX_BUFFER_LENGTH];
#else
__attribute__((section(".dma_buffer"), aligned(32)))
static uint8_t tactileRightRxBuffer[TACTILE_SERVICE_RX_BUFFER_LENGTH];
__attribute__((section(".dma_buffer"), aligned(32)))
static uint8_t tactileLeftRxBuffer[TACTILE_SERVICE_RX_BUFFER_LENGTH];
#endif

static TactileSensorContext_t rightContext = {
  .name = "R",
  .sensor_index = TACTILE_RIGHT_SENSOR_INDEX,
  .huart = &huart1
};
static TactileSensorContext_t leftContext = {
  .name = "L",
  .sensor_index = TACTILE_LEFT_SENSOR_INDEX,
  .huart = &huart2
};

static volatile bool tactileServiceEnabled = true;
static TactileSnapshot_t serviceSnapshot;
static TactileStats_t serviceStats;
static uint32_t fpsWindowCounter[TACTILE_SENSOR_COUNT][TACTILE_UNITS_PER_SENSOR];
static TactileUnitRuntimeStat_t runtimeStats[TACTILE_SENSOR_COUNT][TACTILE_UNITS_PER_SENSOR];

static uint32_t TactileService_MsToTicks(uint32_t milliseconds)
{
  return ((milliseconds * osKernelGetTickFreq()) + 999U) / 1000U;
}

static TactileUnitData_t *TactileService_GetUnit(TactileSensorData_t *sensor,
                                                 uint8_t unitIndex)
{
  return (unitIndex == TACTILE_UNIT_UPPER) ? &sensor->upper : &sensor->lower;
}

static const TactileUnitData_t *TactileService_GetConstUnit(
  const TactileSensorData_t *sensor,
  uint8_t unitIndex)
{
  return (unitIndex == TACTILE_UNIT_UPPER) ? &sensor->upper : &sensor->lower;
}

static uint8_t TactileService_UnitAddress(uint8_t unitIndex)
{
  return (unitIndex == TACTILE_UNIT_UPPER) ?
         TACTILE_UPPER_ADDRESS : TACTILE_LOWER_ADDRESS;
}

static bool TactileService_IsLoggerContext(const TactileSensorContext_t *context)
{
#if (TACTILE_SERVICE_ENABLE_RIGHT_SENSOR != 0U)
  return context->sensor_index == TACTILE_RIGHT_SENSOR_INDEX;
#elif (TACTILE_SERVICE_ENABLE_LEFT_SENSOR != 0U)
  return context->sensor_index == TACTILE_LEFT_SENSOR_INDEX;
#else
  (void)context;
  return false;
#endif
}

static void TactileService_UpdateRuntimeStat(
  TactileSensorContext_t *context,
  uint8_t unitIndex,
  TactileProtocolResult_t protocolResult)
{
  TactileUartTransportStats_t transportStats;
  TactileUnitRuntimeStat_t *stat;

  TactileUartTransport_GetStats(&context->transport, &transportStats);

  taskENTER_CRITICAL();
  stat = &runtimeStats[context->sensor_index][unitIndex];
  stat->attempts++;
  stat->last_rx_len = transportStats.last_rx_len;
  stat->last_hal_error = transportStats.last_hal_error;
  stat->last_protocol = protocolResult;
  taskEXIT_CRITICAL();
}

static void TactileService_RecordError(TactileSensorContext_t *context,
                                       uint8_t unitIndex,
                                       bool transportOk,
                                       TactileProtocolResult_t protocolResult)
{
  TactileUnitData_t *unit = TactileService_GetUnit(&context->sensor_data,
                                                   unitIndex);

  unit->address = TactileService_UnitAddress(unitIndex);
  unit->sensor_index = context->sensor_index;
  unit->unit_index = unitIndex;
  unit->timestamp_ms = HAL_GetTick();
  unit->valid = false;

  taskENTER_CRITICAL();
  if (!transportOk)
  {
    unit->timeout_count++;
    context->sensor_data.timeout_count++;
    serviceStats.timeout_count++;
  }
  else if (protocolResult == TACTILE_PROTOCOL_BAD_CHECKSUM)
  {
    unit->checksum_error_count++;
    context->sensor_data.checksum_error_count++;
    serviceStats.checksum_error_count++;
  }
  else
  {
    unit->frame_error_count++;
    context->sensor_data.frame_error_count++;
    serviceStats.frame_error_count++;
  }
  taskEXIT_CRITICAL();
}

static void TactileService_PrintDiag(TactileSensorContext_t *context,
                                     uint8_t unitIndex,
                                     uint8_t address,
                                     TactileProtocolResult_t protocolResult)
{
#if TACTILE_SERVICE_DIAG_ENABLE
  TactileUartTransportStats_t transportStats;
  uint32_t nowMs = HAL_GetTick();
  char line[220];

  if ((nowMs - context->last_diag_ms[unitIndex]) <
      TACTILE_SERVICE_DIAG_PRINT_MS)
  {
    return;
  }
  context->last_diag_ms[unitIndex] = nowMs;

  TactileUartTransport_GetStats(&context->transport, &transportStats);
  (void)snprintf(line, sizeof(line),
                 "touchdiag: %s,%c,%02X,%lu,%lu,%u,%d,"
                 "%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,"
                 "%02X,%02X,%02X,%02X\r\n",
                 context->name,
                 (unitIndex == TACTILE_UNIT_UPPER) ? 'U' : 'L',
                 address,
                 (unsigned long)transportStats.last_hal_status,
                 (unsigned long)transportStats.last_hal_error,
                 transportStats.last_rx_len,
                 (int)protocolResult,
                 transportStats.last_rx_preview[0],
                 transportStats.last_rx_preview[1],
                 transportStats.last_rx_preview[2],
                 transportStats.last_rx_preview[3],
                 transportStats.last_rx_preview[4],
                 transportStats.last_rx_preview[5],
                 transportStats.last_rx_preview[6],
                 transportStats.last_rx_preview[7],
                 transportStats.last_rx_preview[8],
                 transportStats.last_rx_preview[9],
                 transportStats.last_rx_preview[10],
                 transportStats.last_rx_preview[11]);
  (void)DebugUartTransport_Write(line, 10U);
#else
  (void)context;
  (void)unitIndex;
  (void)address;
  (void)protocolResult;
#endif
}

static bool TactileService_ReadUnit(TactileSensorContext_t *context,
                                    uint8_t unitIndex,
                                    uint8_t *rxBuffer)
{
  uint8_t command[2];
  uint16_t rxLength = 0U;
  uint8_t address = TactileService_UnitAddress(unitIndex);
  TactileProtocolResult_t protocolResult = TACTILE_PROTOCOL_BAD_LENGTH;
  TactileUnitData_t decoded;
  TactileUnitData_t *stored;
  bool transportOk = false;
  uint8_t attempt;

  TactileProtocol_BuildReadCommand(address, command);
  for (attempt = 0U; attempt < TACTILE_SERVICE_MAX_READ_ATTEMPTS; attempt++)
  {
    if (attempt != 0U)
    {
      (void)osDelay(TactileService_MsToTicks(TACTILE_SERVICE_RETRY_DELAY_MS));
    }

    protocolResult = TACTILE_PROTOCOL_BAD_LENGTH;
    rxLength = 0U;
    transportOk = TactileUartTransport_WriteRead(
      &context->transport,
      command, sizeof(command),
      rxBuffer,
      TACTILE_SERVICE_RX_BUFFER_LENGTH,
      TACTILE_FRAME_LENGTH,
      &rxLength,
      TACTILE_SERVICE_UNIT_TIMEOUT_MS);

    if (transportOk)
    {
      stored = TactileService_GetUnit(&context->sensor_data, unitIndex);
      decoded = *stored;
      protocolResult = TactileProtocol_DecodeFrame(address, rxBuffer, rxLength,
                                                  &decoded);
      if (protocolResult == TACTILE_PROTOCOL_OK)
      {
        TactileService_UpdateRuntimeStat(context, unitIndex, protocolResult);
        decoded.address = address;
        decoded.sensor_index = context->sensor_index;
        decoded.unit_index = unitIndex;
        decoded.timestamp_ms = HAL_GetTick();
        decoded.valid = true;
        *stored = decoded;

        taskENTER_CRITICAL();
        fpsWindowCounter[context->sensor_index][unitIndex]++;
        taskEXIT_CRITICAL();
        return true;
      }
    }
  }

  TactileService_UpdateRuntimeStat(context, unitIndex, protocolResult);
  TactileService_RecordError(context, unitIndex, transportOk, protocolResult);
  TactileService_PrintDiag(context, unitIndex, address, protocolResult);

  return false;
}

static uint8_t TactileService_UpdateSensorMask(TactileSensorContext_t *context)
{
  uint8_t mask = 0U;

  if (context->sensor_data.upper.valid)
  {
    mask |= 0x01U;
  }
  if (context->sensor_data.lower.valid)
  {
    mask |= 0x02U;
  }
  context->sensor_data.valid_mask = mask;
  context->sensor_data.complete = (mask == 0x03U);
  return mask;
}

static void TactileService_PublishSensor(TactileSensorContext_t *context)
{
  TactileSnapshot_t localSnapshot;
  uint8_t globalMask = 0U;

  TactileService_UpdateSensorMask(context);

  taskENTER_CRITICAL();
  if (context->sensor_index == TACTILE_RIGHT_SENSOR_INDEX)
  {
    serviceSnapshot.right = context->sensor_data;
  }
  else
  {
    serviceSnapshot.left = context->sensor_data;
  }

  if (serviceSnapshot.right.upper.valid)
  {
    globalMask |= TACTILE_MASK_RIGHT_UPPER;
  }
  if (serviceSnapshot.right.lower.valid)
  {
    globalMask |= TACTILE_MASK_RIGHT_LOWER;
  }
  if (serviceSnapshot.left.upper.valid)
  {
    globalMask |= TACTILE_MASK_LEFT_UPPER;
  }
  if (serviceSnapshot.left.lower.valid)
  {
    globalMask |= TACTILE_MASK_LEFT_LOWER;
  }

  serviceSnapshot.sample_id++;
  serviceSnapshot.timestamp_ms = HAL_GetTick();
  serviceSnapshot.valid_mask = globalMask;
  serviceSnapshot.complete = (globalMask == TACTILE_MASK_ALL);

  serviceStats.sample_count++;
  serviceStats.last_valid_mask = globalMask;
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

static int16_t TactileService_ForceOrZero(const TactileUnitData_t *unit,
                                          bool tangent)
{
  if ((unit == NULL) || (!unit->valid))
  {
    return 0;
  }
  return tangent ? unit->tangent_force : unit->normal_force;
}

static void TactileService_PrintSummary(void)
{
  TactileSnapshot_t snapshot;
  const TactileUnitData_t *ru;
  const TactileUnitData_t *rl;
  const TactileUnitData_t *lu;
  const TactileUnitData_t *ll;
  char line[160];

  if (!TactileData_GetSnapshot(&snapshot, HAL_GetTick()))
  {
    return;
  }

  ru = TactileService_GetConstUnit(&snapshot.right, TACTILE_UNIT_UPPER);
  rl = TactileService_GetConstUnit(&snapshot.right, TACTILE_UNIT_LOWER);
  lu = TactileService_GetConstUnit(&snapshot.left, TACTILE_UNIT_UPPER);
  ll = TactileService_GetConstUnit(&snapshot.left, TACTILE_UNIT_LOWER);

  (void)snprintf(line, sizeof(line),
                 "touch: %lu,%u,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
                 (unsigned long)snapshot.sample_id,
                 snapshot.valid_mask,
                 TactileService_ForceOrZero(ru, false),
                 TactileService_ForceOrZero(ru, true),
                 TactileService_ForceOrZero(rl, false),
                 TactileService_ForceOrZero(rl, true),
                 TactileService_ForceOrZero(lu, false),
                 TactileService_ForceOrZero(lu, true),
                 TactileService_ForceOrZero(ll, false),
                 TactileService_ForceOrZero(ll, true));
  (void)DebugUartTransport_Write(line, 10U);
}

static void TactileService_PrintFrameRate(void)
{
  uint32_t fps[TACTILE_SENSOR_COUNT][TACTILE_UNITS_PER_SENSOR];
  TactileUnitRuntimeStat_t stat[TACTILE_SENSOR_COUNT][TACTILE_UNITS_PER_SENSOR];
  char line[220];

  taskENTER_CRITICAL();
  (void)memcpy(fps, fpsWindowCounter, sizeof(fps));
  (void)memset(fpsWindowCounter, 0, sizeof(fpsWindowCounter));
  (void)memcpy(serviceStats.unit_fps, fps, sizeof(serviceStats.unit_fps));
  (void)memcpy(stat, runtimeStats, sizeof(stat));
  (void)memset(runtimeStats, 0, sizeof(runtimeStats));
  taskEXIT_CRITICAL();

  (void)snprintf(line, sizeof(line),
                 "touchfps: %lu,%lu,%lu,%lu\r\n",
                 (unsigned long)fps[TACTILE_RIGHT_SENSOR_INDEX][TACTILE_UNIT_UPPER],
                 (unsigned long)fps[TACTILE_RIGHT_SENSOR_INDEX][TACTILE_UNIT_LOWER],
                 (unsigned long)fps[TACTILE_LEFT_SENSOR_INDEX][TACTILE_UNIT_UPPER],
                 (unsigned long)fps[TACTILE_LEFT_SENSOR_INDEX][TACTILE_UNIT_LOWER]);
  (void)DebugUartTransport_Write(line, 10U);

  /*
   * touchstat 每组含义为：try,last_rx_len,last_protocol,last_hal_error。
   * last_protocol: 0=OK, 2=长度不足, 3=帧头不匹配, 4=校验错, 5=通道数错。
   */
  (void)snprintf(line, sizeof(line),
                 "touchstat: %lu,%u,%d,%lu,"
                 "%lu,%u,%d,%lu,"
                 "%lu,%u,%d,%lu,"
                 "%lu,%u,%d,%lu\r\n",
                 (unsigned long)stat[TACTILE_RIGHT_SENSOR_INDEX][TACTILE_UNIT_UPPER].attempts,
                 stat[TACTILE_RIGHT_SENSOR_INDEX][TACTILE_UNIT_UPPER].last_rx_len,
                 (int)stat[TACTILE_RIGHT_SENSOR_INDEX][TACTILE_UNIT_UPPER].last_protocol,
                 (unsigned long)stat[TACTILE_RIGHT_SENSOR_INDEX][TACTILE_UNIT_UPPER].last_hal_error,
                 (unsigned long)stat[TACTILE_RIGHT_SENSOR_INDEX][TACTILE_UNIT_LOWER].attempts,
                 stat[TACTILE_RIGHT_SENSOR_INDEX][TACTILE_UNIT_LOWER].last_rx_len,
                 (int)stat[TACTILE_RIGHT_SENSOR_INDEX][TACTILE_UNIT_LOWER].last_protocol,
                 (unsigned long)stat[TACTILE_RIGHT_SENSOR_INDEX][TACTILE_UNIT_LOWER].last_hal_error,
                 (unsigned long)stat[TACTILE_LEFT_SENSOR_INDEX][TACTILE_UNIT_UPPER].attempts,
                 stat[TACTILE_LEFT_SENSOR_INDEX][TACTILE_UNIT_UPPER].last_rx_len,
                 (int)stat[TACTILE_LEFT_SENSOR_INDEX][TACTILE_UNIT_UPPER].last_protocol,
                 (unsigned long)stat[TACTILE_LEFT_SENSOR_INDEX][TACTILE_UNIT_UPPER].last_hal_error,
                 (unsigned long)stat[TACTILE_LEFT_SENSOR_INDEX][TACTILE_UNIT_LOWER].attempts,
                 stat[TACTILE_LEFT_SENSOR_INDEX][TACTILE_UNIT_LOWER].last_rx_len,
                 (int)stat[TACTILE_LEFT_SENSOR_INDEX][TACTILE_UNIT_LOWER].last_protocol,
                 (unsigned long)stat[TACTILE_LEFT_SENSOR_INDEX][TACTILE_UNIT_LOWER].last_hal_error);
  (void)DebugUartTransport_Write(line, 10U);
}

static void TactileService_Task(void *argument)
{
  TactileSensorContext_t *context = (TactileSensorContext_t *)argument;
  uint8_t *rxBuffer = (context->sensor_index == TACTILE_RIGHT_SENSOR_INDEX) ?
                      tactileRightRxBuffer : tactileLeftRxBuffer;
  uint32_t nextTick = osKernelGetTickCount();
  const uint32_t periodTicks =
    TactileService_MsToTicks(TACTILE_SERVICE_PERIOD_MS);
  uint32_t lastSummaryMs = HAL_GetTick();
  uint32_t lastFpsMs = HAL_GetTick();

  TactileUartTransport_Init(&context->transport, context->huart);

  for (;;)
  {
    if (tactileServiceEnabled)
    {
      uint8_t unitIndex = context->first_unit;

      /*
       * 同一片传感器的上下单元共用一个 UART。
       * 每 10ms 只读取一个地址，上下交替，单元采样率仍为 50Hz，
       * 同时避免连续命令导致第二帧短帧或错帧。
       */
      (void)TactileService_ReadUnit(context, unitIndex, rxBuffer);
      context->first_unit =
        (uint8_t)((context->first_unit + 1U) % TACTILE_UNITS_PER_SENSOR);
      TactileService_PublishSensor(context);
    }

    /*
     * 日志只由右侧任务打印，避免两条采集任务同时抢 RS485。
     * 输出频率低于采集频率，保证触觉 50Hz 轮询优先。
     */
    if (TactileService_IsLoggerContext(context))
    {
      uint32_t nowMs = HAL_GetTick();
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
#if (TACTILE_SERVICE_ENABLE_RIGHT_SENSOR != 0U)
  const osThreadAttr_t rightTaskAttr = {
    .name = "touchR",
    .stack_size = TACTILE_SERVICE_TASK_STACK_WORDS * 4U,
    .priority = osPriorityNormal
  };
#endif
#if (TACTILE_SERVICE_ENABLE_LEFT_SENSOR != 0U)
  const osThreadAttr_t leftTaskAttr = {
    .name = "touchL",
    .stack_size = TACTILE_SERVICE_TASK_STACK_WORDS * 4U,
    .priority = osPriorityNormal
  };
#endif
  bool createFailed = false;

  TactileData_Init();
  (void)memset(&serviceSnapshot, 0, sizeof(serviceSnapshot));
  (void)memset(&serviceStats, 0, sizeof(serviceStats));
  (void)memset(&rightContext.sensor_data, 0, sizeof(rightContext.sensor_data));
  (void)memset(&leftContext.sensor_data, 0, sizeof(leftContext.sensor_data));
  (void)memset(rightContext.last_diag_ms, 0, sizeof(rightContext.last_diag_ms));
  (void)memset(leftContext.last_diag_ms, 0, sizeof(leftContext.last_diag_ms));
  (void)memset(fpsWindowCounter, 0, sizeof(fpsWindowCounter));
  (void)memset(runtimeStats, 0, sizeof(runtimeStats));

#if (TACTILE_SERVICE_ENABLE_RIGHT_SENSOR != 0U)
  rightContext.task_handle =
    osThreadNew(TactileService_Task, &rightContext, &rightTaskAttr);
#else
  rightContext.task_handle = NULL;
#endif

#if (TACTILE_SERVICE_ENABLE_LEFT_SENSOR != 0U)
  leftContext.task_handle =
    osThreadNew(TactileService_Task, &leftContext, &leftTaskAttr);
#else
  leftContext.task_handle = NULL;
#endif

#if (TACTILE_SERVICE_ENABLE_RIGHT_SENSOR != 0U)
  if (rightContext.task_handle == NULL)
  {
    createFailed = true;
  }
#endif
#if (TACTILE_SERVICE_ENABLE_LEFT_SENSOR != 0U)
  if (leftContext.task_handle == NULL)
  {
    createFailed = true;
  }
#endif

  if (createFailed)
  {
    (void)DebugUartTransport_Write("touchboot: create_failed\r\n", 10U);
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
