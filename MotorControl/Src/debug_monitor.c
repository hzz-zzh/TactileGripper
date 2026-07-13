#include "debug_monitor.h"

#include "cmsis_os2.h"
#include "main.h"
#include "gripper_config.h"
#include "gripper_service.h"
#include "kth7812_speed_pos_fdbk.h"
#include "mc_api.h"
#include "mc_config.h"
#include "mc_config_common.h"
#include "parameters_conversion.h"
#include "debug_uart_transport.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define DEBUG_POLL_TICKS       20U  /* 10 ms at the 2 kHz RTOS tick: 100 Hz. */
#define DEBUG_IDLE_POLL_TICKS  100U /* 非运行状态降为20 Hz，仍可捕获短时状态变化。 */
#define DEBUG_HOMING_PRINT_MS  100U
#define DEBUG_HOMING_FAST_STALL_MS 200U
#define DEBUG_HOMING_FAST_PRINT_MS 20U
#define DEBUG_TX_TIMEOUT_MS    10U
#define ADC_CAL_SAMPLES        2048U

/* RS485用于连续触觉二进制流时，关闭homing和position周期文本输出。 */
#define DEBUG_RUNTIME_OUTPUT_ENABLE 0U

extern UART_HandleTypeDef huart10;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern TIM_HandleTypeDef htim1;

#if DEBUG_RUNTIME_OUTPUT_ENABLE
static osThreadId_t debugTaskHandle;
#endif
static uint16_t busCurrentZeroRaw = 32768U;

typedef struct
{
  uint32_t sum;
  uint16_t minimum;
  uint16_t maximum;
  uint16_t count;
} AdcCalibrationStats_t;

static void DebugMonitor_Write(const char *text)
{
  (void)DebugUartTransport_Write(text, DEBUG_TX_TIMEOUT_MS);
}

static bool DebugMonitor_ReadAdc(ADC_HandleTypeDef *hadc, uint32_t channel,
                                 uint16_t *value)
{
  ADC_ChannelConfTypeDef config = {0};
  config.Channel = channel;
  config.Rank = ADC_REGULAR_RANK_1;
  config.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;
  config.SingleDiff = ADC_SINGLE_ENDED;
  config.OffsetNumber = ADC_OFFSET_NONE;
  config.Offset = 0;

  if ((HAL_ADC_ConfigChannel(hadc, &config) != HAL_OK) ||
      (HAL_ADC_Start(hadc) != HAL_OK) ||
      (HAL_ADC_PollForConversion(hadc, 2U) != HAL_OK))
  {
    (void)HAL_ADC_Stop(hadc);
    return false;
  }

  *value = (uint16_t)HAL_ADC_GetValue(hadc);
  (void)HAL_ADC_Stop(hadc);
  return true;
}

static bool DebugMonitor_ConfigureSynchronousRegularAdc(ADC_HandleTypeDef *hadc,
                                                        uint32_t channel)
{
  ADC_ChannelConfTypeDef config = {0};

  /* 电流由TIM1 OC4REF产生的TRGO触发，固定在PWM周期的安静采样点。 */
  (void)HAL_ADC_Stop(hadc);
  hadc->Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
  hadc->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  if (HAL_ADC_Init(hadc) != HAL_OK)
  {
    return false;
  }

  config.Channel = channel;
  config.Rank = ADC_REGULAR_RANK_1;
  config.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;
  config.SingleDiff = ADC_SINGLE_ENDED;
  config.OffsetNumber = ADC_OFFSET_NONE;
  config.Offset = 0;
  return (HAL_ADC_ConfigChannel(hadc, &config) == HAL_OK);
}

static void DebugMonitor_StatsReset(AdcCalibrationStats_t *stats)
{
  stats->sum = 0U;
  stats->minimum = UINT16_MAX;
  stats->maximum = 0U;
  stats->count = 0U;
}

static void DebugMonitor_StatsPush(AdcCalibrationStats_t *stats, uint16_t value)
{
  stats->sum += value;
  if (value < stats->minimum)
  {
    stats->minimum = value;
  }
  if (value > stats->maximum)
  {
    stats->maximum = value;
  }
  stats->count++;
}

static uint16_t DebugMonitor_StatsAverage(const AdcCalibrationStats_t *stats)
{
  return (stats->count == 0U) ? 0U : (uint16_t)(stats->sum / stats->count);
}

uint16_t DebugMonitor_CalibrateBusCurrent(void)
{
  AdcCalibrationStats_t stats;
  ADC_AnalogWDGConfTypeDef watchdog = {0};
  uint32_t sample;
  uint16_t value;
  uint32_t center14;
  const uint32_t twoAmpCounts14 = 993U;
  char line[96];

  CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE);
  DebugMonitor_StatsReset(&stats);
  for (sample = 0U; sample < ADC_CAL_SAMPLES; ++sample)
  {
    if (DebugMonitor_ReadAdc(&hadc2, ADC_CHANNEL_3, &value))
    {
      DebugMonitor_StatsPush(&stats, value);
    }
  }

  if (stats.count != 0U)
  {
    busCurrentZeroRaw = DebugMonitor_StatsAverage(&stats);
  }

  center14 = ((uint32_t)busCurrentZeroRaw + 2U) >> 2;
  watchdog.WatchdogNumber = ADC_ANALOGWATCHDOG_1;
  watchdog.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
  watchdog.Channel = ADC_CHANNEL_3;
  watchdog.ITMode = ENABLE;
  watchdog.HighThreshold = center14 + twoAmpCounts14;
  watchdog.LowThreshold = (center14 > twoAmpCounts14) ?
                          (center14 - twoAmpCounts14) : 0U;
  (void)HAL_ADC_AnalogWDGConfig(&hadc2, &watchdog);
  __HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_AWD1);

  (void)snprintf(line, sizeof(line),
                 "CAL,BUS=%u/%u/%u,AWD=%lu/%lu,n=%u\r\n",
                 busCurrentZeroRaw, stats.minimum, stats.maximum,
                 (unsigned long)watchdog.LowThreshold,
                 (unsigned long)watchdog.HighThreshold,
                 (unsigned int)stats.count);
  DebugMonitor_Write(line);
  return busCurrentZeroRaw;
}

uint16_t DebugMonitor_GetBusCurrentZero(void)
{
  return busCurrentZeroRaw;
}

void DebugMonitor_PrintResetReason(uint32_t reset_flags)
{
  char line[256];

  (void)snprintf(line, sizeof(line),
                 "RST,raw=0x%08lX,bor=%u,pin=%u,por=%u,soft=%u,"
                 "iwdg=%u,wwdg=%u,lpwr=%u,cpu=%u\r\n",
                 (unsigned long)reset_flags,
                 (reset_flags & RCC_RSR_BORRSTF) != 0U,
                 (reset_flags & RCC_RSR_PINRSTF) != 0U,
                 (reset_flags & RCC_RSR_PORRSTF) != 0U,
                 (reset_flags & RCC_RSR_SFTRSTF) != 0U,
                 (reset_flags & RCC_RSR_IWDG1RSTF) != 0U,
                 (reset_flags & RCC_RSR_WWDG1RSTF) != 0U,
                 (reset_flags & RCC_RSR_LPWRRSTF) != 0U,
                 (reset_flags & RCC_RSR_CPURSTF) != 0U);
  DebugMonitor_Write(line);
  __HAL_RCC_CLEAR_RESET_FLAGS();
}

void DebugMonitor_RunPwmInspection(uint32_t reset_flags)
{
  const uint16_t currentTripCounts = 1000U;
  const uint32_t outputMask = TIM_CCER_CC1E | TIM_CCER_CC1NE |
                              TIM_CCER_CC2E | TIM_CCER_CC2NE |
                              TIM_CCER_CC3E | TIM_CCER_CC3NE;
  const uint32_t compareCenter = (TIM1->ARR + 1U) / 2U;
  const uint32_t vectorDelta = 10U;
  const uint32_t compareU = compareCenter + vectorDelta;
  const uint32_t compareV = compareCenter - (vectorDelta / 2U);
  const uint32_t compareW = compareV;
  AdcCalibrationStats_t currentStats[2];
  uint32_t sampleErrors = 0U;
  uint32_t tripMask = 0U;
  uint32_t consecutiveU = 0U;
  uint32_t consecutiveW = 0U;
  uint32_t maximumConsecutiveU = 0U;
  uint32_t maximumConsecutiveW = 0U;
  uint32_t startTick;
  int32_t averageDeltaU;
  int32_t averageDeltaW;
  int32_t estimatedDeltaV;
  uint16_t adcU = 0U;
  uint16_t adcV = 0U;
  uint16_t adcW = 0U;
  uint16_t valueU;
  uint16_t valueW;
  uint8_t command = 0U;
  bool synchronousAdcReadyU;
  bool synchronousAdcReadyW;
  char line[256];

  DebugMonitor_PrintResetReason(reset_flags);

  /* 上电默认保持六路PWM关闭，收到明确命令后才执行一次短时测试。 */
  CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE);
  CLEAR_BIT(TIM1->CR1, TIM_CR1_CEN);
  CLEAR_BIT(TIM1->CCER, outputMask);

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, compareU);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, compareV);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, compareW);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,
                        (TIM1->ARR > 1U) ? (TIM1->ARR - 1U) : 0U);
  __HAL_TIM_SET_COUNTER(&htim1, 0U);
  SET_BIT(TIM1->EGR, TIM_EGR_UG);

  (void)DebugMonitor_ReadAdc(&hadc1, ADC_CHANNEL_17, &adcU);
  (void)DebugMonitor_ReadAdc(&hadc1, ADC_CHANNEL_14, &adcV);
  (void)DebugMonitor_ReadAdc(&hadc2, ADC_CHANNEL_19, &adcW);
  synchronousAdcReadyU = DebugMonitor_ConfigureSynchronousRegularAdc(
                           &hadc1, ADC_CHANNEL_17);
  synchronousAdcReadyW = DebugMonitor_ConfigureSynchronousRegularAdc(
                           &hadc2, ADC_CHANNEL_19);
  (void)snprintf(line, sizeof(line),
                 "PWM6,READY,f=10000,time_ms=10,vector=U_POS_VW_NEG,dtg=%lu,"
                 "zero=%u/%u/%u,ccr=%lu/%lu/%lu,limit=%u,sync=%u/%u,send=P\r\n",
                 (unsigned long)(TIM1->BDTR & TIM_BDTR_DTG),
                 adcU, adcV, adcW,
                 (unsigned long)compareU,
                 (unsigned long)compareV,
                 (unsigned long)compareW,
                 currentTripCounts,
                 synchronousAdcReadyU ? 1U : 0U,
                 synchronousAdcReadyW ? 1U : 0U);
  DebugMonitor_Write(line);

  /* 每次上电只接受一次大写P，避免串口噪声导致重复驱动。 */
  for (;;)
  {
    if ((HAL_UART_Receive(&huart10, &command, 1U, 100U) == HAL_OK) &&
        (command == (uint8_t)'P'))
    {
      break;
    }
  }

  if ((!synchronousAdcReadyU) || (!synchronousAdcReadyW) ||
      (HAL_ADC_Start(&hadc1) != HAL_OK) ||
      (HAL_ADC_Start(&hadc2) != HAL_OK))
  {
    DebugMonitor_Write("PWM6,ABORT,reason=adc_sync_start\r\n");
    for (;;)
    {
      CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE);
      HAL_Delay(100U);
    }
  }

  (void)snprintf(line, sizeof(line),
                 "PWM6,START,vector=U_POS_VW_NEG,arr=%lu,ccr=%lu/%lu/%lu,dtg=%lu\r\n",
                 (unsigned long)TIM1->ARR,
                 (unsigned long)compareU,
                 (unsigned long)compareV,
                 (unsigned long)compareW,
                 (unsigned long)(TIM1->BDTR & TIM_BDTR_DTG));
  DebugMonitor_Write(line);

  /* 三相在同一次寄存器写入中使能，只施加预设的极小固定线电压。 */
  SET_BIT(TIM1->CCER, outputMask);
  SET_BIT(TIM1->CR1, TIM_CR1_CEN);
  SET_BIT(TIM1->BDTR, TIM_BDTR_MOE);

  DebugMonitor_StatsReset(&currentStats[0]);
  DebugMonitor_StatsReset(&currentStats[1]);

  /* 同步采集U、W两相电流；V相可由-(U+W)推算，连续3次越界立即关断。 */
  startTick = HAL_GetTick();
  while ((HAL_GetTick() - startTick) < 10U)
  {
    if ((HAL_ADC_PollForConversion(&hadc1, 1U) == HAL_OK) &&
        (HAL_ADC_PollForConversion(&hadc2, 1U) == HAL_OK))
    {
      valueU = (uint16_t)HAL_ADC_GetValue(&hadc1);
      valueW = (uint16_t)HAL_ADC_GetValue(&hadc2);
      DebugMonitor_StatsPush(&currentStats[0], valueU);
      DebugMonitor_StatsPush(&currentStats[1], valueW);

      if (((valueU > adcU) && ((valueU - adcU) > currentTripCounts)) ||
          ((valueU < adcU) && ((adcU - valueU) > currentTripCounts)))
      {
        consecutiveU++;
        if (consecutiveU > maximumConsecutiveU)
        {
          maximumConsecutiveU = consecutiveU;
        }
        if (consecutiveU >= 3U)
        {
          tripMask |= 0x01U;
        }
      }
      else
      {
        consecutiveU = 0U;
      }

      if (((valueW > adcW) && ((valueW - adcW) > currentTripCounts)) ||
          ((valueW < adcW) && ((adcW - valueW) > currentTripCounts)))
      {
        consecutiveW++;
        if (consecutiveW > maximumConsecutiveW)
        {
          maximumConsecutiveW = consecutiveW;
        }
        if (consecutiveW >= 3U)
        {
          tripMask |= 0x04U;
        }
      }
      else
      {
        consecutiveW = 0U;
      }
    }
    else { sampleErrors++; }

    if (tripMask != 0U)
    {
      CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE);
      break;
    }
  }

  /* 先关闭MOE，再停止计数器和通道，保证功率输出优先关断。 */
  CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE);
  CLEAR_BIT(TIM1->CR1, TIM_CR1_CEN);
  CLEAR_BIT(TIM1->CCER, outputMask);
  (void)HAL_ADC_Stop(&hadc1);
  (void)HAL_ADC_Stop(&hadc2);

  averageDeltaU = (int32_t)DebugMonitor_StatsAverage(&currentStats[0]) -
                  (int32_t)adcU;
  averageDeltaW = (int32_t)DebugMonitor_StatsAverage(&currentStats[1]) -
                  (int32_t)adcW;
  estimatedDeltaV = -(averageDeltaU + averageDeltaW);
  (void)snprintf(line, sizeof(line),
                 "PWM6,DONE,U=%u/%u/%u/%ld,W=%u/%u/%u/%ld,Vest=%ld,"
                 "n=%u,maxseq=%lu/%lu,trip=0x%02lX,err=%lu\r\n",
                 adcU, currentStats[0].minimum, currentStats[0].maximum,
                 (long)averageDeltaU,
                 adcW, currentStats[1].minimum, currentStats[1].maximum,
                 (long)averageDeltaW,
                 (long)estimatedDeltaV,
                 currentStats[0].count,
                 (unsigned long)maximumConsecutiveU,
                 (unsigned long)maximumConsecutiveW,
                 (unsigned long)tripMask,
                 (unsigned long)sampleErrors);
  DebugMonitor_Write(line);
  (void)snprintf(line, sizeof(line),
                 "PWM6,OFF,ccer=0x%08lX,bdtr=0x%08lX\r\n",
                 (unsigned long)TIM1->CCER,
                 (unsigned long)TIM1->BDTR);
  DebugMonitor_Write(line);

  for (;;)
  {
    /* 测试完成后保持锁定关断，复位后才能再次执行。 */
    CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE);
    HAL_Delay(100U);
  }
}

static bool DebugMonitor_RunPwm7Pulse(uint32_t ccrU, uint32_t ccrV, uint32_t ccrW,
                                      uint32_t secondChannel, uint16_t zeroU,
                                      uint16_t zeroSecond, AdcCalibrationStats_t *statsU,
                                      AdcCalibrationStats_t *statsSecond,
                                      uint32_t durationMs,
                                      uint32_t *tripMask, uint32_t *errors)
{
  const uint16_t currentTripCounts = 1000U;
  const uint32_t outputMask = TIM_CCER_CC1E | TIM_CCER_CC1NE |
                              TIM_CCER_CC2E | TIM_CCER_CC2NE |
                              TIM_CCER_CC3E | TIM_CCER_CC3NE;
  uint32_t consecutiveU = 0U;
  uint32_t consecutiveSecond = 0U;
  uint32_t startTick;

  if ((!DebugMonitor_ConfigureSynchronousRegularAdc(&hadc1, ADC_CHANNEL_17)) ||
      (!DebugMonitor_ConfigureSynchronousRegularAdc(&hadc2, secondChannel)) ||
      (HAL_ADC_Start(&hadc1) != HAL_OK) ||
      (HAL_ADC_Start(&hadc2) != HAL_OK))
  {
    return false;
  }

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccrU);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccrV);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccrW);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,
                        (TIM1->ARR > 1U) ? (TIM1->ARR - 1U) : 0U);
  __HAL_TIM_SET_COUNTER(&htim1, 0U);
  SET_BIT(TIM1->EGR, TIM_EGR_UG);
  DebugMonitor_StatsReset(statsU);
  DebugMonitor_StatsReset(statsSecond);

  SET_BIT(TIM1->CCER, outputMask);
  SET_BIT(TIM1->CR1, TIM_CR1_CEN);
  SET_BIT(TIM1->BDTR, TIM_BDTR_MOE);
  startTick = HAL_GetTick();
  while ((HAL_GetTick() - startTick) < durationMs)
  {
    if ((HAL_ADC_PollForConversion(&hadc1, 1U) == HAL_OK) &&
        (HAL_ADC_PollForConversion(&hadc2, 1U) == HAL_OK))
    {
      uint16_t valueU = (uint16_t)HAL_ADC_GetValue(&hadc1);
      uint16_t valueSecond = (uint16_t)HAL_ADC_GetValue(&hadc2);
      uint16_t differenceU = (valueU > zeroU) ? (valueU - zeroU) : (zeroU - valueU);
      uint16_t differenceSecond = (valueSecond > zeroSecond) ?
                                  (valueSecond - zeroSecond) : (zeroSecond - valueSecond);
      DebugMonitor_StatsPush(statsU, valueU);
      DebugMonitor_StatsPush(statsSecond, valueSecond);
      consecutiveU = (differenceU > currentTripCounts) ? (consecutiveU + 1U) : 0U;
      consecutiveSecond = (differenceSecond > currentTripCounts) ?
                          (consecutiveSecond + 1U) : 0U;
      if (consecutiveU >= 3U) { *tripMask |= 0x01U; }
      if (consecutiveSecond >= 3U)
      {
        *tripMask |= (secondChannel == ADC_CHANNEL_19) ? 0x04U : 0x02U;
      }
    }
    else
    {
      (*errors)++;
    }
    if (*tripMask != 0U)
    {
      CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE);
      break;
    }
  }

  /* 切换ADC通道前先彻底关闭功率输出，避免重配置期间出现不可控脉冲。 */
  CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE);
  CLEAR_BIT(TIM1->CR1, TIM_CR1_CEN);
  CLEAR_BIT(TIM1->CCER, outputMask);
  (void)HAL_ADC_Stop(&hadc1);
  (void)HAL_ADC_Stop(&hadc2);
  return true;
}

void DebugMonitor_RunPwmVectorInspection(uint32_t reset_flags)
{
  const uint32_t outputMask = TIM_CCER_CC1E | TIM_CCER_CC1NE |
                              TIM_CCER_CC2E | TIM_CCER_CC2NE |
                              TIM_CCER_CC3E | TIM_CCER_CC3NE;
  const uint32_t center = (TIM1->ARR + 1U) / 2U;
  const uint32_t secondChannel[4] = {ADC_CHANNEL_19, ADC_CHANNEL_14,
                                     ADC_CHANNEL_19, ADC_CHANNEL_14};
  const char *const vectorName[4] = {"FWD", "FWD", "REV", "REV"};
  const char *const pairName[4] = {"UW", "UV", "UW", "UV"};
  uint16_t zeroU = 0U, zeroV = 0U, zeroW = 0U;
  uint8_t command = 0U;
  uint32_t step;
  char line[192];

  DebugMonitor_PrintResetReason(reset_flags);
  CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE);
  CLEAR_BIT(TIM1->CR1, TIM_CR1_CEN);
  CLEAR_BIT(TIM1->CCER, outputMask);
  (void)DebugMonitor_ReadAdc(&hadc1, ADC_CHANNEL_17, &zeroU);
  /* V相在PWM7中由ADC2采集，零点也必须使用同一个ADC实例。 */
  (void)DebugMonitor_ReadAdc(&hadc2, ADC_CHANNEL_14, &zeroV);
  (void)DebugMonitor_ReadAdc(&hadc2, ADC_CHANNEL_19, &zeroW);

  for (step = 0U; step < 4U; step++)
  {
    AdcCalibrationStats_t statsU;
    AdcCalibrationStats_t statsSecond;
    uint32_t tripMask = 0U;
    uint32_t errors = 0U;
    uint32_t ccrU, ccrV, ccrW;
    uint16_t zeroSecond = (secondChannel[step] == ADC_CHANNEL_19) ? zeroW : zeroV;
    int32_t deltaU, deltaSecond;

    if (step < 2U)
    {
      ccrU = center + 10U; ccrV = center - 5U; ccrW = center - 5U;
    }
    else
    {
      ccrU = center - 10U; ccrV = center + 5U; ccrW = center + 5U;
    }
    (void)snprintf(line, sizeof(line),
                   "PWM7,READY,step=%lu/4,vec=%s,pair=%s,time_ms=5,zero=%u/%u/%u,send=P\r\n",
                   (unsigned long)(step + 1U), vectorName[step], pairName[step],
                   zeroU, zeroV, zeroW);
    DebugMonitor_Write(line);
    for (;;)
    {
      if ((HAL_UART_Receive(&huart10, &command, 1U, 100U) == HAL_OK) &&
          (command == (uint8_t)'P')) { break; }
    }

    (void)snprintf(line, sizeof(line),
                   "PWM7,START,step=%lu,vec=%s,pair=%s,ccr=%lu/%lu/%lu\r\n",
                   (unsigned long)(step + 1U), vectorName[step], pairName[step],
                   (unsigned long)ccrU, (unsigned long)ccrV, (unsigned long)ccrW);
    DebugMonitor_Write(line);
    if (!DebugMonitor_RunPwm7Pulse(ccrU, ccrV, ccrW, secondChannel[step],
                                   zeroU, zeroSecond, &statsU, &statsSecond,
                                   5U, &tripMask, &errors))
    {
      DebugMonitor_Write("PWM7,ABORT,reason=adc_sync_start\r\n");
      break;
    }

    deltaU = (int32_t)DebugMonitor_StatsAverage(&statsU) - (int32_t)zeroU;
    deltaSecond = (int32_t)DebugMonitor_StatsAverage(&statsSecond) -
                  (int32_t)zeroSecond;
    (void)snprintf(line, sizeof(line),
                   "PWM7,DONE,step=%lu,vec=%s,pair=%s,U=%ld,%s=%ld,n=%u,trip=0x%02lX,err=%lu\r\n",
                   (unsigned long)(step + 1U), vectorName[step], pairName[step],
                   (long)deltaU,
                   (secondChannel[step] == ADC_CHANNEL_19) ? "W" : "V",
                   (long)deltaSecond, statsU.count, (unsigned long)tripMask,
                   (unsigned long)errors);
    DebugMonitor_Write(line);
    DebugMonitor_Write("PWM7,OFF,ccer=0x00000000\r\n");
    if (tripMask != 0U)
    {
      DebugMonitor_Write("PWM7,ABORT,reason=current_trip\r\n");
      break;
    }
    HAL_Delay(100U);
  }

  DebugMonitor_Write("PWM7,COMPLETE,pwm=off\r\n");
  for (;;)
  {
    CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE);
    HAL_Delay(100U);
  }
}

void DebugMonitor_RunPwmBaselineVectorInspection(uint32_t reset_flags)
{
  const uint32_t outputMask = TIM_CCER_CC1E | TIM_CCER_CC1NE |
                              TIM_CCER_CC2E | TIM_CCER_CC2NE |
                              TIM_CCER_CC3E | TIM_CCER_CC3NE;
  const uint32_t center = (TIM1->ARR + 1U) / 2U;
  const uint32_t vectorDelta = 20U;
  const uint32_t pulseMs = 8U;
  const uint32_t secondChannel[4] = {ADC_CHANNEL_19, ADC_CHANNEL_14,
                                     ADC_CHANNEL_19, ADC_CHANNEL_14};
  const char *const vectorName[4] = {"FWD", "FWD", "REV", "REV"};
  const char *const pairName[4] = {"UW", "UV", "UW", "UV"};
  uint16_t zeroU = 0U, zeroV = 0U, zeroW = 0U;
  uint8_t command = 0U;
  uint32_t step;
  char line[224];

  DebugMonitor_PrintResetReason(reset_flags);
  CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE);
  CLEAR_BIT(TIM1->CR1, TIM_CR1_CEN);
  CLEAR_BIT(TIM1->CCER, outputMask);
  (void)DebugMonitor_ReadAdc(&hadc1, ADC_CHANNEL_17, &zeroU);
  /* V相零点固定使用ADC2_IN14读取，保证和后续同步采样通道完全一致。 */
  (void)DebugMonitor_ReadAdc(&hadc2, ADC_CHANNEL_14, &zeroV);
  (void)DebugMonitor_ReadAdc(&hadc2, ADC_CHANNEL_19, &zeroW);

  for (step = 0U; step < 4U; step++)
  {
    AdcCalibrationStats_t baseU;
    AdcCalibrationStats_t baseSecond;
    AdcCalibrationStats_t vecU;
    AdcCalibrationStats_t vecSecond;
    uint32_t tripMask = 0U;
    uint32_t errors = 0U;
    uint32_t ccrU, ccrV, ccrW;
    uint16_t zeroSecond = (secondChannel[step] == ADC_CHANNEL_19) ? zeroW : zeroV;
    uint16_t avgBaseU, avgBaseSecond, avgVecU, avgVecSecond;
    int32_t deltaU, deltaSecond;

    if (step < 2U)
    {
      ccrU = center + vectorDelta;
      ccrV = center - (vectorDelta / 2U);
      ccrW = center - (vectorDelta / 2U);
    }
    else
    {
      ccrU = center - vectorDelta;
      ccrV = center + (vectorDelta / 2U);
      ccrW = center + (vectorDelta / 2U);
    }

    (void)snprintf(line, sizeof(line),
                   "PWM8,READY,step=%lu/4,vec=%s,pair=%s,time_ms=%lu,delta=%lu,zero=%u/%u/%u,send=P\r\n",
                   (unsigned long)(step + 1U), vectorName[step], pairName[step],
                   (unsigned long)pulseMs, (unsigned long)vectorDelta,
                   zeroU, zeroV, zeroW);
    DebugMonitor_Write(line);
    for (;;)
    {
      if ((HAL_UART_Receive(&huart10, &command, 1U, 100U) == HAL_OK) &&
          (command == (uint8_t)'P')) { break; }
    }

    (void)snprintf(line, sizeof(line),
                   "PWM8,BASE,step=%lu,pair=%s,ccr=%lu/%lu/%lu\r\n",
                   (unsigned long)(step + 1U), pairName[step],
                   (unsigned long)center, (unsigned long)center,
                   (unsigned long)center);
    DebugMonitor_Write(line);
    if (!DebugMonitor_RunPwm7Pulse(center, center, center, secondChannel[step],
                                   zeroU, zeroSecond, &baseU, &baseSecond,
                                   pulseMs, &tripMask, &errors))
    {
      DebugMonitor_Write("PWM8,ABORT,reason=adc_sync_start_base\r\n");
      break;
    }
    HAL_Delay(5U);

    (void)snprintf(line, sizeof(line),
                   "PWM8,VEC,step=%lu,vec=%s,pair=%s,ccr=%lu/%lu/%lu\r\n",
                   (unsigned long)(step + 1U), vectorName[step], pairName[step],
                   (unsigned long)ccrU, (unsigned long)ccrV, (unsigned long)ccrW);
    DebugMonitor_Write(line);
    if ((tripMask == 0U) &&
        (!DebugMonitor_RunPwm7Pulse(ccrU, ccrV, ccrW, secondChannel[step],
                                    zeroU, zeroSecond, &vecU, &vecSecond,
                                    pulseMs, &tripMask, &errors)))
    {
      DebugMonitor_Write("PWM8,ABORT,reason=adc_sync_start_vec\r\n");
      break;
    }

    avgBaseU = DebugMonitor_StatsAverage(&baseU);
    avgBaseSecond = DebugMonitor_StatsAverage(&baseSecond);
    avgVecU = DebugMonitor_StatsAverage(&vecU);
    avgVecSecond = DebugMonitor_StatsAverage(&vecSecond);
    deltaU = (int32_t)avgVecU - (int32_t)avgBaseU;
    deltaSecond = (int32_t)avgVecSecond - (int32_t)avgBaseSecond;
    (void)snprintf(line, sizeof(line),
                   "PWM8,DONE,step=%lu,vec=%s,pair=%s,Ubase=%u,Uvec=%u,dU=%ld,%sbase=%u,%svec=%u,d%s=%ld,n=%u/%u,trip=0x%02lX,err=%lu\r\n",
                   (unsigned long)(step + 1U), vectorName[step], pairName[step],
                   avgBaseU, avgVecU, (long)deltaU,
                   (secondChannel[step] == ADC_CHANNEL_19) ? "W" : "V",
                   avgBaseSecond,
                   (secondChannel[step] == ADC_CHANNEL_19) ? "W" : "V",
                   avgVecSecond,
                   (secondChannel[step] == ADC_CHANNEL_19) ? "W" : "V",
                   (long)deltaSecond, baseU.count, vecU.count,
                   (unsigned long)tripMask, (unsigned long)errors);
    DebugMonitor_Write(line);
    DebugMonitor_Write("PWM8,OFF,ccer=0x00000000\r\n");
    if (tripMask != 0U)
    {
      DebugMonitor_Write("PWM8,ABORT,reason=current_trip\r\n");
      break;
    }
    HAL_Delay(100U);
  }

  DebugMonitor_Write("PWM8,COMPLETE,pwm=off\r\n");
  for (;;)
  {
    /* 诊断结束后保持硬关断，避免测试完成后仍存在未受控的门极输入。 */
    CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE);
    HAL_Delay(100U);
  }
}

#if DEBUG_RUNTIME_OUTPUT_ENABLE
static int32_t DebugMonitor_ToMilli(float value)
{
  float scaled = value * 1000.0f;
  return (scaled >= 0.0f) ? (int32_t)(scaled + 0.5f) :
                           (int32_t)(scaled - 0.5f);
}

static int32_t DebugMonitor_ToInteger(float value)
{
  return (value >= 0.0f) ? (int32_t)(value + 0.5f) :
                           (int32_t)(value - 0.5f);
}

static const char *DebugMonitor_GripperStateName(GripperState_t state)
{
  switch (state)
  {
    case GRIPPER_STATE_BOOT:
      return "BOOT";

    case GRIPPER_STATE_PRECHECK:
      return "PRECHECK";

    case GRIPPER_STATE_STARTING:
      return "STARTING";

    case GRIPPER_STATE_HOMING_OPEN:
      return "HOMING_OPEN";

    case GRIPPER_STATE_HOMING_CLOSE:
      return "HOMING_CLOSE";

    case GRIPPER_STATE_MOVING_SAFE:
      return "MOVING_SAFE";

    case GRIPPER_STATE_READY:
      return "READY";

    case GRIPPER_STATE_MOVING:
      return "MOVING";

    case GRIPPER_STATE_HOLDING:
      return "HOLDING";

    case GRIPPER_STATE_STOPPED:
      return "STOPPED";

    case GRIPPER_STATE_FAULT:
      return "FAULT";

    default:
      return "UNKNOWN";
  }
}

static bool DebugMonitor_IsHomingPrintActive(
  const GripperStatus_t *status)
{
  switch (status->state)
  {
    case GRIPPER_STATE_PRECHECK:
    case GRIPPER_STATE_STARTING:
    case GRIPPER_STATE_HOMING_OPEN:
    case GRIPPER_STATE_HOMING_CLOSE:
    case GRIPPER_STATE_MOVING_SAFE:
      return true;

    case GRIPPER_STATE_FAULT:
      return !status->homed;

    default:
      return false;
  }
}

static bool DebugMonitor_ShouldPrintHoming(
  const GripperStatus_t *status,
  GripperState_t *lastState,
  uint32_t *lastPrintTick)
{
  uint32_t now = osKernelGetTickCount();
  uint32_t tickHz = osKernelGetTickFreq();
  uint32_t elapsedMs = 0U;
  uint32_t intervalMs = DEBUG_HOMING_PRINT_MS;

  if (tickHz != 0U)
  {
    elapsedMs = (uint32_t)(((uint64_t)(now - *lastPrintTick) * 1000ULL) /
                           tickHz);
  }

  if (status->stall_elapsed_ms >= DEBUG_HOMING_FAST_STALL_MS)
  {
    intervalMs = DEBUG_HOMING_FAST_PRINT_MS;
  }

  if ((status->state != *lastState) ||
      (elapsedMs >= intervalMs))
  {
    *lastState = status->state;
    *lastPrintTick = now;
    return true;
  }

  return false;
}
#endif

void DebugMonitor_RunCurrentAdcCalibration(void)
{
  AdcCalibrationStats_t stats[4];
  GPIO_InitTypeDef gpio = {0};
  char line[192];

  /* Calibration firmware safety: disconnect TIM1 and force all gate inputs low. */
  CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE);
  CLEAR_BIT(TIM1->CCER, TIM_CCER_CC1E | TIM_CCER_CC1NE |
                         TIM_CCER_CC2E | TIM_CCER_CC2NE |
                         TIM_CCER_CC3E | TIM_CCER_CC3NE);
  HAL_GPIO_WritePin(GPIOE, M1_PWM_UH_Pin | M1_PWM_UL_Pin | M1_PWM_VH_Pin |
                           M1_PWM_VL_Pin | M1_PWM_WH_Pin | M1_PWM_WL_Pin,
                    GPIO_PIN_RESET);
  gpio.Pin = M1_PWM_UH_Pin | M1_PWM_UL_Pin | M1_PWM_VH_Pin |
             M1_PWM_VL_Pin | M1_PWM_WH_Pin | M1_PWM_WL_Pin;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_PULLDOWN;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &gpio);

  for (;;)
  {
    uint32_t errors = 0U;
    uint32_t sample;
    uint16_t value;
    uint32_t index;

    for (index = 0U; index < 4U; ++index)
    {
      DebugMonitor_StatsReset(&stats[index]);
    }

    for (sample = 0U; sample < ADC_CAL_SAMPLES; ++sample)
    {
      if (DebugMonitor_ReadAdc(&hadc1, ADC_CHANNEL_17, &value))
      {
        DebugMonitor_StatsPush(&stats[0], value); /* IA: PA1 */
      }
      else { errors++; }

      if (DebugMonitor_ReadAdc(&hadc1, ADC_CHANNEL_14, &value))
      {
        DebugMonitor_StatsPush(&stats[1], value); /* IB: PA2 */
      }
      else { errors++; }

      if (DebugMonitor_ReadAdc(&hadc2, ADC_CHANNEL_19, &value))
      {
        DebugMonitor_StatsPush(&stats[2], value); /* IC: PA5 */
      }
      else { errors++; }

      if (DebugMonitor_ReadAdc(&hadc2, ADC_CHANNEL_3, &value))
      {
        DebugMonitor_StatsPush(&stats[3], value); /* IBUS: PA6 */
      }
      else { errors++; }
    }

    (void)snprintf(line, sizeof(line),
                   "ADC,U=%u/%u/%u,V=%u/%u/%u,W=%u/%u/%u,"
                   "BUS=%u/%u/%u,err=%lu\r\n",
                   DebugMonitor_StatsAverage(&stats[0]), stats[0].minimum, stats[0].maximum,
                   DebugMonitor_StatsAverage(&stats[1]), stats[1].minimum, stats[1].maximum,
                   DebugMonitor_StatsAverage(&stats[2]), stats[2].minimum, stats[2].maximum,
                   DebugMonitor_StatsAverage(&stats[3]), stats[3].minimum, stats[3].maximum,
                   (unsigned long)errors);
    DebugMonitor_Write(line);
  }
}

#if DEBUG_RUNTIME_OUTPUT_ENABLE
static void DebugMonitor_Task(void *argument)
{
  uint32_t next = osKernelGetTickCount();
  uint32_t lastHomingPrintTick = 0U;
  GripperState_t lastHomingPrintState = GRIPPER_STATE_BOOT;
  uint32_t speedCheckLastTick = 0U;
  int32_t speedCheckLastCount = 0;
  bool speedCheckValid = false;
  char line[384];
  (void)argument;

  for (;;)
  {
    GripperStatus_t gripperStatus;
    MCI_State_t motorState = MC_GetSTMStateMotor1();
    qd_f_t reference = MC_GetIqdrefMotor1_F();
    qd_f_t measured = MC_GetIqdMotor1_F();
    bool homingPrintActive;

    GripperService_GetStatus(&gripperStatus);
    homingPrintActive = DebugMonitor_IsHomingPrintActive(&gripperStatus);

    if ((motorState != ALIGNMENT) && (motorState != RUN))
    {
      /* 非运行态不使用MCSDK的电流缓存，避免把初始化残留值当作真实电流。 */
      reference.q = 0.0f;
      reference.d = 0.0f;
      measured.q = 0.0f;
      measured.d = 0.0f;
    }

    if (homingPrintActive)
    {
      if (DebugMonitor_ShouldPrintHoming(&gripperStatus,
                                         &lastHomingPrintState,
                                         &lastHomingPrintTick))
      {
        uint32_t speedCheckTick = osKernelGetTickCount();
        uint32_t speedCheckDeltaTick = speedCheckTick - speedCheckLastTick;
        int32_t speedCheckRpm = 0;
        int32_t homeTravel =
          gripperStatus.position_count - gripperStatus.open_count;
        int32_t homeTravelMilliTurns =
          (int32_t)(((int64_t)homeTravel * 1000LL) /
                    GRIPPER_MOTOR_COUNTS_PER_TURN);

        /* 使用日志间隔内的count差独立计算转速，用于核对MCSDK速度换算。 */
        if (speedCheckValid && (speedCheckDeltaTick > 0U))
        {
          speedCheckRpm = (int32_t)
            (((int64_t)(gripperStatus.position_count - speedCheckLastCount) *
              (int64_t)osKernelGetTickFreq() * 60LL) /
             ((int64_t)GRIPPER_MOTOR_COUNTS_PER_TURN *
              (int64_t)speedCheckDeltaTick));
        }
        speedCheckLastTick = speedCheckTick;
        speedCheckLastCount = gripperStatus.position_count;
        speedCheckValid = true;

        /* 回零阶段保留字段名，方便直接判断卡在预检查、启动、开端或闭端搜索。 */
//        (void)snprintf(line, sizeof(line),
//                       "homing: state=%s,ms=%lu,stall=%lu,pos=%ld,"
//                       "open=%ld,close=%ld,travel=%ld,travel_mt=%ld,"
//                       "speedr=%ld,speed=%ld,iqr=%ld,iq=%ld,enc=%u,"
//                       "fault=0x%08lX,motor=0x%08lX,mc=0x%04X\r\n",
//                       DebugMonitor_GripperStateName(gripperStatus.state),
//                       (unsigned long)gripperStatus.state_elapsed_ms,
//                       (unsigned long)gripperStatus.stall_elapsed_ms,
//                       (long)gripperStatus.position_count,
//                       (long)gripperStatus.open_count,
//                       (long)gripperStatus.close_count,
//                       (long)homeTravel,
//                       (long)homeTravelMilliTurns,
//                       (long)DebugMonitor_ToInteger(gripperStatus.speed_ref_rpm),
//                       (long)DebugMonitor_ToInteger(gripperStatus.speed_rpm),
//                       (long)DebugMonitor_ToMilli(gripperStatus.iq_ref_a),
//                       (long)DebugMonitor_ToMilli(gripperStatus.iq_a),
//                       gripperStatus.encoder_reliable ? 1U : 0U,
//                       (unsigned long)gripperStatus.faults,
//                       (unsigned long)gripperStatus.motor_faults,
//                       (unsigned int)gripperStatus.mc_faults);
        (void)snprintf(line, sizeof(line),
                       "homing: %d,%lu,"
                       "%ld,%ld,%ld,%ld,%ld,%u\r\n",
						gripperStatus.state,
                       (unsigned long)gripperStatus.stall_elapsed_ms,
                       (long)DebugMonitor_ToInteger(gripperStatus.speed_ref_rpm),
                       (long)DebugMonitor_ToInteger(gripperStatus.speed_rpm),
                       (long)speedCheckRpm,
                       (long)DebugMonitor_ToMilli(gripperStatus.iq_ref_a),
                       (long)DebugMonitor_ToMilli(gripperStatus.iq_a),
                       gripperStatus.encoder_reliable ? 1U : 0U);
        DebugMonitor_Write(line);
      }
    }
    else
    {
      (void)snprintf(line, sizeof(line),
                     "%s: %ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld\r\n",
                     "position",
                     (long)DebugMonitor_ToMilli(reference.q),
                     (long)DebugMonitor_ToMilli(reference.d),
                     (long)DebugMonitor_ToMilli(measured.q),
                     (long)DebugMonitor_ToMilli(measured.d),
                     (long)DebugMonitor_ToInteger(gripperStatus.speed_ref_rpm),
                     (long)DebugMonitor_ToInteger(gripperStatus.speed_rpm),
                     (long)gripperStatus.target_count,
                     (long)gripperStatus.position_count);
      DebugMonitor_Write(line);
    }

//    if ((motorState == ALIGNMENT) || (motorState == RUN))
//    {
//      /* 打印高频环实际使用的扇区、注入ADC和比较值，用于核对采样重构链路。 */
//      (void)snprintf(rawLine, sizeof(rawLine),
//                     "RAW:sec=%u,j1=%lu,j2=%lu,ofs=%lu/%lu/%lu,ccr=%lu/%lu/%lu\r\n",
//                     (unsigned int)PWM_Handle_M1._Super.Sector,
//                     (unsigned long)ADC1->JDR1,
//                     (unsigned long)ADC2->JDR1,
//                     (unsigned long)PWM_Handle_M1.PhaseAOffset,
//                     (unsigned long)PWM_Handle_M1.PhaseBOffset,
//                     (unsigned long)PWM_Handle_M1.PhaseCOffset,
//                     (unsigned long)TIM1->CCR1,
//                     (unsigned long)TIM1->CCR2,
//                     (unsigned long)TIM1->CCR3);
//      DebugMonitor_Write(rawLine);
//    }

    next += (homingPrintActive ||
             (motorState == ALIGNMENT) || (motorState == RUN)) ?
            DEBUG_POLL_TICKS : DEBUG_IDLE_POLL_TICKS;
    (void)osDelayUntil(next);
  }
}
#endif

void DebugMonitor_CreateTask(void)
{
#if DEBUG_RUNTIME_OUTPUT_ENABLE
  const osThreadAttr_t attributes = {
    .name = "debugUart2",
    .stack_size = 512U * 4U,
    .priority = osPriorityLow
  };

  debugTaskHandle = osThreadNew(DebugMonitor_Task, NULL, &attributes);
  (void)debugTaskHandle;
#else
  /* 保留创建接口，当前不启动周期日志任务，避免文本插入触觉二进制帧流。 */
#endif
}
