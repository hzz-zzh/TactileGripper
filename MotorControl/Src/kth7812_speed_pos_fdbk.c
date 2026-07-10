#include "kth7812_speed_pos_fdbk.h"
#include "main.h"
#include "parameters_conversion.h"

#include <limits.h>
#include <string.h>

KTH7812_Handle_t KTH7812_M1 =
{
  ._Super =
  {
    .bElToMecRatio = POLE_PAIR_NUM,
    .SpeedUnit = SPEED_UNIT,
    .bMaximumSpeedErrorsNumber = M1_SS_MEAS_ERRORS_BEFORE_FAULTS,
    .hMaxReliableMecSpeedUnit = (uint16_t)(MAX_APPLICATION_SPEED_UNIT * 115U / 100U),
    .hMinReliableMecSpeedUnit = 0U,
    .hMaxReliableMecAccelUnitP = UINT16_MAX,
    .hMeasurementFrequency = TF_REGULATION_RATE_SCALED,
    .DPPConvFactor = DPP_CONV_FACTOR
  },
  .direction = KTH7812_DIRECTION,
  .reliable = false
};

static KTH7812_Status_t KTH7812_ReadFrame(KTH7812_Handle_t *handle, uint16_t *frame)
{
  uint32_t timeout;
  SPI_TypeDef *spi = handle->spi->Instance;

  /* SPI1 hardware NSS asserts PA15 when SPE is enabled. */
  spi->IFCR = SPI_IFCR_EOTC | SPI_IFCR_TXTFC | SPI_IFCR_OVRC |
              SPI_IFCR_UDRC | SPI_IFCR_CRCEC | SPI_IFCR_TIFREC;
  MODIFY_REG(spi->CR2, SPI_CR2_TSIZE, 1U);
  SET_BIT(spi->CR1, SPI_CR1_SPE);
  SET_BIT(spi->CR1, SPI_CR1_CSTART);

  timeout = 20000U;
  while (((spi->SR & SPI_SR_TXP) == 0U) && (timeout > 0U))
  {
    timeout--;
  }
  if (timeout > 0U)
  {
    *(__IO uint16_t *)&spi->TXDR = 0U;
  }

  while (((spi->SR & SPI_SR_RXP) == 0U) && (timeout > 0U))
  {
    timeout--;
  }
  if (timeout > 0U)
  {
    *frame = *(__IO uint16_t *)&spi->RXDR;
  }

  while (((spi->SR & SPI_SR_EOT) == 0U) && (timeout > 0U))
  {
    timeout--;
  }
  /* Keep NSS low for the KTH7812 SCK-to-CSN hold time (>120 ns). */
  for (volatile uint32_t nss_hold = 0U; nss_hold < 128U; ++nss_hold)
  {
    __NOP();
  }
  spi->IFCR = SPI_IFCR_EOTC | SPI_IFCR_TXTFC | SPI_IFCR_OVRC;
  CLEAR_BIT(spi->CR1, SPI_CR1_SPE);

  if (timeout == 0U)
  {
    handle->spi_error_count++;
    return KTH7812_ERROR_SPI;
  }
  return KTH7812_OK;
}

void KTH7812_Init(KTH7812_Handle_t *handle, SPI_HandleTypeDef *spi,
                  GPIO_TypeDef *nss_port, uint16_t nss_pin)
{
  handle->spi = spi;
  handle->nss_port = nss_port;
  handle->nss_pin = nss_pin;
  handle->electrical_offset = 0;
  /* 初始化时固定使用与当前电机相序匹配的反向编码器方向。 */
  handle->direction = KTH7812_DIRECTION;
  handle->spi_error_count = 0U;
  handle->frame_count = 0U;
  handle->last_frame = 0U;
  handle->plausibility_error_count = 0U;
  handle->consecutive_errors = 0U;
  handle->initialized = false;
  handle->reliable = false;
  (void)KTH7812_Update(handle);
  KTH7812_Clear(handle);
}

void KTH7812_Clear(KTH7812_Handle_t *handle)
{
  handle->multi_turn_count = (int32_t)handle->raw_angle;
  handle->_Super.hMecAngle = (int16_t)handle->multi_turn_count;
  handle->_Super.wMecAngle = handle->multi_turn_count;
  KTH7812_ClearSpeed(handle);
}

void KTH7812_ClearSpeed(KTH7812_Handle_t *handle)
{
  /* 电机重新启动时只清速度估算，保留回零标定使用的多圈位置坐标。 */
  handle->speed_sample_count = handle->multi_turn_count;
  handle->speed_filter_sum = 0;
  handle->speed_filter_index = 0U;
  handle->speed_filter_count = 0U;
  memset(handle->speed_filter_buffer, 0, sizeof(handle->speed_filter_buffer));
  handle->last_raw = (int16_t)handle->raw_angle;
  handle->_Super.hAvrMecSpeedUnit = 0;
  handle->_Super.hElSpeedDpp = 0;
  handle->_Super.InstantaneousElSpeedDpp = 0;
}

KTH7812_Status_t KTH7812_Update(KTH7812_Handle_t *handle)
{
  uint16_t frame;
  uint16_t raw;
  int32_t delta;
  int32_t electrical;
  KTH7812_Status_t status = KTH7812_ReadFrame(handle, &frame);

  if (status == KTH7812_OK)
  {
    handle->last_frame = frame;
    raw = frame;
  }

  if (status != KTH7812_OK)
  {
    if (handle->consecutive_errors < UINT8_MAX)
    {
      handle->consecutive_errors++;
    }
    if (handle->consecutive_errors >= KTH7812_MAX_CONSECUTIVE_ERRORS)
    {
      handle->reliable = false;
      handle->_Super.bSpeedErrorNumber = handle->_Super.bMaximumSpeedErrorsNumber;
    }
    return status;
  }

  if (!handle->initialized)
  {
    handle->raw_angle = raw;
    handle->last_raw = raw;
    handle->initialized = true;
  }

  /* Signed 16-bit subtraction unwraps the 0xFFFF/0x0000 crossing. */
  delta = (int32_t)(int16_t)(raw - handle->last_raw);

  /* 16 kHz下即使达到最高转速，单周期变化也远小于1024；拒绝SPI毛刺角度。 */
  if ((delta > KTH7812_MAX_DELTA_PER_UPDATE) ||
      (delta < -KTH7812_MAX_DELTA_PER_UPDATE))
  {
    handle->plausibility_error_count++;
    if (handle->consecutive_errors < UINT8_MAX)
    {
      handle->consecutive_errors++;
    }
    if (handle->consecutive_errors >= KTH7812_MAX_CONSECUTIVE_ERRORS)
    {
      handle->reliable = false;
      handle->_Super.bSpeedErrorNumber = handle->_Super.bMaximumSpeedErrorsNumber;
    }
    return KTH7812_ERROR_PLAUSIBILITY;
  }

  handle->frame_count++;
  handle->consecutive_errors = 0U;
  handle->reliable = true;
  handle->_Super.bSpeedErrorNumber = 0U;
  delta *= handle->direction;
  handle->multi_turn_count += delta;
  handle->last_raw = raw;
  handle->raw_angle = raw;

  handle->_Super.hMecAngle = (int16_t)handle->multi_turn_count;
  handle->_Super.wMecAngle = handle->multi_turn_count;
  handle->_Super.InstantaneousElSpeedDpp = (int16_t)(delta * POLE_PAIR_NUM);
  handle->_Super.hElSpeedDpp = handle->_Super.InstantaneousElSpeedDpp;

  electrical = ((int32_t)raw * POLE_PAIR_NUM * handle->direction) +
               handle->electrical_offset;
  handle->_Super.hElAngle = (int16_t)electrical;
  return KTH7812_OK;
}

bool KTH7812_CalcAvrgMecSpeedUnit(KTH7812_Handle_t *handle, int16_t *speed_unit)
{
  int32_t delta = handle->multi_turn_count - handle->speed_sample_count;
  int32_t raw_speed = (delta * (int32_t)SPEED_LOOP_FREQUENCY_HZ * SPEED_UNIT) /
                      KTH7812_COUNTS_PER_TURN;
  int32_t speed;

  handle->speed_sample_count = handle->multi_turn_count;
  if (raw_speed > INT16_MAX)
  {
    raw_speed = INT16_MAX;
  }
  else if (raw_speed < INT16_MIN)
  {
    raw_speed = INT16_MIN;
  }

  /* 2点滑动平均抑制量化噪声，同时减少负载扰动传递到速度环的反馈延迟。 */
  handle->speed_filter_sum -=
    handle->speed_filter_buffer[handle->speed_filter_index];
  handle->speed_filter_buffer[handle->speed_filter_index] = (int16_t)raw_speed;
  handle->speed_filter_sum += raw_speed;
  handle->speed_filter_index++;
  if (handle->speed_filter_index >= KTH7812_SPEED_FILTER_SAMPLES)
  {
    handle->speed_filter_index = 0U;
  }
  if (handle->speed_filter_count < KTH7812_SPEED_FILTER_SAMPLES)
  {
    handle->speed_filter_count++;
  }
  speed = handle->speed_filter_sum / (int32_t)handle->speed_filter_count;

  handle->_Super.hAvrMecSpeedUnit = (int16_t)speed;
  *speed_unit = (int16_t)speed;
  return handle->reliable && SPD_IsMecSpeedReliable(&handle->_Super, speed_unit);
}

void KTH7812_SetElectricalOffset(KTH7812_Handle_t *handle, int16_t offset)
{
  handle->electrical_offset = offset;
}

void KTH7812_AlignElectricalAngle(KTH7812_Handle_t *handle, int16_t electrical_angle)
{
  int32_t raw_electrical = (int32_t)handle->raw_angle * POLE_PAIR_NUM * handle->direction;
  handle->electrical_offset = (int16_t)((int32_t)electrical_angle - raw_electrical);
  handle->_Super.hElAngle = electrical_angle;
}

int32_t KTH7812_GetMultiTurnCount(const KTH7812_Handle_t *handle)
{
  return handle->multi_turn_count;
}

bool KTH7812_IsReliable(const KTH7812_Handle_t *handle)
{
  return handle->reliable;
}
