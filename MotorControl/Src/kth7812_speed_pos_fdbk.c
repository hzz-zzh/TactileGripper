#include "kth7812_speed_pos_fdbk.h"
#include "main.h"
#include "parameters_conversion.h"

#include <limits.h>
#include <string.h>

#define KTH7812_SPI_TIMEOUT_LOOPS  5000U

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

static uint8_t KTH7812_Reflect4(uint8_t value)
{
  return (uint8_t)((((value >> 0) & 0x01U) << 3) |
                   (((value >> 1) & 0x01U) << 2) |
                   (((value >> 2) & 0x01U) << 1) |
                   (((value >> 3) & 0x01U) << 0));
}

static uint8_t KTH7812_CalcCrc4Itu(uint16_t position12)
{
  uint8_t crc = 0U;
  int8_t bit;
  uint8_t i;

  /*
   * 手册示例：位置0x0FF时CRC为0x2。按MSB-first输入12位位置数据，
   * 多项式x4+x+1，最后输出反转，可得到与手册一致的结果。
   */
  for (bit = 11; bit >= 0; --bit)
  {
    uint8_t top = (uint8_t)((crc >> 3) & 0x01U);
    crc = (uint8_t)(((crc << 1) & 0x0FU) |
                    ((position12 >> bit) & 0x01U));
    if (top != 0U)
    {
      crc ^= 0x03U;
    }
  }

  for (i = 0U; i < 4U; ++i)
  {
    uint8_t top = (uint8_t)((crc >> 3) & 0x01U);
    crc = (uint8_t)((crc << 1) & 0x0FU);
    if (top != 0U)
    {
      crc ^= 0x03U;
    }
  }

  return KTH7812_Reflect4(crc);
}

static bool KTH7812_DecodeFrame(KTH7812_Handle_t *handle,
                                uint16_t frame,
                                uint16_t *raw_angle)
{
#if KTH7812_SPI_CRC_OUTPUT
  uint16_t position12 = (uint16_t)(frame >> 4);
  uint8_t receivedCrc = (uint8_t)(frame & 0x0FU);
  uint8_t calculatedCrc = KTH7812_CalcCrc4Itu(position12);

  handle->last_position12 = position12;
  handle->last_received_crc = receivedCrc;
  handle->last_calculated_crc = calculatedCrc;

  if (receivedCrc != calculatedCrc)
  {
    handle->crc_error_count++;
#if KTH7812_SPI_CRC_CHECK_ENABLE
    return false;
#endif
  }

  /* C版本只有12bit角度，左移4位后继续保持65536 count/turn的软件单位。 */
  *raw_angle = (uint16_t)(position12 << 4);
#else
  *raw_angle = frame;
#endif
  return true;
}

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

  timeout = KTH7812_SPI_TIMEOUT_LOOPS;
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
  handle->last_position12 = 0U;
  handle->last_received_crc = 0U;
  handle->last_calculated_crc = 0U;
  handle->crc_error_count = 0U;
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
  uint16_t raw = 0U;
  int32_t delta;
  int32_t electrical;
  KTH7812_Status_t status = KTH7812_ReadFrame(handle, &frame);

  if (status == KTH7812_OK)
  {
    handle->last_frame = frame;
    if (!KTH7812_DecodeFrame(handle, frame, &raw))
    {
      status = KTH7812_ERROR_CRC;
    }
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

  /*
   * 当前内部角度单位为65536 count/turn。若上一两帧因CRC或SPI错误被丢弃，
   * 下一帧的真实位移会累计，因此跳变门限按连续错误数适当放宽。
   */

  /* 10kHz下即使达到最高转速，单周期变化也远小于1024 count；拒绝SPI毛刺角度。 */
  if ((delta > ((int32_t)KTH7812_MAX_DELTA_PER_UPDATE *
                ((int32_t)handle->consecutive_errors + 1L))) ||
      (delta < -((int32_t)KTH7812_MAX_DELTA_PER_UPDATE *
                 ((int32_t)handle->consecutive_errors + 1L))))
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
