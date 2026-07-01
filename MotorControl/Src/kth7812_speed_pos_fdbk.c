#include "kth7812_speed_pos_fdbk.h"
#include "main.h"
#include "parameters_conversion.h"

#include <limits.h>

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
  .direction = 1,
  .reliable = false
};

uint8_t KTH7812_Crc4(uint16_t angle12)
{
  uint8_t crc = 0U;
  int8_t bit;

  angle12 &= 0x0FFFU;
  for (bit = 11; bit >= 0; --bit)
  {
    uint8_t feedback = (uint8_t)(((crc >> 3) ^ (angle12 >> bit)) & 1U);
    crc = (uint8_t)((crc << 1) & 0x0FU);
    if (feedback != 0U)
    {
      crc ^= 0x03U;
    }
  }

  crc = (uint8_t)(((crc & 0x01U) << 3) |
                  ((crc & 0x02U) << 1) |
                  ((crc & 0x04U) >> 1) |
                  ((crc & 0x08U) >> 3));
  return crc;
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
  handle->direction = 1;
  handle->crc_error_count = 0U;
  handle->spi_error_count = 0U;
  handle->frame_count = 0U;
  handle->valid_frame_count = 0U;
  handle->last_frame = 0U;
  handle->last_good_frame = 0U;
  handle->received_crc = 0U;
  handle->calculated_crc = 0U;
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
  handle->last_raw = (int16_t)handle->raw_angle;
  handle->_Super.hAvrMecSpeedUnit = 0;
  handle->_Super.hElSpeedDpp = 0;
  handle->_Super.InstantaneousElSpeedDpp = 0;
}

KTH7812_Status_t KTH7812_Update(KTH7812_Handle_t *handle)
{
  uint16_t frame;
  uint16_t raw;
  int16_t delta;
  int32_t electrical;
  KTH7812_Status_t status = KTH7812_ReadFrame(handle, &frame);

  if (status == KTH7812_OK)
  {
    handle->last_frame = frame;
    handle->frame_count++;
    raw = (uint16_t)(frame >> 4);
    handle->received_crc = (uint8_t)(frame & 0x0FU);
    handle->calculated_crc = KTH7812_Crc4(raw);
    if (handle->calculated_crc != handle->received_crc)
    {
      handle->crc_error_count++;
      status = KTH7812_ERROR_CRC;
    }
    else
    {
      handle->last_good_frame = frame;
      handle->valid_frame_count++;
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

  handle->consecutive_errors = 0U;
  handle->reliable = true;
  handle->_Super.bSpeedErrorNumber = 0U;

  if (!handle->initialized)
  {
    handle->raw_angle = raw;
    handle->last_raw = (int16_t)raw;
    handle->initialized = true;
  }

  delta = (int16_t)raw - handle->last_raw;
  if (delta > (KTH7812_COUNTS_PER_TURN / 2))
  {
    delta -= KTH7812_COUNTS_PER_TURN;
  }
  else if (delta < -(KTH7812_COUNTS_PER_TURN / 2))
  {
    delta += KTH7812_COUNTS_PER_TURN;
  }

  delta = (int16_t)(delta * handle->direction);
  handle->multi_turn_count += delta;
  handle->last_raw = (int16_t)raw;
  handle->raw_angle = raw;

  handle->_Super.hMecAngle = (int16_t)(handle->multi_turn_count * 16);
  handle->_Super.wMecAngle = handle->multi_turn_count * 16;
  handle->_Super.InstantaneousElSpeedDpp = (int16_t)(delta * 16 * POLE_PAIR_NUM);
  handle->_Super.hElSpeedDpp = handle->_Super.InstantaneousElSpeedDpp;

  electrical = ((int32_t)raw * 16 * POLE_PAIR_NUM * handle->direction) +
               handle->electrical_offset;
  handle->_Super.hElAngle = (int16_t)electrical;
  return KTH7812_OK;
}

bool KTH7812_CalcAvrgMecSpeedUnit(KTH7812_Handle_t *handle, int16_t *speed_unit)
{
  int32_t delta = handle->multi_turn_count - handle->speed_sample_count;
  int32_t speed = (delta * (int32_t)SPEED_LOOP_FREQUENCY_HZ * SPEED_UNIT) /
                  KTH7812_COUNTS_PER_TURN;

  handle->speed_sample_count = handle->multi_turn_count;
  if (speed > INT16_MAX)
  {
    speed = INT16_MAX;
  }
  else if (speed < INT16_MIN)
  {
    speed = INT16_MIN;
  }

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
  int32_t raw_electrical = (int32_t)handle->raw_angle * 16 * POLE_PAIR_NUM * handle->direction;
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
