#ifndef KTH7812_SPEED_POS_FDBK_H
#define KTH7812_SPEED_POS_FDBK_H

#include "speed_pos_fdbk.h"
#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define KTH7812_COUNTS_PER_TURN       4096
#define KTH7812_MAX_CONSECUTIVE_ERRORS 3U

typedef enum
{
  KTH7812_OK = 0,
  KTH7812_ERROR_SPI,
  KTH7812_ERROR_CRC
} KTH7812_Status_t;

typedef struct
{
  SpeednPosFdbk_Handle_t _Super;
  SPI_HandleTypeDef *spi;
  GPIO_TypeDef *nss_port;
  uint16_t nss_pin;
  int32_t multi_turn_count;
  int32_t speed_sample_count;
  int16_t electrical_offset;
  int16_t last_raw;
  uint16_t raw_angle;
  uint16_t last_frame;
  uint16_t last_good_frame;
  uint32_t frame_count;
  uint32_t valid_frame_count;
  uint32_t crc_error_count;
  uint32_t spi_error_count;
  uint8_t received_crc;
  uint8_t calculated_crc;
  uint8_t consecutive_errors;
  int8_t direction;
  bool initialized;
  bool reliable;
} KTH7812_Handle_t;

extern KTH7812_Handle_t KTH7812_M1;

void KTH7812_Init(KTH7812_Handle_t *handle, SPI_HandleTypeDef *spi,
                  GPIO_TypeDef *nss_port, uint16_t nss_pin);
void KTH7812_Clear(KTH7812_Handle_t *handle);
KTH7812_Status_t KTH7812_Update(KTH7812_Handle_t *handle);
bool KTH7812_CalcAvrgMecSpeedUnit(KTH7812_Handle_t *handle, int16_t *speed_unit);
void KTH7812_SetElectricalOffset(KTH7812_Handle_t *handle, int16_t offset);
void KTH7812_AlignElectricalAngle(KTH7812_Handle_t *handle, int16_t electrical_angle);
int32_t KTH7812_GetMultiTurnCount(const KTH7812_Handle_t *handle);
bool KTH7812_IsReliable(const KTH7812_Handle_t *handle);
uint8_t KTH7812_Crc4(uint16_t angle12);

#ifdef __cplusplus
}
#endif

#endif
