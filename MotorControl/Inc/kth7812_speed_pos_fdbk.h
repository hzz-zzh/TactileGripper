#ifndef KTH7812_SPEED_POS_FDBK_H
#define KTH7812_SPEED_POS_FDBK_H

#include "speed_pos_fdbk.h"
#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define KTH7812_COUNTS_PER_TURN       65536L
#define KTH7812_DIRECTION             (-1) /* 按当前相序将编码器机械角和电角度方向反向。 */
#define KTH7812_MAX_CONSECUTIVE_ERRORS 3U
#define KTH7812_SPEED_FILTER_SAMPLES   2U
/* 10kHz采样时，7790rpm约为851计数/次；保留约20%的速度裕量。 */
#define KTH7812_MAX_DELTA_PER_UPDATE  1024L

typedef enum
{
  KTH7812_OK = 0,
  KTH7812_ERROR_SPI,
  KTH7812_ERROR_PLAUSIBILITY
} KTH7812_Status_t;

typedef struct
{
  SpeednPosFdbk_Handle_t _Super;
  SPI_HandleTypeDef *spi;
  GPIO_TypeDef *nss_port;
  uint16_t nss_pin;
  int32_t multi_turn_count;
  int32_t speed_sample_count;
  int32_t speed_filter_sum;
  int16_t speed_filter_buffer[KTH7812_SPEED_FILTER_SAMPLES];
  uint8_t speed_filter_index;
  uint8_t speed_filter_count;
  int16_t electrical_offset;
  uint16_t last_raw;
  uint16_t raw_angle;
  uint16_t last_frame;
  uint32_t frame_count;
  uint32_t spi_error_count;
  uint32_t plausibility_error_count;
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

#ifdef __cplusplus
}
#endif

#endif
