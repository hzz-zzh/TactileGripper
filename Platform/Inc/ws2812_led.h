#ifndef WS2812_LED_H
#define WS2812_LED_H

#include "stm32h7xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  uint8_t red;
  uint8_t green;
  uint8_t blue;
} Ws2812Color_t;

void Ws2812Led_Init(SPI_HandleTypeDef *hspi);
bool Ws2812Led_SetColor(Ws2812Color_t color);
bool Ws2812Led_IsBusy(void);
uint32_t Ws2812Led_GetErrorCount(void);

#ifdef __cplusplus
}
#endif

#endif
