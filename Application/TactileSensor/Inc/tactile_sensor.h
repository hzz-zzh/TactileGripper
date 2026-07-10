#ifndef TACTILE_SENSOR_H
#define TACTILE_SENSOR_H

#include "stm32h7xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TACTILE_SENSOR_FRAME_SIZE  96U

/* USART2只读取地址0x36，外部只能取得通过校验的完整原始帧。 */
void TactileSensor_Init(UART_HandleTypeDef *uart);
void TactileSensor_CreateTask(void);
void TactileSensor_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size);
void TactileSensor_ErrorCallback(UART_HandleTypeDef *huart);
bool TactileSensor_GetLatestRaw(uint8_t *buffer,
                                uint16_t capacity,
                                uint32_t *frame_count);

#ifdef __cplusplus
}
#endif

#endif
