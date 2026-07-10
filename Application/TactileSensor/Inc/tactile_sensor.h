#ifndef TACTILE_SENSOR_H
#define TACTILE_SENSOR_H

#include "stm32h7xx_hal.h"
#include "tactile_data.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TACTILE_SENSOR_FRAME_SIZE  96U
#define TACTILE_SENSOR_ADDRESS_36  0x36U
#define TACTILE_SENSOR_ADDRESS_37  0x37U
#define TACTILE_SENSOR_PORT_USART1 0U
#define TACTILE_SENSOR_PORT_USART2 1U

/* USART1和USART2均轮询0x36/0x37，只保存通过校验的完整原始帧。 */
void TactileSensor_Init(UART_HandleTypeDef *uart1,
                        UART_HandleTypeDef *uart2);
void TactileSensor_CreateTask(void);
void TactileSensor_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size);
void TactileSensor_ErrorCallback(UART_HandleTypeDef *huart);

/* 获取最近一次四单元均有效的完整快照。 */
bool TactileSensor_GetLatestData(gripper_tactile_data_t *data);

/* 兼容旧调用，默认读取USART2的0x36原始帧。 */
bool TactileSensor_GetLatestRaw(uint8_t *buffer,
                                uint16_t capacity,
                                uint32_t *frame_count);
/* 按地址读取USART2原始帧。 */
bool TactileSensor_GetLatestRawByAddress(uint8_t address,
                                         uint8_t *buffer,
                                         uint16_t capacity,
                                         uint32_t *frame_count);
/* 按USART端口和地址读取四路中的任意一路原始帧。 */
bool TactileSensor_GetLatestRawByPort(uint8_t port_index,
                                      uint8_t address,
                                      uint8_t *buffer,
                                      uint16_t capacity,
                                      uint32_t *frame_count);

#ifdef __cplusplus
}
#endif

#endif
