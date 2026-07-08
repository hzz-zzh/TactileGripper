/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdbool.h>

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* 触觉传感器独立调试模式：只启动触觉采集任务和 RS485 调试输出。 */
#define TACTILE_SENSOR_ONLY_MODE   1U

/* Motor power stage: TIM1 complementary PWM. */
#define M1_PWM_UH_Pin              GPIO_PIN_9
#define M1_PWM_UH_GPIO_Port        GPIOE
#define M1_PWM_UL_Pin              GPIO_PIN_8
#define M1_PWM_UL_GPIO_Port        GPIOE
#define M1_PWM_VH_Pin              GPIO_PIN_11
#define M1_PWM_VH_GPIO_Port        GPIOE
#define M1_PWM_VL_Pin              GPIO_PIN_10
#define M1_PWM_VL_GPIO_Port        GPIOE
#define M1_PWM_WH_Pin              GPIO_PIN_13
#define M1_PWM_WH_GPIO_Port        GPIOE
#define M1_PWM_WL_Pin              GPIO_PIN_12
#define M1_PWM_WL_GPIO_Port        GPIOE

/* Current and voltage feedback. */
#define M1_CURR_AMPL_U_Pin         GPIO_PIN_1
#define M1_CURR_AMPL_U_GPIO_Port   GPIOA
#define M1_CURR_AMPL_V_Pin         GPIO_PIN_2
#define M1_CURR_AMPL_V_GPIO_Port   GPIOA
#define M1_CURR_AMPL_W_Pin         GPIO_PIN_5
#define M1_CURR_AMPL_W_GPIO_Port   GPIOA
#define M1_BUS_CURRENT_Pin         GPIO_PIN_6
#define M1_BUS_CURRENT_GPIO_Port   GPIOA
#define M1_BUS_VOLTAGE_Pin         GPIO_PIN_7
#define M1_BUS_VOLTAGE_GPIO_Port   GPIOA
#define M1_TEMPERATURE_Pin         GPIO_PIN_3
#define M1_TEMPERATURE_GPIO_Port   GPIOA

/* KTH7812 absolute encoder on SPI1. NSS is controlled by software. */
#define KTH7812_SCK_Pin            GPIO_PIN_3
#define KTH7812_SCK_GPIO_Port      GPIOB
#define KTH7812_MISO_Pin           GPIO_PIN_4
#define KTH7812_MISO_GPIO_Port     GPIOB
#define KTH7812_MOSI_Pin           GPIO_PIN_5
#define KTH7812_MOSI_GPIO_Port     GPIOB
#define KTH7812_NSS_Pin            GPIO_PIN_15
#define KTH7812_NSS_GPIO_Port      GPIOA

/* 触觉传感器1串口：USART1，460800-8-N-1。 */
#define TACTILE1_UART_TX_Pin       GPIO_PIN_9
#define TACTILE1_UART_TX_GPIO_Port GPIOA
#define TACTILE1_UART_RX_Pin       GPIO_PIN_10
#define TACTILE1_UART_RX_GPIO_Port GPIOA
#define MOTOR_PILOT_TX_Pin         TACTILE1_UART_TX_Pin
#define MOTOR_PILOT_TX_GPIO_Port   TACTILE1_UART_TX_GPIO_Port
#define MOTOR_PILOT_RX_Pin         TACTILE1_UART_RX_Pin
#define MOTOR_PILOT_RX_GPIO_Port   TACTILE1_UART_RX_GPIO_Port

/* 触觉传感器2串口：USART2，460800-8-N-1。 */
#define TACTILE_UART_TX_Pin        GPIO_PIN_5
#define TACTILE_UART_TX_GPIO_Port  GPIOD
#define TACTILE_UART_RX_Pin        GPIO_PIN_6
#define TACTILE_UART_RX_GPIO_Port  GPIOD
#define DEBUG_UART_TX_Pin          TACTILE_UART_TX_Pin
#define DEBUG_UART_TX_GPIO_Port    TACTILE_UART_TX_GPIO_Port
#define DEBUG_UART_RX_Pin          TACTILE_UART_RX_Pin
#define DEBUG_UART_RX_GPIO_Port    TACTILE_UART_RX_GPIO_Port

/* RS485 调试口：USART10，PC13 为收发方向控制，高电平发送。 */
#define RS485_DEBUG_TX_Pin         GPIO_PIN_3
#define RS485_DEBUG_TX_GPIO_Port   GPIOE
#define RS485_DEBUG_RX_Pin         GPIO_PIN_2
#define RS485_DEBUG_RX_GPIO_Port   GPIOE
#define RS485_DEBUG_DIR_Pin        GPIO_PIN_13
#define RS485_DEBUG_DIR_GPIO_Port  GPIOC

/* WS2812E-1313 status indicator driven by SPI3 MOSI. */
#define STATUS_LED_DATA_Pin        GPIO_PIN_12
#define STATUS_LED_DATA_GPIO_Port  GPIOC

/* CAN2.0B communication on FDCAN1. */
#define GRIPPER_CAN_RX_Pin         GPIO_PIN_8
#define GRIPPER_CAN_RX_GPIO_Port   GPIOB
#define GRIPPER_CAN_TX_Pin         GPIO_PIN_9
#define GRIPPER_CAN_TX_GPIO_Port   GPIOB

/* MCSDK compatibility: low sides are driven by TIM1 complementary outputs. */
#define M1_PWM_EN_U_Pin            GPIO_PIN_0
#define M1_PWM_EN_U_GPIO_Port      GPIOE
#define M1_PWM_EN_V_Pin            GPIO_PIN_0
#define M1_PWM_EN_V_GPIO_Port      GPIOE
#define M1_PWM_EN_W_Pin            GPIO_PIN_0
#define M1_PWM_EN_W_GPIO_Port      GPIOE

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
