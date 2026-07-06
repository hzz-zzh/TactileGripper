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

/* KTH7812绝对编码器使用SPI1，PA15由SPI硬件NSS自动控制。 */
#define KTH7812_SCK_Pin            GPIO_PIN_3
#define KTH7812_SCK_GPIO_Port      GPIOB
#define KTH7812_MISO_Pin           GPIO_PIN_4
#define KTH7812_MISO_GPIO_Port     GPIOB
#define KTH7812_MOSI_Pin           GPIO_PIN_5
#define KTH7812_MOSI_GPIO_Port     GPIOB
#define KTH7812_NSS_Pin            GPIO_PIN_15
#define KTH7812_NSS_GPIO_Port      GPIOA

/* USART1 pins. 当前调试控制台使用PA9/PA10，Motor Pilot/ASPEP暂时停用。 */
#define MOTOR_PILOT_TX_Pin         GPIO_PIN_9
#define MOTOR_PILOT_TX_GPIO_Port   GPIOA
#define MOTOR_PILOT_RX_Pin         GPIO_PIN_10
#define MOTOR_PILOT_RX_GPIO_Port   GPIOA

/* USART2端子保留，当前不作为调试控制台。 */
#define DEBUG_UART_TX_Pin          GPIO_PIN_5
#define DEBUG_UART_TX_GPIO_Port    GPIOD
#define DEBUG_UART_RX_Pin          GPIO_PIN_6
#define DEBUG_UART_RX_GPIO_Port    GPIOD

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
