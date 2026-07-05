/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mc_tasks.h"
#include "mc_config.h"
#include "parameters_conversion.h"
#include "mcp_config.h"
#include "motorcontrol.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart2;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  TSK_HardwareFaultTask();
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM6 global interrupt, DAC1_CH1 and DAC1_CH2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

void ADC_IRQHandler(void)
{
  if (LL_ADC_IsActiveFlag_AWD1(ADC2) != 0U)
  {
    LL_ADC_ClearFlag_AWD1(ADC2);
    PWMC_SwitchOffPWM(&PWM_Handle_M1._Super);
    MCI_FaultProcessing(&Mci[M1], MC_OVER_CURR, 0);
  }
  if (LL_ADC_IsActiveFlag_JEOS(ADC1) != 0U)
  {
    LL_ADC_ClearFlag_JEOS(ADC1);
  }
  TSK_HighFrequencyTask();
}

void TIM1_UP_IRQHandler(void)
{
  LL_TIM_ClearFlag_UPDATE(TIM1);
  R3_2_TIMx_UP_IRQHandler(&PWM_Handle_M1);
}

void TIM1_BRK_IRQHandler(void)
{
  if (LL_TIM_IsActiveFlag_BRK(TIM1) != 0U)
  {
    LL_TIM_ClearFlag_BRK(TIM1);
    PWMC_DP_Handler(&PWM_Handle_M1._Super);
  }
  if (LL_TIM_IsActiveFlag_BRK2(TIM1) != 0U)
  {
    LL_TIM_ClearFlag_BRK2(TIM1);
    PWMC_OVP_Handler(&PWM_Handle_M1._Super, TIM1);
  }
}

void USART1_IRQHandler(void)
{
  uint32_t activeIdleFlag;
  uint32_t isEnabledIdleFlag;

  if (LL_USART_IsActiveFlag_TC(USARTA) != 0U)
  {
    LL_DMA_DisableStream(DMA_TX_A, DMACH_TX_A);
    LL_USART_ClearFlag_TC(USARTA);
    ASPEP_HWDataTransmittedIT(&aspepOverUartA);
  }

  if ((LL_USART_IsActiveFlag_ORE(USARTA) || LL_USART_IsActiveFlag_FE(USARTA) ||
       LL_USART_IsActiveFlag_NE(USARTA)) && LL_USART_IsEnabledIT_ERROR(USARTA))
  {
    LL_USART_DisableIT_ERROR(USARTA);
    LL_USART_EnableIT_IDLE(USARTA);
  }

  activeIdleFlag = LL_USART_IsActiveFlag_IDLE(USARTA);
  isEnabledIdleFlag = LL_USART_IsEnabledIT_IDLE(USARTA);
  if ((activeIdleFlag & isEnabledIdleFlag) != 0U)
  {
    LL_USART_ClearFlag_FE(USARTA);
    LL_USART_ClearFlag_ORE(USARTA);
    LL_USART_ClearFlag_NE(USARTA);
    LL_USART_DisableIT_IDLE(USARTA);
    LL_USART_EnableIT_ERROR(USARTA);
    LL_USART_DisableDMAReq_RX(USARTA);
    (void)LL_USART_ReceiveData8(USARTA);
    LL_USART_EnableDMAReq_RX(USARTA);
    LL_DMA_ClearFlag_TE(DMA_RX_A, DMACH_RX_A);
    LL_DMA_ClearFlag_TC(DMA_RX_A, DMACH_RX_A);
    ASPEP_HWReset(&aspepOverUartA);
  }
}

void USART2_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart2);
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
