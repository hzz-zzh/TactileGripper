/**
******************************************************************************
* @file stdrive102bp_driver.h
* @author Motor Control SDK Team, ST Microelectronics
* @brief Header file for the configuration and management of the 
* 		 STDRIVE102BP gate driver.
******************************************************************************
* @attention
*
* <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
* All rights reserved.</center></h2>
*
* This software component is licensed by ST under Ultimate Liberty license
* SLA0044, the "License"; You may not use this file except in compliance with
* the License. You may obtain a copy of the License at:
* www.st.com/SLA0044
*
******************************************************************************
* @ingroup stdrive102bpDriver
*/
/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef STDRIVE102BP_DRIVER_H
#define STDRIVE102BP_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


/* Includes ------------------------------------------------------------------*/
#include <mc_stm_types.h>

#include <stdint.h>
#include <stdbool.h>

#if ((defined STM32H5xx_LL_SPI_H) || (defined STM32H7xx_LL_SPI_H) || (defined STM32U5xx_LL_SPI_H)) 
  #define STDRIVE102BP_spi_TX_ready LL_SPI_IsActiveFlag_TXP
  #define STDRIVE102BP_spi_RX_ready LL_SPI_IsActiveFlag_RXP
#else
  #define STDRIVE102BP_spi_TX_ready LL_SPI_IsActiveFlag_TXE
  #define STDRIVE102BP_spi_RX_ready LL_SPI_IsActiveFlag_RXNE
#endif 

/* constant delays for SPI communication -----------------------------*/
#define STDRIVE102BP_SPI_TIMEOUT 20 // milliseconds (minimum)
#define STDRIVE102BP_SPI_FRAME_DELAY 2 // microseconds (coarse)
#define STDRIVE102BP_SPI_CS_DELAY 1 // microseconds (coarse)

/* status bits definition and diagnotic management.
Use these #define to decode the meaning of each bit, when reading:
STATUS1, STATUS2 using function STDRIVE102BP_GetStatus2()
STATUS1, STATUS3 using function STDRIVE102BP_GetStatus3()
STATUS1, AFE_STATUS using function  STDRIVE102BP_GetAFEStatus() 
----------------------------------------------------------------------*/

// STATUS 1 - Reg. Address 0x00
#define STDRIVE102BP_STATUS1_SPIERROR       (uint8_t)(1)
#define STDRIVE102BP_STATUS1_COMPEV_L       (uint8_t)(1 << 1)
#define STDRIVE102BP_STATUS1_THDS_L         (uint8_t)(1 << 2)
#define STDRIVE102BP_STATUS1_VDDUVLO_L      (uint8_t)(1 << 3)
#define STDRIVE102BP_STATUS1_VCCUVLO_L      (uint8_t)(1 << 4)
#define STDRIVE102BP_STATUS1_VCPUVLO_L      (uint8_t)(1 << 5)
#define STDRIVE102BP_STATUS1_VDSFAULT_L     (uint8_t)(1 << 6)
#define STDRIVE102BP_STATUS1_LOCKED         (uint8_t)(1 << 7)

// STATUS 2 - Reg. Address 0x01
#define STDRIVE102BP_STATUS2_COMP3EV_L      (uint8_t)(1)
#define STDRIVE102BP_STATUS2_COMP2EV_L      (uint8_t)(1 << 1)
#define STDRIVE102BP_STATUS2_COMP1EV_L      (uint8_t)(1 << 2)
#define STDRIVE102BP_STATUS2_THGOOD         (uint8_t)(1 << 3)
#define STDRIVE102BP_STATUS2_VCCGOOD        (uint8_t)(1 << 4)
#define STDRIVE102BP_STATUS2_VCCUVLO        (uint8_t)(1 << 5)
#define STDRIVE102BP_STATUS2_VCPUVLO        (uint8_t)(1 << 6)
#define STDRIVE102BP_STATUS2_RESET          (uint8_t)(1 << 7)

// STATUS 3 - Reg. Address 0x02
#define STDRIVE102BP_STATUS3_VDSLS3         (uint8_t)(1)
#define STDRIVE102BP_STATUS3_VDSLS2         (uint8_t)(1 << 1)
#define STDRIVE102BP_STATUS3_VDSLS1         (uint8_t)(1 << 2)
#define STDRIVE102BP_STATUS3_VDSHS3         (uint8_t)(1 << 3)
#define STDRIVE102BP_STATUS3_VDSHS2         (uint8_t)(1 << 4)
#define STDRIVE102BP_STATUS3_VDSHS1         (uint8_t)(1 << 5)
#define STDRIVE102BP_STATUS3_VDDUVLO        (uint8_t)(1 << 6)
#define STDRIVE102BP_STATUS3_THSD           (uint8_t)(1 << 7)

// AFE_STATUS - Reg. Address 0x03
#define STDRIVE102BP_AFESTATUS_COMP3_FLT    (uint8_t)(1)
#define STDRIVE102BP_AFESTATUS_COMP2_FLT    (uint8_t)(1 << 1)
#define STDRIVE102BP_AFESTATUS_COMP1_FLT    (uint8_t)(1 << 2)
#define STDRIVE102BP_AFESTATUS_COMP3_EV     (uint8_t)(1 << 5)
#define STDRIVE102BP_AFESTATUS_COMP2_EV     (uint8_t)(1 << 6)
#define STDRIVE102BP_AFESTATUS_COMP1_EV     (uint8_t)(1 << 7)


/* nFAULT_SIG_EN - Reg. Address 0x08
Use these #define with the functions STDRIVE102BP_SetFAULTpinSignal()
to select which are the STDRIVE102 events to be mapped on the nFAULT pin.
Use these #define with the functions STDRIVE102BP_GetFAULTpinSignal()
to detect which are the STDRIVE102 events currently mapped on the nFAULT pin.*/

#define STDRIVE102BP_FAULT_SIG_EN_COMP1         (uint8_t)(1)
#define STDRIVE102BP_FAULT_SIG_EN_COMP2         (uint8_t)(1 << 1)
#define STDRIVE102BP_FAULT_SIG_EN_COMP3         (uint8_t)(1 << 2)
#define STDRIVE102BP_FAULT_SIG_EN_VDS           (uint8_t)(1 << 3)
#define STDRIVE102BP_FAULT_SIG_EN_THSD          (uint8_t)(1 << 4)
#define STDRIVE102BP_FAULT_SIG_EN_VDD_UVLO      (uint8_t)(1 << 5)
#define STDRIVE102BP_FAULT_SIG_EN_CP_UVLO       (uint8_t)(1 << 6)
#define STDRIVE102BP_FAULT_SIG_EN_VCC_UVLO      (uint8_t)(1 << 7)
#define STDRIVE102BP_FAULT_SIG_EN_ALL           (uint8_t)(0xFF)

/* FLAG_SIG_EN1 - Reg. Address 0x09 / FLAG_SIG_EN2 - Reg. Address 0x0A
grouped with reg 0x09 in a single 16 bit word.
Use these #define with the functions STDRIVE102BP_SetFLAGpinSignal()
to select which are the STDRIVE102 events to be mapped on the FLAG pin.
Use these #define with the functions STDRIVE102BP_GetFLAGpinSignal()
to detect which are the STDRIVE102 events currently mapped on the FLAG pin.*/

#define STDRIVE102BP_FLAG_SIG_EN1_COMP1_EV       (uint16_t)(1)
#define STDRIVE102BP_FLAG_SIG_EN1_COMP2_EV       (uint16_t)(1 << 1)
#define STDRIVE102BP_FLAG_SIG_EN1_COMP3_EV       (uint16_t)(1 << 2)
#define STDRIVE102BP_FLAG_SIG_EN1_3FG            (uint16_t)(1 << 4)
#define STDRIVE102BP_FLAG_SIG_EN1_VCC_WARN       (uint16_t)(1 << 5)
#define STDRIVE102BP_FLAG_SIG_EN1_THERM_WARN     (uint16_t)(1 << 6)
#define STDRIVE102BP_FLAG_SIG_EN1_SPI_ERROR      (uint16_t)(1 << 7)

#define STDRIVE102BP_FLAG_SIG_EN2_THSD           (uint16_t)(1 << 8)
#define	STDRIVE102BP_FLAG_SIG_EN2_VDD_UVLO       (uint16_t)(1 << 9)
#define	STDRIVE102BP_FLAG_SIG_EN2_VCC_UVLO       (uint16_t)(1 << 10)
#define	STDRIVE102BP_FLAG_SIG_EN2_CP_UVLO        (uint16_t)(1 << 11)
#define	STDRIVE102BP_FLAG_SIG_EN2_VDS            (uint16_t)(1 << 12)
#define STDRIVE102BP_FLAG_SIG_EN_ALL             (uint16_t)(0x1FF7)


/* Commands codes - Reg. 0x1A - Clear commands ------------------*/
#define STDRIVE102BP_CMD_CLEAR_STATUS_L     (uint8_t)(0x90)
#define STDRIVE102BP_CMD_CLEAR_HW_FAULTS    (uint8_t)(0x05)
#define STDRIVE102BP_CMD_CLEAR_ALL          (STDRIVE102BP_CMD_CLEAR_STATUS_L | STDRIVE102BP_CMD_CLEAR_HW_FAULTS)
#define STDRIVE102BP_CMD_CLEAR_RESET        (uint8_t)(0x68)

/* Commands codes - Reg. 0x1B - Reset commands ------------------*/
#define STDRIVE102BP_CMD_GLOBAL_RESET       (uint8_t)(0xA7)

/* Commands codes - Reg. 0x1C - Lock/Unlock commands ------------*/
#define STDRIVE102BP_CMD_LOCK_REGS          (uint8_t)(0x71)
#define STDRIVE102BP_CMD_UNLOCK_REGS        (uint8_t)(0x4B)


/** @addtogroup MCSDK
* @{
*/
/** @addtogroup stdrive102bpDriver
* @{
*/
/** @defgroup stdrive102bpErrCode STDRIVE102BP driver Error Codes
  * @brief Error codes returned by the functions in the STDRIVE102BP firmware library.
  * In case of no errors, the returned error code is 0.
  * @{
*/
#define STDRIVE102BP_SYS_ERROR           (uint16_t)(0x8000)       /**< @brief Unexpected system error in the firmware library. */
#define STDRIVE102BP_SPI_TIMEOUT_ERROR   (uint16_t)(0x4000)       /**< @brief A timeout has occurred int he SPI communication. */
#define STDRIVE102BP_CONFIG_DATA_ERROR   (uint16_t)(0x2000)       /**< @brief A wrong data format was passed to the FW library fucntions. */
#define STDRIVE102BP_USER2_ERROR         (uint16_t)(0x1000)       /**< @brief Error code not assigned, at user disposal. */
#define STDRIVE102BP_USER1_ERROR         (uint16_t)(0x0800)       /**< @brief Error code not assigned, at user disposal. */
#define STDRIVE102BP_COMP1_ERROR         (uint16_t)(0x0400)       /**< @brief The comparator 1 of the STDRIVE102 has been triggered. */
#define STDRIVE102BP_COMP2_ERROR         (uint16_t)(0x0200)       /**< @brief The comparator 2 of the STDRIVE102 has been triggered. */
#define STDRIVE102BP_COMP3_ERROR         (uint16_t)(0x0100)       /**< @brief The comparator 3 of the STDRIVE102 has been triggered. */
#define STDRIVE102BP_VDS_ERROR           (uint16_t)(0x0040)       /**< @brief The VDS monitoring of the STDRIVE102 has been triggered. */
#define STDRIVE102BP_CHG_PUMP_ERROR      (uint16_t)(0x0020)       /**< @brief The Charge pump UVLO of the STDRIVE102 has been triggered. */
#define STDRIVE102BP_VCC_UVLO_ERROR      (uint16_t)(0x0010)       /**< @brief The VCC UVLO of the STDRIVE102 has been triggered. */
#define STDRIVE102BP_VDD_UVLO_ERROR      (uint16_t)(0x0008)       /**< @brief The VDD UVLO of the STDRIVE102 has been triggered. */
#define STDRIVE102BP_THERMAL_SD_ERROR    (uint16_t)(0x0004)       /**< @brief The thermal protection of the STDRIVE102 has been triggered. */
#define STDRIVE102BP_COMPARATORS_ERROR   (uint16_t)(0x0002)       /**< @brief One of the three comparators of the STDRIVE102 has been triggered. */
#define STDRIVE102BP_SPI_CMD_ERROR       (uint16_t)(0x0001)       /**< @brief An SPI error protocol has been detected by the STDRIVE102. */
  /**
  * @}
  */
/**
* @}
*/
/**
* @}
*/

/** @addtogroup MCSDK
* @{
*/
/** @addtogroup stdrive102bpDriver
* @{
*/
/**
* @brief stdrv102bp_Timings_t Enumerator for the TCC: gate driver timing  selection.
*/
typedef enum
{
  BP_L0_280_140ns = 0,        /**< @brief TCC timing level 0. */
  BP_L1_560_280ns,            /**< @brief TCC timing level 1. */
  BP_L2_840_420ns,            /**< @brief TCC timing level 2. */
  BP_L3_1120_560ns,           /**< @brief TCC timing level 3. */
  BP_L4_1400_700ns,           /**< @brief TCC timing level 4. */
  BP_L5_1680_840ns,           /**< @brief TCC timing level 5. */
  BP_L6_1960_980ns,           /**< @brief TCC timing level 6. */
  BP_L7_2240_1120ns,          /**< @brief TCC timing level 7. */
  BP_L8_2520_1260ns,          /**< @brief TCC timing level 8. */
  BP_L9_2800_1400ns,          /**< @brief TCC timing level 9. */
  BP_L10_3080_1540ns,         /**< @brief TCC timing level 10. */
  BP_L11_3360_1680ns,         /**< @brief TCC timing level 11. */
  BP_L12_3800_1900ns,         /**< @brief TCC timing level 12. */
  BP_L13_4400_2200ns,         /**< @brief TCC timing level 13. */
  BP_L14_4800_2400ns,         /**< @brief TCC timing level 14. */
  BP_L15_5400_2700ns          /**< @brief TCC timing level 15. */
} stdrv102bp_Timings_t;

/**
* @brief stdrv102bp_gateLevels_t Enumerator for the IGATE: gate current selection.
*/
typedef enum
{
  BP_L0_25mA = 0,            /**< @brief IGATE current level 0. */
  BP_L1_75mA,                /**< @brief IGATE current level 1. */
  BP_L2_150mA,               /**< @brief IGATE current level 2. */
  BP_L3_250mA,               /**< @brief IGATE current level 3. */
  BP_L4_300mA,               /**< @brief IGATE current level 4. */
  BP_L5_350mA,               /**< @brief IGATE current level 5. */
  BP_L6_400mA,               /**< @brief IGATE current level 6. */
  BP_L7_500mA,               /**< @brief IGATE current level 7. */
  BP_L8_550mA,               /**< @brief IGATE current level 8. */
  BP_L9_600mA,               /**< @brief IGATE current level 9. */
  BP_L10_650mA,              /**< @brief IGATE current level 10. */
  BP_L11_700mA,              /**< @brief IGATE current level 11. */
  BP_L12_800mA,              /**< @brief IGATE current level 12. */
  BP_L13_850mA,              /**< @brief IGATE current level 13. */
  BP_L14_900mA,              /**< @brief IGATE current level 14. */
  BP_L15_1000mA              /**< @brief IGATE current level 15. */
} stdrv102bp_gateLevels_t;

/**
* @brief stdriv102bp_eqSel_t Enumerator for the EQ mode of the gate drivers.
*/
typedef enum
{
  BP_IGATE_NOT_EQ = 0,                 /**< @brief Selects IGATE,off = 2IGATE,on and TCC,on = 2TCC,off. */
  BP_IGATE_EQ_TCC_HIGH_RANGE = 1,      /**< @brief Selects IGATE,off = IGATE,on and TCC,on = TCC,off (high range TCC). */
  BP_IGATE_EQ_TCC_LOW_RANGE = 3        /**< @brief Selects IGATE,off = IGATE,on and TCC,on = TCC,off (low range TCC). */
} stdriv102bp_eqSel_t;

/**
* @brief stdrv102bp_in_mode_t Enumerator for the digital input mode of the gate drivers.
*/
typedef enum
{
  BP_EN_IN = 0,               /**< @brief Selects ENx/INx mode */
  BP_DIRECT_TCCWAIT = 4,      /**< @brief Selects INHx/INLx direct mode, with insertion of TCC wait. */
  BP_DIRECT_NO_WAIT = 6,      /**< @brief Selects INHx/INLx direct mode, without insertion of TCC wait. */
  BP_NO_INTERLOCKING = 7      /**< @brief Selects INHx/INLx direct mode, with interlocking disabled. */
} stdrv102bp_in_mode_t;

/**
* @brief STDRIVE102BP_GateDrivers_t Structure for the gate drivers configuration.
* This structure sets/gets the parameters of the functions STDRIVE102BP_SetGateDrivers() and STDRIVE102BP_GetGateDrivers().
*/
typedef struct
{
  stdrv102bp_gateLevels_t igate_on;      /**< @brief IGATE: gate current selection. */
  stdrv102bp_Timings_t tcc;              /**< @brief TCC: gate driver timing  selection. */
  stdriv102bp_eqSel_t eq_mode_sel;       /**< @brief EQ mode of the gate drivers. */
  stdrv102bp_in_mode_t drv_mode_sel;     /**< @brief Digital input mode of the gate drivers. */
} STDRIVE102BP_GateDrivers_t;

/**
* @brief stdrv102bp_uvlo_vals_t Enumerator for the undervoltage lock-out threshold selection.
*/
typedef enum
{
  BP_UVLO_5V5 = 0,       /**< @brief Selects the low threshold for the VCC and charge pump UVLO. */
  BP_UVLO_7V8            /**< @brief Selects the high threshold for the VCC and charge pump UVLO. */
} stdrv102bp_uvlo_vals_t;

/**
* @brief stdrv102bp_pwrgood_vals_t Enumerator for the VCC power good threhsold selection.
*/
typedef enum
{
  BP_PGOOD_7V75 = 0,     /**< @brief Selects the low threshold for the VCC power good warning. */
  BP_PGOOD_9V65          /**< @brief Selects the high threshold for the VCC power good warning. */
} stdrv102bp_pwrgood_vals_t;

/**
* @brief STDRIVE102BP_uvlo_t Structure for the UVLO and power good configuration.
* This structure sets/gets the parameters of the functions STDRIVE102BP_SetUvlo() and STDRIVE102BP_GetUvlo().
*/
typedef struct
{
  stdrv102bp_uvlo_vals_t uvlo_sel;           /**< @brief Selects the threshold for the VCC and charge pump UVLO. */
  stdrv102bp_pwrgood_vals_t pwrgood_sel;     /**< @brief Selects the threshold for the VCC power good. */
} STDRIVE102BP_uvlo_t;

/**
* @brief stdrv102bp_vdsDisTime_t Enumerator to select the disable time of the VDS monitoring.
*/
typedef enum
{
  BP_VDS_TDIS_500us = 0,           /**< @brief Disable time: 500 microseconds. */
  BP_VDS_TDIS_750us = 1,           /**< @brief Disable time: 750 microseconds. */
  BP_VDS_TDIS_1ms = 2,             /**< @brief Disable time: 1 millisecond. */
  BP_VDS_TDIS_2ms = 3,             /**< @brief Disable time: 2 milliseconds. */
  BP_VDS_TDIS_3ms = 4,             /**< @brief Disable time: 3 milliseconds. */
  BP_VDS_TDIS_5ms = 5,             /**< @brief Disable time: 5 milliseconds. */
  BP_VDS_TDIS_10ms = 6,            /**< @brief Disable time: 10 milliseconds. */
  BP_VDS_TDIS_20ms = 7             /**< @brief Disable time: 20 milliseconds. */
} stdrv102bp_vdsDisTime_t;

/**
* @brief stdrv102bp_vdsDeglitch_t Enumerator to select the deglitch time of the VDS monitoring filter.
*/
typedef enum 
{
  BP_VDS_DGT_9us = 0,           /**< @brief Deglitch time: 9 microseconds. */
  BP_VDS_DGT_8us = 1,           /**< @brief Deglitch time: 8 microseconds. */
  BP_VDS_DGT_7us = 2,           /**< @brief Deglitch time: 7 microseconds. */
  BP_VDS_DGT_6us = 3,           /**< @brief Deglitch time: 6 microseconds. */
  BP_VDS_DGT_4us5 = 4,          /**< @brief Deglitch time: 4.5 microseconds. */
  BP_VDS_DGT_3us5 = 5,          /**< @brief Deglitch time: 3.5 microseconds. */
  BP_VDS_DGT_2us5 = 6           /**< @brief Deglitch time: 2.5 microseconds. */
} stdrv102bp_vdsDeglitch_t;	

/**
* @brief STDRIVE102BP_VDSmonitor_t Structure to configure the VDS monitoring filter.
* This structure sets/gets the parameters of the functions STDRIVE102BP_SetVDSmonitor() and STDRIVE102BP_GetVDSmonitor().
*/
typedef struct
{
  bool vds_soft_off;                      /**< @brief if true, enables the soft-off feature of the VDS monitoring. */
  bool vds_count_enable;                  /**< @brief if true, enables the counter of the VDS monitoring events. */
  stdrv102bp_vdsDisTime_t vds_disable;    /**< @brief Sets the disable time after a VDS monitoring event. */
  stdrv102bp_vdsDeglitch_t vds_deglitch;  /**< @brief Sets the deglitch time of the VDS monitoring filter. */
  uint8_t vds_counter;                    /**< @brief Sets the numebr of counts of the VDS monitoring. When it reaches 0, a futher event generates a latch. */
} STDRIVE102BP_VDSmonitor_t;

/**
* @brief stdrv102bp_pga_gain_t Enumerator to select the gain of the PGA in the analog front-end of the STDRIVE102.
*/
typedef enum
{
  BP_PGA_GAIN_4 = 0,            /**< @brief PGA gain = 4. */
  BP_PGA_GAIN_8 = 1,            /**< @brief PGA gain = 8. */
  BP_PGA_GAIN_16 = 2,           /**< @brief PGA gain = 16. */
  BP_PGA_GAIN_32 = 3            /**< @brief PGA gain = 32. */
} stdrv102bp_pga_gain_t;

/**
* @brief stdrv102bp_inp_config_t Enumerator to select which is the positive input of the comparator in the analog front-end of the STDRIVE102.
*/
typedef	enum
{
  BP_COMP_DIRECT_INP = 0,       /**< @brief Comparator input connected to the posisive input of the PGA. */
  BP_COMP_PGA_OUTPUT = 1,       /**< @brief Comparator input connected to the output of the PGA. */
  BP_COMP_AUX_INPUT = 2         /**< @brief Comparator input is the auxiliary input (valid only for AFE channel 3. */
} stdrv102bp_inp_config_t;

/**
* @brief stdrv102bp_inn_config_t Enumerator to select which is the negative input of the comparator in the analog front-end of the STDRIVE102.
*/
typedef enum
{
  BP_COMP_TH_INTERNAL = 0,         /**< @brief Use the internal adjustable threshold. */
  BP_COMP_TH_EXT_CREF = 1          /**< @brief Use the voltage reference present on the CREF pin of the STDRIVE102. */
} stdrv102bp_inn_config_t;

/**
* @brief stdrv102bp_compDeglitch_t Enumerator to select the deglitch time of the comparators's filter.
*/
typedef enum
{
  BP_COMP_DGT_disabled = 0,       /**< @brief Deglitch filter is disabled. */
  BP_COMP_DGT_600ns = 1,          /**< @brief Deglitch filter is 600 nanoseconds. */
  BP_COMP_DGT_1300ns = 2,         /**< @brief Deglitch filter is 1300 nanoseconds. */
  BP_COMP_DGT_2500ns = 3          /**< @brief Deglitch filter is 2500 nanoseconds. */
} stdrv102bp_compDeglitch_t;

/**
* @brief stdrv102bp_compDisTime_t Enumerator to select the disable time after a comparator's event.
*/
typedef enum
{
  BP_COMP_DIST_0us = 0,          /**< @brief No Disable time. */
  BP_COMP_DIST_10us = 1,         /**< @brief Disable time is 10 microseconds. */
  BP_COMP_DIST_15us = 2,         /**< @brief Disable time is 15 microseconds. */
  BP_COMP_DIST_20us = 3,         /**< @brief Disable time is 20 microseconds. */
  BP_COMP_DIST_25us = 4,         /**< @brief Disable time is 25 microseconds. */
  BP_COMP_DIST_35us = 5,         /**< @brief Disable time is 35 microseconds. */
  BP_COMP_DIST_45us = 6,         /**< @brief Disable time is 45 microseconds. */
  BP_COMP_DIST_55us = 7,         /**< @brief Disable time is 55 microseconds. */
  BP_COMP_DIST_75us = 8,         /**< @brief Disable time is 75 microseconds. */
  BP_COMP_DIST_110us = 9,        /**< @brief Disable time is 110 microseconds. */
  BP_COMP_DIST_160us = 10,       /**< @brief Disable time is 160 microseconds. */
  BP_COMP_DIST_220us = 11,       /**< @brief Disable time is 220 microseconds. */
  BP_COMP_DIST_330us = 12,       /**< @brief Disable time is 330 microseconds. */
  BP_COMP_DIST_550us = 13,       /**< @brief Disable time is 550 microseconds. */
  BP_COMP_DIST_880us = 14,       /**< @brief Disable time is 880 microseconds. */
  BP_COMP_DIST_1100us = 15       /**< @brief Disable time is 1100 microseconds. */
} stdrv102bp_compDisTime_t;

/**
* @brief STDRIVE102BP_AFEchannel_t Structure to configure the specific AFE channel.
* This structure sets/gets the parameters of the functions STDRIVE102BP_SetAFEchannel() and STDRIVE102BP_GetAFEchannel().
*/
typedef struct
{
  bool pga_enable;                           /**< @brief if true, enables the PGA of the specific AFE channel. */
  bool comp_enable;                          /**< @brief if true, enables the comparator of the specific AFE channel. */
  stdrv102bp_inp_config_t comp_pos_input;    /**< @brief configures the positive input of the comparator. */
  stdrv102bp_inn_config_t comp_neg_input;    /**< @brief configures the negative input of the comparator. */
  bool comp_inversion;                       /**< @brief if true, inverts the output of the comparator. */
  stdrv102bp_pga_gain_t pga_gain;            /**< @brief configure the gain of the PGA. */
  bool comp_soft_off;                        /**< @brief if true, enables the soft-off feature when the comparator is triggered. */
  bool comp_fault_enable;                    /**< @brief if true, enables the fault management for the specific AFE channel. */
  bool comp_counter_enable;                  /**< @brief if true, enables the event counter for the specific AFE comparator. */
  stdrv102bp_compDeglitch_t comp_deglitch;   /**< @brief configures the degltich time of the comparator's filter. */
  stdrv102bp_compDisTime_t comp_disable;     /**< @brief configures the disable time after a comparator event. */
  uint8_t comp_counter;                      /**< @brief Sets the numebr of counts of the AFE comparator. When it reaches 0, a futher event generates a latch. */
} STDRIVE102BP_AFEchannel_t;

/**
* @brief stdrv102bp_AFE_Ch_index_t Enumerator to select which AFE channel to configure.
*/
typedef enum
{
  BP_AFE_CHANNEL_1 = 0,            /**< @brief configure AFE channel 1. */
  BP_AFE_CHANNEL_2 = 1,            /**< @brief configure AFE channel 2. */
  BP_AFE_CHANNEL_3 = 2,            /**< @brief configure AFE channel 3. */
  BP_AFE_ALL_CHANNELS = 3          /**< @brief configure all the AFE channels in the same configuration. */
} stdrv102bp_AFE_Ch_index_t;

/**
* @brief stdrv102bp_counter_index_t Enumerator to select which counter must be written or read.
* Use this enumerator with the functions STDRIVE102BP_SetCounter() and STDRIVE102BP_GetCounter().
*/
typedef enum
{
  BP_VDS_COUNTER = 0x07,           /**< @brief Selects the coutner of the VDS monitoring */
  BP_AFE_CH1_COUNTER = 0x13,       /**< @brief Selects the coutner of the AFE channel 1 */
  BP_AFE_CH2_COUNTER = 0x14,       /**< @brief Selects the coutner of the AFE channel 2 */
  BP_AFE_CH3_COUNTER = 0x15        /**< @brief Selects the coutner of the AFE channel 3 */
} stdrv102bp_counter_index_t;

/**
* @brief stdrv102bp_pga_offset_t Enumerator to select the offset present on the PGA output.
* According to the offset selected it is possible to read unipolar or bipolar signals on the PGA inputs.
*/
typedef enum
{
  BP_PGA_NO_OFFSET = 0,         /**< @brief No offset on PGA output. Suitable for unipolar signals */
  BP_PGA_HALF_VDD = 1           /**< @brief No offset on PGA output. Suitable for bipolar signals */
} stdrv102bp_pga_offset_t;

/**
* @brief stdrv102bp_compInternalThreshold_t Enumerator to select the level of the internal threshold applied to the AFE comparators.
*/
typedef enum
{
  BP_COMP_TH_INT_100mV = 0,         /**< @brief internal threshold is 100 mV */
  BP_COMP_TH_INT_250mV = 1,         /**< @brief internal threshold is 250 mV */
  BP_COMP_TH_INT_300mV = 2,         /**< @brief internal threshold is 300 mV */
  BP_COMP_TH_INT_500mV = 3,         /**< @brief internal threshold is 500 mV */
  BP_COMP_TH_INT_600mV = 4,         /**< @brief internal threshold is 600 mV */
  BP_COMP_TH_INT_1V2 = 5,           /**< @brief internal threshold is 1200 mV */
  BP_COMP_TH_INT_1V65 = 6,          /**< @brief internal threshold is VDD/2 */
  BP_COMP_TH_INT_1V85 = 7,          /**< @brief internal threshold is 11/20 VDD */
  BP_COMP_TH_INT_2V25 = 8,          /**< @brief internal threshold is 14/20 VDD */
  BP_COMP_TH_INT_2V85 = 9,          /**< @brief internal threshold is 17/20 VDD */
  BP_COMP_TH_INT_3V15 = 10          /**< @brief internal threshold is 19/20 VDD */
} stdrv102bp_compInternalThreshold_t;

/**
* @brief stdrv102bp_safeStateSel_t Enumerator to select the safe state of the power stage after one of the comparators is triggered.
*/	
typedef enum
{
  BP_MOS_ALL_OFF = 0,          /**< @brief All the MOSFETs in the power stage are turned off. */
  BP_MOS_SINGLE_CH_OFF = 1,    /**< @brief Only the half-bridge corresponding to the triggered comparator is turned off. */
  BP_MOS_HS_ONLY_OFF = 2,      /**< @brief Only the high side MOSFETs in the power stage are turned off. */
  BP_MOS_FAULT_BYPASS = 3      /**< @brief No MOSFETs in the power stage is turned off. */
} stdrv102bp_safeStateSel_t;

/**
* @brief STDRIVE102BP_AFEcommon_t Structure to configure the common configuration of the AFE.
* This structure sets/gets the parameters of the functions STDRIVE102BP_SetAFEcommon() and STDRIVE102BP_GetAFEcommon().
*/	
typedef struct
{
  stdrv102bp_pga_offset_t pga_ref_sel;                    /**< @brief Configures the output offset of the three PGAs. */
  stdrv102bp_compInternalThreshold_t comp_internal_th;    /**< @brief Configures the internal threshold for the three comparators. */
  stdrv102bp_safeStateSel_t driver_safe_state;            /**< @brief Configures the safe state of the power stage after a comparator event. */
} STDRIVE102BP_AFEcommon_t;

/**
* @brief STDRIVE102BP_hw_interface_t Structure to define the SPI interface of the STDRIVE102.
*/
typedef struct
{
  SPI_TypeDef *spi_hdl;               /**< @brief SPI handler. */
  GPIO_TypeDef *nCS_gpio_port;        /**< @brief SPI nCS/nSS GPIO port (SW managed). */
  uint32_t nCS_gpio_pin;              /**< @brief SPI nCS/nSS GPIO pin (SW managed). */
} STDRIVE102BP_hw_interface_t;

/**
* @brief STDRIVE102BP_defaultParams_t This structure is used to manage all the configurations of the STDRIVE102.
*/
typedef struct
{
  STDRIVE102BP_hw_interface_t spi;             /**< @brief SPI configuration structure. */
  STDRIVE102BP_GateDrivers_t gd;               /**< @brief Gate drivers configuration structure. */
  STDRIVE102BP_uvlo_t uv;                      /**< @brief UVLO thresholds configuration structure. */
  STDRIVE102BP_VDSmonitor_t vdsm;              /**< @brief VDS monitoring configuration structure. */
  STDRIVE102BP_AFEchannel_t afe_ch1;           /**< @brief AFE channel 1 configuration structure. */
  STDRIVE102BP_AFEchannel_t afe_ch2;           /**< @brief AFE channel 2 configuration structure. */
  STDRIVE102BP_AFEchannel_t afe_ch3;           /**< @brief AFE channel 3 configuration structure. */
  STDRIVE102BP_AFEcommon_t afe_cm;             /**< @brief AFE common configuration structure. */
}STDRIVE102BP_defaultParams_t;


/**
* @brief STDRIVE102BP_Handle_t handler definition. 
* The handler is passed to all the library function to manage a spedific instance of the STDRIVE102BP.
*/
typedef struct
{
	const STDRIVE102BP_defaultParams_t *defaultParams;
}STDRIVE102BP_Handle_t;


/* Exported functions ------------------------------------------------------- */

__weak void STDRIVE102BP_Init(STDRIVE102BP_Handle_t *pHandler, const STDRIVE102BP_defaultParams_t *params);
            
__weak void STDRIVE102BP_FaultManagement( STDRIVE102BP_Handle_t *pHandler);
__weak void STDRIVE102BP_StartManagement( STDRIVE102BP_Handle_t *pHandler);
__weak void STDRIVE102BP_StopManagement( STDRIVE102BP_Handle_t *pHandler);
__weak void STDRIVE102BP_MF_Management( STDRIVE102BP_Handle_t *pHandler);

uint16_t STDRIVE102BP_ReadReg (STDRIVE102BP_Handle_t *pHandler, uint8_t reg_addr, uint8_t *reg_value, uint8_t *status1);
uint16_t STDRIVE102BP_WriteReg (STDRIVE102BP_Handle_t *pHandler, uint8_t reg_addr, uint8_t reg_value, uint8_t *status1, uint8_t *status2);

uint16_t STDRIVE102BP_SetGateDrivers (STDRIVE102BP_Handle_t *pHandler, STDRIVE102BP_GateDrivers_t gd_par);
uint16_t STDRIVE102BP_GetGateDrivers (STDRIVE102BP_Handle_t *pHandler, STDRIVE102BP_GateDrivers_t *gd_par);

uint16_t STDRIVE102BP_SetUvlo (STDRIVE102BP_Handle_t *pHandler, STDRIVE102BP_uvlo_t uv_par);
uint16_t STDRIVE102BP_GetUvlo (STDRIVE102BP_Handle_t *pHandler, STDRIVE102BP_uvlo_t *uv_par);

uint16_t STDRIVE102BP_SetVDSmonitor (STDRIVE102BP_Handle_t *pHandler, STDRIVE102BP_VDSmonitor_t vds_par);
uint16_t STDRIVE102BP_GetVDSmonitor (STDRIVE102BP_Handle_t *pHandler, STDRIVE102BP_VDSmonitor_t *vds_par);

uint16_t STDRIVE102BP_SetAFEchannel (STDRIVE102BP_Handle_t *pHandler, stdrv102bp_AFE_Ch_index_t ch, STDRIVE102BP_AFEchannel_t afe_par);
uint16_t STDRIVE102BP_GetAFEchannel (STDRIVE102BP_Handle_t *pHandler, stdrv102bp_AFE_Ch_index_t ch, STDRIVE102BP_AFEchannel_t *afe_par);
 
uint16_t STDRIVE102BP_SetAFEcommon (STDRIVE102BP_Handle_t *pHandler, STDRIVE102BP_AFEcommon_t afe_par);
uint16_t STDRIVE102BP_GetAFEcommon (STDRIVE102BP_Handle_t *pHandler, STDRIVE102BP_AFEcommon_t *afe_par);

uint16_t STDRIVE102BP_AFE_Enable (STDRIVE102BP_Handle_t *pHandler);
uint16_t STDRIVE102BP_AFE_Disable (STDRIVE102BP_Handle_t *pHandler);

uint16_t STDRIVE102BP_SetCounter (STDRIVE102BP_Handle_t *pHandler, stdrv102bp_counter_index_t cnt, uint8_t value);
uint16_t STDRIVE102BP_GetCounter (STDRIVE102BP_Handle_t *pHandler, stdrv102bp_counter_index_t cnt, uint8_t *value);

uint16_t STDRIVE102BP_SetFAULTpinSignal (STDRIVE102BP_Handle_t *pHandler, uint8_t val);
uint16_t STDRIVE102BP_GetFAULTpinSignal (STDRIVE102BP_Handle_t *pHandler, uint8_t *val);

uint16_t STDRIVE102BP_SetFLAGpinSignal (STDRIVE102BP_Handle_t *pHandler, uint16_t val);
uint16_t STDRIVE102BP_GetFLAGpinSignal (STDRIVE102BP_Handle_t *pHandler, uint16_t *val);

uint16_t STDRIVE102BP_GetStatus2 (STDRIVE102BP_Handle_t *pHandler, uint8_t *status1, uint8_t *status2);
uint16_t STDRIVE102BP_GetStatus3 (STDRIVE102BP_Handle_t *pHandler, uint8_t *status3, uint8_t *status1);
uint16_t STDRIVE102BP_GetAFEStatus (STDRIVE102BP_Handle_t *pHandler, uint8_t *afe_status, uint8_t *status1);

uint16_t STDRIVE102BP_ClearFault_Cmd (STDRIVE102BP_Handle_t *pHandler, uint8_t cmd);
uint16_t STDRIVE102BP_DevReset_Cmd (STDRIVE102BP_Handle_t *pHandler);
uint16_t STDRIVE102BP_Lock_Cmd (STDRIVE102BP_Handle_t *pHandler);
uint16_t STDRIVE102BP_Unlock_Cmd (STDRIVE102BP_Handle_t *pHandler);


/**
  * @}
  */

/**
  * @}
  */


#ifdef __cplusplus
}
#endif /* __cpluplus */


#endif /* STDRIVE102BP_DRIVER_H */

/************************ (C) COPYRIGHT 2025 STMicroelectronics *****END OF FILE****/
