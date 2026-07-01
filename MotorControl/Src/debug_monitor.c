#include "debug_monitor.h"

#include "cmsis_os2.h"
#include "main.h"
#include "gripper_motor_service.h"
#include "mc_api.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define DEBUG_POLL_TICKS       200U  /* 100 ms at the 2 kHz RTOS tick. */
#define DEBUG_STATUS_DIVIDER   5U    /* Full status every 500 ms. */
#define DEBUG_TX_TIMEOUT_MS    30U

extern UART_HandleTypeDef huart2;

static osThreadId_t debugTaskHandle;

static const char *DebugMonitor_StateName(GripperMotorState_t state)
{
  switch (state)
  {
    case GRIPPER_MOTOR_BOOT:         return "BOOT";
    case GRIPPER_MOTOR_PRECHECK:     return "PRECHECK";
    case GRIPPER_MOTOR_ALIGNING:     return "ALIGNING";
    case GRIPPER_MOTOR_HOMING_OPEN:  return "HOME_OPEN";
    case GRIPPER_MOTOR_HOMING_CLOSE: return "HOME_CLOSE";
    case GRIPPER_MOTOR_MOVE_SAFE:    return "MOVE_SAFE";
    case GRIPPER_MOTOR_READY:        return "READY";
    case GRIPPER_MOTOR_POSITIONING:  return "POSITIONING";
    case GRIPPER_MOTOR_STOPPED:      return "STOPPED";
    case GRIPPER_MOTOR_FAULT:        return "FAULT";
    default:                         return "UNKNOWN";
  }
}

static int32_t DebugMonitor_ToMilli(float value)
{
  float scaled = value * 1000.0f;
  return (scaled >= 0.0f) ? (int32_t)(scaled + 0.5f) : (int32_t)(scaled - 0.5f);
}

static void DebugMonitor_Write(const char *text)
{
  size_t length = strlen(text);
  if (length > 0U)
  {
    (void)HAL_UART_Transmit(&huart2, (const uint8_t *)text, (uint16_t)length,
                            DEBUG_TX_TIMEOUT_MS);
  }
}

static void DebugMonitor_Task(void *argument)
{
  uint32_t next = osKernelGetTickCount();
  uint32_t statusDivider = 0U;
  GripperMotorState_t previousState = GRIPPER_MOTOR_BOOT;
  uint32_t previousFaults = UINT32_MAX;
  char line[224];
  (void)argument;

  DebugMonitor_Write("\r\n[BOOT] STM32H723 tactile gripper\r\n"
                     "[BOOT] UART2 PD5/PD6 115200-8-N-1\r\n");

  for (;;)
  {
    GripperMotorStatus_t status;
    GripperMotor_GetStatus(&status);

    if ((status.state != previousState) || (status.faults != previousFaults))
    {
      (void)snprintf(line, sizeof(line),
                     "[STATE] app=%s(%u) mc=%u homed=%u fault=0x%08lX "
                     "mc_now=0x%04X mc_seen=0x%04X\r\n",
                     DebugMonitor_StateName(status.state), (unsigned int)status.state,
                     (unsigned int)MC_GetSTMStateMotor1(), status.homed ? 1U : 0U,
                     (unsigned long)status.faults, (unsigned int)status.mc_faults,
                     (unsigned int)status.mc_occurred_faults);
      DebugMonitor_Write(line);
      previousState = status.state;
      previousFaults = status.faults;
    }

    if (++statusDivider >= DEBUG_STATUS_DIVIDER)
    {
      statusDivider = 0U;
      (void)snprintf(line, sizeof(line),
                     "[STAT] pos=%d target=%d raw=%ld open=%ld close=%ld "
                     "speed_mrpm=%ld iq_mA=%ld ibus_mA=%ld vbus=%u\r\n",
                     (int)status.position_permille, (int)status.target_permille,
                     (long)status.position_count, (long)status.open_count,
                     (long)status.close_count,
                     (long)DebugMonitor_ToMilli(status.speed_rpm),
                     (long)DebugMonitor_ToMilli(status.iq_a),
                     (long)DebugMonitor_ToMilli(status.bus_current_a),
                     (unsigned int)status.bus_voltage_v);
      DebugMonitor_Write(line);

      (void)snprintf(line, sizeof(line),
                     "[ENC] frame=0x%04X good_frame=0x%04X angle=%u "
                     "rx_crc=0x%X calc_crc=0x%X frames=%lu valid=%lu "
                     "crc_err=%lu spi_err=%lu consec=%u reliable=%u "
                     "cs=%u miso=%u\r\n",
                     (unsigned int)status.encoder_last_frame,
                     (unsigned int)status.encoder_last_good_frame,
                     (unsigned int)(status.encoder_last_frame >> 4),
                     (unsigned int)status.encoder_received_crc,
                     (unsigned int)status.encoder_calculated_crc,
                     (unsigned long)status.encoder_frames,
                     (unsigned long)status.encoder_valid_frames,
                     (unsigned long)status.encoder_crc_errors,
                     (unsigned long)status.encoder_spi_errors,
                     (unsigned int)status.encoder_consecutive_errors,
                     status.encoder_reliable ? 1U : 0U,
                     HAL_GPIO_ReadPin(KTH7812_NSS_GPIO_Port, KTH7812_NSS_Pin) == GPIO_PIN_SET ? 1U : 0U,
                     HAL_GPIO_ReadPin(KTH7812_MISO_GPIO_Port, KTH7812_MISO_Pin) == GPIO_PIN_SET ? 1U : 0U);
      DebugMonitor_Write(line);

      (void)snprintf(line, sizeof(line),
                     "[ADC] ibus_raw=%u ibus_mA=%ld vbus_raw=%u vbus=%u ntc_raw=%u\r\n",
                     (unsigned int)status.bus_current_raw,
                     (long)DebugMonitor_ToMilli(status.bus_current_a),
                     (unsigned int)status.bus_voltage_raw,
                     (unsigned int)status.bus_voltage_v,
                     (unsigned int)status.temperature_raw);
      DebugMonitor_Write(line);
    }

    next += DEBUG_POLL_TICKS;
    (void)osDelayUntil(next);
  }
}

void DebugMonitor_CreateTask(void)
{
  const osThreadAttr_t attributes = {
    .name = "debugUart2",
    .stack_size = 512U * 4U,
    .priority = osPriorityLow
  };

  debugTaskHandle = osThreadNew(DebugMonitor_Task, NULL, &attributes);
  (void)debugTaskHandle;
}
