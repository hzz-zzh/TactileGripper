#ifndef MOTOR_CONTROL_SERVICE_H
#define MOTOR_CONTROL_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  MOTOR_CONTROL_STATE_BOOT = 0,
  MOTOR_CONTROL_STATE_IDLE,
  MOTOR_CONTROL_STATE_STARTING,
  MOTOR_CONTROL_STATE_RUNNING,
  MOTOR_CONTROL_STATE_STOPPING,
  MOTOR_CONTROL_STATE_FAULT
} MotorControlState_t;

typedef enum
{
  MOTOR_CONTROL_FAULT_NONE        = 0,
  MOTOR_CONTROL_FAULT_ENCODER     = (1UL << 0),
  MOTOR_CONTROL_FAULT_MCSDK       = (1UL << 1),
  MOTOR_CONTROL_FAULT_BUS_VOLTAGE = (1UL << 2),
  MOTOR_CONTROL_FAULT_OVER_CURRENT = (1UL << 3),
  MOTOR_CONTROL_FAULT_START_TIMEOUT = (1UL << 4)
} MotorControlFault_t;

typedef struct
{
  MotorControlState_t state;
  uint32_t faults;
  int32_t position_count;
  float speed_ref_rpm;
  float speed_rpm;
  float iq_ref_a;
  float id_ref_a;
  float iq_a;
  float id_a;
  float bus_current_a;
  uint16_t bus_current_raw;
  uint16_t bus_voltage_raw;
  uint16_t temperature_raw;
  uint16_t bus_voltage_v;
  uint16_t mc_faults;
  uint16_t mc_occurred_faults;
  uint16_t encoder_last_frame;
  uint32_t encoder_frames;
  uint32_t encoder_spi_errors;
  uint8_t encoder_consecutive_errors;
  bool encoder_reliable;
} MotorControlStatus_t;

void MotorControlService_CreateTasks(void);
bool MotorControlService_Start(void);
void MotorControlService_Stop(void);
bool MotorControlService_ClearFaults(void);
bool MotorControlService_SetSpeed(float speed_rpm, uint16_t ramp_ms);
bool MotorControlService_SetCurrent(float iq_a, float id_a);
void MotorControlService_SetSpeedCurrentLimit(float current_a);
void MotorControlService_ResetSpeedController(void);
void MotorControlService_GetStatus(MotorControlStatus_t *status);

#ifdef __cplusplus
}
#endif

#endif
