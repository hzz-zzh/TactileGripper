#ifndef GRIPPER_MOTOR_SERVICE_H
#define GRIPPER_MOTOR_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  GRIPPER_MOTOR_BOOT = 0,
  GRIPPER_MOTOR_PRECHECK,
  GRIPPER_MOTOR_ALIGNING,
  GRIPPER_MOTOR_HOMING_OPEN,
  GRIPPER_MOTOR_HOMING_CLOSE,
  GRIPPER_MOTOR_MOVE_SAFE,
  GRIPPER_MOTOR_READY,
  GRIPPER_MOTOR_POSITIONING,
  GRIPPER_MOTOR_STOPPED,
  GRIPPER_MOTOR_FAULT
} GripperMotorState_t;

typedef enum
{
  GRIPPER_FAULT_NONE = 0,
  GRIPPER_FAULT_ENCODER = (1UL << 0),
  GRIPPER_FAULT_MOTOR_CONTROL = (1UL << 1),
  GRIPPER_FAULT_HOME_TIMEOUT = (1UL << 2),
  GRIPPER_FAULT_HOME_TRAVEL = (1UL << 3),
  GRIPPER_FAULT_BUS_VOLTAGE = (1UL << 4)
} GripperMotorFault_t;

typedef struct
{
  GripperMotorState_t state;
  uint32_t faults;
  int32_t position_count;
  int32_t open_count;
  int32_t close_count;
  int16_t position_permille;
  int16_t target_permille;
  float speed_rpm;
  float iq_a;
  float bus_current_a;
  uint16_t bus_current_raw;
  uint16_t bus_voltage_raw;
  uint16_t temperature_raw;
  uint16_t bus_voltage_v;
  uint16_t encoder_last_frame;
  uint32_t encoder_frames;
  uint32_t encoder_spi_errors;
  uint8_t encoder_consecutive_errors;
  bool encoder_reliable;
  uint16_t mc_faults;
  uint16_t mc_occurred_faults;
  bool homed;
} GripperMotorStatus_t;

void GripperMotorService_CreateTasks(void);
bool GripperMotor_SetPosition(int16_t position_permille);
bool GripperMotor_SetSpeed(float motor_rpm);
bool GripperMotor_SetCurrent(float iq_a);
bool GripperMotor_Rehome(void);
void GripperMotor_Stop(void);
bool GripperMotor_ClearFaults(void);
void GripperMotor_GetStatus(GripperMotorStatus_t *status);

#ifdef __cplusplus
}
#endif

#endif
