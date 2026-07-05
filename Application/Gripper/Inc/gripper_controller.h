#ifndef GRIPPER_CONTROLLER_H
#define GRIPPER_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  GRIPPER_STATE_BOOT = 0,
  GRIPPER_STATE_PRECHECK,
  GRIPPER_STATE_STARTING,
  GRIPPER_STATE_HOMING_OPEN,
  GRIPPER_STATE_HOMING_CLOSE,
  GRIPPER_STATE_MOVING_SAFE,
  GRIPPER_STATE_READY,
  GRIPPER_STATE_MOVING,
  GRIPPER_STATE_HOLDING,
  GRIPPER_STATE_STOPPED,
  GRIPPER_STATE_FAULT
} GripperState_t;

typedef enum
{
  GRIPPER_FAULT_NONE             = 0,
  GRIPPER_FAULT_MOTOR            = (1UL << 0),
  GRIPPER_FAULT_ENCODER          = (1UL << 1),
  GRIPPER_FAULT_HOME_TIMEOUT     = (1UL << 2),
  GRIPPER_FAULT_HOME_TRAVEL      = (1UL << 3),
  GRIPPER_FAULT_COMMAND_REJECTED = (1UL << 4)
} GripperFault_t;

typedef struct
{
  uint32_t control_period_ms;
  int8_t open_direction;
  int32_t counts_per_turn;
  float home_speed_rpm;
  float home_current_limit_a;
  float stall_current_a;
  float stall_speed_rpm;
  uint32_t stall_ignore_ms;
  uint32_t stall_time_ms;
  uint32_t home_timeout_ms;
  int32_t max_home_travel_counts;
  int32_t min_home_travel_counts;
  int16_t safe_open_position_permille;
  float position_kp_rpm_per_turn;
  float position_max_speed_rpm;
  int32_t position_deadband_counts;
  float operation_current_limit_a;
} GripperControllerConfig_t;

typedef struct
{
  bool motor_idle;
  bool motor_starting;
  bool motor_running;
  bool encoder_reliable;
  uint32_t motor_faults;
  int32_t position_count;
  float speed_rpm;
  float iq_ref_a;
  float iq_a;
} GripperControllerFeedback_t;

typedef struct
{
  bool start_motor;
  bool stop_motor;
  float speed_ref_rpm;
  float current_limit_a;
} GripperControllerOutput_t;

typedef struct
{
  GripperControllerConfig_t config;
  GripperState_t state;
  uint32_t faults;
  uint32_t state_elapsed_ms;
  uint32_t stall_elapsed_ms;
  int32_t open_count;
  int32_t close_count;
  int32_t target_count;
  int16_t target_position_permille;
  int16_t position_permille;
  bool homed;
} GripperController_t;

void GripperController_Init(GripperController_t *controller,
                            const GripperControllerConfig_t *config);
void GripperController_RequestHome(GripperController_t *controller);
bool GripperController_SetPosition(GripperController_t *controller,
                                   int16_t position_permille);
void GripperController_Stop(GripperController_t *controller);
bool GripperController_ClearFaults(GripperController_t *controller,
                                   bool motor_fault_cleared);
GripperControllerOutput_t GripperController_Update(
  GripperController_t *controller,
  const GripperControllerFeedback_t *feedback);

#ifdef __cplusplus
}
#endif

#endif
