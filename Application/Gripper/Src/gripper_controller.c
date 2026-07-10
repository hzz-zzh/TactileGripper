#include "gripper_controller.h"

#include <string.h>

static int32_t GripperController_Abs32(int32_t value)
{
  return (value < 0) ? -value : value;
}

static float GripperController_AbsF(float value)
{
  return (value < 0.0f) ? -value : value;
}

static float GripperController_ClampF(float value,
                                      float minimum,
                                      float maximum)
{
  if (value < minimum)
  {
    return minimum;
  }
  if (value > maximum)
  {
    return maximum;
  }
  return value;
}

static int16_t GripperController_ClampPosition(int32_t position_permille)
{
  if (position_permille < 0)
  {
    return 0;
  }
  if (position_permille > 1000)
  {
    return 1000;
  }
  return position_permille;
}

static void GripperController_EnterState(GripperController_t *controller,
                                         GripperState_t state)
{
  controller->state = state;
  controller->state_elapsed_ms = 0U;
  controller->stall_elapsed_ms = 0U;
}

static void GripperController_LatchFault(GripperController_t *controller,
                                         uint32_t fault)
{
  controller->faults |= fault;
  controller->homed = false;
  GripperController_EnterState(controller, GRIPPER_STATE_FAULT);
}

static int32_t GripperController_PositionToCount(
  const GripperController_t *controller,
  int16_t position_permille)
{
  int32_t travel = controller->close_count - controller->open_count;
  return controller->open_count +
         (int32_t)(((int64_t)travel * position_permille) / 1000LL);
}

static int16_t GripperController_CountToPosition(
  const GripperController_t *controller,
  int32_t position_count)
{
  int32_t travel = controller->close_count - controller->open_count;
  int32_t position;

  if (travel == 0)
  {
    return 0;
  }
  position = (int32_t)(((int64_t)(position_count - controller->open_count) *
                        1000LL) / travel);
  return GripperController_ClampPosition(position);
}

static bool GripperController_StallDetected(
  GripperController_t *controller,
  const GripperControllerFeedback_t *feedback)
{
  bool speedLow;
  bool torqueHigh;

  speedLow = GripperController_AbsF(feedback->speed_rpm) <=
             controller->config.stall_speed_rpm;
  torqueHigh =
    (GripperController_AbsF(feedback->iq_a) >=
     controller->config.stall_current_a) ||
    (GripperController_AbsF(feedback->iq_ref_a) >=
     controller->config.stall_current_a);

  if (controller->state_elapsed_ms < controller->config.stall_ignore_ms)
  {
    /*
     * 起步阶段会经过静摩擦和齿隙消除，先忽略一段时间，避免刚启动误判限位。
     */
    controller->stall_elapsed_ms = 0U;
  }
  else if (speedLow && torqueHigh)
  {
    controller->stall_elapsed_ms += controller->config.control_period_ms;
  }
  else
  {
    controller->stall_elapsed_ms = 0U;
  }

  return controller->stall_elapsed_ms >= controller->config.stall_time_ms;
}

static float GripperController_RunPositionLoop(
  GripperController_t *controller,
  const GripperControllerFeedback_t *feedback,
  bool *target_reached)
{
  int32_t error = controller->target_count - feedback->position_count;
  float error_turn;
  float speed_ref;

  *target_reached =
    GripperController_Abs32(error) <=
    controller->config.position_deadband_counts;
  if (*target_reached)
  {
    return 0.0f;
  }

  error_turn = (float)error / (float)controller->config.counts_per_turn;
  speed_ref = error_turn * controller->config.position_kp_rpm_per_turn;
  return GripperController_ClampF(
    speed_ref,
    -controller->config.position_max_speed_rpm,
    controller->config.position_max_speed_rpm);
}

void GripperController_Init(GripperController_t *controller,
                            const GripperControllerConfig_t *config)
{
  if ((controller == NULL) || (config == NULL))
  {
    return;
  }

  memset(controller, 0, sizeof(*controller));
  controller->config = *config;
  controller->state = GRIPPER_STATE_BOOT;
}

void GripperController_RequestHome(GripperController_t *controller)
{
  if (controller == NULL)
  {
    return;
  }

  controller->faults = GRIPPER_FAULT_NONE;
  controller->homed = false;
  controller->open_count = 0;
  controller->close_count = 0;
  controller->target_count = 0;
  controller->target_position_permille = 0;
  controller->position_permille = 0;
  GripperController_EnterState(controller, GRIPPER_STATE_PRECHECK);
}

bool GripperController_SetPosition(GripperController_t *controller,
                                   int16_t position_permille)
{
  if ((controller == NULL) || !controller->homed ||
      (controller->faults != GRIPPER_FAULT_NONE) ||
      (controller->state == GRIPPER_STATE_FAULT))
  {
    return false;
  }

  controller->target_position_permille =
    GripperController_ClampPosition(position_permille);
  controller->target_count = GripperController_PositionToCount(
    controller, controller->target_position_permille);
  GripperController_EnterState(controller, GRIPPER_STATE_MOVING);
  return true;
}

void GripperController_Stop(GripperController_t *controller)
{
  if (controller != NULL)
  {
    GripperController_EnterState(controller, GRIPPER_STATE_STOPPED);
  }
}

bool GripperController_ClearFaults(GripperController_t *controller,
                                   bool motor_fault_cleared)
{
  bool preserve_homing;

  if ((controller == NULL) || !motor_fault_cleared)
  {
    return false;
  }

  /* 纯通信故障不影响机械零点，其他故障清除后必须重新回零。 */
  preserve_homing = controller->homed &&
    ((controller->faults & ~GRIPPER_FAULT_COMMUNICATION) == 0U);
  controller->faults = GRIPPER_FAULT_NONE;
  controller->homed = preserve_homing;
  GripperController_EnterState(controller, GRIPPER_STATE_STOPPED);
  return true;
}

void GripperController_LatchExternalFault(GripperController_t *controller,
                                          uint32_t fault)
{
  if (controller == NULL)
  {
    return;
  }

  controller->faults |= fault;
  /* CAN超时只停止运动，不破坏已经完成的夹爪行程标定。 */
  if ((controller->faults & ~GRIPPER_FAULT_COMMUNICATION) != 0U)
  {
    controller->homed = false;
  }
  GripperController_EnterState(controller, GRIPPER_STATE_FAULT);
}

GripperControllerOutput_t GripperController_Update(
  GripperController_t *controller,
  const GripperControllerFeedback_t *feedback)
{
  GripperControllerOutput_t output = {0};
  bool targetReached = false;

  if ((controller == NULL) || (feedback == NULL))
  {
    return output;
  }

  controller->state_elapsed_ms += controller->config.control_period_ms;
  output.current_limit_a = controller->config.operation_current_limit_a;

  if ((feedback->motor_faults != 0U) &&
      (controller->state != GRIPPER_STATE_FAULT))
  {
    GripperController_LatchFault(controller, GRIPPER_FAULT_MOTOR);
  }
  else if (!feedback->encoder_reliable &&
           (controller->state != GRIPPER_STATE_BOOT) &&
           (controller->state != GRIPPER_STATE_PRECHECK) &&
           (controller->state != GRIPPER_STATE_FAULT))
  {
    GripperController_LatchFault(controller, GRIPPER_FAULT_ENCODER);
  }

  switch (controller->state)
  {
    case GRIPPER_STATE_BOOT:
      break;

    case GRIPPER_STATE_PRECHECK:
      output.current_limit_a = controller->config.home_current_limit_a;
      if (feedback->encoder_reliable && (feedback->motor_faults == 0U))
      {
        output.start_motor = true;
        GripperController_EnterState(controller, GRIPPER_STATE_STARTING);
      }
      else if (controller->state_elapsed_ms >=
               controller->config.home_timeout_ms)
      {
        GripperController_LatchFault(controller, GRIPPER_FAULT_ENCODER);
      }
      break;

    case GRIPPER_STATE_STARTING:
      output.current_limit_a = controller->config.home_current_limit_a;
      output.start_motor = feedback->motor_idle;
      if (feedback->motor_running)
      {
        GripperController_EnterState(controller, GRIPPER_STATE_HOMING_OPEN);
      }
      else if (controller->state_elapsed_ms >=
               controller->config.home_timeout_ms)
      {
        GripperController_LatchFault(controller, GRIPPER_FAULT_HOME_TIMEOUT);
      }
      break;

    case GRIPPER_STATE_HOMING_OPEN:
      output.current_limit_a = controller->config.home_current_limit_a;
      output.speed_ref_rpm =
        (float)controller->config.open_direction *
        controller->config.home_speed_rpm;
      if (GripperController_StallDetected(controller, feedback))
      {
        controller->open_count = feedback->position_count;
        output.speed_ref_rpm = 0.0f;
        GripperController_EnterState(controller, GRIPPER_STATE_HOMING_CLOSE);
      }
      else if (controller->state_elapsed_ms >=
               controller->config.home_timeout_ms)
      {
        GripperController_LatchFault(controller, GRIPPER_FAULT_HOME_TIMEOUT);
      }
      break;

    case GRIPPER_STATE_HOMING_CLOSE:
      output.current_limit_a = controller->config.home_current_limit_a;
      output.speed_ref_rpm =
        (float)-controller->config.open_direction *
        controller->config.home_speed_rpm;
      if (GripperController_Abs32(feedback->position_count -
                                  controller->open_count) >
          controller->config.max_home_travel_counts)
      {
        GripperController_LatchFault(controller, GRIPPER_FAULT_HOME_TRAVEL);
      }
      else if (GripperController_StallDetected(controller, feedback))
      {
        controller->close_count = feedback->position_count;
        output.speed_ref_rpm = 0.0f;
        if (GripperController_Abs32(controller->close_count -
                                    controller->open_count) <
            controller->config.min_home_travel_counts)
        {
          GripperController_LatchFault(controller,
                                       GRIPPER_FAULT_HOME_TRAVEL);
        }
        else
        {
          controller->homed = true;
          controller->target_position_permille =
            controller->config.safe_open_position_permille;
          controller->target_count = GripperController_PositionToCount(
            controller, controller->target_position_permille);
          GripperController_EnterState(controller,
                                       GRIPPER_STATE_MOVING_SAFE);
        }
      }
      else if (controller->state_elapsed_ms >=
               controller->config.home_timeout_ms)
      {
        GripperController_LatchFault(controller, GRIPPER_FAULT_HOME_TIMEOUT);
      }
      break;

    case GRIPPER_STATE_MOVING_SAFE:
    case GRIPPER_STATE_MOVING:
      if (feedback->motor_idle)
      {
        /* STOP或通信故障恢复后，首条位置指令负责重新启动电机。 */
        output.start_motor = true;
      }
      else if (feedback->motor_running)
      {
        output.speed_ref_rpm = GripperController_RunPositionLoop(
          controller, feedback, &targetReached);
        if (targetReached)
        {
          GripperController_EnterState(controller,
                                       (controller->state ==
                                        GRIPPER_STATE_MOVING_SAFE) ?
                                       GRIPPER_STATE_READY :
                                       GRIPPER_STATE_HOLDING);
        }
      }
      break;

    case GRIPPER_STATE_READY:
    case GRIPPER_STATE_HOLDING:
      if (feedback->motor_idle)
      {
        output.start_motor = true;
      }
      else if (feedback->motor_running)
      {
        /* 保持状态仍运行位置外环，外力造成偏移时自动回到目标位置。 */
        output.speed_ref_rpm = GripperController_RunPositionLoop(
          controller, feedback, &targetReached);
      }
      break;

    case GRIPPER_STATE_STOPPED:
      output.stop_motor = !feedback->motor_idle;
      break;

    case GRIPPER_STATE_FAULT:
      output.stop_motor = true;
      break;

    default:
      GripperController_LatchFault(controller,
                                   GRIPPER_FAULT_COMMAND_REJECTED);
      output.stop_motor = true;
      break;
  }

  if (controller->homed)
  {
    controller->position_permille = GripperController_CountToPosition(
      controller, feedback->position_count);
  }
  if (controller->state == GRIPPER_STATE_FAULT)
  {
    /* 故障在本周期内产生时也要立即覆盖运动指令，不能延迟到下一次更新。 */
    output.speed_ref_rpm = 0.0f;
    output.stop_motor = true;
  }
  return output;
}
