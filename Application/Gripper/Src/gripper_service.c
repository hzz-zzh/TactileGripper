#include "gripper_service.h"

#include "gripper_config.h"
#include "motor_control_service.h"
#include "debug_uart_command.h"
#include "debug_uart_transport.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

#include <stdio.h>
#include <string.h>

#define GRIPPER_COMMAND_QUEUE_DEPTH  8U
#define GRIPPER_SPEED_DIRECTION_EPSILON_RPM  1.0f

typedef enum
{
  GRIPPER_COMMAND_HOME = 0,
  GRIPPER_COMMAND_SET_POSITION,
  GRIPPER_COMMAND_STOP,
  GRIPPER_COMMAND_CLEAR_FAULT
} GripperCommandType_t;

typedef struct
{
  GripperCommandType_t type;
  int16_t position_permille;
} GripperCommand_t;

extern UART_HandleTypeDef huart1;

static osThreadId_t gripperTaskHandle;
static osMessageQueueId_t gripperCommandQueue;
static GripperController_t gripperController;
static volatile GripperStatus_t gripperStatus;

static int8_t GripperService_GetSpeedDirection(float speed_rpm)
{
  if (speed_rpm > GRIPPER_SPEED_DIRECTION_EPSILON_RPM)
  {
    return 1;
  }
  if (speed_rpm < -GRIPPER_SPEED_DIRECTION_EPSILON_RPM)
  {
    return -1;
  }
  return 0;
}

static void GripperService_Write(const char *text)
{
  (void)DebugUartTransport_Write(text, 10U);
}

static bool GripperService_SubmitCommand(GripperCommandType_t type,
                                         int16_t position_permille)
{
  GripperCommand_t command;

  if (gripperCommandQueue == NULL)
  {
    return false;
  }
  command.type = type;
  command.position_permille = position_permille;
  return osMessageQueuePut(gripperCommandQueue, &command, 0U, 0U) == osOK;
}

static GripperControllerConfig_t GripperService_BuildConfig(void)
{
  GripperControllerConfig_t config;

  config.control_period_ms = GRIPPER_CONTROL_PERIOD_MS;
  config.open_direction = GRIPPER_OPEN_DIRECTION;
  config.counts_per_turn = GRIPPER_MOTOR_COUNTS_PER_TURN;
  config.home_speed_rpm = GRIPPER_HOME_SPEED_RPM;
  config.home_current_limit_a = GRIPPER_HOME_CURRENT_LIMIT_A;
  config.stall_current_a = GRIPPER_HOME_STALL_CURRENT_A;
  config.stall_speed_rpm = GRIPPER_HOME_STALL_SPEED_RPM;
  config.stall_ignore_ms = GRIPPER_HOME_STALL_IGNORE_MS;
  config.stall_time_ms = GRIPPER_HOME_STALL_TIME_MS;
  config.home_timeout_ms = GRIPPER_HOME_TIMEOUT_MS;
  config.max_home_travel_counts =
    GRIPPER_HOME_MAX_TRAVEL_TURNS * GRIPPER_MOTOR_COUNTS_PER_TURN;
  config.min_home_travel_counts = GRIPPER_HOME_MIN_TRAVEL_COUNTS;
  config.safe_open_position_permille =
    GRIPPER_SAFE_OPEN_POSITION_PERMILLE;
  config.position_kp_rpm_per_turn = GRIPPER_POSITION_KP_RPM_PER_TURN;
  config.position_max_speed_rpm = GRIPPER_POSITION_MAX_SPEED_RPM;
  config.position_deadband_counts = GRIPPER_POSITION_DEADBAND_COUNTS;
  config.operation_current_limit_a = GRIPPER_OPERATION_CURRENT_LIMIT_A;
  return config;
}

static void GripperService_ProcessCommand(const GripperCommand_t *command)
{
  if (command == NULL)
  {
    return;
  }

  switch (command->type)
  {
    case GRIPPER_COMMAND_HOME:
      if (gripperStatus.motor_state != MOTOR_CONTROL_STATE_IDLE)
      {
        MotorControlService_Stop();
      }
      GripperController_RequestHome(&gripperController);
      break;

    case GRIPPER_COMMAND_SET_POSITION:
      if (!GripperController_SetPosition(&gripperController,
                                         command->position_permille))
      {
        GripperService_Write("GRIPPER,ERR,position_rejected\r\n");
      }
      break;

    case GRIPPER_COMMAND_STOP:
      GripperController_Stop(&gripperController);
      break;

    case GRIPPER_COMMAND_CLEAR_FAULT:
    {
      bool motorCleared = MotorControlService_ClearFaults();
      if (!GripperController_ClearFaults(&gripperController, motorCleared))
      {
        GripperService_Write("GRIPPER,ERR,clear_fault_rejected\r\n");
      }
      break;
    }

    default:
      break;
  }
}

static void GripperService_PrintStatus(void)
{
  GripperStatus_t status;
  char line[192];

  GripperService_GetStatus(&status);
  (void)snprintf(line, sizeof(line),
                 "gripper: %u,%lu,%u,%d,%d,%ld,%ld,%ld,%ld\r\n",
                 (unsigned int)status.state,
                 (unsigned long)status.faults,
                 status.homed ? 1U : 0U,
                 (int)status.target_position_permille,
                 (int)status.position_permille,
                 (long)status.target_count,
                 (long)status.position_count,
                 (long)status.open_count,
                 (long)status.close_count);
  GripperService_Write(line);
}

static void GripperService_ProcessUart(void)
{
  DebugUartCommand_t uartCommand;
  bool accepted;

  while (DebugUartCommand_Poll(&uartCommand))
  {
    accepted = true;
    switch (uartCommand.type)
    {
      case DEBUG_UART_COMMAND_START:
      case DEBUG_UART_COMMAND_HOME:
        accepted = GripperService_Home();
        break;

      case DEBUG_UART_COMMAND_STOP:
        accepted = GripperService_Stop();
        break;

      case DEBUG_UART_COMMAND_CLEAR_FAULT:
        accepted = GripperService_ClearFaults();
        break;

      case DEBUG_UART_COMMAND_STATUS:
        GripperService_PrintStatus();
        break;

      case DEBUG_UART_COMMAND_TARGET_POSITION:
        accepted = GripperService_SetPosition(
          uartCommand.target_position_permille);
        break;

      case DEBUG_UART_COMMAND_TARGET_TURN:
      case DEBUG_UART_COMMAND_ZERO:
        GripperService_Write("GRIPPER,ERR,motor_turn_command_disabled\r\n");
        break;

      case DEBUG_UART_COMMAND_INVALID:
        GripperService_Write("GRIPPER,ERR,invalid_command\r\n");
        break;

      default:
        break;
    }
    if (!accepted)
    {
      GripperService_Write("GRIPPER,ERR,command_queue_full\r\n");
    }
  }
}

static void GripperService_UpdateStatus(
  const MotorControlStatus_t *motor,
  const GripperControllerOutput_t *output)
{
  taskENTER_CRITICAL();
  gripperStatus.state = gripperController.state;
  gripperStatus.faults = gripperController.faults;
  gripperStatus.state_elapsed_ms = gripperController.state_elapsed_ms;
  gripperStatus.stall_elapsed_ms = gripperController.stall_elapsed_ms;
  gripperStatus.homed = gripperController.homed;
  gripperStatus.target_position_permille =
    gripperController.target_position_permille;
  gripperStatus.position_permille = gripperController.position_permille;
  gripperStatus.target_count = gripperController.target_count;
  gripperStatus.position_count = motor->position_count;
  gripperStatus.open_count = gripperController.open_count;
  gripperStatus.close_count = gripperController.close_count;
  gripperStatus.speed_ref_rpm = output->speed_ref_rpm;
  gripperStatus.speed_rpm = motor->speed_rpm;
  gripperStatus.iq_ref_a = motor->iq_ref_a;
  gripperStatus.id_ref_a = motor->id_ref_a;
  gripperStatus.iq_a = motor->iq_a;
  gripperStatus.id_a = motor->id_a;
  gripperStatus.bus_current_a = motor->bus_current_a;
  gripperStatus.bus_voltage_v = motor->bus_voltage_v;
  gripperStatus.temperature_raw = motor->temperature_raw;
  gripperStatus.mc_faults = motor->mc_faults;
  gripperStatus.encoder_spi_errors = motor->encoder_spi_errors;
  gripperStatus.motor_state = (uint8_t)motor->state;
  gripperStatus.motor_faults = motor->faults;
  gripperStatus.encoder_reliable = motor->encoder_reliable;
  taskEXIT_CRITICAL();
}

static void GripperService_Task(void *argument)
{
  uint32_t next = osKernelGetTickCount();
  GripperControllerConfig_t config = GripperService_BuildConfig();
  int8_t lastSpeedDirection = 0;
  (void)argument;

  memset((void *)&gripperStatus, 0, sizeof(gripperStatus));
  GripperController_Init(&gripperController, &config);
#if GRIPPER_AUTO_HOME_ON_BOOT
  GripperController_RequestHome(&gripperController);
#endif

  for (;;)
  {
    MotorControlStatus_t motor;
    GripperControllerFeedback_t feedback;
    GripperControllerOutput_t output;
    GripperCommand_t command;
    GripperState_t previousState;
    int8_t speedDirection;

    next += GRIPPER_CONTROL_PERIOD_TICKS;
    GripperService_ProcessUart();
    while (osMessageQueueGet(gripperCommandQueue, &command, NULL, 0U) == osOK)
    {
      GripperService_ProcessCommand(&command);
    }

    MotorControlService_GetStatus(&motor);
    feedback.motor_idle = motor.state == MOTOR_CONTROL_STATE_IDLE;
    feedback.motor_starting = motor.state == MOTOR_CONTROL_STATE_STARTING;
    feedback.motor_running = motor.state == MOTOR_CONTROL_STATE_RUNNING;
    feedback.encoder_reliable = motor.encoder_reliable;
    feedback.motor_faults = motor.faults;
    feedback.position_count = motor.position_count;
    feedback.speed_rpm = motor.speed_rpm;
    feedback.iq_ref_a = motor.iq_ref_a;
    feedback.iq_a = motor.iq_a;

    previousState = gripperController.state;
    output = GripperController_Update(&gripperController, &feedback);
    speedDirection = GripperService_GetSpeedDirection(output.speed_ref_rpm);

    /* 两端堵转后都会改变运动方向，换向前清除上一方向积累的速度积分。 */
    if (((previousState == GRIPPER_STATE_HOMING_OPEN) &&
         (gripperController.state == GRIPPER_STATE_HOMING_CLOSE)) ||
        ((previousState == GRIPPER_STATE_HOMING_CLOSE) &&
         (gripperController.state == GRIPPER_STATE_MOVING_SAFE)))
    {
      MotorControlService_ResetSpeedController();
    }

    /* 位置误差越过目标点时清除旧方向积分，防止速度目标和Iq长期反向。 */
    if ((speedDirection != 0) &&
        (lastSpeedDirection != 0) &&
        (speedDirection != lastSpeedDirection))
    {
      MotorControlService_ResetSpeedController();
    }
    if (speedDirection != 0)
    {
      lastSpeedDirection = speedDirection;
    }
    if (output.stop_motor)
    {
      lastSpeedDirection = 0;
    }
    MotorControlService_SetSpeedCurrentLimit(output.current_limit_a);
    if (output.stop_motor &&
        (motor.state != MOTOR_CONTROL_STATE_IDLE) &&
        (motor.state != MOTOR_CONTROL_STATE_STOPPING) &&
        (motor.state != MOTOR_CONTROL_STATE_FAULT))
    {
      MotorControlService_Stop();
    }
    else
    {
      if (output.start_motor)
      {
        (void)MotorControlService_Start();
      }
      if (feedback.motor_running)
      {
        uint16_t speedRampMs = GRIPPER_POSITION_SPEED_RAMP_MS;

        if ((gripperController.state == GRIPPER_STATE_HOMING_OPEN) ||
            (gripperController.state == GRIPPER_STATE_HOMING_CLOSE))
        {
          speedRampMs = GRIPPER_HOME_SPEED_RAMP_MS;
        }
        (void)MotorControlService_SetSpeed(output.speed_ref_rpm,
                                           speedRampMs);
      }
    }
    GripperService_UpdateStatus(&motor, &output);
    (void)osDelayUntil(next);
  }
}

void GripperService_CreateTask(void)
{
  const osThreadAttr_t taskAttr = {
    .name = "gripperControl",
    .stack_size = 768U * 4U,
    .priority = osPriorityNormal
  };

  gripperCommandQueue = osMessageQueueNew(GRIPPER_COMMAND_QUEUE_DEPTH,
                                          sizeof(GripperCommand_t), NULL);
  DebugUartCommand_Init(&huart1);
  if (gripperCommandQueue != NULL)
  {
    gripperTaskHandle = osThreadNew(GripperService_Task, NULL, &taskAttr);
  }
  if ((gripperCommandQueue == NULL) || (gripperTaskHandle == NULL))
  {
    MotorControlService_Stop();
  }
}

bool GripperService_Home(void)
{
  return GripperService_SubmitCommand(GRIPPER_COMMAND_HOME, 0);
}

bool GripperService_SetPosition(int16_t position_permille)
{
  if ((position_permille < 0) || (position_permille > 1000))
  {
    return false;
  }
  return GripperService_SubmitCommand(GRIPPER_COMMAND_SET_POSITION,
                                      position_permille);
}

bool GripperService_Stop(void)
{
  return GripperService_SubmitCommand(GRIPPER_COMMAND_STOP, 0);
}

bool GripperService_ClearFaults(void)
{
  return GripperService_SubmitCommand(GRIPPER_COMMAND_CLEAR_FAULT, 0);
}

void GripperService_GetStatus(GripperStatus_t *status)
{
  if (status != NULL)
  {
    taskENTER_CRITICAL();
    memcpy(status, (const void *)&gripperStatus, sizeof(*status));
    taskEXIT_CRITICAL();
  }
}
