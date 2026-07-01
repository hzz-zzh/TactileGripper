#include "gripper_motor_service.h"

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "mc_api.h"
#include "mc_config.h"
#include "mc_tasks.h"
#include "mc_config_common.h"
#include "mcp_config.h"
#include "parameters_conversion.h"
#include "kth7812_speed_pos_fdbk.h"

#include "stm32h7xx_ll_dma.h"

#include <math.h>
#include <string.h>

#define SERVICE_PERIOD_TICKS          10U  /* 5 ms at 2 kHz */
#define PRECHECK_DELAY_TICKS          1000U
#define HOME_TIMEOUT_STEPS            2000U
#define HOME_STALL_STEPS              60U
#define HOME_MAX_TRAVEL_COUNTS        (20L * KTH7812_COUNTS_PER_TURN)
#define HOME_MIN_TRAVEL_COUNTS        256L
#define HOME_SPEED_RPM                120.0f
#define HOME_CURRENT_LIMIT_A          0.30f
#define STALL_CURRENT_A               0.25f
#define STALL_SPEED_RPM               5.0f
#define POSITION_MAX_SPEED_RPM        600.0f
#define POSITION_DEADBAND_COUNTS      4L
#define SAFE_OPEN_POSITION_PERMILLE   50

static osThreadId_t mediumTaskHandle;
static osThreadId_t safetyTaskHandle;
static osThreadId_t serviceTaskHandle;

static volatile GripperMotorStatus_t serviceStatus;
static uint32_t stateSteps;
static uint32_t stallSteps;
static bool startIssued;

static int32_t Abs32(int32_t value)
{
  return (value < 0) ? -value : value;
}

static float AbsF(float value)
{
  return (value < 0.0f) ? -value : value;
}

static void SetSpeedCurrentLimit(float current_a)
{
  int16_t limit = (int16_t)(current_a * CURRENT_CONV_FACTOR);
  PID_SetUpperOutputLimit(&PIDSpeedHandle_M1, limit);
  PID_SetLowerOutputLimit(&PIDSpeedHandle_M1, (int16_t)-limit);
  PID_SetUpperIntegralTermLimit(&PIDSpeedHandle_M1, (int32_t)limit * SP_KIDIV);
  PID_SetLowerIntegralTermLimit(&PIDSpeedHandle_M1, -(int32_t)limit * SP_KIDIV);
}

static void CommandSpeed(float rpm)
{
  MC_ProgramSpeedRampMotor1_F(rpm, 20U);
}

static void LatchFault(uint32_t fault)
{
  serviceStatus.faults |= fault;
  serviceStatus.state = GRIPPER_MOTOR_FAULT;
  serviceStatus.homed = false;
  (void)MC_StopMotor1();
}

static bool StallDetected(void)
{
  qd_f_t current = MC_GetIqdMotor1_F();
  float speed = MC_GetAverageMecSpeedMotor1_F();

  if ((AbsF(speed) <= STALL_SPEED_RPM) && (AbsF(current.q) >= STALL_CURRENT_A))
  {
    if (stallSteps < HOME_STALL_STEPS)
    {
      stallSteps++;
    }
  }
  else
  {
    stallSteps = 0U;
  }
  return stallSteps >= HOME_STALL_STEPS;
}

static int16_t PositionPermille(int32_t count)
{
  int32_t travel = serviceStatus.close_count - serviceStatus.open_count;
  int32_t value;
  if (travel == 0)
  {
    return 0;
  }
  value = ((count - serviceStatus.open_count) * 1000L) / travel;
  if (value < 0)
  {
    value = 0;
  }
  else if (value > 1000)
  {
    value = 1000;
  }
  return (int16_t)value;
}

static void RunPositionLoop(void)
{
  int32_t travel = serviceStatus.close_count - serviceStatus.open_count;
  int32_t target = serviceStatus.open_count +
                   (travel * serviceStatus.target_permille) / 1000L;
  int32_t error = target - serviceStatus.position_count;
  float speed;

  if (Abs32(error) <= POSITION_DEADBAND_COUNTS)
  {
    CommandSpeed(0.0f);
    if (serviceStatus.state == GRIPPER_MOTOR_MOVE_SAFE)
    {
      serviceStatus.state = GRIPPER_MOTOR_READY;
    }
    else
    {
      serviceStatus.state = GRIPPER_MOTOR_READY;
    }
    return;
  }

  speed = (float)error * 0.8f;
  if (speed > POSITION_MAX_SPEED_RPM)
  {
    speed = POSITION_MAX_SPEED_RPM;
  }
  else if (speed < -POSITION_MAX_SPEED_RPM)
  {
    speed = -POSITION_MAX_SPEED_RPM;
  }
  CommandSpeed(speed);
}

static void ResetHomeState(void)
{
  serviceStatus.homed = false;
  serviceStatus.open_count = 0;
  serviceStatus.close_count = 0;
  serviceStatus.target_permille = 0;
  stateSteps = 0U;
  stallSteps = 0U;
  startIssued = false;
  SetSpeedCurrentLimit(HOME_CURRENT_LIMIT_A);
  serviceStatus.state = GRIPPER_MOTOR_PRECHECK;
}

static void MediumTask(void *argument)
{
  uint32_t next = osKernelGetTickCount();
  (void)argument;
  for (;;)
  {
    next += 1U;
    if (LL_DMA_IsActiveFlag_TC(DMA_RX_A, DMACH_RX_A))
    {
      LL_DMA_ClearFlag_TC(DMA_RX_A, DMACH_RX_A);
      ASPEP_HWDataReceivedIT(&aspepOverUartA);
    }
    MC_RunMotorControlTasks();
    (void)osDelayUntil(next);
  }
}

static void SafetyTask(void *argument)
{
  uint32_t next = osKernelGetTickCount();
  (void)argument;
  for (;;)
  {
    next += 1U;
    TSK_SafetyTask();
    (void)osDelayUntil(next);
  }
}

static void ServiceTask(void *argument)
{
  uint32_t next = osKernelGetTickCount();
  (void)argument;
  ResetHomeState();

  for (;;)
  {
    qd_f_t current;
    next += SERVICE_PERIOD_TICKS;
    serviceStatus.position_count = KTH7812_GetMultiTurnCount(&KTH7812_M1);
    serviceStatus.speed_rpm = MC_GetAverageMecSpeedMotor1_F();
    current = MC_GetIqdMotor1_F();
    serviceStatus.iq_a = current.q;
    serviceStatus.bus_current_raw = RCM_GetRegularConv(&BusCurrentRegConv_M1);
    serviceStatus.bus_voltage_raw = RCM_GetRegularConv(&VbusRegConv_M1);
    serviceStatus.temperature_raw = RCM_GetRegularConv(&TempRegConv_M1);
    serviceStatus.bus_current_a = ((int32_t)serviceStatus.bus_current_raw - 32768L) *
                                  CURRENT_CONV_FACTOR_INV;
    serviceStatus.bus_voltage_v = VBS_GetAvBusVoltage_V(&BusVoltageSensor_M1._Super);
    serviceStatus.mc_faults = MC_GetCurrentFaultsMotor1();
    serviceStatus.mc_occurred_faults = MC_GetOccurredFaultsMotor1();
    serviceStatus.encoder_last_frame = KTH7812_M1.last_frame;
    serviceStatus.encoder_last_good_frame = KTH7812_M1.last_good_frame;
    serviceStatus.encoder_frames = KTH7812_M1.frame_count;
    serviceStatus.encoder_valid_frames = KTH7812_M1.valid_frame_count;
    serviceStatus.encoder_crc_errors = KTH7812_M1.crc_error_count;
    serviceStatus.encoder_spi_errors = KTH7812_M1.spi_error_count;
    serviceStatus.encoder_received_crc = KTH7812_M1.received_crc;
    serviceStatus.encoder_calculated_crc = KTH7812_M1.calculated_crc;
    serviceStatus.encoder_consecutive_errors = KTH7812_M1.consecutive_errors;
    serviceStatus.encoder_reliable = KTH7812_IsReliable(&KTH7812_M1);

    if (!KTH7812_IsReliable(&KTH7812_M1))
    {
      LatchFault(GRIPPER_FAULT_ENCODER);
    }
    else if (serviceStatus.mc_faults != 0U)
    {
      LatchFault(GRIPPER_FAULT_MOTOR_CONTROL);
    }

    switch (serviceStatus.state)
    {
      case GRIPPER_MOTOR_PRECHECK:
        stateSteps++;
        if (stateSteps >= (PRECHECK_DELAY_TICKS / SERVICE_PERIOD_TICKS))
        {
          if ((serviceStatus.bus_voltage_v < 10U) || (serviceStatus.bus_voltage_v > 30U))
          {
            LatchFault(GRIPPER_FAULT_BUS_VOLTAGE);
          }
          else
          {
            CommandSpeed(-HOME_SPEED_RPM);
            startIssued = MC_StartMotor1();
            if (startIssued)
            {
              serviceStatus.state = GRIPPER_MOTOR_ALIGNING;
              stateSteps = 0U;
            }
          }
        }
        break;

      case GRIPPER_MOTOR_ALIGNING:
        if (MC_GetSTMStateMotor1() == RUN)
        {
          CommandSpeed(-HOME_SPEED_RPM);
          serviceStatus.state = GRIPPER_MOTOR_HOMING_OPEN;
          stateSteps = 0U;
          stallSteps = 0U;
        }
        else if (++stateSteps > HOME_TIMEOUT_STEPS)
        {
          LatchFault(GRIPPER_FAULT_HOME_TIMEOUT);
        }
        break;

      case GRIPPER_MOTOR_HOMING_OPEN:
        stateSteps++;
        if (StallDetected())
        {
          serviceStatus.open_count = serviceStatus.position_count;
          CommandSpeed(HOME_SPEED_RPM);
          serviceStatus.state = GRIPPER_MOTOR_HOMING_CLOSE;
          stateSteps = 0U;
          stallSteps = 0U;
        }
        else if (stateSteps > HOME_TIMEOUT_STEPS)
        {
          LatchFault(GRIPPER_FAULT_HOME_TIMEOUT);
        }
        break;

      case GRIPPER_MOTOR_HOMING_CLOSE:
        stateSteps++;
        if (Abs32(serviceStatus.position_count - serviceStatus.open_count) > HOME_MAX_TRAVEL_COUNTS)
        {
          LatchFault(GRIPPER_FAULT_HOME_TRAVEL);
        }
        else if (StallDetected())
        {
          serviceStatus.close_count = serviceStatus.position_count;
          if (Abs32(serviceStatus.close_count - serviceStatus.open_count) < HOME_MIN_TRAVEL_COUNTS)
          {
            LatchFault(GRIPPER_FAULT_HOME_TRAVEL);
          }
          else
          {
            serviceStatus.homed = true;
            serviceStatus.target_permille = SAFE_OPEN_POSITION_PERMILLE;
            serviceStatus.state = GRIPPER_MOTOR_MOVE_SAFE;
            SetSpeedCurrentLimit(1.0f);
          }
        }
        else if (stateSteps > HOME_TIMEOUT_STEPS)
        {
          LatchFault(GRIPPER_FAULT_HOME_TIMEOUT);
        }
        break;

      case GRIPPER_MOTOR_MOVE_SAFE:
      case GRIPPER_MOTOR_POSITIONING:
        RunPositionLoop();
        break;

      case GRIPPER_MOTOR_READY:
        CommandSpeed(0.0f);
        break;

      case GRIPPER_MOTOR_FAULT:
        (void)MC_StopMotor1();
        break;

      default:
        break;
    }

    if (serviceStatus.homed)
    {
      serviceStatus.position_permille = PositionPermille(serviceStatus.position_count);
    }
    (void)osDelayUntil(next);
  }
}

void GripperMotorService_CreateTasks(void)
{
  const osThreadAttr_t mediumAttr = {
    .name = "mcMedium",
    .stack_size = 768U * 4U,
    .priority = osPriorityRealtime
  };
  const osThreadAttr_t safetyAttr = {
    .name = "mcSafety",
    .stack_size = 384U * 4U,
    .priority = osPriorityHigh
  };
  const osThreadAttr_t serviceAttr = {
    .name = "gripperMotor",
    .stack_size = 768U * 4U,
    .priority = osPriorityAboveNormal
  };

  mediumTaskHandle = osThreadNew(MediumTask, NULL, &mediumAttr);
  safetyTaskHandle = osThreadNew(SafetyTask, NULL, &safetyAttr);
  serviceTaskHandle = osThreadNew(ServiceTask, NULL, &serviceAttr);
  if ((mediumTaskHandle == NULL) || (safetyTaskHandle == NULL) || (serviceTaskHandle == NULL))
  {
    LatchFault(GRIPPER_FAULT_MOTOR_CONTROL);
  }
}

bool GripperMotor_SetPosition(int16_t position_permille)
{
  if ((!serviceStatus.homed) || (serviceStatus.state == GRIPPER_MOTOR_FAULT))
  {
    return false;
  }
  if (position_permille < 0)
  {
    position_permille = 0;
  }
  else if (position_permille > 1000)
  {
    position_permille = 1000;
  }
  serviceStatus.target_permille = position_permille;
  serviceStatus.state = GRIPPER_MOTOR_POSITIONING;
  return true;
}

bool GripperMotor_SetSpeed(float motor_rpm)
{
  if ((!serviceStatus.homed) || (AbsF(motor_rpm) > MAX_APPLICATION_SPEED_RPM))
  {
    return false;
  }
  CommandSpeed(motor_rpm);
  return true;
}

bool GripperMotor_SetCurrent(float iq_a)
{
  qd_f_t current = {0.0f, 0.0f};
  if ((!serviceStatus.homed) || (AbsF(iq_a) > 1.0f))
  {
    return false;
  }
  current.q = iq_a;
  MC_SetCurrentReferenceMotor1_F(current);
  return true;
}

bool GripperMotor_Rehome(void)
{
  if (serviceStatus.state == GRIPPER_MOTOR_FAULT)
  {
    return false;
  }
  (void)MC_StopMotor1();
  ResetHomeState();
  return true;
}

void GripperMotor_Stop(void)
{
  (void)MC_StopMotor1();
  serviceStatus.state = GRIPPER_MOTOR_STOPPED;
}

bool GripperMotor_ClearFaults(void)
{
  if (!MC_AcknowledgeFaultMotor1())
  {
    return false;
  }
  serviceStatus.faults = GRIPPER_FAULT_NONE;
  ResetHomeState();
  return true;
}

void GripperMotor_GetStatus(GripperMotorStatus_t *status)
{
  if (status != NULL)
  {
    taskENTER_CRITICAL();
    memcpy(status, (const void *)&serviceStatus, sizeof(*status));
    taskEXIT_CRITICAL();
  }
}
