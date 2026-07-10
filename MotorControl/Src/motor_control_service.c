#include "motor_control_service.h"

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
#include "debug_monitor.h"

#include "stm32h7xx_ll_dma.h"

#include <string.h>

#define MOTOR_FEEDBACK_PERIOD_TICKS       10U  /* FreeRTOS 2 kHz，反馈任务周期5 ms。 */
#define MOTOR_START_TIMEOUT_STEPS         400U
#define MOTOR_STARTUP_GRACE_STEPS         100U
#define MOTOR_MIN_BUS_VOLTAGE_V           10U
#define MOTOR_MAX_BUS_VOLTAGE_V           30U
#define MOTOR_MAX_COMMAND_SPEED_RPM       6000.0f
#define MOTOR_MAX_COMMAND_CURRENT_A       2.0f
#define MOTOR_SOFTWARE_TRIP_CURRENT_A     2.3f
#define MOTOR_DEFAULT_CURRENT_LIMIT_A     0.6f
#define MOTOR_SPEED_INTEGRAL_LIMIT_A      1.5f
#define MOTOR_ALIGNMENT_ELECTRICAL_DEG    0

static osThreadId_t mediumTaskHandle;
static osThreadId_t safetyTaskHandle;
static osThreadId_t feedbackTaskHandle;
static volatile MotorControlStatus_t motorStatus;
static uint32_t feedbackSteps;
static uint32_t startSteps;

static float MotorControlService_AbsF(float value)
{
  return (value < 0.0f) ? -value : value;
}

static float MotorControlService_ClampF(float value,
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

static void MotorControlService_LatchFault(uint32_t fault)
{
  motorStatus.faults |= fault;
  motorStatus.state = MOTOR_CONTROL_STATE_FAULT;
  motorStatus.speed_ref_rpm = 0.0f;
  motorStatus.iq_ref_a = 0.0f;
  motorStatus.id_ref_a = 0.0f;
  (void)MC_StopMotor1();
}

void MotorControlService_SetSpeedCurrentLimit(float current_a)
{
  int16_t outputLimit;
  int16_t integralLimit;
  float integralLimitA;

  current_a = MotorControlService_ClampF(current_a, 0.0f,
                                         MOTOR_MAX_COMMAND_CURRENT_A);
  integralLimitA = MotorControlService_ClampF(current_a, 0.0f,
                                               MOTOR_SPEED_INTEGRAL_LIMIT_A);
  outputLimit = (int16_t)(current_a * CURRENT_CONV_FACTOR);
  integralLimit = (int16_t)(integralLimitA * CURRENT_CONV_FACTOR);

  /* 总Iq允许短时提供较大转矩，积分项单独收紧，防止堵转时长时间饱和。 */
  PID_SetUpperOutputLimit(&PIDSpeedHandle_M1, outputLimit);
  PID_SetLowerOutputLimit(&PIDSpeedHandle_M1, (int16_t)-outputLimit);
  PID_SetUpperIntegralTermLimit(&PIDSpeedHandle_M1,
                                (int32_t)integralLimit * SP_KIDIV);
  PID_SetLowerIntegralTermLimit(&PIDSpeedHandle_M1,
                                -(int32_t)integralLimit * SP_KIDIV);
}

void MotorControlService_ResetSpeedController(void)
{
  /* 堵转换向前清除速度积分，避免原方向饱和转矩造成反向启动延迟。 */
  taskENTER_CRITICAL();
  PID_SetIntegralTerm(&PIDSpeedHandle_M1, 0);
  taskEXIT_CRITICAL();
}

static void MotorControlService_UpdateFeedback(void)
{
  qd_f_t measured = MC_GetIqdMotor1_F();
  qd_f_t reference = MC_GetIqdrefMotor1_F();

  motorStatus.position_count = KTH7812_GetMultiTurnCount(&KTH7812_M1);
  motorStatus.speed_rpm = MC_GetAverageMecSpeedMotor1_F();
  motorStatus.iq_a = measured.q;
  motorStatus.id_a = measured.d;
  motorStatus.iq_ref_a = reference.q;
  motorStatus.id_ref_a = reference.d;
  motorStatus.bus_current_raw = RCM_GetRegularConv(&BusCurrentRegConv_M1);
  motorStatus.bus_voltage_raw = RCM_GetRegularConv(&VbusRegConv_M1);
  motorStatus.temperature_raw = RCM_GetRegularConv(&TempRegConv_M1);
  motorStatus.bus_current_a =
    ((int32_t)motorStatus.bus_current_raw -
     (int32_t)DebugMonitor_GetBusCurrentZero()) * CURRENT_CONV_FACTOR_INV;
  motorStatus.bus_voltage_v =
    VBS_GetAvBusVoltage_V(&BusVoltageSensor_M1._Super);
  motorStatus.mc_faults = MC_GetCurrentFaultsMotor1();
  motorStatus.mc_occurred_faults = MC_GetOccurredFaultsMotor1();
  motorStatus.encoder_last_frame = KTH7812_M1.last_frame;
  motorStatus.encoder_frames = KTH7812_M1.frame_count;
  motorStatus.encoder_spi_errors = KTH7812_M1.spi_error_count;
  motorStatus.encoder_consecutive_errors = KTH7812_M1.consecutive_errors;
  motorStatus.encoder_reliable = KTH7812_IsReliable(&KTH7812_M1);
}

static void MotorControlService_MediumTask(void *argument)
{
  uint32_t next = osKernelGetTickCount();
  (void)argument;

  for (;;)
  {
    next += 1U;
    /*
     * USART1只保留硬件初始化，ASPEP未启动，不能轮询其DMA接收标志。
     * 恢复Motor Pilot时需要与mc_tasks.c中的ASPEP_start同步恢复。
     */
    MC_RunMotorControlTasks();
    (void)osDelayUntil(next);
  }
}

static void MotorControlService_SafetyTask(void *argument)
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

static void MotorControlService_FeedbackTask(void *argument)
{
  uint32_t next = osKernelGetTickCount();
  (void)argument;

  memset((void *)&motorStatus, 0, sizeof(motorStatus));
  motorStatus.state = MOTOR_CONTROL_STATE_BOOT;
  MotorControlService_SetSpeedCurrentLimit(MOTOR_DEFAULT_CURRENT_LIMIT_A);

  for (;;)
  {
    MCI_State_t mcState;
    next += MOTOR_FEEDBACK_PERIOD_TICKS;
    feedbackSteps++;
    MotorControlService_UpdateFeedback();
    mcState = MC_GetSTMStateMotor1();

    /* 编码器与母线滤波值在启动后需要短暂建立，宽限期内不锁存故障。 */
    if (feedbackSteps >= MOTOR_STARTUP_GRACE_STEPS)
    {
      if (!motorStatus.encoder_reliable)
      {
        MotorControlService_LatchFault(MOTOR_CONTROL_FAULT_ENCODER);
      }
      else if ((motorStatus.bus_voltage_v < MOTOR_MIN_BUS_VOLTAGE_V) ||
               (motorStatus.bus_voltage_v > MOTOR_MAX_BUS_VOLTAGE_V))
      {
        MotorControlService_LatchFault(MOTOR_CONTROL_FAULT_BUS_VOLTAGE);
      }
    }

    if ((feedbackSteps >= MOTOR_STARTUP_GRACE_STEPS) &&
        (motorStatus.state != MOTOR_CONTROL_STATE_FAULT) &&
        (motorStatus.mc_faults != 0U))
    {
      MotorControlService_LatchFault(MOTOR_CONTROL_FAULT_MCSDK);
    }

    if (((mcState == ALIGNMENT) || (mcState == RUN)) &&
        ((MotorControlService_AbsF(motorStatus.iq_a) >
          MOTOR_SOFTWARE_TRIP_CURRENT_A) ||
         (MotorControlService_AbsF(motorStatus.id_a) >
          MOTOR_SOFTWARE_TRIP_CURRENT_A)))
    {
      MotorControlService_LatchFault(MOTOR_CONTROL_FAULT_OVER_CURRENT);
    }

    if (motorStatus.state == MOTOR_CONTROL_STATE_BOOT)
    {
      if (feedbackSteps >= MOTOR_STARTUP_GRACE_STEPS)
      {
        motorStatus.state = MOTOR_CONTROL_STATE_IDLE;
      }
    }
    else if (motorStatus.state == MOTOR_CONTROL_STATE_STARTING)
    {
      if (mcState == RUN)
      {
        motorStatus.state = MOTOR_CONTROL_STATE_RUNNING;
        startSteps = 0U;
      }
      else if (++startSteps > MOTOR_START_TIMEOUT_STEPS)
      {
        MotorControlService_LatchFault(MOTOR_CONTROL_FAULT_START_TIMEOUT);
      }
    }
    else if ((motorStatus.state == MOTOR_CONTROL_STATE_STOPPING) &&
             (mcState == IDLE))
    {
      motorStatus.state = MOTOR_CONTROL_STATE_IDLE;
    }

    (void)osDelayUntil(next);
  }
}

void MotorControlService_CreateTasks(void)
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
  const osThreadAttr_t feedbackAttr = {
    .name = "motorFeedback",
    .stack_size = 512U * 4U,
    .priority = osPriorityAboveNormal
  };

  mediumTaskHandle = osThreadNew(MotorControlService_MediumTask, NULL,
                                 &mediumAttr);
  safetyTaskHandle = osThreadNew(MotorControlService_SafetyTask, NULL,
                                 &safetyAttr);
  feedbackTaskHandle = osThreadNew(MotorControlService_FeedbackTask, NULL,
                                   &feedbackAttr);
  if ((mediumTaskHandle == NULL) || (safetyTaskHandle == NULL) ||
      (feedbackTaskHandle == NULL))
  {
    MotorControlService_LatchFault(MOTOR_CONTROL_FAULT_MCSDK);
  }
}

bool MotorControlService_Start(void)
{
  bool accepted;

  taskENTER_CRITICAL();
  if ((motorStatus.state == MOTOR_CONTROL_STATE_RUNNING) ||
      (motorStatus.state == MOTOR_CONTROL_STATE_STARTING))
  {
    taskEXIT_CRITICAL();
    return true;
  }
  if ((motorStatus.state != MOTOR_CONTROL_STATE_IDLE) ||
      (motorStatus.faults != MOTOR_CONTROL_FAULT_NONE))
  {
    taskEXIT_CRITICAL();
    return false;
  }
  motorStatus.state = MOTOR_CONTROL_STATE_STARTING;
  startSteps = 0U;
  taskEXIT_CRITICAL();

  TSK_SetAlignmentAngleM1(MOTOR_ALIGNMENT_ELECTRICAL_DEG);
  MC_ProgramSpeedRampMotor1_F(0.0f, 0U);
  accepted = MC_StartMotor1();
  if (!accepted)
  {
    taskENTER_CRITICAL();
    motorStatus.state = MOTOR_CONTROL_STATE_IDLE;
    taskEXIT_CRITICAL();
  }
  return accepted;
}

void MotorControlService_Stop(void)
{
  (void)MC_StopMotor1();
  taskENTER_CRITICAL();
  motorStatus.speed_ref_rpm = 0.0f;
  motorStatus.iq_ref_a = 0.0f;
  motorStatus.id_ref_a = 0.0f;
  if (motorStatus.state != MOTOR_CONTROL_STATE_FAULT)
  {
    motorStatus.state = MOTOR_CONTROL_STATE_STOPPING;
  }
  taskEXIT_CRITICAL();
}

bool MotorControlService_ClearFaults(void)
{
  if ((motorStatus.state == MOTOR_CONTROL_STATE_STARTING) ||
      (motorStatus.state == MOTOR_CONTROL_STATE_RUNNING) ||
      (motorStatus.state == MOTOR_CONTROL_STATE_STOPPING))
  {
    return false;
  }
  if ((MC_GetCurrentFaultsMotor1() != 0U) &&
      !MC_AcknowledgeFaultMotor1())
  {
    return false;
  }
  if (MC_GetCurrentFaultsMotor1() != 0U)
  {
    return false;
  }

  taskENTER_CRITICAL();
  motorStatus.faults = MOTOR_CONTROL_FAULT_NONE;
  motorStatus.state = MOTOR_CONTROL_STATE_IDLE;
  taskEXIT_CRITICAL();
  return true;
}

bool MotorControlService_SetSpeed(float speed_rpm, uint16_t ramp_ms)
{
  if ((motorStatus.state != MOTOR_CONTROL_STATE_STARTING) &&
      (motorStatus.state != MOTOR_CONTROL_STATE_RUNNING))
  {
    return false;
  }

  speed_rpm = MotorControlService_ClampF(speed_rpm,
                                         -MOTOR_MAX_COMMAND_SPEED_RPM,
                                         MOTOR_MAX_COMMAND_SPEED_RPM);
  /* 相同目标无需重复规划斜坡，否则周期调用会不断重启同一条速度斜坡。 */
  if (MotorControlService_AbsF(speed_rpm - motorStatus.speed_ref_rpm) < 0.1f)
  {
    return true;
  }
  motorStatus.speed_ref_rpm = speed_rpm;
  MC_ProgramSpeedRampMotor1_F(speed_rpm, ramp_ms);
  return true;
}

bool MotorControlService_SetCurrent(float iq_a, float id_a)
{
  qd_f_t current = {0.0f, 0.0f};

  if (motorStatus.state != MOTOR_CONTROL_STATE_RUNNING)
  {
    return false;
  }
  if ((MotorControlService_AbsF(iq_a) > MOTOR_MAX_COMMAND_CURRENT_A) ||
      (MotorControlService_AbsF(id_a) > MOTOR_MAX_COMMAND_CURRENT_A))
  {
    return false;
  }

  current.q = iq_a;
  current.d = id_a;
  motorStatus.iq_ref_a = iq_a;
  motorStatus.id_ref_a = id_a;
  MC_SetCurrentReferenceMotor1_F(current);
  return true;
}

void MotorControlService_GetStatus(MotorControlStatus_t *status)
{
  if (status != NULL)
  {
    taskENTER_CRITICAL();
    memcpy(status, (const void *)&motorStatus, sizeof(*status));
    taskEXIT_CRITICAL();
  }
}
