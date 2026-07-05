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
#include "debug_monitor.h"

#include "stm32h7xx_ll_dma.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define SERVICE_PERIOD_TICKS          10U  /* 5 ms at 2 kHz */
#define SERVICE_PERIOD_MS             5U
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
#define MOTOR_TO_OUTPUT_GEAR_RATIO    10.0f  /* 当前夹爪使用10:1减速器，编码器计数仍按电机轴计算。 */
#define CURRENT_LOOP_COMMISSIONING_MODE 1
#define COMMISSIONING_CONTROL_MODE_CURRENT 0U
#define COMMISSIONING_CONTROL_MODE_SPEED   1U
/* 调试主模式切换：CURRENT=纯电流环，SPEED=速度外环加电流内环。 */
#define COMMISSIONING_CONTROL_MODE COMMISSIONING_CONTROL_MODE_CURRENT
#define COMMISSIONING_CURRENT_IQ_A       0.250f  /* 当前电流环测试只给Iq轴0.5A，Id轴保持0A。 */
#define COMMISSIONING_CURRENT_ID_A       0.00f
#define COMMISSIONING_CURRENT_RAMP_MS    800U   /* 保留短斜坡，避免按P瞬间给满0.5A。 */
#define COMMISSIONING_SPEED_TARGET_RPM 3000.0f /* 电机轴3000rpm对应减速器输出端约300rpm。 */
#define COMMISSIONING_SPEED_RAMP_MS    3000U  /* 使用3秒斜坡升速，避免齿轮箱受到突然冲击。 */
#define COMMISSIONING_SPEED_KP         4000
#define COMMISSIONING_SPEED_KI         1600
#define COMMISSIONING_CURRENT_LIMIT_A  0.60f  /* 速度环最大只允许输出600mA。 */
#define COMMISSIONING_TRIP_CURRENT_A   1.50f  /* 0.5A电流环测试时，任一dq电流超过900mA立即关PWM。 */
#define COMMISSIONING_START_TIMEOUT    400U  /* 2 s at 5 ms. */
#define COMMISSIONING_SCAN_COUNT        1U
#define COMMISSIONING_MAX_TRAVEL_COUNTS (KTH7812_COUNTS_PER_TURN * 4L)
#define COMMISSIONING_MAX_SPEED_RPM     6000.0f /* 目标3000rpm，超过3500rpm立即停机。 */

/* 编码器方向确认后，使用受限电流的3000rpm速度环验证高速连续运行。 */
static const int16_t commissioningScanAngles[COMMISSIONING_SCAN_COUNT] =
{
  0
};
static const char *const commissioningReadyText[COMMISSIONING_SCAN_COUNT] =
{
#if (COMMISSIONING_CONTROL_MODE == COMMISSIONING_CONTROL_MODE_CURRENT)
  "FOC32,READY,mode=current,enc_dir=-1,iq_mA=500,id_mA=0,ramp_ms=800,trip_mA=900,start=P,stop=S\r\n"
#else
  "FOC31,READY,mode=speed,enc_dir=-1,speed_rpm=3000,output_rpm=300,ramp_ms=3000,limit_mA=600,overspeed=3500,start=P,stop=S\r\n"
#endif
};
static const char *const commissioningStartText[COMMISSIONING_SCAN_COUNT] =
{
#if (COMMISSIONING_CONTROL_MODE == COMMISSIONING_CONTROL_MODE_CURRENT)
  "FOC32,START,mode=current,enc_dir=-1,align_deg=0,align_id_mA=300,align_ms=500\r\n"
#else
  "FOC31,START,mode=speed,enc_dir=-1,align_deg=0,align_id_mA=300,align_ms=500\r\n"
#endif
};

extern UART_HandleTypeDef huart2;

static osThreadId_t mediumTaskHandle;
static osThreadId_t safetyTaskHandle;
static osThreadId_t serviceTaskHandle;

static volatile GripperMotorStatus_t serviceStatus;
static uint32_t stateSteps;
static uint32_t stallSteps;
static bool startIssued;

static void CommissioningWrite(const char *text)
{
  /* 调试阶段仅发送短文本，避免阻塞电机控制任务过久。 */
  (void)HAL_UART_Transmit(&huart2, (const uint8_t *)text,
                          (uint16_t)strlen(text), 10U);
}

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

static void CommandSpeedRamp(float rpm, uint16_t duration_ms)
{
  MC_ProgramSpeedRampMotor1_F(rpm, duration_ms);
}

static void CommandCurrentReference(float iq_a, float id_a)
{
  qd_f_t current = {0.0f, 0.0f};
  /* 直接写dq电流参考，绕开速度外环，用于单独验证电流环和编码器方向。 */
  current.q = iq_a;
  current.d = id_a;
  MC_SetCurrentReferenceMotor1_F(current);
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
    serviceStatus.bus_current_a = ((int32_t)serviceStatus.bus_current_raw -
                                  (int32_t)DebugMonitor_GetBusCurrentZero()) *
                                  CURRENT_CONV_FACTOR_INV;
    serviceStatus.bus_voltage_v = VBS_GetAvBusVoltage_V(&BusVoltageSensor_M1._Super);
    serviceStatus.mc_faults = MC_GetCurrentFaultsMotor1();
    serviceStatus.mc_occurred_faults = MC_GetOccurredFaultsMotor1();
    serviceStatus.encoder_last_frame = KTH7812_M1.last_frame;
    serviceStatus.encoder_frames = KTH7812_M1.frame_count;
    serviceStatus.encoder_spi_errors = KTH7812_M1.spi_error_count;
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

static void CurrentLoopCommissioningTask(void *argument)
{
  uint32_t next = osKernelGetTickCount();
  uint32_t steps = 0U;
  uint8_t command = 0U;
  uint8_t scanIndex = 0U;
  int32_t torqueStartPosition = 0;
  float currentRampTargetIqA = 0.0f;
  bool precheckPassed = false;
  bool scanComplete = false;
  (void)argument;

  memset((void *)&serviceStatus, 0, sizeof(serviceStatus));
  serviceStatus.state = GRIPPER_MOTOR_PRECHECK;

  for (;;)
  {
    qd_f_t measured;
    next += SERVICE_PERIOD_TICKS;

    measured = MC_GetIqdMotor1_F();
    serviceStatus.position_count = KTH7812_GetMultiTurnCount(&KTH7812_M1);
    serviceStatus.speed_rpm = MC_GetAverageMecSpeedMotor1_F();
    serviceStatus.iq_a = measured.q;
    serviceStatus.bus_current_raw = RCM_GetRegularConv(&BusCurrentRegConv_M1);
    serviceStatus.bus_current_a = ((int32_t)serviceStatus.bus_current_raw -
                                  (int32_t)DebugMonitor_GetBusCurrentZero()) *
                                  CURRENT_CONV_FACTOR_INV;
    serviceStatus.bus_voltage_v = VBS_GetAvBusVoltage_V(&BusVoltageSensor_M1._Super);
    serviceStatus.mc_faults = MC_GetCurrentFaultsMotor1();
    serviceStatus.mc_occurred_faults = MC_GetOccurredFaultsMotor1();
    serviceStatus.encoder_last_frame = KTH7812_M1.last_frame;
    serviceStatus.encoder_reliable = KTH7812_IsReliable(&KTH7812_M1);

    if (((MC_GetSTMStateMotor1() == ALIGNMENT) ||
         (MC_GetSTMStateMotor1() == RUN)) &&
        (serviceStatus.state != GRIPPER_MOTOR_FAULT) &&
        ((AbsF(measured.q) > COMMISSIONING_TRIP_CURRENT_A) ||
         (AbsF(measured.d) > COMMISSIONING_TRIP_CURRENT_A)))
    {
      ab_f_t phase = MC_GetIabMotor1_F();
      char tripLine[176];
      /* 记录保护瞬间的dq电流、三相电流和实际FOC角度，便于直接判断反馈方向。 */
      (void)snprintf(tripLine, sizeof(tripLine),
                     "FOC32,TRIP,iq=%ld,id=%ld,ia=%ld,ib=%ld,ic=%ld,el=%d,reason=over_900mA\r\n",
                     (long)(measured.q * 1000.0f),
                     (long)(measured.d * 1000.0f),
                     (long)(phase.a * 1000.0f),
                     (long)(phase.b * 1000.0f),
                     (long)((-phase.a - phase.b) * 1000.0f),
                     (int)MC_GetElAngledppMotor1());
      CommissioningWrite(tripLine);
      LatchFault(GRIPPER_FAULT_MOTOR_CONTROL);
    }

    if ((serviceStatus.state != GRIPPER_MOTOR_PRECHECK) &&
        (serviceStatus.state != GRIPPER_MOTOR_STOPPED) &&
        (serviceStatus.mc_faults != 0U))
    {
      LatchFault(GRIPPER_FAULT_MOTOR_CONTROL);
    }

    switch (serviceStatus.state)
    {
      case GRIPPER_MOTOR_PRECHECK:
        if ((!precheckPassed) && (++steps >= 100U))
        {
          if (!serviceStatus.encoder_reliable)
          {
            LatchFault(GRIPPER_FAULT_ENCODER);
          }
          else if ((serviceStatus.bus_voltage_v < 10U) ||
                   (serviceStatus.bus_voltage_v > 30U))
          {
            LatchFault(GRIPPER_FAULT_BUS_VOLTAGE);
          }
          else if (serviceStatus.mc_faults != 0U)
          {
            LatchFault(GRIPPER_FAULT_MOTOR_CONTROL);
          }
          else if ((PWM_Handle_M1.PhaseAOffset < 1000U) ||
                   (PWM_Handle_M1.PhaseBOffset < 1000U) ||
                   (PWM_Handle_M1.PhaseCOffset < 1000U))
          {
            /* 在开PWM前确认三相零偏都已生成，避免单ADC状态进入闭环。 */
            CommissioningWrite("FOC32,ABORT,reason=phase_offset_invalid\r\n");
            LatchFault(GRIPPER_FAULT_MOTOR_CONTROL);
          }
          else
          {
            precheckPassed = true;
            steps = 0U;
            CommissioningWrite(commissioningReadyText[scanIndex]);
          }
        }
        else if (precheckPassed &&
                 (HAL_UART_Receive(&huart2, &command, 1U, 0U) == HAL_OK) &&
                 (command == (uint8_t)'P'))
        {
          /* 启动前设置本档静态电角度，运行过程中不突变角度。 */
          TSK_SetAlignmentAngleM1(commissioningScanAngles[scanIndex]);
          CommissioningWrite(commissioningStartText[scanIndex]);
          SetSpeedCurrentLimit(COMMISSIONING_CURRENT_LIMIT_A);
#if (COMMISSIONING_CONTROL_MODE == COMMISSIONING_CONTROL_MODE_CURRENT)
          CommandCurrentReference(0.0f, 0.0f);
#else
          CommandSpeedRamp(0.0f, 0U);
#endif
          if (MC_StartMotor1())
          {
            serviceStatus.state = GRIPPER_MOTOR_ALIGNING;
            steps = 0U;
          }
          else
          {
            CommissioningWrite("FOC32,ABORT,reason=mc_start_rejected\r\n");
            LatchFault(GRIPPER_FAULT_MOTOR_CONTROL);
          }
        }
        break;

      case GRIPPER_MOTOR_ALIGNING:
        if (MC_GetSTMStateMotor1() == RUN)
        {
          char runLine[192];
          torqueStartPosition = serviceStatus.position_count;
          currentRampTargetIqA = 0.0f;
#if (COMMISSIONING_CONTROL_MODE == COMMISSIONING_CONTROL_MODE_CURRENT)
          CommandCurrentReference(0.0f, COMMISSIONING_CURRENT_ID_A);
          serviceStatus.state = GRIPPER_MOTOR_POSITIONING;
          steps = 0U;
          /* 编码器方向已经确认，这里进入纯Iq电流环，速度环不再给目标。 */
          (void)snprintf(runLine, sizeof(runLine),
                         "FOC32,RUN,mode=current,iq_mA=500,id_mA=0,ramp_ms=800,"
                         "trip_mA=900,iq_kp=%d,iq_ki=%d,id_kp=%d,id_ki=%d,pos0=%ld\r\n",
                         PID_TORQUE_KP_DEFAULT,
                         PID_TORQUE_KI_DEFAULT,
                         PID_FLUX_KP_DEFAULT,
                         PID_FLUX_KI_DEFAULT,
                         (long)torqueStartPosition);
          CommissioningWrite(runLine);
#else
          SetSpeedCurrentLimit(COMMISSIONING_CURRENT_LIMIT_A);
          PID_SetKP(&PIDSpeedHandle_M1, COMMISSIONING_SPEED_KP);
          PID_SetKI(&PIDSpeedHandle_M1, COMMISSIONING_SPEED_KI);
          PID_SetIntegralTerm(&PIDSpeedHandle_M1, 0);
          CommandSpeedRamp(COMMISSIONING_SPEED_TARGET_RPM, COMMISSIONING_SPEED_RAMP_MS);
          serviceStatus.state = GRIPPER_MOTOR_POSITIONING;
          steps = 0U;
          /* 编码器方向已经验证，从零速用3秒斜坡进入3000rpm速度闭环。 */
          (void)snprintf(runLine, sizeof(runLine),
                         "FOC31,RUN,enc_dir=-1,target_rpm=3000,output_rpm=300,ramp_ms=3000,kp=4000,ki=1600,limit_mA=600,pos0=%ld\r\n",
                         (long)torqueStartPosition);
          CommissioningWrite(runLine);
#endif
        }
        else if (++steps > COMMISSIONING_START_TIMEOUT)
        {
          LatchFault(GRIPPER_FAULT_MOTOR_CONTROL);
        }
        break;

      case GRIPPER_MOTOR_POSITIONING:
      {
        int32_t travel = serviceStatus.position_count - torqueStartPosition;
        bool stopRequested = ((HAL_UART_Receive(&huart2, &command, 1U, 0U) == HAL_OK) &&
                              ((command == (uint8_t)'S') || (command == (uint8_t)'s')));
        if (stopRequested)
        {
          char stopLine[144];
          (void)MC_StopMotor1();
          serviceStatus.state = GRIPPER_MOTOR_STOPPED;
          /* 速度环运行期间收到S就立即关闭PWM。 */
#if (COMMISSIONING_CONTROL_MODE == COMMISSIONING_CONTROL_MODE_CURRENT)
          (void)snprintf(stopLine, sizeof(stopLine),
                         "FOC32,STOP,pwm=off,reason=uart_s,delta=%ld,speed_mrpm=%ld\r\n",
                         (long)travel, (long)(serviceStatus.speed_rpm * 1000.0f));
#else
          (void)snprintf(stopLine, sizeof(stopLine),
                         "FOC31,STOP,pwm=off,reason=uart_s,delta=%ld,speed_mrpm=%ld\r\n",
                         (long)travel, (long)(serviceStatus.speed_rpm * 1000.0f));
#endif
          CommissioningWrite(stopLine);
        }
#if (COMMISSIONING_CONTROL_MODE == COMMISSIONING_CONTROL_MODE_CURRENT)
        else
        {
          float rampStep = (COMMISSIONING_CURRENT_IQ_A * (float)SERVICE_PERIOD_MS) /
                           (float)COMMISSIONING_CURRENT_RAMP_MS;
          if (currentRampTargetIqA < COMMISSIONING_CURRENT_IQ_A)
          {
            currentRampTargetIqA += rampStep;
            if (currentRampTargetIqA > COMMISSIONING_CURRENT_IQ_A)
            {
              currentRampTargetIqA = COMMISSIONING_CURRENT_IQ_A;
            }
          }
          /* 运行态持续写Iq=0.5A、Id=0A，用于观察纯电流环是否稳定。 */
          CommandCurrentReference(currentRampTargetIqA, COMMISSIONING_CURRENT_ID_A);
          if (AbsF(serviceStatus.speed_rpm) > COMMISSIONING_MAX_SPEED_RPM)
          {
            char motionTripLine[128];
            (void)snprintf(motionTripLine, sizeof(motionTripLine),
                           "FOC32,TRIP,reason=overspeed_3500rpm,current_mode=1,delta=%ld,speed_mrpm=%ld\r\n",
                           (long)travel, (long)(serviceStatus.speed_rpm * 1000.0f));
            CommissioningWrite(motionTripLine);
            LatchFault(GRIPPER_FAULT_MOTOR_CONTROL);
          }
        }
#else
        else if (AbsF(serviceStatus.speed_rpm) > COMMISSIONING_MAX_SPEED_RPM)
        {
          char motionTripLine[128];
          (void)snprintf(motionTripLine, sizeof(motionTripLine),
                         "FOC31,TRIP,reason=overspeed_3500rpm,delta=%ld,speed_mrpm=%ld\r\n",
                         (long)travel, (long)(serviceStatus.speed_rpm * 1000.0f));
          CommissioningWrite(motionTripLine);
          LatchFault(GRIPPER_FAULT_MOTOR_CONTROL);
        }
#endif
        break;
      }

      case GRIPPER_MOTOR_FAULT:
        (void)MC_StopMotor1();
        break;

      case GRIPPER_MOTOR_STOPPED:
        if ((!scanComplete) && (MC_GetSTMStateMotor1() == IDLE))
        {
          scanIndex++;
          if (scanIndex < COMMISSIONING_SCAN_COUNT)
          {
            /* 等待下一次人工按P，防止电机连续跨角度运行。 */
            precheckPassed = false;
            steps = 0U;
            serviceStatus.state = GRIPPER_MOTOR_PRECHECK;
          }
          else
          {
            scanComplete = true;
#if (COMMISSIONING_CONTROL_MODE == COMMISSIONING_CONTROL_MODE_CURRENT)
            CommissioningWrite("FOC32,COMPLETE,current_loop_test=1,pwm=off\r\n");
#else
            CommissioningWrite("FOC31,COMPLETE,high_speed_loop_test=1,pwm=off\r\n");
#endif
          }
        }
        break;

      default:
        break;
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
  serviceTaskHandle = osThreadNew((CURRENT_LOOP_COMMISSIONING_MODE != 0) ?
                                  CurrentLoopCommissioningTask : ServiceTask,
                                  NULL, &serviceAttr);
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
