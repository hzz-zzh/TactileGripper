#include "communication_service.h"

#include "communication_types.h"
#include "gripper_can_config.h"
#include "gripper_can_protocol.h"
#include "gripper_service.h"
#include "can_transport.h"
#include "fdcan.h"
#include "cmsis_os2.h"

#include <stdbool.h>
#include <stdint.h>

static osThreadId_t communicationTaskHandle;

static uint32_t CommunicationService_MsToTicks(uint32_t ms)
{
  uint32_t freq = osKernelGetTickFreq();
  uint32_t ticks = (ms * freq + 999U) / 1000U;

  return (ticks == 0U) ? 1U : ticks;
}

static bool CommunicationService_IsMotionState(GripperState_t state)
{
  return (state == GRIPPER_STATE_HOMING_OPEN) ||
         (state == GRIPPER_STATE_HOMING_CLOSE) ||
         (state == GRIPPER_STATE_MOVING_SAFE) ||
         (state == GRIPPER_STATE_MOVING);
}

static void CommunicationService_SendStatus(void)
{
  GripperStatus_t status;
  CanTransportFrame_t frame;

  GripperService_GetStatus(&status);
  GripperCanProtocol_PackStatus(&status,
                                GRIPPER_CAN_NODE_ID,
                                GRIPPER_CAN_MASTER_NODE_ID,
                                &frame);
  (void)CanTransport_Send(&frame);
}

static void CommunicationService_SendFault(void)
{
  GripperStatus_t status;
  CanTransportFrame_t frame;

  GripperService_GetStatus(&status);
  GripperCanProtocol_PackFault(&status,
                               GRIPPER_CAN_NODE_ID,
                               GRIPPER_CAN_MASTER_NODE_ID,
                               &frame);
  (void)CanTransport_Send(&frame);
}

static void CommunicationService_SendDiag(void)
{
  GripperStatus_t status;
  CanTransportFrame_t frame;

  GripperService_GetStatus(&status);
  GripperCanProtocol_PackDiag(&status,
                              GRIPPER_CAN_NODE_ID,
                              GRIPPER_CAN_MASTER_NODE_ID,
                              &frame);
  (void)CanTransport_Send(&frame);
}

static void CommunicationService_SendHeartbeat(void)
{
  GripperStatus_t status;
  CanTransportFrame_t frame;

  GripperService_GetStatus(&status);
  GripperCanProtocol_PackHeartbeat(&status,
                                   GRIPPER_CAN_NODE_ID,
                                   GRIPPER_CAN_MASTER_NODE_ID,
                                   &frame);
  (void)CanTransport_Send(&frame);
}

static void CommunicationService_ProcessControl(
  const CommunicationControlFrame_t *control,
  bool *can_control_active,
  bool *last_sequence_valid,
  uint8_t *last_sequence)
{
  bool duplicate = false;

  if ((control == NULL) || (can_control_active == NULL) ||
      (last_sequence_valid == NULL) || (last_sequence == NULL))
  {
    return;
  }

  if (*last_sequence_valid && (control->sequence == *last_sequence) &&
      (control->command != COMMUNICATION_CONTROL_NOP) &&
      (control->command != COMMUNICATION_CONTROL_STOP) &&
      (control->command != COMMUNICATION_CONTROL_STATUS_ONESHOT))
  {
    duplicate = true;
  }

  if ((control->command != COMMUNICATION_CONTROL_NOP) && !duplicate)
  {
    *last_sequence = control->sequence;
    *last_sequence_valid = true;
  }

  if (duplicate)
  {
    return;
  }

  switch (control->command)
  {
    case COMMUNICATION_CONTROL_NOP:
      break;

    case COMMUNICATION_CONTROL_HOME:
      if (GripperService_Home())
      {
        *can_control_active = true;
      }
      break;

    case COMMUNICATION_CONTROL_SET_POSITION:
      if (GripperService_SetPosition(control->position_permille))
      {
        *can_control_active = true;
      }
      else
      {
        CommunicationService_SendStatus();
        CommunicationService_SendFault();
      }
      break;

    case COMMUNICATION_CONTROL_STOP:
      (void)GripperService_Stop();
      *can_control_active = false;
      break;

    case COMMUNICATION_CONTROL_CLEAR_FAULT:
      (void)GripperService_ClearFaults();
      *can_control_active = false;
      break;

    case COMMUNICATION_CONTROL_STATUS_ONESHOT:
      CommunicationService_SendStatus();
      CommunicationService_SendFault();
      CommunicationService_SendDiag();
      break;

    default:
      break;
  }
}

static void CommunicationService_Task(void *argument)
{
  const uint32_t taskPeriodTicks =
    CommunicationService_MsToTicks(GRIPPER_CAN_TASK_PERIOD_MS);
  const uint32_t statusPeriodTicks =
    CommunicationService_MsToTicks(GRIPPER_CAN_STATUS_PERIOD_MS);
  const uint32_t diagPeriodTicks =
    CommunicationService_MsToTicks(GRIPPER_CAN_DIAG_PERIOD_MS);
  const uint32_t heartbeatPeriodTicks =
    CommunicationService_MsToTicks(GRIPPER_CAN_HEARTBEAT_PERIOD_MS);
  const uint32_t timeoutTicks =
    CommunicationService_MsToTicks(GRIPPER_CAN_CONTROL_TIMEOUT_MS);
  uint32_t next = osKernelGetTickCount();
  uint32_t lastStatusTick = next;
  uint32_t lastDiagTick = next;
  uint32_t lastHeartbeatTick = next;
  uint32_t lastControlTick = next;
  uint32_t lastFaults = 0U;
  uint32_t lastMotorFaults = 0U;
  uint8_t lastSequence = 0U;
  bool lastSequenceValid = false;
  bool hostSeen = false;
  bool canControlActive = false;
  bool timeoutLatched = false;
  (void)argument;

  CommunicationService_SendHeartbeat();

  for (;;)
  {
    CanTransportFrame_t rxFrame;
    uint32_t now;
    GripperStatus_t status;

    next += taskPeriodTicks;
    now = osKernelGetTickCount();

    while (CanTransport_Poll(&rxFrame))
    {
      CommunicationControlFrame_t control;

      if (GripperCanProtocol_DecodeControl(&rxFrame,
                                           GRIPPER_CAN_NODE_ID,
                                           GRIPPER_CAN_BROADCAST_NODE_ID,
                                           &control))
      {
        hostSeen = true;
        timeoutLatched = false;
        lastControlTick = now;
        CommunicationService_ProcessControl(&control,
                                            &canControlActive,
                                            &lastSequenceValid,
                                            &lastSequence);
      }
    }

    GripperService_GetStatus(&status);
    if (hostSeen && canControlActive &&
        CommunicationService_IsMotionState(status.state) &&
        ((uint32_t)(now - lastControlTick) >= timeoutTicks) &&
        !timeoutLatched)
    {
      /*
       * CAN接管运动后需要主站持续在线。超时只触发一次，避免队列被重复故障填满。
       */
      (void)GripperService_Stop();
      (void)GripperService_LatchExternalFault(GRIPPER_FAULT_COMMUNICATION);
      canControlActive = false;
      timeoutLatched = true;
    }

    if ((uint32_t)(now - lastStatusTick) >= statusPeriodTicks)
    {
      CommunicationService_SendStatus();
      lastStatusTick = now;
    }
    if (((status.faults != lastFaults) ||
         (status.motor_faults != lastMotorFaults)) ||
        ((uint32_t)(now - lastDiagTick) >= diagPeriodTicks))
    {
      CommunicationService_SendFault();
      CommunicationService_SendDiag();
      lastFaults = status.faults;
      lastMotorFaults = status.motor_faults;
      lastDiagTick = now;
    }
    if ((uint32_t)(now - lastHeartbeatTick) >= heartbeatPeriodTicks)
    {
      CommunicationService_SendHeartbeat();
      lastHeartbeatTick = now;
    }

    (void)osDelayUntil(next);
  }
}

void CommunicationService_CreateTask(void)
{
  const osThreadAttr_t taskAttr = {
    .name = "communication",
    .stack_size = 768U * 4U,
    .priority = osPriorityNormal
  };

  if (!CanTransport_Init(&hfdcan1,
                         GRIPPER_CAN_NODE_ID,
                         GRIPPER_CAN_BROADCAST_NODE_ID) ||
      !CanTransport_Start())
  {
    (void)GripperService_LatchExternalFault(GRIPPER_FAULT_COMMUNICATION);
    return;
  }

  communicationTaskHandle =
    osThreadNew(CommunicationService_Task, NULL, &taskAttr);
  if (communicationTaskHandle == NULL)
  {
    (void)GripperService_LatchExternalFault(GRIPPER_FAULT_COMMUNICATION);
  }
}
