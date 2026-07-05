#include "gripper_can_protocol.h"

#include "motor_control_service.h"

#include <string.h>

#define GRIPPER_CAN_PRIORITY_DEFAULT  6U
#define GRIPPER_CAN_PRIORITY_SHIFT    26U
#define GRIPPER_CAN_TYPE_SHIFT        18U
#define GRIPPER_CAN_SOURCE_SHIFT      10U
#define GRIPPER_CAN_TARGET_SHIFT      2U
#define GRIPPER_CAN_NODE_MASK         0xFFU
#define GRIPPER_CAN_TYPE_MASK         0xFFU
#define GRIPPER_CAN_PRIORITY_MASK     0x07U

static void GripperCanProtocol_WriteU16(uint8_t *data,
                                        uint8_t offset,
                                        uint16_t value)
{
  data[offset] = (uint8_t)(value & 0xFFU);
  data[offset + 1U] = (uint8_t)((value >> 8U) & 0xFFU);
}

static void GripperCanProtocol_WriteI16(uint8_t *data,
                                        uint8_t offset,
                                        int16_t value)
{
  GripperCanProtocol_WriteU16(data, offset, (uint16_t)value);
}

static void GripperCanProtocol_WriteU32(uint8_t *data,
                                        uint8_t offset,
                                        uint32_t value)
{
  data[offset] = (uint8_t)(value & 0xFFU);
  data[offset + 1U] = (uint8_t)((value >> 8U) & 0xFFU);
  data[offset + 2U] = (uint8_t)((value >> 16U) & 0xFFU);
  data[offset + 3U] = (uint8_t)((value >> 24U) & 0xFFU);
}

static int16_t GripperCanProtocol_SaturateI16(int32_t value)
{
  if (value > 32767L)
  {
    return 32767;
  }
  if (value < -32768L)
  {
    return -32768;
  }
  return (int16_t)value;
}

static uint16_t GripperCanProtocol_SaturateU16(uint32_t value)
{
  return (value > 65535UL) ? 65535U : (uint16_t)value;
}

uint32_t GripperCanProtocol_MakeId(uint8_t priority,
                                   GripperCanMessageType_t type,
                                   uint8_t source_node,
                                   uint8_t target_node)
{
  return (((uint32_t)priority & GRIPPER_CAN_PRIORITY_MASK)
          << GRIPPER_CAN_PRIORITY_SHIFT) |
         (((uint32_t)type & GRIPPER_CAN_TYPE_MASK)
          << GRIPPER_CAN_TYPE_SHIFT) |
         (((uint32_t)source_node & GRIPPER_CAN_NODE_MASK)
          << GRIPPER_CAN_SOURCE_SHIFT) |
         (((uint32_t)target_node & GRIPPER_CAN_NODE_MASK)
          << GRIPPER_CAN_TARGET_SHIFT);
}

uint8_t GripperCanProtocol_GetTargetNode(uint32_t id)
{
  return (uint8_t)((id >> GRIPPER_CAN_TARGET_SHIFT) &
                   GRIPPER_CAN_NODE_MASK);
}

uint8_t GripperCanProtocol_GetSourceNode(uint32_t id)
{
  return (uint8_t)((id >> GRIPPER_CAN_SOURCE_SHIFT) &
                   GRIPPER_CAN_NODE_MASK);
}

GripperCanMessageType_t GripperCanProtocol_GetMessageType(uint32_t id)
{
  return (GripperCanMessageType_t)((id >> GRIPPER_CAN_TYPE_SHIFT) &
                                   GRIPPER_CAN_TYPE_MASK);
}

bool GripperCanProtocol_DecodeControl(const CanTransportFrame_t *frame,
                                      uint8_t local_node_id,
                                      uint8_t broadcast_node_id,
                                      CommunicationControlFrame_t *control)
{
  uint8_t target;
  uint8_t command;

  if ((frame == NULL) || (control == NULL) || !frame->extended ||
      (frame->len < 2U))
  {
    return false;
  }
  if (GripperCanProtocol_GetMessageType(frame->id) !=
      GRIPPER_CAN_MSG_COMMAND)
  {
    return false;
  }

  target = GripperCanProtocol_GetTargetNode(frame->id);
  if ((target != local_node_id) && (target != broadcast_node_id))
  {
    return false;
  }

  command = frame->data[0];
  if (command > (uint8_t)COMMUNICATION_CONTROL_STATUS_ONESHOT)
  {
    return false;
  }

  memset(control, 0, sizeof(*control));
  control->command = (CommunicationControlCommand_t)command;
  control->sequence = frame->data[1];
  if (frame->len >= 4U)
  {
    control->position_permille =
      (int16_t)((uint16_t)frame->data[2] |
                ((uint16_t)frame->data[3] << 8U));
  }
  return true;
}

void GripperCanProtocol_PackStatus(const GripperStatus_t *status,
                                   uint8_t local_node_id,
                                   uint8_t master_node_id,
                                   CanTransportFrame_t *frame)
{
  uint8_t flags = 0U;

  if ((status == NULL) || (frame == NULL))
  {
    return;
  }

  memset(frame, 0, sizeof(*frame));
  frame->id = GripperCanProtocol_MakeId(GRIPPER_CAN_PRIORITY_DEFAULT,
                                        GRIPPER_CAN_MSG_STATUS,
                                        local_node_id,
                                        master_node_id);
  frame->extended = true;
  frame->len = 8U;
  if (status->homed)
  {
    flags |= 0x01U;
  }
  if (status->encoder_reliable)
  {
    flags |= 0x02U;
  }
  if (status->motor_state == (uint8_t)MOTOR_CONTROL_STATE_RUNNING)
  {
    flags |= 0x04U;
  }
  if ((status->faults != 0U) || (status->motor_faults != 0U))
  {
    flags |= 0x08U;
  }

  frame->data[0] = (uint8_t)status->state;
  frame->data[1] = flags;
  GripperCanProtocol_WriteI16(frame->data, 2U, status->position_permille);
  GripperCanProtocol_WriteI16(
    frame->data, 4U,
    GripperCanProtocol_SaturateI16((int32_t)status->speed_rpm));
  GripperCanProtocol_WriteI16(
    frame->data, 6U,
    GripperCanProtocol_SaturateI16((int32_t)(status->iq_a * 1000.0f)));
}

void GripperCanProtocol_PackFault(const GripperStatus_t *status,
                                  uint8_t local_node_id,
                                  uint8_t master_node_id,
                                  CanTransportFrame_t *frame)
{
  if ((status == NULL) || (frame == NULL))
  {
    return;
  }

  memset(frame, 0, sizeof(*frame));
  frame->id = GripperCanProtocol_MakeId(GRIPPER_CAN_PRIORITY_DEFAULT,
                                        GRIPPER_CAN_MSG_FAULT,
                                        local_node_id,
                                        master_node_id);
  frame->extended = true;
  frame->len = 8U;
  GripperCanProtocol_WriteU32(frame->data, 0U, status->faults);
  GripperCanProtocol_WriteU32(frame->data, 4U, status->motor_faults);
}

void GripperCanProtocol_PackDiag(const GripperStatus_t *status,
                                 uint8_t local_node_id,
                                 uint8_t master_node_id,
                                 CanTransportFrame_t *frame)
{
  if ((status == NULL) || (frame == NULL))
  {
    return;
  }

  memset(frame, 0, sizeof(*frame));
  frame->id = GripperCanProtocol_MakeId(GRIPPER_CAN_PRIORITY_DEFAULT,
                                        GRIPPER_CAN_MSG_DIAG,
                                        local_node_id,
                                        master_node_id);
  frame->extended = true;
  frame->len = 8U;
  GripperCanProtocol_WriteU16(frame->data, 0U, status->mc_faults);
  GripperCanProtocol_WriteU16(
    frame->data, 2U,
    GripperCanProtocol_SaturateU16((uint32_t)status->bus_voltage_v * 100UL));
  GripperCanProtocol_WriteU16(frame->data, 4U, status->temperature_raw);
  GripperCanProtocol_WriteU16(
    frame->data, 6U,
    GripperCanProtocol_SaturateU16(status->encoder_spi_errors));
}

void GripperCanProtocol_PackHeartbeat(const GripperStatus_t *status,
                                      uint8_t local_node_id,
                                      uint8_t master_node_id,
                                      CanTransportFrame_t *frame)
{
  if ((status == NULL) || (frame == NULL))
  {
    return;
  }

  memset(frame, 0, sizeof(*frame));
  frame->id = GripperCanProtocol_MakeId(GRIPPER_CAN_PRIORITY_DEFAULT,
                                        GRIPPER_CAN_MSG_HEARTBEAT,
                                        local_node_id,
                                        master_node_id);
  frame->extended = true;
  frame->len = 1U;
  frame->data[0] = (uint8_t)status->state;
}
