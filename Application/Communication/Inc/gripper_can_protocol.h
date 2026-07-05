#ifndef GRIPPER_CAN_PROTOCOL_H
#define GRIPPER_CAN_PROTOCOL_H

#include "communication_types.h"
#include "gripper_service.h"
#include "can_transport.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  GRIPPER_CAN_MSG_COMMAND = 0x01U,
  GRIPPER_CAN_MSG_STATUS = 0x02U,
  GRIPPER_CAN_MSG_FAULT = 0x03U,
  GRIPPER_CAN_MSG_DIAG = 0x04U,
  GRIPPER_CAN_MSG_HEARTBEAT = 0x05U
} GripperCanMessageType_t;

uint32_t GripperCanProtocol_MakeId(uint8_t priority,
                                   GripperCanMessageType_t type,
                                   uint8_t source_node,
                                   uint8_t target_node);
uint8_t GripperCanProtocol_GetTargetNode(uint32_t id);
uint8_t GripperCanProtocol_GetSourceNode(uint32_t id);
GripperCanMessageType_t GripperCanProtocol_GetMessageType(uint32_t id);

bool GripperCanProtocol_DecodeControl(const CanTransportFrame_t *frame,
                                      uint8_t local_node_id,
                                      uint8_t broadcast_node_id,
                                      CommunicationControlFrame_t *control);
void GripperCanProtocol_PackStatus(const GripperStatus_t *status,
                                   uint8_t local_node_id,
                                   uint8_t master_node_id,
                                   CanTransportFrame_t *frame);
void GripperCanProtocol_PackFault(const GripperStatus_t *status,
                                  uint8_t local_node_id,
                                  uint8_t master_node_id,
                                  CanTransportFrame_t *frame);
void GripperCanProtocol_PackDiag(const GripperStatus_t *status,
                                 uint8_t local_node_id,
                                 uint8_t master_node_id,
                                 CanTransportFrame_t *frame);
void GripperCanProtocol_PackHeartbeat(const GripperStatus_t *status,
                                      uint8_t local_node_id,
                                      uint8_t master_node_id,
                                      CanTransportFrame_t *frame);

#ifdef __cplusplus
}
#endif

#endif
