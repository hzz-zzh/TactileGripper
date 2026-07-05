#ifndef COMMUNICATION_TYPES_H
#define COMMUNICATION_TYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  COMMUNICATION_CONTROL_NOP = 0,
  COMMUNICATION_CONTROL_HOME = 1,
  COMMUNICATION_CONTROL_SET_POSITION = 2,
  COMMUNICATION_CONTROL_STOP = 3,
  COMMUNICATION_CONTROL_CLEAR_FAULT = 4,
  COMMUNICATION_CONTROL_STATUS_ONESHOT = 5
} CommunicationControlCommand_t;

typedef struct
{
  CommunicationControlCommand_t command;
  uint8_t sequence;
  int16_t position_permille;
} CommunicationControlFrame_t;

#ifdef __cplusplus
}
#endif

#endif
