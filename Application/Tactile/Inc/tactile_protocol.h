#ifndef TACTILE_PROTOCOL_H
#define TACTILE_PROTOCOL_H

#include "tactile_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  TACTILE_PROTOCOL_OK = 0,
  TACTILE_PROTOCOL_BAD_ARGUMENT,
  TACTILE_PROTOCOL_BAD_LENGTH,
  TACTILE_PROTOCOL_BAD_HEADER,
  TACTILE_PROTOCOL_BAD_CHECKSUM,
  TACTILE_PROTOCOL_BAD_CHANNEL_COUNT
} TactileProtocolResult_t;

void TactileProtocol_BuildReadCommand(uint8_t address, uint8_t command[2]);
TactileProtocolResult_t TactileProtocol_DecodeFrame(uint8_t address,
                                                    const uint8_t *frame,
                                                    uint16_t len,
                                                    TactileUnitData_t *unit);

#ifdef __cplusplus
}
#endif

#endif
