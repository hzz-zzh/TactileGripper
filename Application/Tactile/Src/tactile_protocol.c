#include "tactile_protocol.h"

#include <stddef.h>

#define TACTILE_HEADER0               0x55U
#define TACTILE_HEADER1               0xAAU
#define TACTILE_DATA_COMMAND          0x85U
#define TACTILE_CHECKSUM_DATA_LENGTH  94U

static uint16_t TactileProtocol_ReadLe16(const uint8_t *data)
{
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static uint32_t TactileProtocol_ReadLe24(const uint8_t *data)
{
  return (uint32_t)data[0] |
         ((uint32_t)data[1] << 8) |
         ((uint32_t)data[2] << 16);
}

static uint16_t TactileProtocol_CalcChecksum(const uint8_t *data,
                                             uint16_t len)
{
  uint16_t sum = 0U;
  uint16_t index;

  for (index = 0U; index < len; index++)
  {
    sum = (uint16_t)(sum + data[index]);
  }

  return sum;
}

static const uint8_t *TactileProtocol_FindFrame(const uint8_t *frame,
                                                uint16_t len,
                                                uint8_t address)
{
  uint16_t offset;

  if ((frame == NULL) || (len < TACTILE_FRAME_LENGTH))
  {
    return NULL;
  }

  for (offset = 0U; offset <= (uint16_t)(len - TACTILE_FRAME_LENGTH); offset++)
  {
    if ((frame[offset] == TACTILE_HEADER0) &&
        (frame[offset + 1U] == TACTILE_HEADER1) &&
        (frame[offset + 2U] == address) &&
        (frame[offset + 3U] == TACTILE_DATA_COMMAND))
    {
      return &frame[offset];
    }
  }

  return NULL;
}

void TactileProtocol_BuildReadCommand(uint8_t address, uint8_t command[2])
{
  if (command == NULL)
  {
    return;
  }

  command[0] = address;
  command[1] = (uint8_t)(0xFFU - address);
}

TactileProtocolResult_t TactileProtocol_DecodeFrame(uint8_t address,
                                                    const uint8_t *frame,
                                                    uint16_t len,
                                                    TactileUnitData_t *unit)
{
  const uint8_t *payload;
  uint16_t expectedChecksum;
  uint16_t actualChecksum;
  uint8_t channel;

  if ((frame == NULL) || (unit == NULL))
  {
    return TACTILE_PROTOCOL_BAD_ARGUMENT;
  }
  if (len < TACTILE_FRAME_LENGTH)
  {
    return TACTILE_PROTOCOL_BAD_LENGTH;
  }

  payload = TactileProtocol_FindFrame(frame, len, address);
  if (payload == NULL)
  {
    return TACTILE_PROTOCOL_BAD_HEADER;
  }
  if (payload[5] != TACTILE_CHANNEL_COUNT)
  {
    return TACTILE_PROTOCOL_BAD_CHANNEL_COUNT;
  }

  /* 校验覆盖帧头到 status 字节，帧尾两个字节为低字节在前的 16 位累加和。 */
  expectedChecksum = TactileProtocol_ReadLe16(&payload[94]);
  actualChecksum = TactileProtocol_CalcChecksum(payload,
                                                TACTILE_CHECKSUM_DATA_LENGTH);
  if (expectedChecksum != actualChecksum)
  {
    return TACTILE_PROTOCOL_BAD_CHECKSUM;
  }

  unit->address = address;
  unit->sequence = payload[4];
  unit->channel_count = payload[5];
  for (channel = 0U; channel < TACTILE_CHANNEL_COUNT; channel++)
  {
    unit->raw[channel] =
      TactileProtocol_ReadLe24(&payload[6U + ((uint16_t)channel * 3U)]);
  }

  unit->normal_force = (int16_t)TactileProtocol_ReadLe16(&payload[84]);
  unit->tangent_force = (int16_t)TactileProtocol_ReadLe16(&payload[86]);
  unit->tangent_angle = TactileProtocol_ReadLe16(&payload[88]);
  unit->proximity = TactileProtocol_ReadLe24(&payload[90]);
  unit->status = payload[93];
  unit->valid = true;

  return TACTILE_PROTOCOL_OK;
}
