#include "tactile_protocol.h"

#include <string.h>

#define TACTILE_HEADER0                    0x55U
#define TACTILE_HEADER1                    0xAAU
#define TACTILE_DATA_COMMAND               0x85U
#define TACTILE_CHANNEL_COUNT              0x1AU
#define TACTILE_CHECKSUM_DATA_LENGTH       94U
#define TACTILE_TAXEL_OFFSET               6U
#define TACTILE_TAXEL_WIDTH                3U
#define TACTILE_PROXIMITY_RAW_OFFSET       81U
#define TACTILE_NORMAL_FORCE_OFFSET        84U
#define TACTILE_TANGENTIAL_FORCE_OFFSET    86U
#define TACTILE_TANGENTIAL_ANGLE_OFFSET    88U
#define TACTILE_PROXIMITY_DELTA_OFFSET     90U
#define TACTILE_FORCE_SCALE                1000.0f

static uint16_t TactileProtocol_ReadLe16(const uint8_t *data)
{
  return (uint16_t)data[0] |
         (uint16_t)((uint16_t)data[1] << 8U);
}

static uint32_t TactileProtocol_ReadLe24(const uint8_t *data)
{
  return (uint32_t)data[0] |
         ((uint32_t)data[1] << 8U) |
         ((uint32_t)data[2] << 16U);
}

static uint16_t TactileProtocol_CalcChecksum(const uint8_t *data)
{
  uint32_t sum = 0U;
  uint16_t index;

  for (index = 0U; index < TACTILE_CHECKSUM_DATA_LENGTH; index++)
  {
    sum += data[index];
  }
  return (uint16_t)(sum & 0xFFFFU);
}

bool TactileProtocol_IsAddress(uint8_t address)
{
  return (address == TACTILE_PROTOCOL_ADDRESS_36) ||
         (address == TACTILE_PROTOCOL_ADDRESS_37);
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

tactile_protocol_result_t TactileProtocol_DecodeFrame(
  uint8_t expected_address,
  const uint8_t *frame,
  uint16_t length,
  tactile_unit_data_t *data)
{
  uint16_t received_checksum;
  uint16_t calculated_checksum;
  uint16_t direction;
  uint32_t index;

  if ((frame == NULL) || (data == NULL) ||
      !TactileProtocol_IsAddress(expected_address))
  {
    return TACTILE_PROTOCOL_BAD_ARGUMENT;
  }
  if (length != TACTILE_PROTOCOL_FRAME_SIZE)
  {
    return TACTILE_PROTOCOL_BAD_LENGTH;
  }
  if ((frame[0] != TACTILE_HEADER0) ||
      (frame[1] != TACTILE_HEADER1) ||
      (frame[2] != expected_address))
  {
    return TACTILE_PROTOCOL_BAD_HEADER;
  }
  if (frame[3] != TACTILE_DATA_COMMAND)
  {
    return TACTILE_PROTOCOL_BAD_COMMAND;
  }
  if (frame[5] != TACTILE_CHANNEL_COUNT)
  {
    return TACTILE_PROTOCOL_BAD_CHANNEL_COUNT;
  }

  received_checksum = TactileProtocol_ReadLe16(
    &frame[TACTILE_CHECKSUM_DATA_LENGTH]);
  calculated_checksum = TactileProtocol_CalcChecksum(frame);
  if (received_checksum != calculated_checksum)
  {
    return TACTILE_PROTOCOL_BAD_CHECKSUM;
  }

  memset(data, 0, sizeof(*data));
  for (index = 0U; index < TACTILE_TAXEL_COUNT_PER_UNIT; index++)
  {
    uint16_t offset = (uint16_t)(TACTILE_TAXEL_OFFSET +
                                 index * TACTILE_TAXEL_WIDTH);
    data->taxel_delta[index] =
      (int32_t)TactileProtocol_ReadLe24(&frame[offset]);
  }

  data->proximity_raw =
    TactileProtocol_ReadLe24(&frame[TACTILE_PROXIMITY_RAW_OFFSET]);
  data->proximity_delta =
    (int32_t)TactileProtocol_ReadLe24(
      &frame[TACTILE_PROXIMITY_DELTA_OFFSET]);
  data->normal_force_n =
    (float)(int16_t)TactileProtocol_ReadLe16(
      &frame[TACTILE_NORMAL_FORCE_OFFSET]) / TACTILE_FORCE_SCALE;
  data->tangential_force_n =
    (float)(int16_t)TactileProtocol_ReadLe16(
      &frame[TACTILE_TANGENTIAL_FORCE_OFFSET]) / TACTILE_FORCE_SCALE;

  direction = TactileProtocol_ReadLe16(
    &frame[TACTILE_TANGENTIAL_ANGLE_OFFSET]);
  data->tangential_direction_deg =
    (direction <= 359U) ? direction : TACTILE_DIRECTION_INVALID_DEG;
  data->valid_mask = TACTILE_UNIT_DATA_VALID_TAXEL |
                     TACTILE_UNIT_DATA_VALID_PROXIMITY |
                     TACTILE_UNIT_DATA_VALID_FORCE;
  return TACTILE_PROTOCOL_OK;
}
