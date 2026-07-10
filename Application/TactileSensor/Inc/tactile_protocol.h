#ifndef TACTILE_PROTOCOL_H
#define TACTILE_PROTOCOL_H

#include "tactile_data.h"

#include <stdbool.h>
#include <stdint.h>

#define TACTILE_PROTOCOL_FRAME_SIZE   96U
#define TACTILE_PROTOCOL_ADDRESS_36   0x36U
#define TACTILE_PROTOCOL_ADDRESS_37   0x37U

typedef enum
{
  TACTILE_PROTOCOL_OK = 0,
  TACTILE_PROTOCOL_BAD_ARGUMENT,
  TACTILE_PROTOCOL_BAD_LENGTH,
  TACTILE_PROTOCOL_BAD_HEADER,
  TACTILE_PROTOCOL_BAD_COMMAND,
  TACTILE_PROTOCOL_BAD_CHANNEL_COUNT,
  TACTILE_PROTOCOL_BAD_CHECKSUM
} tactile_protocol_result_t;

bool TactileProtocol_IsAddress(uint8_t address);
void TactileProtocol_BuildReadCommand(uint8_t address, uint8_t command[2]);
/* 校验并解析一个完整96字节响应，输出单位已经转换后的单元数据。 */
tactile_protocol_result_t TactileProtocol_DecodeFrame(
  uint8_t expected_address,
  const uint8_t *frame,
  uint16_t length,
  tactile_unit_data_t *data);

#endif
