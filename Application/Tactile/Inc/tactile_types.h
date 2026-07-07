#ifndef TACTILE_TYPES_H
#define TACTILE_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TACTILE_SENSOR_COUNT              2U
#define TACTILE_UNITS_PER_SENSOR          2U
#define TACTILE_CHANNEL_COUNT             26U
#define TACTILE_FRAME_LENGTH              96U
#define TACTILE_HISTORY_DEPTH             32U

#define TACTILE_RIGHT_SENSOR_INDEX        0U
#define TACTILE_LEFT_SENSOR_INDEX         1U

#define TACTILE_UPPER_ADDRESS             0x6DU
#define TACTILE_LOWER_ADDRESS             0x36U

#define TACTILE_MASK_RIGHT_UPPER          (1U << 0)
#define TACTILE_MASK_RIGHT_LOWER          (1U << 1)
#define TACTILE_MASK_LEFT_UPPER           (1U << 2)
#define TACTILE_MASK_LEFT_LOWER           (1U << 3)
#define TACTILE_MASK_ALL                  (TACTILE_MASK_RIGHT_UPPER | \
                                           TACTILE_MASK_RIGHT_LOWER | \
                                           TACTILE_MASK_LEFT_UPPER | \
                                           TACTILE_MASK_LEFT_LOWER)

typedef enum
{
  TACTILE_SENSOR_RIGHT = TACTILE_RIGHT_SENSOR_INDEX,
  TACTILE_SENSOR_LEFT = TACTILE_LEFT_SENSOR_INDEX
} TactileSensorSide_t;

typedef enum
{
  TACTILE_UNIT_UPPER = 0,
  TACTILE_UNIT_LOWER = 1
} TactileUnitIndex_t;

typedef struct
{
  uint8_t address;
  uint8_t sensor_index;
  uint8_t unit_index;
  uint8_t sequence;
  uint8_t channel_count;
  uint8_t status;
  uint32_t raw[TACTILE_CHANNEL_COUNT];
  int16_t normal_force;
  int16_t tangent_force;
  uint16_t tangent_angle;
  uint32_t proximity;
  uint32_t timestamp_ms;
  bool valid;
  uint32_t checksum_error_count;
  uint32_t timeout_count;
  uint32_t frame_error_count;
  uint32_t transport_error_count;
} TactileUnitData_t;

typedef struct
{
  TactileUnitData_t upper;
  TactileUnitData_t lower;
  uint8_t valid_mask;
  bool complete;
  uint32_t timeout_count;
  uint32_t checksum_error_count;
  uint32_t frame_error_count;
  uint32_t transport_error_count;
} TactileSensorData_t;

typedef struct
{
  uint32_t sample_id;
  uint32_t timestamp_ms;
  TactileSensorData_t right;
  TactileSensorData_t left;
  uint8_t valid_mask;
  bool complete;
  uint32_t age_ms;
} TactileSnapshot_t;

typedef struct
{
  uint32_t sample_count;
  uint32_t complete_count;
  uint32_t partial_count;
  uint32_t timeout_count;
  uint32_t checksum_error_count;
  uint32_t frame_error_count;
  uint32_t transport_error_count;
  uint32_t unit_fps[TACTILE_SENSOR_COUNT][TACTILE_UNITS_PER_SENSOR];
  uint8_t last_valid_mask;
} TactileStats_t;

#ifdef __cplusplus
}
#endif

#endif
