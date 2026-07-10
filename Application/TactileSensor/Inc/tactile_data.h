#ifndef TACTILE_DATA_H
#define TACTILE_DATA_H

#include <stdint.h>

/* 二指夹爪：左指、右指。 */
#define TACTILE_FINGER_COUNT              2U

/* 单个触觉模块由上下两个触觉单元组成。 */
#define TACTILE_UNIT_COUNT_PER_MODULE     2U

/* 单个触觉单元为5x5阵列。 */
#define TACTILE_UNIT_ROW_COUNT            5U
#define TACTILE_UNIT_COL_COUNT            5U
#define TACTILE_TAXEL_COUNT_PER_UNIT      \
  (TACTILE_UNIT_ROW_COUNT * TACTILE_UNIT_COL_COUNT)

/* 切向力方向无效值；有效方向为0~359 deg。 */
#define TACTILE_DIRECTION_INVALID_DEG     UINT16_MAX

typedef enum
{
  TACTILE_FINGER_LEFT = 0,
  TACTILE_FINGER_RIGHT = 1
} tactile_finger_id_t;

typedef enum
{
  TACTILE_UNIT_UPPER = 0,
  TACTILE_UNIT_LOWER = 1
} tactile_unit_id_t;

typedef enum
{
  TACTILE_UNIT_DATA_VALID_TAXEL     = (1U << 0),
  TACTILE_UNIT_DATA_VALID_PROXIMITY = (1U << 1),
  TACTILE_UNIT_DATA_VALID_FORCE     = (1U << 2)
} tactile_unit_data_valid_mask_t;

/**
 * @brief 单个5x5触觉单元的一帧动态数据。
 *
 * taxel_delta索引规则：index = row * TACTILE_UNIT_COL_COUNT + col。
 */
typedef struct
{
  int32_t taxel_delta[TACTILE_TAXEL_COUNT_PER_UNIT];
  uint32_t proximity_raw;
  int32_t proximity_delta;
  float normal_force_n;
  float tangential_force_n;
  uint16_t tangential_direction_deg;
  uint16_t valid_mask;
} tactile_unit_data_t;

typedef struct
{
  tactile_unit_data_t unit[TACTILE_UNIT_COUNT_PER_MODULE];
} tactile_module_data_t;

/**
 * @brief 二指夹爪的一帧完整触觉数据。
 */
typedef struct
{
  tactile_module_data_t finger[TACTILE_FINGER_COUNT];
  uint32_t sequence;
  uint32_t timestamp_us;
} gripper_tactile_data_t;

#endif
