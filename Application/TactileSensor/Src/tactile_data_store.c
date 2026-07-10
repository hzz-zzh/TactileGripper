#include "tactile_data_store.h"

#include "FreeRTOS.h"
#include "stm32h7xx_hal.h"
#include "task.h"

#include <string.h>

#define TACTILE_DATA_ALL_UNITS_MASK  \
  ((1U << (TACTILE_FINGER_COUNT * TACTILE_UNIT_COUNT_PER_MODULE)) - 1U)

static gripper_tactile_data_t tactileWorkingData;
static gripper_tactile_data_t tactilePublishedData;
static volatile uint32_t tactilePublishVersion;
static volatile uint8_t tactileCycleValidMask;

static uint8_t TactileDataStore_UnitMask(tactile_finger_id_t finger,
                                         tactile_unit_id_t unit)
{
  uint32_t bit = (uint32_t)finger * TACTILE_UNIT_COUNT_PER_MODULE +
                 (uint32_t)unit;
  return (uint8_t)(1U << bit);
}

static void TactileDataStore_SetInvalidDirections(
  gripper_tactile_data_t *data)
{
  uint32_t finger;
  uint32_t unit;

  for (finger = 0U; finger < TACTILE_FINGER_COUNT; finger++)
  {
    for (unit = 0U; unit < TACTILE_UNIT_COUNT_PER_MODULE; unit++)
    {
      data->finger[finger].unit[unit].tangential_direction_deg =
        TACTILE_DIRECTION_INVALID_DEG;
    }
  }
}

void TactileDataStore_Init(void)
{
  memset(&tactileWorkingData, 0, sizeof(tactileWorkingData));
  memset(&tactilePublishedData, 0, sizeof(tactilePublishedData));
  TactileDataStore_SetInvalidDirections(&tactileWorkingData);
  TactileDataStore_SetInvalidDirections(&tactilePublishedData);
  tactilePublishVersion = 0U;
  tactileCycleValidMask = 0U;
}

void TactileDataStore_BeginCycle(void)
{
  taskENTER_CRITICAL();
  tactileCycleValidMask = 0U;
  taskEXIT_CRITICAL();
}

void TactileDataStore_UpdateUnit(tactile_finger_id_t finger,
                                 tactile_unit_id_t unit,
                                 const tactile_unit_data_t *data)
{
  uint8_t unit_mask;

  if ((data == NULL) ||
      ((uint32_t)finger >= TACTILE_FINGER_COUNT) ||
      ((uint32_t)unit >= TACTILE_UNIT_COUNT_PER_MODULE))
  {
    return;
  }

  tactileWorkingData.finger[finger].unit[unit] = *data;
  unit_mask = TactileDataStore_UnitMask(finger, unit);
  tactileCycleValidMask |= unit_mask;
  if (tactileCycleValidMask != TACTILE_DATA_ALL_UNITS_MASK)
  {
    return;
  }

  tactileWorkingData.sequence = tactilePublishedData.sequence + 1U;
  tactileWorkingData.timestamp_us = HAL_GetTick() * 1000U;

  /* 奇数版本表示正在发布，读取方会在复制后再次核对版本。 */
  tactilePublishVersion++;
  __DMB();
  memcpy(&tactilePublishedData,
         &tactileWorkingData,
         sizeof(tactilePublishedData));
  __DMB();
  tactilePublishVersion++;
  tactileCycleValidMask = 0U;
}

bool TactileDataStore_GetLatest(gripper_tactile_data_t *data)
{
  uint32_t version_before;
  uint32_t version_after;

  if (data == NULL)
  {
    return false;
  }

  for (;;)
  {
    version_before = tactilePublishVersion;
    if ((version_before & 1U) != 0U)
    {
      continue;
    }
    __DMB();
    memcpy(data, &tactilePublishedData, sizeof(*data));
    __DMB();
    version_after = tactilePublishVersion;
    if ((version_before == version_after) &&
        ((version_after & 1U) == 0U))
    {
      break;
    }
  }

  return data->sequence != 0U;
}
