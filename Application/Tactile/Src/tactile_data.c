#include "tactile_data.h"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

static TactileSnapshot_t latestSnapshot;
static TactileSnapshot_t history[TACTILE_HISTORY_DEPTH];
static uint8_t historyWriteIndex;
static uint8_t historyCount;

void TactileData_Init(void)
{
  taskENTER_CRITICAL();
  (void)memset(&latestSnapshot, 0, sizeof(latestSnapshot));
  (void)memset(history, 0, sizeof(history));
  historyWriteIndex = 0U;
  historyCount = 0U;
  taskEXIT_CRITICAL();
}

void TactileData_Publish(const TactileSnapshot_t *snapshot)
{
  if (snapshot == NULL)
  {
    return;
  }

  taskENTER_CRITICAL();
  latestSnapshot = *snapshot;
  history[historyWriteIndex] = *snapshot;
  historyWriteIndex = (uint8_t)((historyWriteIndex + 1U) %
                                TACTILE_HISTORY_DEPTH);
  if (historyCount < TACTILE_HISTORY_DEPTH)
  {
    historyCount++;
  }
  taskEXIT_CRITICAL();
}

bool TactileData_GetSnapshot(TactileSnapshot_t *snapshot, uint32_t now_ms)
{
  if (snapshot == NULL)
  {
    return false;
  }

  taskENTER_CRITICAL();
  *snapshot = latestSnapshot;
  taskEXIT_CRITICAL();

  snapshot->age_ms = now_ms - snapshot->timestamp_ms;
  return snapshot->sample_id != 0U;
}

bool TactileData_GetHistory(uint8_t back_index,
                            TactileSnapshot_t *snapshot,
                            uint32_t now_ms)
{
  uint8_t readIndex;

  if (snapshot == NULL)
  {
    return false;
  }

  taskENTER_CRITICAL();
  if (back_index >= historyCount)
  {
    taskEXIT_CRITICAL();
    return false;
  }

  readIndex = (uint8_t)((historyWriteIndex + TACTILE_HISTORY_DEPTH - 1U -
                         back_index) % TACTILE_HISTORY_DEPTH);
  *snapshot = history[readIndex];
  taskEXIT_CRITICAL();

  snapshot->age_ms = now_ms - snapshot->timestamp_ms;
  return snapshot->sample_id != 0U;
}
