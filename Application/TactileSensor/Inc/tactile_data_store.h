#ifndef TACTILE_DATA_STORE_H
#define TACTILE_DATA_STORE_H

#include "tactile_data.h"

#include <stdbool.h>

void TactileDataStore_Init(void);
void TactileDataStore_BeginCycle(void);
void TactileDataStore_UpdateUnit(tactile_finger_id_t finger,
                                 tactile_unit_id_t unit,
                                 const tactile_unit_data_t *data);
bool TactileDataStore_GetLatest(gripper_tactile_data_t *data);

#endif
