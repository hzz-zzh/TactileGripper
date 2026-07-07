#ifndef TACTILE_DATA_H
#define TACTILE_DATA_H

#include "tactile_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void TactileData_Init(void);
void TactileData_Publish(const TactileSnapshot_t *snapshot);
bool TactileData_GetSnapshot(TactileSnapshot_t *snapshot, uint32_t now_ms);
bool TactileData_GetHistory(uint8_t back_index,
                            TactileSnapshot_t *snapshot,
                            uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif
