#ifndef TACTILE_SERVICE_H
#define TACTILE_SERVICE_H

#include "tactile_types.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void TactileService_CreateTask(void);
bool TactileService_GetSnapshot(TactileSnapshot_t *snapshot);
void TactileService_GetStats(TactileStats_t *stats);
void TactileService_SetEnabled(bool enabled);

#ifdef __cplusplus
}
#endif

#endif
