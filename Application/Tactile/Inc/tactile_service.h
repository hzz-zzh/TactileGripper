#ifndef TACTILE_SERVICE_H
#define TACTILE_SERVICE_H

#include "tactile_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void TactileService_CreateTask(void);
bool TactileService_GetSnapshot(TactileSnapshot_t *snapshot);
void TactileService_GetStats(TactileStats_t *stats);
void TactileService_SetEnabled(bool enabled);
void TactileService_USART2IdleIRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif
