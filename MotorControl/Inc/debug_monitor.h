#ifndef DEBUG_MONITOR_H
#define DEBUG_MONITOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void DebugMonitor_CreateTask(void);
void DebugMonitor_PrintResetReason(uint32_t reset_flags);
void DebugMonitor_RunCurrentAdcCalibration(void);
void DebugMonitor_RunPwmInspection(uint32_t reset_flags);
void DebugMonitor_RunPwmVectorInspection(uint32_t reset_flags);
void DebugMonitor_RunPwmBaselineVectorInspection(uint32_t reset_flags);
uint16_t DebugMonitor_CalibrateBusCurrent(void);
uint16_t DebugMonitor_GetBusCurrentZero(void);

#ifdef __cplusplus
}
#endif

#endif
