#ifndef __FDCAN_H__
#define __FDCAN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern FDCAN_HandleTypeDef hfdcan1;

void MX_FDCAN1_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_H__ */
