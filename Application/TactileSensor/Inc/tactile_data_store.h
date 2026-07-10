#ifndef TACTILE_DATA_STORE_H
#define TACTILE_DATA_STORE_H

#include "tactile_data.h"

#include <stdbool.h>

void TactileDataStore_Init(void);
/* 开始新的25ms采样周期，清除上一周期的四单元到达标志。 */
void TactileDataStore_BeginCycle(void);
/* 由UART接收回调写入单元数据，四个单元到齐后自动发布快照。 */
void TactileDataStore_UpdateUnit(tactile_finger_id_t finger,
                                 tactile_unit_id_t unit,
                                 const tactile_unit_data_t *data);
/* 使用版本校验读取稳定快照，不长时间屏蔽UART中断。 */
bool TactileDataStore_GetLatest(gripper_tactile_data_t *data);

#endif
