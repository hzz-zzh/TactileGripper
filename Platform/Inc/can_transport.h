#ifndef CAN_TRANSPORT_H
#define CAN_TRANSPORT_H

#include "main.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CAN_TRANSPORT_MAX_DATA_LEN  8U

typedef struct
{
  uint32_t id;
  uint8_t data[CAN_TRANSPORT_MAX_DATA_LEN];
  uint8_t len;
  bool extended;
} CanTransportFrame_t;

bool CanTransport_Init(FDCAN_HandleTypeDef *hfdcan,
                       uint8_t node_id,
                       uint8_t broadcast_id);
bool CanTransport_Start(void);
bool CanTransport_Send(const CanTransportFrame_t *frame);
bool CanTransport_Poll(CanTransportFrame_t *frame);
void CanTransport_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                                  uint32_t rx_fifo0_it);
void CanTransport_ErrorCallback(FDCAN_HandleTypeDef *hfdcan);
void CanTransport_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan,
                                      uint32_t error_status_it);
uint32_t CanTransport_GetRxDroppedCount(void);
uint32_t CanTransport_GetTxErrorCount(void);
uint32_t CanTransport_GetBusErrorCount(void);

#ifdef __cplusplus
}
#endif

#endif
