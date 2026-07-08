#include "can_transport.h"

#include <string.h>

#define CAN_TRANSPORT_RX_QUEUE_DEPTH  16U
#define CAN_TRANSPORT_ID_TARGET_SHIFT 2U
#define CAN_TRANSPORT_ID_TARGET_MASK  (0xFFUL << CAN_TRANSPORT_ID_TARGET_SHIFT)

static FDCAN_HandleTypeDef *canHandle;
static volatile uint8_t rxWriteIndex;
static volatile uint8_t rxReadIndex;
static CanTransportFrame_t rxQueue[CAN_TRANSPORT_RX_QUEUE_DEPTH];
static volatile uint32_t rxDroppedCount;
static volatile uint32_t txErrorCount;
static volatile uint32_t busErrorCount;

static uint32_t CanTransport_LengthToDlc(uint8_t len)
{
  switch (len)
  {
    case 0U: return FDCAN_DLC_BYTES_0;
    case 1U: return FDCAN_DLC_BYTES_1;
    case 2U: return FDCAN_DLC_BYTES_2;
    case 3U: return FDCAN_DLC_BYTES_3;
    case 4U: return FDCAN_DLC_BYTES_4;
    case 5U: return FDCAN_DLC_BYTES_5;
    case 6U: return FDCAN_DLC_BYTES_6;
    case 7U: return FDCAN_DLC_BYTES_7;
    default: return FDCAN_DLC_BYTES_8;
  }
}

static uint8_t CanTransport_DlcToLength(uint32_t dlc)
{
  switch (dlc)
  {
    case FDCAN_DLC_BYTES_0: return 0U;
    case FDCAN_DLC_BYTES_1: return 1U;
    case FDCAN_DLC_BYTES_2: return 2U;
    case FDCAN_DLC_BYTES_3: return 3U;
    case FDCAN_DLC_BYTES_4: return 4U;
    case FDCAN_DLC_BYTES_5: return 5U;
    case FDCAN_DLC_BYTES_6: return 6U;
    case FDCAN_DLC_BYTES_7: return 7U;
    default: return 8U;
  }
}

static bool CanTransport_ConfigTargetFilter(uint32_t filter_index,
                                            uint8_t target_id)
{
  FDCAN_FilterTypeDef filter;

  memset(&filter, 0, sizeof(filter));
  filter.IdType = FDCAN_EXTENDED_ID;
  filter.FilterIndex = filter_index;
  filter.FilterType = FDCAN_FILTER_MASK;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1 = ((uint32_t)target_id << CAN_TRANSPORT_ID_TARGET_SHIFT);
  filter.FilterID2 = CAN_TRANSPORT_ID_TARGET_MASK;

  return HAL_FDCAN_ConfigFilter(canHandle, &filter) == HAL_OK;
}

bool CanTransport_Init(FDCAN_HandleTypeDef *hfdcan,
                       uint8_t node_id,
                       uint8_t broadcast_id)
{
  if (hfdcan == NULL)
  {
    return false;
  }

  canHandle = hfdcan;
  rxWriteIndex = 0U;
  rxReadIndex = 0U;
  rxDroppedCount = 0U;
  txErrorCount = 0U;
  busErrorCount = 0U;

  if (!CanTransport_ConfigTargetFilter(0U, node_id))
  {
    return false;
  }
  if (!CanTransport_ConfigTargetFilter(1U, broadcast_id))
  {
    return false;
  }

  /*
   * 标准帧暂不作为业务通道，扩展帧按目标节点过滤后进入FIFO0。
   * 这样可以明确保持CAN2.0B接口，不和后续CANopen标准帧混在一起。
   */
  if (HAL_FDCAN_ConfigGlobalFilter(canHandle,
                                   FDCAN_REJECT,
                                   FDCAN_REJECT,
                                   FDCAN_REJECT_REMOTE,
                                   FDCAN_REJECT_REMOTE) != HAL_OK)
  {
    return false;
  }

  return true;
}

bool CanTransport_Start(void)
{
  if (canHandle == NULL)
  {
    return false;
  }
  if (HAL_FDCAN_ActivateNotification(canHandle,
                                     FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                                     FDCAN_IT_BUS_OFF |
                                     FDCAN_IT_ERROR_WARNING |
                                     FDCAN_IT_ERROR_PASSIVE,
                                     0U) != HAL_OK)
  {
    return false;
  }
  return HAL_FDCAN_Start(canHandle) == HAL_OK;
}

bool CanTransport_Send(const CanTransportFrame_t *frame)
{
  FDCAN_TxHeaderTypeDef txHeader;
  uint8_t data[CAN_TRANSPORT_MAX_DATA_LEN] = {0};

  if ((canHandle == NULL) || (frame == NULL) ||
      (frame->len > CAN_TRANSPORT_MAX_DATA_LEN))
  {
    return false;
  }
  if (HAL_FDCAN_GetTxFifoFreeLevel(canHandle) == 0U)
  {
    txErrorCount++;
    return false;
  }

  memset(&txHeader, 0, sizeof(txHeader));
  txHeader.Identifier = frame->id & 0x1FFFFFFFUL;
  txHeader.IdType = frame->extended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
  txHeader.TxFrameType = FDCAN_DATA_FRAME;
  txHeader.DataLength = CanTransport_LengthToDlc(frame->len);
  txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  txHeader.BitRateSwitch = FDCAN_BRS_OFF;
  txHeader.FDFormat = FDCAN_CLASSIC_CAN;
  txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  txHeader.MessageMarker = 0U;

  if (frame->len > 0U)
  {
    memcpy(data, frame->data, frame->len);
  }
  if (HAL_FDCAN_AddMessageToTxFifoQ(canHandle, &txHeader, data) != HAL_OK)
  {
    txErrorCount++;
    return false;
  }
  return true;
}

bool CanTransport_Poll(CanTransportFrame_t *frame)
{
  if ((frame == NULL) || (rxReadIndex == rxWriteIndex))
  {
    return false;
  }

  *frame = rxQueue[rxReadIndex];
  rxReadIndex = (uint8_t)((rxReadIndex + 1U) % CAN_TRANSPORT_RX_QUEUE_DEPTH);
  return true;
}

void CanTransport_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                                  uint32_t rx_fifo0_it)
{
  FDCAN_RxHeaderTypeDef rxHeader;
  CanTransportFrame_t frame;
  uint8_t nextWriteIndex;

  if ((hfdcan != canHandle) ||
      ((rx_fifo0_it & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U))
  {
    return;
  }

  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
  {
    memset(&frame, 0, sizeof(frame));
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0,
                               &rxHeader, frame.data) != HAL_OK)
    {
      busErrorCount++;
      return;
    }

    frame.id = rxHeader.Identifier;
    frame.extended = rxHeader.IdType == FDCAN_EXTENDED_ID;
    frame.len = CanTransport_DlcToLength(rxHeader.DataLength);

    nextWriteIndex =
      (uint8_t)((rxWriteIndex + 1U) % CAN_TRANSPORT_RX_QUEUE_DEPTH);
    if (nextWriteIndex == rxReadIndex)
    {
      rxDroppedCount++;
    }
    else
    {
      rxQueue[rxWriteIndex] = frame;
      rxWriteIndex = nextWriteIndex;
    }
  }
}

void CanTransport_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
  if (hfdcan == canHandle)
  {
    busErrorCount++;
  }
}

void CanTransport_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan,
                                      uint32_t error_status_it)
{
  if ((hfdcan == canHandle) && (error_status_it != 0U))
  {
    busErrorCount++;
  }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs)
{
  CanTransport_RxFifo0Callback(hfdcan, RxFifo0ITs);
}

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
  CanTransport_ErrorCallback(hfdcan);
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan,
                                   uint32_t ErrorStatusITs)
{
  CanTransport_ErrorStatusCallback(hfdcan, ErrorStatusITs);
}

uint32_t CanTransport_GetRxDroppedCount(void)
{
  return rxDroppedCount;
}

uint32_t CanTransport_GetTxErrorCount(void)
{
  return txErrorCount;
}

uint32_t CanTransport_GetBusErrorCount(void)
{
  return busErrorCount;
}
