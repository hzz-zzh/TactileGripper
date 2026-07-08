#ifndef TACTILE_UART_TRANSPORT_H
#define TACTILE_UART_TRANSPORT_H

#include "stm32h7xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  uint32_t tx_error_count;
  uint32_t rx_error_count;
  uint32_t timeout_count;
  uint32_t ring_overrun_count;
  uint32_t idle_event_count;
  uint32_t last_result;
  uint32_t last_hal_status;
  uint32_t last_hal_error;
  uint16_t last_rx_len;
  uint16_t last_rx_capacity;
  uint16_t last_tx_len;
  uint8_t last_tx_preview[2];
  uint8_t last_rx_preview[16];
} TactileUartTransportStats_t;

typedef struct
{
  UART_HandleTypeDef *huart;
  TactileUartTransportStats_t stats;
  uint8_t *dma_rx_buffer;
  uint16_t dma_rx_length;
  volatile uint16_t dma_last_pos;
  uint8_t *ring_buffer;
  uint16_t ring_length;
  volatile uint16_t ring_head;
  volatile uint16_t ring_tail;
  bool circular_rx_started;
} TactileUartTransport_t;

typedef enum
{
  TACTILE_UART_RESULT_OK = 0,
  TACTILE_UART_RESULT_BAD_ARGUMENT,
  TACTILE_UART_RESULT_RX_START_ERROR,
  TACTILE_UART_RESULT_TX_ERROR,
  TACTILE_UART_RESULT_RX_TIMEOUT,
  TACTILE_UART_RESULT_RX_INCOMPLETE
} TactileUartTransportResult_t;

void TactileUartTransport_Init(TactileUartTransport_t *transport,
                               UART_HandleTypeDef *huart);
bool TactileUartTransport_WriteRead(TactileUartTransport_t *transport,
                                    const uint8_t *tx_data,
                                    uint16_t tx_len,
                                    uint8_t *rx_data,
                                    uint16_t rx_capacity,
                                    uint16_t expected_rx_len,
                                    uint16_t *actual_rx_len,
                                    uint32_t timeout_ms);
bool TactileUartTransport_WriteReadPolling(TactileUartTransport_t *transport,
                                           const uint8_t *tx_data,
                                           uint16_t tx_len,
                                           uint8_t *rx_data,
                                           uint16_t rx_capacity,
                                           uint16_t expected_rx_len,
                                           uint16_t *actual_rx_len,
                                           uint32_t timeout_ms);
bool TactileUartTransport_StartCircularRx(TactileUartTransport_t *transport,
                                          uint8_t *dma_rx_buffer,
                                          uint16_t dma_rx_length,
                                          uint8_t *ring_buffer,
                                          uint16_t ring_length);
bool TactileUartTransport_WriteReadCircular(TactileUartTransport_t *transport,
                                            const uint8_t *tx_data,
                                            uint16_t tx_len,
                                            uint8_t expected_address,
                                            uint8_t *rx_data,
                                            uint16_t rx_capacity,
                                            uint16_t expected_rx_len,
                                            uint16_t *actual_rx_len,
                                            uint32_t timeout_ms);
void TactileUartTransport_HandleIdleIrq(TactileUartTransport_t *transport);
void TactileUartTransport_GetStats(const TactileUartTransport_t *transport,
                                   TactileUartTransportStats_t *stats);

#ifdef __cplusplus
}
#endif

#endif
