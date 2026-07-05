#include "ws2812_led.h"

#define WS2812_ENCODED_FRAME_SIZE       12U
#define WS2812_RESET_TIME_MS             1U
#define WS2812_SYMBOL_ZERO               0x08U  /* 1000：高电平1个SPI时钟。 */
#define WS2812_SYMBOL_ONE                0x0CU  /* 1100：高电平2个SPI时钟。 */
#define WS2812_DMA_BUFFER_ADDRESS        0x24000000UL

static SPI_HandleTypeDef *ws2812Spi;
static volatile bool ws2812Busy;
static volatile bool ws2812FrameCompleted;
static volatile uint32_t ws2812LastCompleteTick;
static volatile uint32_t ws2812ErrorCount;

/*
 * DMA1无法访问默认DTCM(0x20000000)，发送缓存固定放入AXI SRAM。
 * 该地址由本工程保留给状态灯，后续启用IRAM2或D-Cache时需要同步检查内存布局。
 */
#if defined(__CC_ARM)
__attribute__((at(WS2812_DMA_BUFFER_ADDRESS), aligned(32)))
static uint8_t ws2812TxBuffer[WS2812_ENCODED_FRAME_SIZE];
#elif defined(__ARMCC_VERSION)
__attribute__((section(".ARM.__at_0x24000000"), aligned(32)))
static uint8_t ws2812TxBuffer[WS2812_ENCODED_FRAME_SIZE];
#else
__attribute__((section(".dma_buffer"), aligned(32)))
static uint8_t ws2812TxBuffer[WS2812_ENCODED_FRAME_SIZE];
#endif

static void Ws2812Led_EncodeByte(uint8_t value, uint8_t *destination)
{
  uint32_t encoded = 0U;
  uint32_t bit;

  for (bit = 0U; bit < 8U; ++bit)
  {
    encoded <<= 4U;
    encoded |= ((value & 0x80U) != 0U) ?
               WS2812_SYMBOL_ONE : WS2812_SYMBOL_ZERO;
    value <<= 1U;
  }

  destination[0] = (uint8_t)(encoded >> 24U);
  destination[1] = (uint8_t)(encoded >> 16U);
  destination[2] = (uint8_t)(encoded >> 8U);
  destination[3] = (uint8_t)encoded;
}

void Ws2812Led_Init(SPI_HandleTypeDef *hspi)
{
  ws2812Spi = hspi;
  ws2812Busy = false;
  ws2812FrameCompleted = false;
  ws2812LastCompleteTick = 0U;
  ws2812ErrorCount = 0U;
}

bool Ws2812Led_SetColor(Ws2812Color_t color)
{
  uint32_t now;

  if ((ws2812Spi == NULL) || ws2812Busy)
  {
    return false;
  }

  now = HAL_GetTick();
  if (ws2812FrameCompleted &&
      ((now - ws2812LastCompleteTick) < WS2812_RESET_TIME_MS))
  {
    return false;
  }

  /* WS2812协议按GRB顺序发送，每个数据位编码为4个3MHz SPI位。 */
  Ws2812Led_EncodeByte(color.green, &ws2812TxBuffer[0]);
  Ws2812Led_EncodeByte(color.red, &ws2812TxBuffer[4]);
  Ws2812Led_EncodeByte(color.blue, &ws2812TxBuffer[8]);

  ws2812Busy = true;
  if (HAL_SPI_Transmit_DMA(ws2812Spi, ws2812TxBuffer,
                           WS2812_ENCODED_FRAME_SIZE) != HAL_OK)
  {
    ws2812Busy = false;
    ws2812ErrorCount++;
    return false;
  }
  return true;
}

bool Ws2812Led_IsBusy(void)
{
  return ws2812Busy;
}

uint32_t Ws2812Led_GetErrorCount(void)
{
  return ws2812ErrorCount;
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if ((ws2812Spi != NULL) && (hspi == ws2812Spi))
  {
    ws2812Busy = false;
    ws2812FrameCompleted = true;
    ws2812LastCompleteTick = HAL_GetTick();
  }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  if ((ws2812Spi != NULL) && (hspi == ws2812Spi))
  {
    ws2812Busy = false;
    ws2812ErrorCount++;
  }
}
