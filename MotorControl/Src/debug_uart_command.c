#include "debug_uart_command.h"
#include "debug_uart_transport.h"
#include "tactile_sensor.h"

#define DEBUG_UART_COMMAND_BUFFER_SIZE      32U
#define DEBUG_UART_COMMAND_IDLE_TIMEOUT_MS  50U
#define DEBUG_UART_COMMAND_RX_RING_SIZE     128U

static UART_HandleTypeDef *debugUartHandle;
static uint8_t debugUartRxByte;
static volatile uint16_t debugUartRxHead;
static volatile uint16_t debugUartRxTail;
static volatile uint8_t debugUartRxRing[DEBUG_UART_COMMAND_RX_RING_SIZE];

static void DebugUartCommand_FlushHardwareRx(void)
{
  if (debugUartHandle == NULL)
  {
    return;
  }

  __HAL_UART_CLEAR_FLAG(debugUartHandle, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                         UART_CLEAR_FEF | UART_CLEAR_PEF);
  __HAL_UART_SEND_REQ(debugUartHandle, UART_RXDATA_FLUSH_REQUEST);
}

static bool DebugUartCommand_IsSpace(char value)
{
  return (value == ' ') || (value == '\t');
}

static bool DebugUartCommand_IsDigit(char value)
{
  return (value >= '0') && (value <= '9');
}

static void DebugUartCommand_PushByte(uint8_t value)
{
  uint16_t nextHead = (uint16_t)((debugUartRxHead + 1U) %
                                 DEBUG_UART_COMMAND_RX_RING_SIZE);

  if (nextHead != debugUartRxTail)
  {
    debugUartRxRing[debugUartRxHead] = value;
    debugUartRxHead = nextHead;
  }
}

static bool DebugUartCommand_PopByte(uint8_t *value)
{
  if (debugUartRxTail == debugUartRxHead)
  {
    return false;
  }

  *value = debugUartRxRing[debugUartRxTail];
  debugUartRxTail = (uint16_t)((debugUartRxTail + 1U) %
                               DEBUG_UART_COMMAND_RX_RING_SIZE);
  return true;
}

static bool DebugUartCommand_ParseTurnMilli(const char *text,
                                            int32_t *turn_milli)
{
  int32_t integerPart = 0;
  int32_t fractionPart = 0;
  int32_t fractionScale = 100;
  int32_t sign = 1;
  bool hasDigit = false;
  bool hasDecimalPoint = false;

  while (DebugUartCommand_IsSpace(*text))
  {
    text++;
  }

  if ((*text == '-') || (*text == '+'))
  {
    sign = (*text == '-') ? -1 : 1;
    text++;
  }

  while (DebugUartCommand_IsDigit(*text))
  {
    hasDigit = true;
    if (integerPart <= 100000L)
    {
      integerPart = (integerPart * 10) + (int32_t)(*text - '0');
    }
    text++;
  }

  if (*text == '.')
  {
    hasDecimalPoint = true;
    text++;
    while (DebugUartCommand_IsDigit(*text))
    {
      hasDigit = true;
      if (fractionScale > 0)
      {
        fractionPart += (int32_t)(*text - '0') * fractionScale;
        fractionScale /= 10;
      }
      text++;
    }
  }

  while (DebugUartCommand_IsSpace(*text))
  {
    text++;
  }

  if ((!hasDigit) || (*text != '\0'))
  {
    return false;
  }

  if (hasDecimalPoint)
  {
    *turn_milli = sign * ((integerPart * 1000L) + fractionPart);
  }
  else
  {
    /* 无小数点时按毫圈处理：T5000 表示 5.000 turn。 */
    *turn_milli = sign * integerPart;
  }
  return true;
}

static bool DebugUartCommand_ParsePosition(const char *text,
                                           int16_t *position_permille)
{
  int32_t value = 0;
  bool hasDigit = false;

  while (DebugUartCommand_IsSpace(*text))
  {
    text++;
  }
  while (DebugUartCommand_IsDigit(*text))
  {
    hasDigit = true;
    value = (value * 10L) + (int32_t)(*text - '0');
    if (value > 1000L)
    {
      return false;
    }
    text++;
  }
  while (DebugUartCommand_IsSpace(*text))
  {
    text++;
  }
  if ((!hasDigit) || (*text != '\0'))
  {
    return false;
  }
  *position_permille = (int16_t)value;
  return true;
}

static bool DebugUartCommand_ParseLine(const char *line,
                                       DebugUartCommand_t *command)
{
  while (DebugUartCommand_IsSpace(*line))
  {
    line++;
  }

  command->type = DEBUG_UART_COMMAND_INVALID;
  command->target_turn_milli = 0;
  command->target_position_permille = 0;

  if (((*line == 'P') || (*line == 'p')) && (line[1] == '\0'))
  {
    command->type = DEBUG_UART_COMMAND_START;
    return true;
  }
  if (((*line == 'S') || (*line == 's')) && (line[1] == '\0'))
  {
    command->type = DEBUG_UART_COMMAND_STOP;
    return true;
  }
  if (((*line == 'H') || (*line == 'h')) && (line[1] == '\0'))
  {
    command->type = DEBUG_UART_COMMAND_HOME;
    return true;
  }
  if (((*line == 'C') || (*line == 'c')) && (line[1] == '\0'))
  {
    command->type = DEBUG_UART_COMMAND_CLEAR_FAULT;
    return true;
  }
  if (((*line == 'Q') || (*line == 'q')) && (line[1] == '\0'))
  {
    command->type = DEBUG_UART_COMMAND_STATUS;
    return true;
  }
  if (((*line == 'Z') || (*line == 'z')) && (line[1] == '\0'))
  {
    command->type = DEBUG_UART_COMMAND_ZERO;
    return true;
  }
  if ((*line == 'T') || (*line == 't'))
  {
    line++;
    if (DebugUartCommand_ParseTurnMilli(line, &command->target_turn_milli))
    {
      command->type = DEBUG_UART_COMMAND_TARGET_TURN;
    }
    return true;
  }
  if ((*line == 'G') || (*line == 'g'))
  {
    line++;
    if (DebugUartCommand_ParsePosition(line,
                                       &command->target_position_permille))
    {
      command->type = DEBUG_UART_COMMAND_TARGET_POSITION;
    }
    return true;
  }

  return true;
}

void DebugUartCommand_Init(UART_HandleTypeDef *huart)
{
  debugUartHandle = huart;
  debugUartRxHead = 0U;
  debugUartRxTail = 0U;

  if (debugUartHandle != NULL)
  {
    DebugUartCommand_FlushHardwareRx();
    (void)HAL_UART_Receive_IT(debugUartHandle, &debugUartRxByte, 1U);
  }
}

bool DebugUartCommand_Poll(DebugUartCommand_t *command)
{
  static char commandBuffer[DEBUG_UART_COMMAND_BUFFER_SIZE];
  static uint8_t commandLength = 0U;
  static uint32_t lastReceiveTick = 0U;
  uint8_t received;

  if (command == NULL)
  {
    return false;
  }

  command->type = DEBUG_UART_COMMAND_NONE;
  command->target_turn_milli = 0;
  command->target_position_permille = 0;

  while (DebugUartCommand_PopByte(&received))
  {
    char ch = (char)received;
    lastReceiveTick = HAL_GetTick();

    if ((ch == '\r') || (ch == '\n'))
    {
      if (commandLength == 0U)
      {
        continue;
      }
      commandBuffer[commandLength] = '\0';
      commandLength = 0U;
      return DebugUartCommand_ParseLine(commandBuffer, command);
    }

    if ((commandLength == 0U) &&
        ((ch == 'P') || (ch == 'p') || (ch == 'S') || (ch == 's') ||
         (ch == 'H') || (ch == 'h') || (ch == 'C') || (ch == 'c') ||
         (ch == 'Q') || (ch == 'q') || (ch == 'Z') || (ch == 'z')))
    {
      commandBuffer[0] = ch;
      commandBuffer[1] = '\0';
      return DebugUartCommand_ParseLine(commandBuffer, command);
    }

    if (commandLength < (DEBUG_UART_COMMAND_BUFFER_SIZE - 1U))
    {
      commandBuffer[commandLength++] = ch;
    }
    else
    {
      commandLength = 0U;
      command->type = DEBUG_UART_COMMAND_INVALID;
      return true;
    }
  }

  if ((commandLength > 0U) &&
      ((HAL_GetTick() - lastReceiveTick) >= DEBUG_UART_COMMAND_IDLE_TIMEOUT_MS))
  {
    commandBuffer[commandLength] = '\0';
    commandLength = 0U;
    return DebugUartCommand_ParseLine(commandBuffer, command);
  }

  return false;
}

void DebugUartCommand_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if ((debugUartHandle != NULL) && (huart == debugUartHandle))
  {
    if (!DebugUartTransport_IsTxActive(huart))
    {
      /* 485调试口发送日志时可能回读到自身数据，避免把日志当命令解析。 */
      DebugUartCommand_PushByte(debugUartRxByte);
    }
    (void)HAL_UART_Receive_IT(debugUartHandle, &debugUartRxByte, 1U);
  }
}

void DebugUartCommand_ErrorCallback(UART_HandleTypeDef *huart)
{
  if ((debugUartHandle != NULL) && (huart == debugUartHandle))
  {
    DebugUartCommand_FlushHardwareRx();
    (void)HAL_UART_Receive_IT(debugUartHandle, &debugUartRxByte, 1U);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  DebugUartCommand_RxCpltCallback(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  DebugUartCommand_ErrorCallback(huart);
  TactileSensor_ErrorCallback(huart);
}
