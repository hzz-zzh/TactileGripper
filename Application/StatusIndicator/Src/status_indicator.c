#include "status_indicator.h"

#include "gripper_service.h"
#include "ws2812_led.h"
#include "cmsis_os2.h"
#include "main.h"

#include <stdbool.h>
#include <stdint.h>

#define STATUS_INDICATOR_PERIOD_TICKS       50U  /* FreeRTOS 2kHz，对应25ms。 */

extern SPI_HandleTypeDef hspi3;

static osThreadId_t statusIndicatorTaskHandle;

static const Ws2812Color_t COLOR_RED     = {32U, 0U, 0U};
static const Ws2812Color_t COLOR_GREEN   = {0U, 10U, 0U};
static const Ws2812Color_t COLOR_PURPLE  = {24U, 0U, 24U};

static bool StatusIndicator_ColorEqual(Ws2812Color_t first,
                                       Ws2812Color_t second)
{
  return (first.red == second.red) &&
         (first.green == second.green) &&
         (first.blue == second.blue);
}

static Ws2812Color_t StatusIndicator_SelectColor(
  const GripperStatus_t *status)
{
  if ((status->faults != GRIPPER_FAULT_NONE) ||
      (status->motor_faults != 0U) ||
      (status->mc_faults != 0U) ||
      (status->state == GRIPPER_STATE_FAULT))
  {
    return COLOR_RED;
  }

  switch (status->state)
  {
    case GRIPPER_STATE_PRECHECK:
    case GRIPPER_STATE_STARTING:
    case GRIPPER_STATE_HOMING_OPEN:
    case GRIPPER_STATE_HOMING_CLOSE:
    case GRIPPER_STATE_MOVING_SAFE:
      return COLOR_PURPLE;

    default:
      return COLOR_GREEN;
  }
}

static void StatusIndicator_Task(void *argument)
{
  uint32_t next = osKernelGetTickCount();
  Ws2812Color_t lastColor = {255U, 255U, 255U};
  bool pendingUpdate = true;
  (void)argument;

  Ws2812Led_Init(&hspi3);

  for (;;)
  {
    GripperStatus_t status;
    Ws2812Color_t targetColor;

    next += STATUS_INDICATOR_PERIOD_TICKS;
    GripperService_GetStatus(&status);
    targetColor = StatusIndicator_SelectColor(&status);
    if (!StatusIndicator_ColorEqual(targetColor, lastColor))
    {
      pendingUpdate = true;
    }
    if (pendingUpdate && Ws2812Led_SetColor(targetColor))
    {
      lastColor = targetColor;
      pendingUpdate = false;
    }
    (void)osDelayUntil(next);
  }
}

void StatusIndicator_CreateTask(void)
{
  const osThreadAttr_t taskAttr = {
    .name = "statusLed",
    .stack_size = 256U * 4U,
    .priority = osPriorityLow
  };

  statusIndicatorTaskHandle = osThreadNew(StatusIndicator_Task, NULL,
                                          &taskAttr);
  if (statusIndicatorTaskHandle == NULL)
  {
    /* 指示灯不是安全链路，创建失败时不影响电机控制任务。 */
  }
}
