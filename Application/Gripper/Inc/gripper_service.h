#ifndef GRIPPER_SERVICE_H
#define GRIPPER_SERVICE_H

#include "gripper_controller.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  GripperState_t state;
  uint32_t faults;
  uint32_t state_elapsed_ms;
  uint32_t stall_elapsed_ms;
  bool homed;
  int16_t target_position_permille;
  int16_t position_permille;
  int32_t target_count;
  int32_t position_count;
  int32_t open_count;
  int32_t close_count;
  float speed_ref_rpm;
  float speed_rpm;
  float iq_ref_a;
  float id_ref_a;
  float iq_a;
  float id_a;
  float bus_current_a;
  uint16_t bus_voltage_v;
  uint16_t temperature_raw;
  uint16_t mc_faults;
  uint32_t encoder_spi_errors;
  uint8_t motor_state;
  uint32_t motor_faults;
  bool encoder_reliable;
} GripperStatus_t;

void GripperService_CreateTask(void);
bool GripperService_Home(void);
bool GripperService_SetPosition(int16_t position_permille);
bool GripperService_Stop(void);
bool GripperService_ClearFaults(void);
bool GripperService_LatchExternalFault(uint32_t fault);
void GripperService_GetStatus(GripperStatus_t *status);

#ifdef __cplusplus
}
#endif

#endif
