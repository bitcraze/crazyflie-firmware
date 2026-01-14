#pragma once
#include <stdbool.h>

#include "stabilizer_types.h"

void controllerForceTorqueExternalInit(void);
bool controllerForceTorqueExternalTest(void);
void controllerForceTorqueExternal(control_t *control,
                                   const setpoint_t *setpoint,
                                   const sensorData_t *sensors,
                                   const state_t *state, const uint32_t tick);
