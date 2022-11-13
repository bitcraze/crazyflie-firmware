#ifndef __CONTROLLER_LQR_H__
#define __CONTROLLER_LQR_H__

#include "stabilizer_types.h"

void controllerLqrInit(void);
bool controllerLqrTest(void);
void controllerLqr(control_t *control, setpoint_t *setpoint,
                                        const sensorData_t *sensor,
                                        const state_t *state,
                                        const uint32_t tick);

#endif