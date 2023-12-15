
#ifndef __CONTROLLER_OMNI_ATT_H__
#define __CONTROLLER_OMNI_ATT_H__

#include "stabilizer_types.h"

void controllerOmniAttInit(void);
bool controllerOmniAttTest(void);
void controllerOmniAtt(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const stabilizerStep_t stabilizerStep);

#endif 
