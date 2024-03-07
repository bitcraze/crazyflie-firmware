

/**
 * Authored by Mike Wen, 2024.March
 *
 * ============================================================================
 */

#ifndef GIMBAL2D_CONTROLLER_H_
#define GIMBAL2D_CONTROLLER_H_

#include "stabilizer_types.h"

void controllerGimbal2DInit(void);
bool controllerGimbal2DTest(void);
void controllerGimbal2D(control_t *control,
                        const setpoint_t *setpoint,
                        const sensorData_t *sensors,
                        const state_t *state,
                        const stabilizerStep_t stabilizerStep);
