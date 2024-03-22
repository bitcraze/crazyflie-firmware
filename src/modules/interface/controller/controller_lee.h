/*
The MIT License (MIT)

Copyright (c) 2024 Khaled Wahba

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#ifndef __CONTROLLER_LEE_H__
#define __CONTROLLER_LEE_H__

#include "stabilizer_types.h"

// This structure contains the mutable state and inmutable parameters
typedef struct controllerLee_s {
    float mass;
    float thrustSi;
    struct vec J; // Inertia matrix (diagonal matrix); kg m^2

    // Position PID
    struct vec Kpos_P; // Kp in paper
    float Kpos_P_limit;
    struct vec Kpos_D; // Kv in paper
    float Kpos_D_limit;
    struct vec Kpos_I; // not in paper
    float Kpos_I_limit;
    struct vec i_error_pos;
    struct vec p_error;
    struct vec v_error;
    // Attitude PID
    struct vec KR;
    struct vec Komega;
    struct vec KI;
    struct vec i_error_att;
    // Logging variables
    struct vec rpy;
    struct vec rpy_des;
    struct mat33 R_des;
    struct vec omega;
    struct vec omega_r;
    struct vec u;
} controllerLee_t;


void controllerLeeInit(controllerLee_t* self);
void controllerLeeReset(controllerLee_t* self);
void controllerLee(controllerLee_t* self, control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);

#ifdef CRAZYFLIE_FW
void controllerLeeFirmwareInit(void);
bool controllerLeeFirmwareTest(void);
void controllerLeeFirmware(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);
#endif // CRAZYFLIE_FW defined

#endif //__CONTROLLER_LEE_H__
