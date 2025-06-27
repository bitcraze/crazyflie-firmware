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

#define FILTER_SIZE 50

// This structure contains the mutable state and inmutable parameters
typedef struct controllerLee_s {
    // Quadrotor parameters
    float m;
    struct vec J; // Inertia matrix (diagonal matrix); kg m^2

    // Gains
    float kx;
    float kv;
    float ki;
    float c1;
    float sigma;

    float kR;
    float kW;
    float kI;
    float c2;

    // Errors
    struct vec ex;
    struct vec ev;
    struct vec ei;

    struct vec eR;
    struct vec eW;
    struct vec eI;

    // Wrench
    float f;
    struct vec M;

    // Previous values
    struct mat33 R_d_prev;
    struct vec W_d_prev;

    struct vec W_d_raw[FILTER_SIZE];
    struct vec W_d_dot_raw[FILTER_SIZE];

    struct vec W_d;
    struct vec W_d_dot;

    uint8_t track_attitude_rate;
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
