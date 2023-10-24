/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */
#ifndef __CONTROLLER_LEE_H__
#define __CONTROLLER_LEE_H__

#include "stabilizer_types.h"

// This structure contains the mutable state and inmutable parameters
typedef struct controllerLee_s {
    float mass; // TODO: should be CF global for other modules
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
    struct vec qr;
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
