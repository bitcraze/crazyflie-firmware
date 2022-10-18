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
#ifndef __CONTROLLER_LEE_PAYLOAD_H__
#define __CONTROLLER_LEE_PAYLOAD_H__

#include "stabilizer_types.h"

// This structure contains the mutable state and inmutable parameters
typedef struct controllerLeePayload_s {
    float mass; // TODO: should be CF global for other modules
    float mp; // mass payload
    float l; // length of cable;
    float thrustSI;
    struct vec J; // Inertia matrix (diagonal matrix); kg m^2
    struct vec offset; // offset for reference
    //Position PID
    struct vec Kpos_P;
    float Kpos_P_limit;
    struct vec Kpos_D;
    float Kpos_D_limit;
    struct vec Kpos_I;
    float Kpos_I_limit;

    struct vec i_error_pos;
    struct vec i_error_att;

   // Cable PD 
    struct vec K_q;
    struct vec K_w;

   // Cable errors
    struct vec plp_error;
    struct vec plv_error;

   //Attitude PID
    struct vec KR;
    struct vec Komega;
    struct vec KI;

    // Logging variables
    struct vec qi_prev;
    struct vec qdi_prev;
    struct vec payload_vel_prev;
    struct vec rpy;
    struct vec rpy_des;
    struct vec qr;
    struct mat33 R_des;
    struct quat q;
    struct mat33 R;
    struct vec omega;
    struct vec omega_r;
    struct vec u;
    struct vec u_i;
    struct vec qidot_prev;
    struct vec acc_prev;

    // -----------------------FOR QP----------------------------//
    // P_alloc matrix 
    struct mat36 P_alloc;
    // Weight matrix of the QP
    struct mat66 P;
    // desired value from the QP
    struct vec desVirtInp;
     // inequality matrix A
    struct mat26 A_in;
    // value: 0,1: defines which part of the vector in desVirtInp goes to which UAV
    float value;
    // angle limit for hyper plane
    struct vec rpyPlane1;
    float yawPlane1; // rpy and yaw for one plane of the first UAV
    struct vec rpyPlane2; 
    float yawPlane2; //rpy and yaw for one plane second UAV
} controllerLeePayload_t;


void controllerLeePayloadInit(controllerLeePayload_t* self);
void controllerLeePayloadReset(controllerLeePayload_t* self);
void controllerLeePayload(controllerLeePayload_t* self, control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);

#ifdef CRAZYFLIE_FW
void controllerLeePayloadFirmwareInit(void);
bool controllerLeePayloadFirmwareTest(void);
void controllerLeePayloadFirmware(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);
#endif //   CRAZYFLIE_FW defined

#endif //__CONTROLLER_LEE_PAYLOAD_H__
