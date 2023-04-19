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
 * controller_mellinger.h - Mellinger Controller Interface
 */
#ifndef __CONTROLLER_MELLINGER_H__
#define __CONTROLLER_MELLINGER_H__

#include "stabilizer_types.h"
#include "math3d.h"

typedef struct {
    float mass;
    float massThrust;

    // XY Position PID
    float kp_xy;      // P
    float kd_xy;      // D
    float ki_xy;      // I
    float i_range_xy;

    // Z Position
    float kp_z;       // P
    float kd_z;       // D
    float ki_z;       // I
    float i_range_z;

    // Attitude
    float kR_xy;      // P
    float kw_xy;      // D
    float ki_m_xy;    // I
    float i_range_m_xy;

    // Yaw
    float kR_z;       // P
    float kw_z;       // D
    float ki_m_z;     // I
    float i_range_m_z;

    // roll and pitch angular velocity
    float kd_omega_rp; // D

    // Helper variables
    float i_error_x;
    float i_error_y;
    float i_error_z;

    float prev_omega_roll;
    float prev_omega_pitch;
    float prev_setpoint_omega_roll;
    float prev_setpoint_omega_pitch;

    float i_error_m_x;
    float i_error_m_y;
    float i_error_m_z;

    // Logging variables
    struct vec z_axis_desired;

    float cmd_thrust;
    float cmd_roll;
    float cmd_pitch;
    float cmd_yaw;
    float r_roll;
    float r_pitch;
    float r_yaw;
    float accelz;
} controllerMellinger_t;

void controllerMellingerInit(controllerMellinger_t* self);
bool controllerMellingerTest(controllerMellinger_t* self);
void controllerMellinger(controllerMellinger_t* self, control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const stabilizerStep_t stabilizerStep);

#ifdef CRAZYFLIE_FW

void controllerMellingerFirmwareInit(void);
bool controllerMellingerFirmwareTest(void);
void controllerMellingerFirmware(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const stabilizerStep_t stabilizerStep);

#endif // CRAZYFLIE_FW

#endif //__CONTROLLER_MELLINGER_H__
