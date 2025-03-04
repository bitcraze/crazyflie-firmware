/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2021-2023 Bitcraze AB
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
 * supervisor.h - Keep track of system state
 */

#pragma once

#include "stabilizer_types.h"

/**
 * @brief Update the supervisor state.
 *
 * The overall process includes:
 *   - collecting data from the system (conditions)
 *   - possibly change state, based on the conditions
 *
 * @param sensors        Latest sensor data
 * @param setpoint       Current setpoint
 * @param stabilizerStep Stabilizer step for rate control
 */
void supervisorUpdate(const sensorData_t *sensors, const setpoint_t* setpoint, stabilizerStep_t stabilizerStep);

/**
 * @brief Replace the values in the current setpoint, if required.
 *
 * If the supervisor thinks it is necessary to take precautions, for instance in case of an emergency stop,
 * if may replace values in the current setpoint.
 *
 * @param setpoint The current setpoint
 */
void supervisorOverrideSetpoint(setpoint_t* setpoint);

/**
 * @brief Check if it is OK to spin the motors
 *
 * @return true  OK to spin motors
 * @return false Not OK to spin motors
 */
bool supervisorAreMotorsAllowedToRun();

/**
 * @brief Is the system ready to fly
 *
 * @return true
 * @return false
 */
bool supervisorCanFly(void);

/**
 * @brief Is the Crazyflie flying
 *
 * @return true
 * @return false
 */
bool supervisorIsFlying(void);

/**
 * @brief Is the Crazyflie tumbled
 *
 * @return true
 * @return false
 */
bool supervisorIsTumbled(void);

/**
 * @brief Request the system to be armed or disarmed. The system can only be armed under certain conditions, this is
 * indicated by the supervisorCanArm() function.
 *
 * @param doArm  true - request arming. false - request disarming.
 * @return true   The request was granted
 * @return false  The request could not be granted
 */
bool supervisorRequestArming(const bool doArm);

/**
 * @brief Query if the system can be armed
 *
 * @return true
 * @return false
 */
bool supervisorCanArm();

/**
 * @brief Query if the system is armed
 *
 * @return true
 * @return false
 */
bool supervisorIsArmed();

/**
 * @brief Query if the system is locked (due to an emergency stop or watchdog timeout)
 *
 * @return true
 * @return false
 */
bool supervisorIsLocked();

/**
 * @brief Query if the system is crashed
 *
 * @return true
 * @return false
 */
bool supervisorIsCrashed();

/**
 * @brief Request the system to be recover if crashed.
 *
 * @param doRecover true - request recovery. false - request crashed.
 * @return true   The request was granted
 * @return false  The request could not be granted
 */
bool supervisorRequestCrashRecovery(const bool doRecover);