/*
 *    ______
 *   / ____/________ _____  __  ________      ______ __________ ___
 *  / /   / ___/ __ `/_  / / / / / ___/ | /| / / __ `/ ___/ __ `__ \
 * / /___/ /  / /_/ / / /_/ /_/ (__  )| |/ |/ / /_/ / /  / / / / / /
 * \____/_/   \__,_/ /___/\__, /____/ |__/|__/\__,_/_/  /_/ /_/ /_/
 *                       /____/
 *
 * Crazyswarm advanced control firmware for Crazyflie
 *

The MIT License (MIT)

Copyright (c) 2018 Wolfgang Hoenig and James Alan Preiss

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

/*
Header file for high-level commander that computes smooth setpoints based on high-level inputs.
*/

#ifndef CRTP_COMMANDER_HIGH_LEVEL_H_
#define CRTP_COMMANDER_HIGH_LEVEL_H_

#include <stdbool.h>
#include <stdint.h>

#include "math3d.h"

#include "stabilizer_types.h"

#define NUM_TRAJECTORY_DEFINITIONS 31

typedef enum {
  CRTP_CHL_TRAJECTORY_TYPE_POLY4D = 0, // struct poly4d, see pptraj.h
  CRTP_CHL_TRAJECTORY_TYPE_POLY4D_COMPRESSED = 1, // see pptraj_compressed.h
  // Future types might include versions without yaw
} crtpCommanderTrajectoryType_t;

/* Public functions */
void crtpCommanderHighLevelInit(void);

// Retrieves the current setpoint. Returns false if the high-level commander is
// disabled, i.e. it does not have an "opinion" on what the setpoint should be.
bool crtpCommanderHighLevelGetSetpoint(setpoint_t* setpoint, const state_t *state, stabilizerStep_t stabilizerStep);

// When flying sequences of high-level commands, the high-level commander uses
// its own history of commands to determine the initial conditions of the next
// trajectory it plans. However, when switching from a low-level streaming
// setpoint mode to high-level, any past command history is invalid because of
// the intervening low-level commands. Therefore, we must tell the high-level
// commander what initial conditions to use for trajectory planning.
void crtpCommanderHighLevelTellState(const state_t *state);

// True if we have landed or stopped.
bool crtpCommanderHighLevelIsStopped();

// Public API - can be used from an app

/**
 * @brief vertical takeoff from current x-y position to given height
 *
 * @param absoluteHeight_m absolute target height (m)
 * @param duration_s       time it should take until target height is reached (s)
 * @return zero if the command succeeded, an error code otherwise
 */
int crtpCommanderHighLevelTakeoff(const float absoluteHeight_m, const float duration_s);

/**
 * @brief vertical takeoff from current x-y position to given absolute or relative
 * height with given velocity
 *
 * @param height_m         absolute or relative target height (m)
 * @param velocity_m_s     takeoff velocity (m/s)
 * @param relative         whether the height is relative to the current position
 * @return zero if the command succeeded, an error code otherwise
 */
int crtpCommanderHighLevelTakeoffWithVelocity(const float height_m, const float velocity_m_s, bool relative);


/**
 * @brief vertical takeoff from current x-y position to given height
 *
 * @param absoluteHeight_m absolute target height (m)
 * @param duration_s       time it should take until target height is reached (s)
 * @param yaw              yaw (rad)
 * @return zero if the command succeeded, an error code otherwise
 */
int crtpCommanderHighLevelTakeoffYaw(const float absoluteHeight_m, const float duration_s, const float yaw);

/**
 * @brief vertical land from current x-y position to given height
 *
 * @param absoluteHeight_m absolute target height (m)
 * @param duration_s       time it should take until target height is reached (s)
 * @return zero if the command succeeded, an error code otherwise
 */
int crtpCommanderHighLevelLand(const float absoluteHeight_m, const float duration_s);

/**
 * @brief vertical land from current x-y position to given absolute or relative
 * height with given velocity
 *
 * @param height_m         absolute or relative target height (m)
 * @param velocity_m_s     landing velocity (m/s)
 * @param relative         whether the height is relative to the current position
 * @return zero if the command succeeded, an error code otherwise
 */
int crtpCommanderHighLevelLandWithVelocity(const float height_m, const float velocity_m_s, bool relative);

/**
 * @brief vertical land from current x-y position to given height
 *
 * @param absoluteHeight_m absolute target height (m)
 * @param duration_s       time it should take until target height is reached (s)
 * @param yaw              yaw (rad)
 * @return zero if the command succeeded, an error code otherwise
 */
int crtpCommanderHighLevelLandYaw(const float absoluteHeight_m, const float duration_s, const float yaw);

/**
 * @brief stops the current trajectory (turns off the motors)
 *
 * Send the trajectory planner to stopped state, where it requests motors off.
 *
 * @return zero if the command succeeded, an error code otherwise. The function
 * should never fail, but we provide the error code nevertheless for sake of
 * consistency with the other high-level commander functions.
 */
int crtpCommanderHighLevelStop();

/**
 * @brief stops the current trajectory (without requesting motors off)
 *
 * Send the trajectory planner to disabled state, where it does not generate
 * setpoints, but also does not request motors off.
 *
 * @return zero if the command succeeded, an error code otherwise. The function
 * should never fail, but we provide the error code nevertheless for sake of
 * consistency with the other high-level commander functions.
 */
int crtpCommanderHighLevelDisable();

/**
 * @brief Block/unblock the use of the high level commander.
 *
 * This function is called from the stabilizer loop. The purpose is to provide a way for the supervisor to block a user
 * (or app) from starting a trajectory when the system is not in a flyable state.
 *
 * When entering the blocked state, the planer will stop any running trajectory and go to the stopped state. If
 * the planner already is in the stopped or disabled state, it will remain.
 *
 * When blocked, functions that plans a new trajectory will be blocked.
 *
 * @param doBlock Enter blocked state if true, unblock if false
 * @return zero if the command succeeded, an error code otherwise. The function
 * should never fail, but we provide the error code nevertheless for sake of
 * consistency with the other high-level commander functions.
 */
int crtpCommanderBlock(bool doBlock);

/**
 * @brief Check if the high level commander is blocked by the supervisor
 *
 * @return true   If the high level commander is blocked by the supervisor
 * @return false  If not blocked
 */
bool crtpCommanderHighLevelIsBlocked();

/**
 * @brief Go to an absolute or relative position (will be deprecated, use crtpCommanderHighLevelGoTo2)
 *
 * @param x          x (m)
 * @param y          y (m)
 * @param z          z (m)
 * @param yaw        yaw (rad)
 * @param duration_s time it should take to reach the position (s)
 * @param relative   true if x, y, z is relative to the current position
 * @return zero if the command succeeded, an error code otherwise
 */
int crtpCommanderHighLevelGoTo(const float x, const float y, const float z, const float yaw, const float duration_s, const bool relative);

/**
 * @brief Go to an absolute or relative position
 *
 * @param x          x (m)
 * @param y          y (m)
 * @param z          z (m)
 * @param yaw        yaw (rad)
 * @param duration_s time it should take to reach the position (s)
 * @param relative   true if x, y, z is relative to the current position
 * @param linear     true if linear interpolation should be used instead of a smooth polynomial
 * @return zero if the command succeeded, an error code otherwise
 */
int crtpCommanderHighLevelGoTo2(const float x, const float y, const float z, const float yaw, const float duration_s, const bool relative, const bool linear);

/**
 * @brief Follow a spiral segment (spline approximation of and arc for <= 90-degree segments)
 *
 * @param phi         spiral angle (rad), limited to +/- 2pi
 * @param r0          initial radius (m), must be positive
 * @param rf          final radius (m), must be positive
 * @param dz          altitude gain (m), positive to climb, negative to descent
 * @param duration_s  time it should take to reach the end of the spiral (s)
 * @param sideways    true if crazyflie should spiral sideways instead of forward
 * @return zero if the command succeeded, an error code otherwise
 */
int crtpCommanderHighLevelSpiral(const float phi, const float r0, const float rf, const float dz, const float duration_s, const bool sideways);

/**
 * @brief Returns whether the trajectory with the given ID is defined
 *
 * @param trajectoryId The id of the trajectory
 */
bool crtpCommanderHighLevelIsTrajectoryDefined(uint8_t trajectoryId);

/**
 * @brief starts executing a specified trajectory
 *
 * @param trajectoryId id of the trajectory (previously defined by define_trajectory)
 * @param timeScale    time factor; 1.0 = original speed;
 *                                  >1.0: slower;
 *                                  <1.0: faster
 * @param relative     set to True, if trajectory should be shifted to current setpoint
 * @param reversed     set to True, if trajectory should be executed in reverse
 * @return zero if the command succeeded, an error code otherwise
 */
int crtpCommanderHighLevelStartTrajectory(const uint8_t trajectoryId, const float timeScale, const bool relative, const bool reversed);

/**
 * @brief Define a trajectory that has previously been uploaded to memory.
 *
 * @param trajectoryId The id of the trajectory
 * @param type         The type of trajectory that is stored in memory.
 * @param offset       offset in uploaded memory (bytes)
 * @param nPieces      Nr of pieces in the trajectory
 * @return zero if the command succeeded, an error code otherwise
 */
int crtpCommanderHighLevelDefineTrajectory(const uint8_t trajectoryId, const crtpCommanderTrajectoryType_t type, const uint32_t offset, const uint8_t nPieces);

/**
 * @brief Get the size of the allocated trajectory memory
 *
 * @return uint32_t The size of the trajectory memory in bytes
 */
uint32_t crtpCommanderHighLevelTrajectoryMemSize();

/**
 * @brief Copy trajectory data to the trajectory memeory. After the copy crtpCommanderHighLevelDefineTrajectory()
 *        must be called before the trajectory can be used.
 *
 * @param offset    offset in uploaded memory (bytes)
 * @param length    Length of the data (bytes) to copy to the trajectory memory
 * @param data  pointer to the trajectory data source
 *
 * @return true   If data was copied
 * @return false  If data is too large
 */
bool crtpCommanderHighLevelWriteTrajectory(const uint32_t offset, const uint32_t length, const uint8_t* data);

/**
 * @brief Copy data from the trajectory memory.
 *
 * @param offset             Offset in the trajectory memory (bytes)
 * @param length             Length of the data to copy
 * @param destination [out]  Pointer to copy data to
 * @return true              If data was copied
 * @return false             If length is too large
 */
bool crtpCommanderHighLevelReadTrajectory(const uint32_t offset, const uint32_t length, uint8_t* destination);

/**
 * @brief Query if the current trajectory has finished
 *
 * @return true   The trajectory has reached the end
 * @return false  The trejectory is still running
 */
bool crtpCommanderHighLevelIsTrajectoryFinished();


/**
 * @brief Query the current state of the planner in high level control
 *
 * @return trajectory state enum defined in planner.h
 */

enum trajectory_state crtpCommanderHighLevelGetPlannerState();

#endif /* CRTP_COMMANDER_HIGH_LEVEL_H_ */
