/*
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
This controller is based on the following publication:

Daniel Mellinger, Vijay Kumar:
Minimum snap trajectory generation and control for quadrotors.
IEEE International Conference on Robotics and Automation (ICRA), 2011.

We added the following:
 * Integral terms (compensates for: battery voltage drop over time, unbalanced center of mass due to asymmetries, and uneven wear on propellers and motors)
 * D-term for angular velocity
 * Support to use this controller as an attitude-only controller for manual flight
*/

#include <math.h>

#include "param.h"
#include "log.h"
#include "position_controller.h"
#include "controller_mellinger.h"
#include "physicalConstants.h"
#include "platform_defaults.h"

// Global state variable used in the
// firmware as the only instance and in bindings
// to hold the default values
static controllerMellinger_t g_self = {
  .mass = CF_MASS,
  .massThrust = 132000,

  // XY Position PID
  .kp_xy = 0.4,       // P
  .kd_xy = 0.2,       // D
  .ki_xy = 0.05,      // I
  .i_range_xy = 2.0,

  // Z Position
  .kp_z = 1.25,       // P
  .kd_z = 0.4,        // D
  .ki_z = 0.05,       // I
  .i_range_z  = 0.4,

  // Attitude
  .kR_xy = 70000, // P
  .kw_xy = 20000, // D
  .ki_m_xy = 0.0, // I
  .i_range_m_xy = 1.0,

  // Yaw
  .kR_z = 60000, // P
  .kw_z = 12000, // D
  .ki_m_z = 500, // I
  .i_range_m_z  = 1500,

  // roll and pitch angular velocity
  .kd_omega_rp = 200, // D


  // Helper variables
  .i_error_x = 0,
  .i_error_y = 0,
  .i_error_z = 0,

  .i_error_m_x = 0,
  .i_error_m_y = 0,
  .i_error_m_z = 0,
};


void controllerMellingerReset(controllerMellinger_t* self)
{
  self->i_error_x = 0;
  self->i_error_y = 0;
  self->i_error_z = 0;
  self->i_error_m_x = 0;
  self->i_error_m_y = 0;
  self->i_error_m_z = 0;
}

void controllerMellingerInit(controllerMellinger_t* self)
{
  // copy default values (bindings), or does nothing (firmware)
  *self = g_self;

  controllerMellingerReset(self);
}

bool controllerMellingerTest(controllerMellinger_t* self)
{
  return true;
}

void controllerMellinger(controllerMellinger_t* self, control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const stabilizerStep_t stabilizerStep)
{
  struct vec r_error;
  struct vec v_error;
  struct vec target_thrust;
  struct vec z_axis;
  float current_thrust;
  struct vec x_axis_desired;
  struct vec y_axis_desired;
  struct vec x_c_des;
  struct vec eR, ew, M;
  float dt;
  float desiredYaw = 0; //deg

  control->controlMode = controlModeLegacy;

  if (!RATE_DO_EXECUTE(ATTITUDE_RATE, stabilizerStep)) {
    return;
  }

  dt = (float)(1.0f/ATTITUDE_RATE);
  struct vec setpointPos = mkvec(setpoint->position.x, setpoint->position.y, setpoint->position.z);
  struct vec setpointVel = mkvec(setpoint->velocity.x, setpoint->velocity.y, setpoint->velocity.z);
  struct vec statePos = mkvec(state->position.x, state->position.y, state->position.z);
  struct vec stateVel = mkvec(state->velocity.x, state->velocity.y, state->velocity.z);

  // Position Error (ep)
  r_error = vsub(setpointPos, statePos);

  // Velocity Error (ev)
  v_error = vsub(setpointVel, stateVel);

  // Integral Error
  self->i_error_z += r_error.z * dt;
  self->i_error_z = clamp(self->i_error_z, -self->i_range_z, self->i_range_z);

  self->i_error_x += r_error.x * dt;
  self->i_error_x = clamp(self->i_error_x, -self->i_range_xy, self->i_range_xy);

  self->i_error_y += r_error.y * dt;
  self->i_error_y = clamp(self->i_error_y, -self->i_range_xy, self->i_range_xy);

  // Desired thrust [F_des]
  if (setpoint->mode.x == modeAbs) {
    target_thrust.x = self->mass * setpoint->acceleration.x                       + self->kp_xy * r_error.x + self->kd_xy * v_error.x + self->ki_xy * self->i_error_x;
    target_thrust.y = self->mass * setpoint->acceleration.y                       + self->kp_xy * r_error.y + self->kd_xy * v_error.y + self->ki_xy * self->i_error_y;
    target_thrust.z = self->mass * (setpoint->acceleration.z + GRAVITY_MAGNITUDE) + self->kp_z  * r_error.z + self->kd_z  * v_error.z + self->ki_z  * self->i_error_z;
  } else {
    target_thrust.x = -sinf(radians(setpoint->attitude.pitch));
    target_thrust.y = -sinf(radians(setpoint->attitude.roll));
    // In case of a timeout, the commander tries to level, ie. x/y are disabled, but z will use the previous setting
    // In that case we ignore the last feedforward term for acceleration
    if (setpoint->mode.z == modeAbs) {
      target_thrust.z = self->mass * GRAVITY_MAGNITUDE + self->kp_z  * r_error.z + self->kd_z  * v_error.z + self->ki_z  * self->i_error_z;
    } else {
      target_thrust.z = 1;
    }
  }

  // Rate-controlled YAW is moving YAW angle setpoint
  if (setpoint->mode.yaw == modeVelocity) {
    desiredYaw = state->attitude.yaw + setpoint->attitudeRate.yaw * dt;
  } else if (setpoint->mode.yaw == modeAbs) {
    desiredYaw = setpoint->attitude.yaw;
  } else if (setpoint->mode.quat == modeAbs) {
    struct quat setpoint_quat = mkquat(setpoint->attitudeQuaternion.x, setpoint->attitudeQuaternion.y, setpoint->attitudeQuaternion.z, setpoint->attitudeQuaternion.w);
    struct vec rpy = quat2rpy(setpoint_quat);
    desiredYaw = degrees(rpy.z);
  }

  // Z-Axis [zB]
  struct quat q = mkquat(state->attitudeQuaternion.x, state->attitudeQuaternion.y, state->attitudeQuaternion.z, state->attitudeQuaternion.w);
  struct mat33 R = quat2rotmat(q);
  z_axis = mcolumn(R, 2);

  // yaw correction (only if position control is not used)
  if (setpoint->mode.x != modeAbs) {
    struct vec x_yaw = mcolumn(R, 0);
    x_yaw.z = 0;
    x_yaw = vnormalize(x_yaw);
    struct vec y_yaw = vcross(mkvec(0, 0, 1), x_yaw);
    struct mat33 R_yaw_only = mcolumns(x_yaw, y_yaw, mkvec(0, 0, 1));
    target_thrust = mvmul(R_yaw_only, target_thrust);
  }

  // Current thrust [F]
  current_thrust = vdot(target_thrust, z_axis);

  // Calculate axis [zB_des]
  self->z_axis_desired = vnormalize(target_thrust);

  // [xC_des]
  // x_axis_desired = z_axis_desired x [sin(yaw), cos(yaw), 0]^T
  x_c_des.x = cosf(radians(desiredYaw));
  x_c_des.y = sinf(radians(desiredYaw));
  x_c_des.z = 0;
  // [yB_des]
  y_axis_desired = vnormalize(vcross(self->z_axis_desired, x_c_des));
  // [xB_des]
  x_axis_desired = vcross(y_axis_desired, self->z_axis_desired);

  // [eR]
  // Slow version
  // struct mat33 Rdes = mcolumns(
  //   mkvec(x_axis_desired.x, x_axis_desired.y, x_axis_desired.z),
  //   mkvec(y_axis_desired.x, y_axis_desired.y, y_axis_desired.z),
  //   mkvec(z_axis_desired.x, z_axis_desired.y, z_axis_desired.z));

  // struct mat33 R_transpose = mtranspose(R);
  // struct mat33 Rdes_transpose = mtranspose(Rdes);

  // struct mat33 eRM = msub(mmult(Rdes_transpose, R), mmult(R_transpose, Rdes));

  // eR.x = eRM.m[2][1];
  // eR.y = -eRM.m[0][2];
  // eR.z = eRM.m[1][0];

  // Fast version (generated using Mathematica)
  float x = q.x;
  float y = q.y;
  float z = q.z;
  float w = q.w;
  eR.x = (-1 + 2*fsqr(x) + 2*fsqr(y))*y_axis_desired.z + self->z_axis_desired.y - 2*(x*y_axis_desired.x*z + y*y_axis_desired.y*z - x*y*self->z_axis_desired.x + fsqr(x)*self->z_axis_desired.y + fsqr(z)*self->z_axis_desired.y - y*z*self->z_axis_desired.z) +    2*w*(-(y*y_axis_desired.x) - z*self->z_axis_desired.x + x*(y_axis_desired.y + self->z_axis_desired.z));
  eR.y = x_axis_desired.z - self->z_axis_desired.x - 2*(fsqr(x)*x_axis_desired.z + y*(x_axis_desired.z*y - x_axis_desired.y*z) - (fsqr(y) + fsqr(z))*self->z_axis_desired.x + x*(-(x_axis_desired.x*z) + y*self->z_axis_desired.y + z*self->z_axis_desired.z) + w*(x*x_axis_desired.y + z*self->z_axis_desired.y - y*(x_axis_desired.x + self->z_axis_desired.z)));
  eR.z = y_axis_desired.x - 2*(y*(x*x_axis_desired.x + y*y_axis_desired.x - x*y_axis_desired.y) + w*(x*x_axis_desired.z + y*y_axis_desired.z)) + 2*(-(x_axis_desired.z*y) + w*(x_axis_desired.x + y_axis_desired.y) + x*y_axis_desired.z)*z - 2*y_axis_desired.x*fsqr(z) + x_axis_desired.y*(-1 + 2*fsqr(x) + 2*fsqr(z));

  // Account for Crazyflie coordinate system
  eR.y = -eR.y;

  // [ew]
  float err_d_roll = 0;
  float err_d_pitch = 0;

  float stateAttitudeRateRoll = radians(sensors->gyro.x);
  float stateAttitudeRatePitch = -radians(sensors->gyro.y);
  float stateAttitudeRateYaw = radians(sensors->gyro.z);

  ew.x = radians(setpoint->attitudeRate.roll) - stateAttitudeRateRoll;
  ew.y = -radians(setpoint->attitudeRate.pitch) - stateAttitudeRatePitch;
  ew.z = radians(setpoint->attitudeRate.yaw) - stateAttitudeRateYaw;
  if (self->prev_omega_roll == self->prev_omega_roll) { /*d part initialized*/
    err_d_roll = ((radians(setpoint->attitudeRate.roll) - self->prev_setpoint_omega_roll) - (stateAttitudeRateRoll - self->prev_omega_roll)) / dt;
    err_d_pitch = (-(radians(setpoint->attitudeRate.pitch) - self->prev_setpoint_omega_pitch) - (stateAttitudeRatePitch - self->prev_omega_pitch)) / dt;
  }
  self->prev_omega_roll = stateAttitudeRateRoll;
  self->prev_omega_pitch = stateAttitudeRatePitch;
  self->prev_setpoint_omega_roll = radians(setpoint->attitudeRate.roll);
  self->prev_setpoint_omega_pitch = radians(setpoint->attitudeRate.pitch);

  // Integral Error
  self->i_error_m_x += (-eR.x) * dt;
  self->i_error_m_x = clamp(self->i_error_m_x, -self->i_range_m_xy, self->i_range_m_xy);

  self->i_error_m_y += (-eR.y) * dt;
  self->i_error_m_y = clamp(self->i_error_m_y, -self->i_range_m_xy, self->i_range_m_xy);

  self->i_error_m_z += (-eR.z) * dt;
  self->i_error_m_z = clamp(self->i_error_m_z, -self->i_range_m_z, self->i_range_m_z);

  // Moment:
  M.x = -self->kR_xy * eR.x + self->kw_xy * ew.x + self->ki_m_xy * self->i_error_m_x + self->kd_omega_rp * err_d_roll;
  M.y = -self->kR_xy * eR.y + self->kw_xy * ew.y + self->ki_m_xy * self->i_error_m_y + self->kd_omega_rp * err_d_pitch;
  M.z = -self->kR_z  * eR.z + self->kw_z  * ew.z + self->ki_m_z  * self->i_error_m_z;

  // Output
  if (setpoint->mode.z == modeDisable) {
    control->thrust = setpoint->thrust;
  } else {
    control->thrust = self->massThrust * current_thrust;
  }

  self->cmd_thrust = control->thrust;
  self->r_roll = radians(sensors->gyro.x);
  self->r_pitch = -radians(sensors->gyro.y);
  self->r_yaw = radians(sensors->gyro.z);
  self->accelz = sensors->acc.z;

  if (control->thrust > 0) {
    control->roll = clamp(M.x, -32000, 32000);
    control->pitch = clamp(M.y, -32000, 32000);
    control->yaw = clamp(-M.z, -32000, 32000);

    self->cmd_roll = control->roll;
    self->cmd_pitch = control->pitch;
    self->cmd_yaw = control->yaw;

  } else {
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;

    self->cmd_roll = control->roll;
    self->cmd_pitch = control->pitch;
    self->cmd_yaw = control->yaw;

    controllerMellingerReset(self);
  }
}


void controllerMellingerFirmwareInit(void)
{
  controllerMellingerInit(&g_self);
}

bool controllerMellingerFirmwareTest(void)
{
  return controllerMellingerTest(&g_self);
}

void controllerMellingerFirmware(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const stabilizerStep_t stabilizerStep)
{
  controllerMellinger(&g_self, control, setpoint, sensors, state, stabilizerStep);
}


/**
 * Tunning variables for the full state Mellinger Controller
 */
PARAM_GROUP_START(ctrlMel)
/**
 * @brief Position P-gain (horizontal xy plane)
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, kp_xy, &g_self.kp_xy)
/**
 * @brief Position D-gain (horizontal xy plane)
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, kd_xy, &g_self.kd_xy)
/**
 * @brief Position I-gain (horizontal xy plane)
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, ki_xy, &g_self.ki_xy)
/**
 * @brief Attitude maximum accumulated error (roll and pitch)
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, i_range_xy, &g_self.i_range_xy)
/**
 * @brief Position P-gain (vertical z plane)
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, kp_z, &g_self.kp_z)
/**
 * @brief Position D-gain (vertical z plane)
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, kd_z, &g_self.kd_z)
/**
 * @brief Position I-gain (vertical z plane)
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, ki_z, &g_self.ki_z)
/**
 * @brief Position maximum accumulated error (vertical z plane)
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, i_range_z, &g_self.i_range_z)
/**
 * @brief total mass [kg]
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, mass, &g_self.mass)
/**
 * @brief Force to PWM stretch factor
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, massThrust, &g_self.massThrust)
/**
 * @brief Attitude P-gain (roll and pitch)
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, kR_xy, &g_self.kR_xy)
/**
 * @brief Attitude P-gain (yaw)
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, kR_z, &g_self.kR_z)
/**
 * @brief Attitude D-gain (roll and pitch)
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, kw_xy, &g_self.kw_xy)
/**
 * @brief Attitude D-gain (yaw)
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, kw_z, &g_self.kw_z)
/**
 * @brief Attitude I-gain (roll and pitch)
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, ki_m_xy, &g_self.ki_m_xy)
/**
 * @brief Attitude I-gain (yaw)
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, ki_m_z, &g_self.ki_m_z)
/**
 * @brief Angular velocity D-Gain (roll and pitch)
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, kd_omega_rp, &g_self.kd_omega_rp)
/**
 * @brief Attitude maximum accumulated error (roll and pitch)
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, i_range_m_xy, &g_self.i_range_m_xy)
/**
 * @brief Attitude maximum accumulated error (yaw)
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, i_range_m_z, &g_self.i_range_m_z)
PARAM_GROUP_STOP(ctrlMel)

/**
 * Logging variables for the command and reference signals for the
 * Mellinger controller
 */
LOG_GROUP_START(ctrlMel)
LOG_ADD(LOG_FLOAT, cmd_thrust, &g_self.cmd_thrust)
LOG_ADD(LOG_FLOAT, cmd_roll, &g_self.cmd_roll)
LOG_ADD(LOG_FLOAT, cmd_pitch, &g_self.cmd_pitch)
LOG_ADD(LOG_FLOAT, cmd_yaw, &g_self.cmd_yaw)
LOG_ADD(LOG_FLOAT, r_roll, &g_self.r_roll)
LOG_ADD(LOG_FLOAT, r_pitch, &g_self.r_pitch)
LOG_ADD(LOG_FLOAT, r_yaw, &g_self.r_yaw)
LOG_ADD(LOG_FLOAT, accelz, &g_self.accelz)
LOG_ADD(LOG_FLOAT, zdx, &g_self.z_axis_desired.x)
LOG_ADD(LOG_FLOAT, zdy, &g_self.z_axis_desired.y)
LOG_ADD(LOG_FLOAT, zdz, &g_self.z_axis_desired.z)
LOG_ADD(LOG_FLOAT, i_err_x, &g_self.i_error_x)
LOG_ADD(LOG_FLOAT, i_err_y, &g_self.i_error_y)
LOG_ADD(LOG_FLOAT, i_err_z, &g_self.i_error_z)
LOG_GROUP_STOP(ctrlMel)
