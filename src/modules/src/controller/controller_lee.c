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

/*
This controller is based on the following publications:

[1] Taeyoung Lee, Melvin Leok, and N. Harris McClamroch
Control of Complex Maneuvers for a Quadrotor UAV using Geometric Methods on SE(3)
CDC 2010, updated on arXiv 2011
https://arxiv.org/pdf/1003.2005

[2] Farhad Goodarzi, Daewon Lee, Taeyoung Lee
Geometric Nonlinear PID Control of a Quadrotor UAV on SE(3)
ECC 2013
https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=6669644


* Difference to Mellinger:
  * Different angular velocity error
  * Higher-order terms in attitude controller
*/

#include <math.h>
#include <string.h>

#include "math3d.h"
#include "controller_lee.h"
#include "physicalConstants.h"
#include "power_distribution.h"
#include "platform_defaults.h"

static controllerLee_t g_self = {
  .mass = CF_MASS,

  // Inertia matrix (diagonal matrix), see
  // System Identification of the Crazyflie 2.0 Nano Quadrocopter
  // BA theses, Julian Foerster, ETHZ
  // https://polybox.ethz.ch/index.php/s/20dde63ee00ffe7085964393a55a91c7
  .J = {16.571710e-6, 16.655602e-6, 29.261652e-6}, // kg m^2

  // Position PID
  .Kpos_P = {7.0, 7.0, 7.0}, // Kp in paper
  .Kpos_P_limit = 100,
  .Kpos_D = {4.0, 4.0, 4.0}, // Kv in paper
  .Kpos_D_limit = 100,
  .Kpos_I = {1.0, 1.0, 1.0}, // not in paper
  .Kpos_I_limit = 2,
  .c_pos_multiplier = 3.6, // c1 in [2], used to scale the integral error

  // Attitude PID
  .KR = {0.007, 0.007, 0.008},
  .Komega = {0.00115, 0.00115, 0.002},
  .KI = {0.0005, 0.0005, 0.0005},
  .c_att_multiplier = 0.8, // c2 in [2], used to scale the integral error
};

static inline struct vec vclampscl(struct vec value, float min, float max) {
  return mkvec(
    clamp(value.x, min, max),
    clamp(value.y, min, max),
    clamp(value.z, min, max));
}

static inline struct vec mvee(struct mat33 R) {
	return mkvec(R.m[2][1], R.m[0][2], R.m[1][0]);
}

void controllerLeeReset(controllerLee_t* self)
{
  self->i_error_pos = vzero();
  self->i_error_att = vzero();
}

void controllerLeeInit(controllerLee_t* self)
{
  // copy default values (bindings), or NOP (firmware)
  *self = g_self;

  controllerLeeReset(self);
}

bool controllerLeeTest(controllerLee_t* self)
{
  return true;
}

void controllerLee(controllerLee_t* self, control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{

  if (!RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    return;
  }

  // uint64_t startTime = usecTimestamp();

  // States
  struct vec statePos = mkvec(state->position.x, state->position.y, state->position.z); // position in the world frame (m)
  struct vec stateVel = mkvec(state->velocity.x, state->velocity.y, state->velocity.z); // velocity in the world frame (m/s)
  struct quat q = mkquat(state->attitudeQuaternion.x, state->attitudeQuaternion.y, state->attitudeQuaternion.z, state->attitudeQuaternion.w);
  self->rpy = quat2rpy(q);
  struct mat33 R = quat2rotmat(q); // rotation matrix from the world frame to the body frame
  self->omega = mkvec(radians(sensors->gyro.x), radians(sensors->gyro.y), radians(sensors->gyro.z)); // angular velocity in the body frame (rad/s)

  float dt = (float)(1.0f/ATTITUDE_RATE);
  // struct vec dessnap = vzero();
  // Address inconsistency in firmware where we need to compute our own desired yaw angle
  // Rate-controlled YAW is moving YAW angle setpoint
  float desiredYaw = 0; //rad
  if (setpoint->mode.yaw == modeVelocity) {
    desiredYaw = radians(state->attitude.yaw + setpoint->attitudeRate.yaw * dt);
  } else if (setpoint->mode.yaw == modeAbs) {
    desiredYaw = radians(setpoint->attitude.yaw);
  } else if (setpoint->mode.quat == modeAbs) {
    struct quat setpoint_quat = mkquat(setpoint->attitudeQuaternion.x, setpoint->attitudeQuaternion.y, setpoint->attitudeQuaternion.z, setpoint->attitudeQuaternion.w);
    self->rpy_des = quat2rpy(setpoint_quat);
    desiredYaw = self->rpy_des.z;
  }

  // Position controller
  if (   setpoint->mode.x == modeAbs
      || setpoint->mode.y == modeAbs
      || setpoint->mode.z == modeAbs) {
    struct vec pos_d = mkvec(setpoint->position.x, setpoint->position.y, setpoint->position.z);
    struct vec vel_d = mkvec(setpoint->velocity.x, setpoint->velocity.y, setpoint->velocity.z);
    struct vec acc_d = mkvec(setpoint->acceleration.x, setpoint->acceleration.y, setpoint->acceleration.z);

    // errors
    struct vec pos_e = vclampscl(vsub(pos_d, statePos), -self->Kpos_P_limit, self->Kpos_P_limit);
    struct vec vel_e = vclampscl(vsub(vel_d, stateVel), -self->Kpos_D_limit, self->Kpos_D_limit);
    self->i_error_pos = vadd(self->i_error_pos, vscl(dt, vadd(vel_e, vscl(self->c_pos_multiplier, pos_e))));
    self->i_error_pos = vclampscl(self->i_error_pos, -self->Kpos_I_limit, self->Kpos_I_limit);
    self->p_error = pos_e;
    self->v_error = vel_e;

    struct vec F_d = vscl(self->mass, vadd(vadd4(
      veltmul(self->Kpos_P, pos_e),
      veltmul(self->Kpos_D, vel_e),
      veltmul(self->Kpos_I, self->i_error_pos),
      acc_d),
      vscl(GRAVITY_MAGNITUDE, vbasis(2))));

    self->thrustSi = vdot(F_d , mvmul(R, vbasis(2)));

    // Reset the accumulated error while on the ground
    if (self->thrustSi < 0.01f) {
      controllerLeeReset(self);
    }

    // Compute Desired Rotation matrix
    struct vec b1_d = mkvec(cosf(desiredYaw), sinf(desiredYaw), 0);
    struct vec b3_d = self->thrustSi > 0 ? vnormalize(F_d) : vbasis(2);
    struct vec b2_d = vnormalize(vcross(b3_d, b1_d));

    self->R_des = mcolumns(vcross(b2_d, b3_d), b2_d, b3_d);

  } else {
    if (setpoint->mode.z == modeDisable) {
      if (setpoint->thrust < 1000) {
          control->controlMode = controlModeForceTorque;
          control->thrustSi  = 0;
          control->torque[0] = 0;
          control->torque[1] = 0;
          control->torque[2] = 0;
          controllerLeeReset(self);
          return;
      }
    }
    const float max_thrust = powerDistributionGetMaxThrust(); // N
    self->thrustSi = setpoint->thrust / UINT16_MAX * max_thrust;

    self->R_des = quat2rotmat(rpy2quat(mkvec(
        radians(setpoint->attitude.roll),
        -radians(setpoint->attitude.pitch), // This is in the legacy coordinate system where pitch is inverted
        desiredYaw)));
  }

  // Attitude controller

  // desired rotation [Rdes]
  self->rpy_des = quat2rpy(mat2quat(self->R_des));

  // rotation error
  struct vec eR = vscl(0.5f, mvee(msub(
    mmul(mtranspose(self->R_des), R),
    mmul(mtranspose(R), self->R_des))));

  // Compute desired omega (TODO, zero for now)
  struct vec omega_des = vzero();

  if (setpoint->mode.roll == modeVelocity) {
    omega_des.x = radians(setpoint->attitudeRate.roll);
  }
  if (setpoint->mode.pitch == modeVelocity) {
    omega_des.y = radians(setpoint->attitudeRate.pitch);
  }
  if (setpoint->mode.yaw == modeVelocity) {
    omega_des.z = radians(setpoint->attitudeRate.yaw);
  }

  self->omega_r = mvmul(mmul(mtranspose(R), self->R_des), omega_des);

  struct vec omega_error = vsub(self->omega, self->omega_r);
  
  // Integral part on angle
  self->i_error_att = vadd(self->i_error_att, vscl(dt, vadd(omega_error, vscl(self->c_att_multiplier, eR))));

  // compute moments
  self->torqueSi = vadd4(
    vneg(veltmul(self->KR, eR)),
    vneg(veltmul(self->Komega, omega_error)),
    vneg(veltmul(self->KI, self->i_error_att)),
    vcross(self->omega_r, veltmul(self->J, self->omega_r)));

  control->controlMode = controlModeForceTorque;
  control->thrustSi = self->thrustSi;
  control->torque[0] = self->torqueSi.x;
  control->torque[1] = self->torqueSi.y;
  control->torque[2] = self->torqueSi.z;

  // ticks = usecTimestamp() - startTime;
}

#ifdef CRAZYFLIE_FW

#include "param.h"
#include "log.h"

void controllerLeeFirmwareInit(void)
{
  controllerLeeInit(&g_self);
}

bool controllerLeeFirmwareTest(void)
{
  return true;
}

void controllerLeeFirmware(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
  controllerLee(&g_self, control, setpoint, sensors, state, tick);
}

PARAM_GROUP_START(ctrlLee)
PARAM_ADD(PARAM_FLOAT, KR_x, &g_self.KR.x)
PARAM_ADD(PARAM_FLOAT, KR_y, &g_self.KR.y)
PARAM_ADD(PARAM_FLOAT, KR_z, &g_self.KR.z)
// Attitude I
PARAM_ADD(PARAM_FLOAT, KI_x, &g_self.KI.x)
PARAM_ADD(PARAM_FLOAT, KI_y, &g_self.KI.y)
PARAM_ADD(PARAM_FLOAT, KI_z, &g_self.KI.z)
// Attitude D
PARAM_ADD(PARAM_FLOAT, Kw_x, &g_self.Komega.x)
PARAM_ADD(PARAM_FLOAT, Kw_y, &g_self.Komega.y)
PARAM_ADD(PARAM_FLOAT, Kw_z, &g_self.Komega.z)

// J
PARAM_ADD(PARAM_FLOAT, J_x, &g_self.J.x)
PARAM_ADD(PARAM_FLOAT, J_y, &g_self.J.y)
PARAM_ADD(PARAM_FLOAT, J_z, &g_self.J.z)

// Position P
PARAM_ADD(PARAM_FLOAT, Kpos_Px, &g_self.Kpos_P.x)
PARAM_ADD(PARAM_FLOAT, Kpos_Py, &g_self.Kpos_P.y)
PARAM_ADD(PARAM_FLOAT, Kpos_Pz, &g_self.Kpos_P.z)
PARAM_ADD(PARAM_FLOAT, Kpos_P_limit, &g_self.Kpos_P_limit)
// Position D
PARAM_ADD(PARAM_FLOAT, Kpos_Dx, &g_self.Kpos_D.x)
PARAM_ADD(PARAM_FLOAT, Kpos_Dy, &g_self.Kpos_D.y)
PARAM_ADD(PARAM_FLOAT, Kpos_Dz, &g_self.Kpos_D.z)
PARAM_ADD(PARAM_FLOAT, Kpos_D_limit, &g_self.Kpos_D_limit)
// Position I
PARAM_ADD(PARAM_FLOAT, Kpos_Ix, &g_self.Kpos_I.x)
PARAM_ADD(PARAM_FLOAT, Kpos_Iy, &g_self.Kpos_I.y)
PARAM_ADD(PARAM_FLOAT, Kpos_Iz, &g_self.Kpos_I.z)
PARAM_ADD(PARAM_FLOAT, Kpos_I_limit, &g_self.Kpos_I_limit)

PARAM_ADD(PARAM_FLOAT, mass, &g_self.mass)
PARAM_GROUP_STOP(ctrlLee)


LOG_GROUP_START(ctrlLee)

LOG_ADD(LOG_FLOAT, thrustSi, &g_self.thrustSi)
LOG_ADD(LOG_FLOAT, torquex, &g_self.torqueSi.x)
LOG_ADD(LOG_FLOAT, torquey, &g_self.torqueSi.y)
LOG_ADD(LOG_FLOAT, torquez, &g_self.torqueSi.z)

// current angles
LOG_ADD(LOG_FLOAT, rpyx, &g_self.rpy.x)
LOG_ADD(LOG_FLOAT, rpyy, &g_self.rpy.y)
LOG_ADD(LOG_FLOAT, rpyz, &g_self.rpy.z)

// desired angles
LOG_ADD(LOG_FLOAT, rpydx, &g_self.rpy_des.x)
LOG_ADD(LOG_FLOAT, rpydy, &g_self.rpy_des.y)
LOG_ADD(LOG_FLOAT, rpydz, &g_self.rpy_des.z)

// errors
LOG_ADD(LOG_FLOAT, error_posx, &g_self.p_error.x)
LOG_ADD(LOG_FLOAT, error_posy, &g_self.p_error.y)
LOG_ADD(LOG_FLOAT, error_posz, &g_self.p_error.z)

LOG_ADD(LOG_FLOAT, error_velx, &g_self.v_error.x)
LOG_ADD(LOG_FLOAT, error_vely, &g_self.v_error.y)
LOG_ADD(LOG_FLOAT, error_velz, &g_self.v_error.z)

// omega
LOG_ADD(LOG_FLOAT, omegax, &g_self.omega.x)
LOG_ADD(LOG_FLOAT, omegay, &g_self.omega.y)
LOG_ADD(LOG_FLOAT, omegaz, &g_self.omega.z)

// omega_r
LOG_ADD(LOG_FLOAT, omegarx, &g_self.omega_r.x)
LOG_ADD(LOG_FLOAT, omegary, &g_self.omega_r.y)
LOG_ADD(LOG_FLOAT, omegarz, &g_self.omega_r.z)

// LOG_ADD(LOG_UINT32, ticks, &ticks)

LOG_GROUP_STOP(ctrlLee)

#endif // CRAZYFLIE_FW defined
