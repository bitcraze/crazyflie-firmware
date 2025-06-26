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
  .m = CF_MASS, // kg
  .J = {16.571710e-6, 16.655602e-6, 29.261652e-6}, // kg m^2

  .kx = 7.0,
  .kv = 4.0,
  .ki = 1.0,
  .c1 = 3.6,
  .sigma = 1.0,

  .kR = 0.007,
  .kW = 0.002,
  .kI = 0.0005,
  .c2 = 0.8,

  // Enable this for tracking aggressive maneuvers
  // 0: W_d and W_d_dot are set to zero
  // Nonzero: W_d and W_d_dot are calculated using an euler approximation
  .track_attitude_rate = 1,
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

// Compute the matrix logarithm of a rotation matrix
static inline struct mat33 mlog(struct mat33 R) {
	float acosinput = (R.m[0][0] + R.m[1][1] + R.m[2][2] - 1.0f) / 2.0f;
	if (acosinput >= 1.0f) {
		return mzero();
	} else if (acosinput <= -1.0f) {
		struct vec omg;
		if (!(fabsf(1.0f + R.m[2][2]) < 1e-6f)) {
			omg = vdiv(mkvec(R.m[0][2], R.m[1][2], 1.0f + R.m[2][2]), sqrtf(2.0f * (1.0f + R.m[2][2])));
		} else if (!(fabsf(1.0f + R.m[1][1]) < 1e-6f)) {
			omg = vdiv(mkvec(R.m[0][1], 1.0f + R.m[1][1], R.m[2][1]), sqrtf(2.0f * (1.0f + R.m[1][1])));
		} else {
			omg = vdiv(mkvec(1.0f + R.m[0][0], R.m[1][0], R.m[2][0]), sqrtf(2.0f * (1.0f + R.m[0][0])));
		}
		return mcrossmat(vscl(M_PI_F, omg));
	} else {
		float theta = acosf(acosinput);
		return mscl(theta / 2.0f / sinf(theta), msub(R, mtranspose(R)));
	}
}

static inline struct vec averagingFilter(struct vec* arr, int size) {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  for (int i = 0; i < size; i++) {
    x += arr[i].x;
    y += arr[i].y;
    z += arr[i].z;
  }
  x /= size;
  y /= size;
  z /= size;

  return mkvec(x, y, z);
}

void controllerLeeReset(controllerLee_t* self) {
  self->ei = vzero();
  self->eI = vzero();

  for (int i = 0; i < FILTER_SIZE; i++) {
    self->W_d_raw[i] = vzero();
    self->W_d_dot_raw[i] = vzero();
  }
}

void controllerLeeInit(controllerLee_t* self) {
  // copy default values (bindings), or NOP (firmware)
  *self = g_self;

  self->f = 0;
  self->M = vzero();
  
  self->ex = vzero();
  self->ev = vzero();

  self->eR = vzero();
  self->eW = vzero();

  self->R_d_prev = mcolumns(vrepeat(NAN), vrepeat(NAN), vrepeat(NAN));
  self->W_d_prev = vrepeat(NAN);

  self->W_d = vzero();
  self->W_d_dot = vzero();

  controllerLeeReset(self);
}

bool controllerLeeTest(controllerLee_t* self) {
  return true;
}

void controllerLee(controllerLee_t* self, control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick) {
  if (!RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    return;
  }
  
  float dt = (float)(1.0f/ATTITUDE_RATE);

  // States
  struct vec x = mkvec(state->position.x, state->position.y, state->position.z);
  struct vec v = mkvec(state->velocity.x, state->velocity.y, state->velocity.z);
  struct quat q = mkquat(state->attitudeQuaternion.x, state->attitudeQuaternion.y, state->attitudeQuaternion.z, state->attitudeQuaternion.w);
  struct mat33 R = quat2rotmat(q);
  struct vec W = mkvec(radians(sensors->gyro.x), radians(sensors->gyro.y), radians(sensors->gyro.z));

  float desiredYaw = 0;
  if (setpoint->mode.yaw == modeVelocity) {
    desiredYaw = radians(state->attitude.yaw + setpoint->attitudeRate.yaw * dt);
  } else if (setpoint->mode.yaw == modeAbs) {
    desiredYaw = radians(setpoint->attitude.yaw);
  } else if (setpoint->mode.quat == modeAbs) {
    struct quat setpoint_quat = mkquat(setpoint->attitudeQuaternion.x, setpoint->attitudeQuaternion.y, setpoint->attitudeQuaternion.z, setpoint->attitudeQuaternion.w);
    desiredYaw = quat2rpy(setpoint_quat).z;
  }
  
  // Calculate f and R_d
  struct mat33 R_d;

  // Position setpoint ([2] Sec. IV)
  if (setpoint->mode.x == modeAbs && setpoint->mode.y == modeAbs && setpoint->mode.z == modeAbs) {
    struct vec x_d = mkvec(setpoint->position.x, setpoint->position.y, setpoint->position.z);
    struct vec v_d = mkvec(setpoint->velocity.x, setpoint->velocity.y, setpoint->velocity.z);
    struct vec a_d = mkvec(setpoint->acceleration.x, setpoint->acceleration.y, setpoint->acceleration.z);
    struct vec b1_d = mkvec(cosf(desiredYaw), sinf(desiredYaw), 0);

    self->ex = vsub(x, x_d);
    self->ev = vsub(v, v_d);
    self->ei = vadd(self->ei, vscl(dt, vadd(self->ev, vscl(self->c1, self->ex))));
    self->ei = vclampscl(self->ei, -self->sigma, self->sigma);
    
    struct vec F_d = vscl(self->m, vadd(vadd4(
      vscl(-self->kx, self->ex),
      vscl(-self->kv, self->ev),
      vscl(-self->ki, self->ei),
      a_d),
      vscl(GRAVITY_MAGNITUDE, vbasis(2))));
    self->f = vdot(F_d, mvmul(R, vbasis(2)));
    
    if (self->f < 0.01f) {
      controllerLeeReset(self);
    }

    struct vec b3_d = vnormalize(F_d);
    struct vec b2_d = vnormalize(vcross(b3_d, b1_d));
    R_d = mcolumns(vcross(b2_d, b3_d), b2_d, b3_d);

  // Velocity setpoint ([1] Sec. VI)
  } else if (setpoint->mode.x == modeVelocity && setpoint->mode.y == modeVelocity && setpoint->mode.z == modeVelocity) {
    struct vec v_d = mkvec(setpoint->velocity.x, setpoint->velocity.y, setpoint->velocity.z);
    struct vec a_d = mkvec(setpoint->acceleration.x, setpoint->acceleration.y, setpoint->acceleration.z);
    struct vec b1_d = mkvec(cosf(desiredYaw), sinf(desiredYaw), 0);

    self->ev = vsub(v, v_d);

    struct vec F_d = vscl(self->m, vadd3(
      vscl(-self->kv, self->ev),
      a_d,
      vscl(GRAVITY_MAGNITUDE, vbasis(2))));
    self->f = vdot(F_d, mvmul(R, vbasis(2)));

    if (self->f < 0.01f) {
      controllerLeeReset(self);
    }

    struct vec b3_d = vnormalize(F_d);
    struct vec b2_d = vnormalize(vcross(b3_d, b1_d));
    R_d = mcolumns(vcross(b2_d, b3_d), b2_d, b3_d);

  } else {
    // zDistance setpoint ([1] Sec. IV)
    if (setpoint->mode.z == modeAbs) {
      float x3 = state->position.z;
      float x3_d = setpoint->position.z;

      float v3 = state->velocity.z;
      float v3_d = setpoint->velocity.z;

      float a3_d = setpoint->acceleration.z;
      
      self->f = self->m*(-self->kx*(x3 - x3_d) - self->kv*(v3 - v3_d) + a3_d + GRAVITY_MAGNITUDE) / R.m[2][2];

    // altHold setpoint (adapted from [1] Sec. IV)
    } else if (setpoint->mode.z == modeVelocity) {
      float v3 = state->velocity.z;
      float v3_d = setpoint->velocity.z;

      float a3_d = setpoint->acceleration.z;
      
      self->f = self->m*(-self->kv*(v3 - v3_d) + a3_d + GRAVITY_MAGNITUDE) / R.m[2][2];

    // Manual setpoint
    } else {
      if (setpoint->mode.z == modeDisable && setpoint->thrust < 1000) {
        control->controlMode = controlModeForceTorque;
        control->thrustSi  = 0;
        control->torque[0] = 0;
        control->torque[1] = 0;
        control->torque[2] = 0;
        controllerLeeReset(self);
        return;
      }
      
      const float max_thrust = powerDistributionGetMaxThrust(); // N
      self->f = setpoint->thrust / UINT16_MAX * max_thrust;
    }

    R_d = quat2rotmat(rpy2quat(mkvec(
      radians(setpoint->attitude.roll),
      -radians(setpoint->attitude.pitch), // This is in the legacy coordinate system where pitch is inverted
      desiredYaw)));
  }

  // Calculate M
  // Attitude control ([2] Sec. III)
  if (vneq(mcolumn(self->R_d_prev, 0), vrepeat(NAN)) && vneq(mcolumn(self->R_d_prev, 1), vrepeat(NAN)) && vneq(mcolumn(self->R_d_prev, 2), vrepeat(NAN))) {
    if (self->track_attitude_rate) {
      // Update W_d_raw buffer
      for (int i = 0; i < FILTER_SIZE - 1; i++) {
        self->W_d_raw[i] = self->W_d_raw[i + 1];
      }
      self->W_d_raw[FILTER_SIZE - 1] = vdiv(mvee(mlog(mmul(mtranspose(self->R_d_prev), R_d))), dt);
      self->W_d = averagingFilter(self->W_d_raw, FILTER_SIZE);
    } else {
      self->W_d = vzero();
    }
    
    // If the attitude rate is in the setpoint, use the setpoint values instead of approximations
    if (setpoint->mode.roll == modeVelocity) {
      self->W_d.x = radians(setpoint->attitudeRate.roll);
    }
    if (setpoint->mode.pitch == modeVelocity) {
      self->W_d.y = radians(setpoint->attitudeRate.pitch);
    }
    if (setpoint->mode.yaw == modeVelocity) {
      self->W_d.z = radians(setpoint->attitudeRate.yaw);
    }

    if (vneq(self->W_d_prev, vrepeat(NAN))) {
      if (self->track_attitude_rate) {
        // Update W_d_dot_raw buffer
        for (int i = 0; i < FILTER_SIZE - 1; i++) {
          self->W_d_dot_raw[i] = self->W_d_dot_raw[i + 1];
        }
        self->W_d_dot_raw[FILTER_SIZE - 1] = vdiv(vsub(self->W_d, self->W_d_prev), dt);
        self->W_d_dot = averagingFilter(self->W_d_dot_raw, FILTER_SIZE);
      } else {
        self->W_d_dot = vzero();
      }

      self->eR = vscl(0.5f, mvee(msub(
        mmul(mtranspose(R_d), R),
        mmul(mtranspose(R), R_d))));
      self->eW = vsub(W, mvmul(mtranspose(R), mvmul(R_d, self->W_d)));
      self->eI = vadd(self->eI, vscl(dt, vadd(self->eW, vscl(self->c2, self->eR))));

      self->M = vadd(vadd4(
        vscl(-self->kR, self->eR),
        vscl(-self->kW, self->eW),
        vscl(-self->kI, self->eI),
        vcross(mvmul(mtranspose(R), mvmul(R_d, self->W_d)), veltmul(self->J, mvmul(mtranspose(R), mvmul(R_d, self->W_d))))),
        veltmul(self->J, mvmul(mtranspose(R), mvmul(R_d, self->W_d_dot))));
    }
    self->W_d_prev = self->W_d;
  }
  self->R_d_prev = R_d;

  control->controlMode = controlModeForceTorque;
  control->thrustSi = self->f;
  control->torque[0] = self->M.x;
  control->torque[1] = self->M.y;
  control->torque[2] = self->M.z;
}

#ifdef CRAZYFLIE_FW

#include "param.h"
#include "log.h"

void controllerLeeFirmwareInit(void) {
  controllerLeeInit(&g_self);
}

bool controllerLeeFirmwareTest(void) {
  return true;
}

void controllerLeeFirmware(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick) {
  controllerLee(&g_self, control, setpoint, sensors, state, tick);
}

PARAM_GROUP_START(ctrlLee)

PARAM_ADD(PARAM_FLOAT, m, &g_self.m)

PARAM_ADD(PARAM_FLOAT, J1, &g_self.J.x)
PARAM_ADD(PARAM_FLOAT, J2, &g_self.J.y)
PARAM_ADD(PARAM_FLOAT, J3, &g_self.J.z)

PARAM_ADD(PARAM_FLOAT, kx, &g_self.kx)
PARAM_ADD(PARAM_FLOAT, kv, &g_self.kv)
PARAM_ADD(PARAM_FLOAT, ki, &g_self.ki)
PARAM_ADD(PARAM_FLOAT, c1, &g_self.c1)
PARAM_ADD(PARAM_FLOAT, sigma, &g_self.sigma)

PARAM_ADD(PARAM_FLOAT, kR, &g_self.kR)
PARAM_ADD(PARAM_FLOAT, kW, &g_self.kW)
PARAM_ADD(PARAM_FLOAT, kI, &g_self.kI)
PARAM_ADD(PARAM_FLOAT, c2, &g_self.c2)

PARAM_ADD(PARAM_UINT8, track_att_rate, &g_self.track_attitude_rate)

PARAM_GROUP_STOP(ctrlLee)


LOG_GROUP_START(ctrlLee)

// Wrench
LOG_ADD(LOG_FLOAT, f, &g_self.f)
LOG_ADD(LOG_FLOAT, M1, &g_self.M.x)
LOG_ADD(LOG_FLOAT, M2, &g_self.M.y)
LOG_ADD(LOG_FLOAT, M3, &g_self.M.z)

// Errors
LOG_ADD(LOG_FLOAT, ex1, &g_self.ex.x)
LOG_ADD(LOG_FLOAT, ex2, &g_self.ex.y)
LOG_ADD(LOG_FLOAT, ex3, &g_self.ex.z)

LOG_ADD(LOG_FLOAT, ev1, &g_self.ev.x)
LOG_ADD(LOG_FLOAT, ev2, &g_self.ev.y)
LOG_ADD(LOG_FLOAT, ev3, &g_self.ev.z)

LOG_ADD(LOG_FLOAT, eR1, &g_self.eR.x)
LOG_ADD(LOG_FLOAT, eR2, &g_self.eR.y)
LOG_ADD(LOG_FLOAT, eR3, &g_self.eR.z)

LOG_ADD(LOG_FLOAT, eW1, &g_self.eW.x)
LOG_ADD(LOG_FLOAT, eW2, &g_self.eW.y)
LOG_ADD(LOG_FLOAT, eW3, &g_self.eW.z)

LOG_ADD(LOG_FLOAT, eI1, &g_self.eI.x)
LOG_ADD(LOG_FLOAT, eI2, &g_self.eI.y)
LOG_ADD(LOG_FLOAT, eI3, &g_self.eI.z)

// Desired attitude rate
LOG_ADD(LOG_FLOAT, W_d1, &g_self.W_d.x)
LOG_ADD(LOG_FLOAT, W_d2, &g_self.W_d.y)
LOG_ADD(LOG_FLOAT, W_d3, &g_self.W_d.z)

LOG_ADD(LOG_FLOAT, W_d_dot1, &g_self.W_d_dot.x)
LOG_ADD(LOG_FLOAT, W_d_dot2, &g_self.W_d_dot.y)
LOG_ADD(LOG_FLOAT, W_d_dot3, &g_self.W_d_dot.z)

LOG_GROUP_STOP(ctrlLee)

#endif // CRAZYFLIE_FW defined
