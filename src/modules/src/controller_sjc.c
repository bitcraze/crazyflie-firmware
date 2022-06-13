/*
The MIT License (MIT)

Copyright (c) 2019 Wolfgang Hoenig

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
This controller is based on the following publication (section 7):

Daniel Morgan, Giri P Subramanian, Soon-Jo Chung, Fred Y Hadaegh
Swarm assignment and trajectory optimization using variable-swarm, distributed auction assignment and sequential convex programming 
IJRR 2016

Notes:
  * There is a typo in the paper, in eq 71: It should be -K (omega - omega_r) (i.e., no dot)
  * We make the following changes:
    1. The first term of the control law (J omega_r_dot) is treated as a damping term on omega_r
       to capture higher-order effects such as motor delays. Thus, we allow to tune this J (the J
       in the second part is the real inertia matrix)
    2. We use the current yaw angle instead of the desired yaw angle to compute desired roll/pitch angles
    3. We add a D-term on omega_r - omega to account for motor delays
  * This implementation is inspired by the implementation of the controller by 
    Giri P Subramanian for CF 1.0
  * Runtime attitude controller: 6us
  * Note that a quaternion-based version is too complex for on-board execution if an analytical model
    of Z^-1 and Z_dot^-1 is to be used
*/

#include <math.h>

#include "param.h"
#include "log.h"
#include "math3d.h"
#include "controller_sjc.h"
#include "usec_time.h"
// #include "debug.h"

#define GRAVITY_MAGNITUDE (9.81f)

static uint32_t ticks;

static uint8_t trigReset;

static float g_vehicleMass = 0.032; // TODO: should be CF global for other modules

// Attitude P on omega
static struct vec K = {0.0005, 0.0005, 0.001};
// static float K_limit = 8.7; // ~500 deg/s

// Attitude P on angle
static struct vec lambda = {10, 10, 10};
// static float lambda_limit = M_PI / 2.0; // 90 deg

// Attitude I on omega
static struct vec Katt_I = {0.00025, 0.00025, 0.0005};
static float Katt_I_limit = 2;
static struct vec i_error_att;

// D on omega_dot
static struct vec omega_error_prev;
static struct vec omega_error_dot;
static struct vec Katt_Dw = {0, 0, 0};

// Position gains
static struct vec Kpos_P = {12, 12, 12};
static float Kpos_P_limit = 100;
static struct vec Kpos_D = {8, 8, 8};
static float Kpos_D_limit = 100;
static struct vec Kpos_I = {8, 8, 8};
static float Kpos_I_limit = 2;
static struct vec i_error_pos;

static float Kpos_I_decay = 1.0f;

static struct vec Kpos_A = {0, 0, 0};
static float Kpos_A_limit = 100;

// delay compensation idea #1
static float T_d_last = 0;
static float T_d_lambda = 0.0; // 0 means disabled

// Inertia matrix (diagonal matrix), see
// System Identification of the Crazyflie 2.0 Nano Quadrocopter
// BA theses, Julian Foerster, ETHZ
// https://polybox.ethz.ch/index.php/s/20dde63ee00ffe7085964393a55a91c7
static struct vec J = {16.571710e-6, 16.655602e-6, 29.261652e-6}; // kg m^2

// We treat the first part of the control law (J omega_r_dot) as damping on omega_r
// by allowing to change J
static struct vec Jtune = {48e-6, 48e-6, 29e-6};

// logging variables
static struct vec u;
static struct vec qr;
static struct vec q;
static struct vec omega;
static struct vec omega_r;

static struct vec qrp;
static struct vec qr_dot;

static inline struct vec vclampscl(struct vec value, float min, float max) {
  return mkvec(
    clamp(value.x, min, max),
    clamp(value.y, min, max),
    clamp(value.z, min, max));
}

// subtract d, b and c from a.
static inline struct vec vsub3(struct vec a, struct vec b, struct vec c, struct vec d) {
  return vadd4(a, vneg(b), vneg(c), vneg(d));
}

static inline struct vec vsub4(struct vec a, struct vec b, struct vec c, struct vec d, struct vec e) {
  return vadd(vadd4(a, vneg(b), vneg(c), vneg(d)), vneg(e));
}

void controllerSJCReset(void)
{
  i_error_pos = vzero();
  i_error_att = vzero();
  omega_error_prev = vzero();
  omega_error_dot = vzero();
}

void controllerSJCInit(void)
{
  controllerSJCReset();
}

bool controllerSJCTest(void)
{
  return true;
}

void controllerSJC(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
  if (!RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    return;
  }

  if (trigReset == 1) {
    controllerSJCReset();
  } else if (trigReset == 2) {
    i_error_pos.z = 0;
  }
  trigReset = 0;

  float dt = (float)(1.0f/ATTITUDE_RATE);

  // Address inconsistency in firmware where we need to compute our own desired yaw angle
  // Rate-controlled YAW is moving YAW angle setpoint
  float desiredYaw = 0; //rad
  if (setpoint->mode.yaw == modeVelocity) {
    desiredYaw = radians(state->attitude.yaw + setpoint->attitudeRate.yaw * dt);
  } else if (setpoint->mode.yaw == modeAbs) {
    desiredYaw = radians(setpoint->attitude.yaw);
  } else if (setpoint->mode.quat == modeAbs) {
    struct quat setpoint_quat = mkquat(setpoint->attitudeQuaternion.x, setpoint->attitudeQuaternion.y, setpoint->attitudeQuaternion.z, setpoint->attitudeQuaternion.w);
    struct vec rpy = quat2rpy(setpoint_quat);
    desiredYaw = rpy.z;
  }

  // qr: Desired/reference angles in rad
  // struct vec qr;

  // Position controller
  if (   setpoint->mode.x == modeAbs
      || setpoint->mode.y == modeAbs
      || setpoint->mode.z == modeAbs) {

    if (RATE_DO_EXECUTE(POSITION_RATE, tick)) {
      struct vec pos_d = mkvec(setpoint->position.x, setpoint->position.y, setpoint->position.z);
      struct vec vel_d = mkvec(setpoint->velocity.x, setpoint->velocity.y, setpoint->velocity.z);
      struct vec acc_d = mkvec(setpoint->acceleration.x, setpoint->acceleration.y, setpoint->acceleration.z + GRAVITY_MAGNITUDE);
      struct vec statePos = mkvec(state->position.x, state->position.y, state->position.z);
      struct vec stateVel = mkvec(state->velocity.x, state->velocity.y, state->velocity.z);
      struct vec stateAcc = vscl(9.81, mkvec(state->acc.x, state->acc.y, state->acc.z + 1)); // Gs -> m/s^2

      // errors
      struct vec pos_e = vclampscl(vsub(pos_d, statePos), -Kpos_P_limit, Kpos_P_limit);
      struct vec vel_e = vclampscl(vsub(vel_d, stateVel), -Kpos_D_limit, Kpos_D_limit);
      // This is for delay compensation (idea #2): feedback term on acceleration
      struct vec acc_e = vclampscl(vsub(acc_d, stateAcc), -Kpos_A_limit, Kpos_A_limit);
      i_error_pos = vclampscl(vadd(vscl(Kpos_I_decay, i_error_pos), vscl(dt, pos_e)), -Kpos_I_limit, Kpos_I_limit);

      struct vec F_d = vscl(g_vehicleMass, vadd5(
        acc_d,
        veltmul(Kpos_A, acc_e),
        veltmul(Kpos_D, vel_e),
        veltmul(Kpos_P, pos_e),
        veltmul(Kpos_I, i_error_pos)));

      control->thrustSI = vmag(F_d);

      // This is for delay compensation (idea #1)
      // replace by T_d by T = T_d + T_d dot / lambda
      float T_d_dot = (control->thrustSI - T_d_last) * POSITION_RATE;
      if (T_d_lambda > 0) {
        control->thrustSI = control->thrustSI + T_d_dot / T_d_lambda;
      }
      T_d_last = control->thrustSI;

      // Reset the accumulated error while on the ground
      if (control->thrustSI < 0.01f) {
        controllerSJCReset();
      }

      // Use current yaw instead of desired yaw for roll/pitch
      float yaw = radians(state->attitude.yaw);
      qr = mkvec(
        asinf((F_d.x * sinf(yaw) - F_d.y * cosf(yaw)) / control->thrustSI),
        atanf((F_d.x * cosf(yaw) + F_d.y * sinf(yaw)) / F_d.z),
        desiredYaw);
    }
  } else {
    if (setpoint->mode.z == modeDisable) {
      if (setpoint->thrust < 1000) {
          control->controlMode = controlModeForceTorque;
          control->thrustSI = 0;
          control->torque[0] = 0;
          control->torque[1] = 0;
          control->torque[2] = 0;
          controllerSJCReset();
          return;
      }
    }
    // On CF2, thrust is mapped 65536 <==> 4 * 12 grams
    const float max_thrust = 4 * 12.0 / 1000.0 * 9.81; // N
    control->thrustSI = setpoint->thrust / 65536.0f * max_thrust;

    qr = mkvec(
      radians(setpoint->attitude.roll),
      -radians(setpoint->attitude.pitch), // This is in the legacy coordinate system where pitch is inverted
      radians(desiredYaw));
  }

  // Attitude controller
  // DEBUG_PRINT("q %d %d %d %d\n", (int)(qr.x * 1000), (int)(qr.y * 1000), (int)(qr.z * 1000), (int)(control->thrustSI * 1000));

  // q: vector of three-dimensional attitude representation
  //    here: Euler angles in rad
  q = mkvec(
    radians(state->attitude.roll),
    radians(-state->attitude.pitch), // This is in the legacy coordinate system where pitch is inverted
    radians(state->attitude.yaw));

  // omega: angular velocity in rad/s
  omega = mkvec(
    radians(sensors->gyro.x),
    radians(sensors->gyro.y),
    radians(sensors->gyro.z));

  // q_dot = Z(q)*omega
  // For euler angles Z(q) is defined in Giri P Subramanian MS thesis, equation 8
  //        [1  sin(r)tan(p)  cos(r)tan(p)]
  // Z(q) = [0  cos(r)        -sin(r)     ]
  //        [0  sin(r)sec(p)  cos(r)sec(p)]
  // TODO: add check for division by zero?
  struct vec q_dot = mkvec(
    omega.x + sinf(q.x)*tanf(q.y)*omega.y + cosf(q.x)*tanf(q.y)*omega.z,
              cosf(q.x)          *omega.y - sinf(q.x)          *omega.z,
              sinf(q.x)/cosf(q.y)*omega.y + cosf(q.x)/cosf(q.y)*omega.z);

  // qr_dot desired/reference angular velocity in rad/s
  qr_dot = mkvec(
    radians(setpoint->attitudeRate.roll),
    radians(setpoint->attitudeRate.pitch),
    radians(setpoint->attitudeRate.yaw));

  // qr_ddot desired/reference angular acceleration in rad/s^2
  struct vec qr_ddot = vzero();

  // omega_r: desired/reference angular velocity in rad/s
  // omega_r = Z(q)^-1 * (qr_dot + lambda (qr - q)) = Z(q)^-1 * qrp
  // For euler angles Z(q)^-1 is:
  //           [1  0       -sin(p)     ]
  // Z(q)^-1 = [0  cos(r)  cos(p)sin(r)]
  //           [0 -sin(r)  cos(p)cos(r)]
  qrp = vadd(qr_dot, veltmul(lambda, vsub(qr, q)));
  omega_r = mkvec(
    qrp.x +                 -sinf(q.y)           *qrp.z,
            cosf(q.x)*qrp.y + cosf(q.y)*sinf(q.x)*qrp.z,
          - sinf(q.x)*qrp.y + cosf(q.y)*cosf(q.x)*qrp.z);

  // omegar_dot =   Z(q)^-1_dot*qr_dot         + Z(q)^-1*qr_ddot
  //              + Z(q)^-1_dot*lambda(qr - q) + Z(q)^-1*lambda*(qr_dot - q_dot)
  //            =   Z(q)^-1_dot (qr_dot + lambda (qr - q))
  //              + Z(q)^-1 (qr_ddot + lambda (qr_dot - q_dot))
  //            =   Z(q)^-1_dot qrp + Z(q)^-1 qrp_dot
  // For euler angles Z(q)^-1_dot is:
  //               [0  0            -cos(p)p_dot                        ]
  // Z(q)^-1_dot = [0 -sin(r)r_dot  -sin(p)sin(r)p_dot+cos(p)cos(r)r_dot]
  //               [0 -cos(r)r_dot  -cos(r)sin(p)p_dot-cos(p)sin(r)r_dot]
  struct vec qrp_dot = vadd(qr_ddot, veltmul(lambda, vsub(qr_dot, q_dot)));
  struct vec omega_r_dot = mkvec(
                             -cosf(q.y)             *q_dot.y                             *qrp.z + qrp_dot.x                     -sinf(q.y)           *qrp_dot.z,
    -sinf(q.x)*q_dot.x*qrp.y + (-sinf(q.y)*sinf(q.x)*q_dot.y+cosf(q.y)*cosf(q.x)*q_dot.x)*qrp.z +           cosf(q.x)*qrp_dot.y + cosf(q.y)*sinf(q.x)*qrp_dot.z,
    -cosf(q.x)*q_dot.x*qrp.y + (-cosf(q.x)*sinf(q.y)*q_dot.y-cosf(q.y)*sinf(q.x)*q_dot.x)*qrp.z +          -sinf(q.x)*qrp_dot.y + cosf(q.y)*cosf(q.x)*qrp_dot.z);

  // Integral part on omega (as in paper)
  struct vec omega_error = vsub(omega, omega_r);
  i_error_att = vclampscl(vadd(i_error_att, vscl(dt, omega_error)), -Katt_I_limit, Katt_I_limit);

  // D part on omega error (to deal with motor delays)
  omega_error_dot = vdiv(vsub(omega_error, omega_error_prev), dt);
  // filter estimate
  omega_error_prev = omega_error;

  // // Integral part on angle
  // struct vec q_error = vsub(qr, q);
  // i_error_att = vclampscl(vadd(i_error_att, vscl(dt, q_error)), -Katt_I_limit, Katt_I_limit);

  // compute moments (note there is a typo on the paper in equation 71)
  // u = J omega_r_dot - (J omega) x omega_r - K(omega - omega_r) - Katt_I \integral(w-w_r, dt)
  u = vsub4(
    veltmul(Jtune, omega_r_dot),
    vcross(veltmul(J, omega), omega_r),
    veltmul(K, omega_error),
    veltmul(Katt_I, i_error_att),
    veltmul(Katt_Dw, omega_error_dot));

  control->controlMode = controlModeForceTorque;
  control->torque[0] = u.x;
  control->torque[1] = u.y;
  control->torque[2] = u.z;
}

PARAM_GROUP_START(ctrlSJC)
// Attitude P
PARAM_ADD(PARAM_FLOAT, Katt_Px, &lambda.x)
PARAM_ADD(PARAM_FLOAT, Katt_Py, &lambda.y)
PARAM_ADD(PARAM_FLOAT, Katt_Pz, &lambda.z)
// Attitude D
PARAM_ADD(PARAM_FLOAT, Katt_Dx, &K.x)
PARAM_ADD(PARAM_FLOAT, Katt_Dy, &K.y)
PARAM_ADD(PARAM_FLOAT, Katt_Dz, &K.z)
// Attitude I
PARAM_ADD(PARAM_FLOAT, Katt_Ix, &Katt_I.x)
PARAM_ADD(PARAM_FLOAT, Katt_Iy, &Katt_I.y)
PARAM_ADD(PARAM_FLOAT, Katt_Iz, &Katt_I.z)
PARAM_ADD(PARAM_FLOAT, Katt_I_limit, &Katt_I_limit)
// Attitude Dw
PARAM_ADD(PARAM_FLOAT, Katt_Dwx, &Katt_Dw.x)
PARAM_ADD(PARAM_FLOAT, Katt_Dwy, &Katt_Dw.y)
PARAM_ADD(PARAM_FLOAT, Katt_Dwz, &Katt_Dw.z)
// Position P
PARAM_ADD(PARAM_FLOAT, Kpos_Px, &Kpos_P.x)
PARAM_ADD(PARAM_FLOAT, Kpos_Py, &Kpos_P.y)
PARAM_ADD(PARAM_FLOAT, Kpos_Pz, &Kpos_P.z)
PARAM_ADD(PARAM_FLOAT, Kpos_P_limit, &Kpos_P_limit)
// Position D
PARAM_ADD(PARAM_FLOAT, Kpos_Dx, &Kpos_D.x)
PARAM_ADD(PARAM_FLOAT, Kpos_Dy, &Kpos_D.y)
PARAM_ADD(PARAM_FLOAT, Kpos_Dz, &Kpos_D.z)
PARAM_ADD(PARAM_FLOAT, Kpos_D_limit, &Kpos_D_limit)

// Position A
PARAM_ADD(PARAM_FLOAT, Kpos_Ax, &Kpos_A.x)
PARAM_ADD(PARAM_FLOAT, Kpos_Ay, &Kpos_A.y)
PARAM_ADD(PARAM_FLOAT, Kpos_Az, &Kpos_A.z)
PARAM_ADD(PARAM_FLOAT, Kpos_A_limit, &Kpos_A_limit)

// Position I
PARAM_ADD(PARAM_FLOAT, Kpos_Ix, &Kpos_I.x)
PARAM_ADD(PARAM_FLOAT, Kpos_Iy, &Kpos_I.y)
PARAM_ADD(PARAM_FLOAT, Kpos_Iz, &Kpos_I.z)
PARAM_ADD(PARAM_FLOAT, Kpos_I_limit, &Kpos_I_limit)
PARAM_ADD(PARAM_FLOAT, Kpos_I_decay, &Kpos_I_decay)

// Jtune
PARAM_ADD(PARAM_FLOAT, Jtune_x, &Jtune.x)
PARAM_ADD(PARAM_FLOAT, Jtune_y, &Jtune.y)
PARAM_ADD(PARAM_FLOAT, Jtune_z, &Jtune.z)

PARAM_ADD(PARAM_FLOAT, mass, &g_vehicleMass)

PARAM_ADD(PARAM_UINT8, trigReset, &trigReset)

// delay compensation (0 == disabled)
PARAM_ADD(PARAM_UINT8, T_d_lambda, &T_d_lambda)

PARAM_GROUP_STOP(ctrlSJC)

LOG_GROUP_START(ctrlSJC)
LOG_ADD(LOG_FLOAT, torquex, &u.x)
LOG_ADD(LOG_FLOAT, torquey, &u.y)
LOG_ADD(LOG_FLOAT, torquez, &u.z)
// current angles
LOG_ADD(LOG_FLOAT, qx, &q.x)
LOG_ADD(LOG_FLOAT, qy, &q.y)
LOG_ADD(LOG_FLOAT, qz, &q.z)
// desired angles
LOG_ADD(LOG_FLOAT, qrx, &qr.x)
LOG_ADD(LOG_FLOAT, qry, &qr.y)
LOG_ADD(LOG_FLOAT, qrz, &qr.z)

// errors
LOG_ADD(LOG_FLOAT, i_error_attx, &i_error_att.x)
LOG_ADD(LOG_FLOAT, i_error_atty, &i_error_att.y)
LOG_ADD(LOG_FLOAT, i_error_attz, &i_error_att.z)

LOG_ADD(LOG_FLOAT, i_error_posx, &i_error_pos.x)
LOG_ADD(LOG_FLOAT, i_error_posy, &i_error_pos.y)
LOG_ADD(LOG_FLOAT, i_error_posz, &i_error_pos.z)

// omega
LOG_ADD(LOG_FLOAT, omegax, &omega.x)
LOG_ADD(LOG_FLOAT, omegay, &omega.y)
LOG_ADD(LOG_FLOAT, omegaz, &omega.z)

// omega dot hat
LOG_ADD(LOG_FLOAT, domegax, &omega_error_dot.x)
LOG_ADD(LOG_FLOAT, domegay, &omega_error_dot.y)
LOG_ADD(LOG_FLOAT, domegaz, &omega_error_dot.z)

// omega_r
LOG_ADD(LOG_FLOAT, omegarx, &omega_r.x)
LOG_ADD(LOG_FLOAT, omegary, &omega_r.y)
LOG_ADD(LOG_FLOAT, omegarz, &omega_r.z)

LOG_ADD(LOG_FLOAT, qrpx, &qrp.x)
LOG_ADD(LOG_FLOAT, qrpz, &qrp.z)

LOG_ADD(LOG_FLOAT, qrdotz, &qr_dot.z)

LOG_ADD(LOG_UINT32, ticks, &ticks)

LOG_GROUP_STOP(ctrlSJC)