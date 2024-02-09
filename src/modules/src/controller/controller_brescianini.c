/**
 * Authored by Michael Hamer (http://www.mikehamer.info), November 2016.
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
 * ============================================================================
 *
 * The controller implemented in this file is based on the paper:
 *
 * "Nonlinear Quadrocopter Attitude Control"
 * http://e-collection.library.ethz.ch/eserv/eth:7387/eth-7387-01.pdf
 *
 * Academic citation would be appreciated.
 *
 * BIBTEX ENTRIES:
      @ARTICLE{BrescianiniNonlinearController2013,
               title={Nonlinear quadrocopter attitude control},
               author={Brescianini, Dario and Hehn, Markus and D'Andrea, Raffaello},
               year={2013},
               publisher={ETH Zurich}}
 *
 * ============================================================================
 */

#include "controller_brescianini.h"
#include "log.h"
#include "param.h"
#include "num.h"
#include "math3d.h"
#include "physicalConstants.h"
#include "platform_defaults.h"

static struct mat33 CRAZYFLIE_INERTIA =
    {{{16.6e-6f, 0.83e-6f, 0.72e-6f},
      {0.83e-6f, 16.6e-6f, 1.8e-6f},
      {0.72e-6f, 1.8e-6f, 29.3e-6f}}};


// tau is a time constant, lower -> more aggressive control (weight on position error)
// zeta is a damping factor, higher -> more damping (weight on velocity error)

static float tau_xy = 0.3;
static float zeta_xy = 0.85; // this gives good performance down to 0.4, the lower the more aggressive (less damping)

static float tau_z = 0.3;
static float zeta_z = 0.85;

// time constant of body angle (thrust direction) control
static float tau_rp = 0.25;
// what percentage is yaw control speed in terms of roll/pitch control speed \in [0, 1], 0 means yaw not controlled
static float mixing_factor = 1.0;

// time constant of rotational rate control
static float tau_rp_rate = 0.015;
static float tau_yaw_rate = 0.0075;

// minimum and maximum thrusts
static float coll_min = 1;
static float coll_max = 18;
// if too much thrust is commanded, which axis is reduced to meet maximum thrust?
// 1 -> even reduction across x, y, z
// 0 -> z gets what it wants (eg. maintain height at all costs)
static float thrust_reduction_fairness = 0.25;

// minimum and maximum body rates
static float omega_rp_max = 30;
static float omega_yaw_max = 10;
static float heuristic_rp = 12;
static float heuristic_yaw = 5;


// Struct for logging position information
static bool isInit = false;

void controllerBrescianiniInit(void) {
  if (isInit) {
    return;
  }

  isInit = true;
}


#define UPDATE_RATE RATE_100_HZ


void controllerBrescianini(control_t *control,
                                 const setpoint_t *setpoint,
                                 const sensorData_t *sensors,
                                 const state_t *state,
                                 const stabilizerStep_t stabilizerStep) {

  static float control_omega[3];
  static struct vec control_torque;
  static float control_thrust;

  // define this here, since we do body-rate control at 1000Hz below the following if statement
  float omega[3] = {0};
  omega[0] = radians(sensors->gyro.x);
  omega[1] = radians(sensors->gyro.y);
  omega[2] = radians(sensors->gyro.z);

  if (RATE_DO_EXECUTE(UPDATE_RATE, stabilizerStep)) {
    // desired accelerations
    struct vec accDes = vzero();
    // desired thrust
    float collCmd = 0;

    // attitude error as computed by the reduced attitude controller
    struct quat attErrorReduced = qeye();

    // attitude error as computed by the full attitude controller
    struct quat attErrorFull = qeye();

    // desired attitude as computed by the full attitude controller
    struct quat attDesiredFull = qeye();

    // current attitude
    struct quat attitude = mkquat(
      state->attitudeQuaternion.x,
      state->attitudeQuaternion.y,
      state->attitudeQuaternion.z,
      state->attitudeQuaternion.w);

    // inverse of current attitude
    struct quat attitudeI = qinv(attitude);

    // body frame -> inertial frame :  vI = R * vB
    // float R[3][3] = {0};
    // struct quat q = attitude;
    // R[0][0] = q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z;
    // R[0][1] = 2 * q.x * q.y - 2 * q.w * q.z;
    // R[0][2] = 2 * q.x * q.z + 2 * q.w * q.y;

    // R[1][0] = 2 * q.x * q.y + 2 * q.w * q.z;
    // R[1][1] = q.w * q.w - q.x * q.x + q.y * q.y - q.z * q.z;
    // R[1][2] = 2 * q.y * q.z - 2 * q.w * q.x;

    // R[2][0] = 2 * q.x * q.z - 2 * q.w * q.y;
    // R[2][1] = 2 * q.y * q.z + 2 * q.w * q.x;
    // R[2][2] = q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z;

    // We don't need all terms of R, only compute the necessary parts

    float R02 = 2 * attitude.x * attitude.z + 2 * attitude.w * attitude.y;
    float R12 = 2 * attitude.y * attitude.z - 2 * attitude.w * attitude.x;
    float R22 = attitude.w * attitude.w - attitude.x * attitude.x - attitude.y * attitude.y + attitude.z * attitude.z;

    // a few temporary quaternions
    struct quat temp1 = qeye();
    struct quat temp2 = qeye();

    // compute the position and velocity errors
    struct vec pError = mkvec(setpoint->position.x - state->position.x,
                              setpoint->position.y - state->position.y,
                              setpoint->position.z - state->position.z);

    struct vec vError = mkvec(setpoint->velocity.x - state->velocity.x,
                              setpoint->velocity.y - state->velocity.y,
                              setpoint->velocity.z - state->velocity.z);


    // ====== LINEAR CONTROL ======

    // compute desired accelerations in X, Y and Z
    accDes.x = 0;
    accDes.x += 1.0f / tau_xy / tau_xy * pError.x;
    accDes.x += 2.0f * zeta_xy / tau_xy * vError.x;
    accDes.x += setpoint->acceleration.x;
    accDes.x = constrain(accDes.x, -coll_max, coll_max);

    accDes.y = 0;
    accDes.y += 1.0f / tau_xy / tau_xy * pError.y;
    accDes.y += 2.0f * zeta_xy / tau_xy * vError.y;
    accDes.y += setpoint->acceleration.y;
    accDes.y = constrain(accDes.y, -coll_max, coll_max);

    accDes.z = GRAVITY_MAGNITUDE;
    accDes.z += 1.0f / tau_z / tau_z * pError.z;
    accDes.z += 2.0f * zeta_z / tau_z * vError.z;
    accDes.z += setpoint->acceleration.z;
    accDes.z = constrain(accDes.z, -coll_max, coll_max);


    // ====== THRUST CONTROL ======

    // compute commanded thrust required to achieve the z acceleration
    collCmd = accDes.z / R22;

    if (fabsf(collCmd) > coll_max) {
      // exceeding the thrust threshold
      // we compute a reduction factor r based on fairness f \in [0,1] such that:
      // collMax^2 = (r*x)^2 + (r*y)^2 + (r*f*z + (1-f)z + g)^2
      float x = accDes.x;
      float y = accDes.y;
      float z = accDes.z - GRAVITY_MAGNITUDE;
      float g = GRAVITY_MAGNITUDE;
      float f = constrain(thrust_reduction_fairness, 0, 1);

      float r = 0;

      // solve as a quadratic
      float a = powf(x, 2) + powf(y, 2) + powf(z*f, 2);
      if (a<0) { a = 0; }

      float b = 2 * z*f*((1-f)*z + g);
      float c = powf(coll_max, 2) - powf((1-f)*z + g, 2);
      if (c<0) { c = 0; }

      if (fabsf(a)<1e-6f) {
        r = 0;
      } else {
        float sqrtterm = powf(b, 2) + 4.0f*a*c;
        r = (-b + sqrtf(sqrtterm))/(2.0f*a);
        r = constrain(r,0,1);
      }
      accDes.x = r*x;
      accDes.y = r*y;
      accDes.z = (r*f+(1-f))*z + g;
    }
    collCmd = constrain(accDes.z / R22, coll_min, coll_max);

    // FYI: this thrust will result in the accelerations
    // xdd = R02*coll
    // ydd = R12*coll

    // a unit vector pointing in the direction of the desired thrust (ie. the direction of body's z axis in the inertial frame)
    struct vec zI_des = vnormalize(accDes);

    // a unit vector pointing in the direction of the current thrust
    struct vec zI_cur = vnormalize(mkvec(R02, R12, R22));

    // a unit vector pointing in the direction of the inertial frame z-axis
    struct vec zI = mkvec(0, 0, 1);



    // ====== REDUCED ATTITUDE CONTROL ======

    // compute the error angle between the current and the desired thrust directions
    float dotProd = vdot(zI_cur, zI_des);
    dotProd = constrain(dotProd, -1, 1);
    float alpha = acosf(dotProd);

    // the axis around which this rotation needs to occur in the inertial frame (ie. an axis orthogonal to the two)
    struct vec rotAxisI = vzero();
    if (fabsf(alpha) > 1 * ARCMINUTE) {
      rotAxisI = vnormalize(vcross(zI_cur, zI_des));
    } else {
      rotAxisI = mkvec(1, 1, 0);
    }

    // the attitude error quaternion
    attErrorReduced.w = cosf(alpha / 2.0f);
    attErrorReduced.x = sinf(alpha / 2.0f) * rotAxisI.x;
    attErrorReduced.y = sinf(alpha / 2.0f) * rotAxisI.y;
    attErrorReduced.z = sinf(alpha / 2.0f) * rotAxisI.z;

    // choose the shorter rotation
    if (sinf(alpha / 2.0f) < 0) {
      rotAxisI = vneg(rotAxisI);
    }
    if (cosf(alpha / 2.0f) < 0) {
      rotAxisI = vneg(rotAxisI);
      attErrorReduced = qneg(attErrorReduced);
    }

    attErrorReduced = qnormalize(attErrorReduced);


    // ====== FULL ATTITUDE CONTROL ======

    // compute the error angle between the inertial and the desired thrust directions
    dotProd = vdot(zI, zI_des);
    dotProd = constrain(dotProd, -1, 1);
    alpha = acosf(dotProd);

    // the axis around which this rotation needs to occur in the inertial frame (ie. an axis orthogonal to the two)
    if (fabsf(alpha) > 1 * ARCMINUTE) {
      rotAxisI = vnormalize(vcross(zI, zI_des));
    } else {
      rotAxisI = mkvec(1, 1, 0);
    }

    // the quaternion corresponding to a roll and pitch around this axis
    struct quat attFullReqPitchRoll = mkquat(sinf(alpha / 2.0f) * rotAxisI.x,
                                             sinf(alpha / 2.0f) * rotAxisI.y,
                                             sinf(alpha / 2.0f) * rotAxisI.z,
                                             cosf(alpha / 2.0f));

    // the quaternion corresponding to a rotation to the desired yaw
    struct quat attFullReqYaw = mkquat(0, 0, sinf(radians(setpoint->attitude.yaw) / 2.0f), cosf(radians(setpoint->attitude.yaw) / 2.0f));

    // the full rotation (roll & pitch, then yaw)
    attDesiredFull = qqmul(attFullReqPitchRoll, attFullReqYaw);

    // back transform from the current attitude to get the error between rotations
    attErrorFull = qqmul(attitudeI, attDesiredFull);

    // correct rotation
    if (attErrorFull.w < 0) {
      attErrorFull = qneg(attErrorFull);
      attDesiredFull = qqmul(attitude, attErrorFull);
    }

    attErrorFull = qnormalize(attErrorFull);
    attDesiredFull = qnormalize(attDesiredFull);


    // ====== MIXING FULL & REDUCED CONTROL ======

    struct quat attError = qeye();

    if (mixing_factor <= 0) {
      // 100% reduced control (no yaw control)
      attError = attErrorReduced;
    } else if (mixing_factor >= 1) {
      // 100% full control (yaw controlled with same time constant as roll & pitch)
      attError = attErrorFull;
    } else {
      // mixture of reduced and full control

      // calculate rotation between the two errors
      temp1 = qinv(attErrorReduced);
      temp2 = qnormalize(qqmul(temp1, attErrorFull));

      // by defintion this rotation has the form [cos(alpha/2), 0, 0, sin(alpha/2)]
      // where the first element gives the rotation angle, and the last the direction
      alpha = 2.0f * acosf(constrain(temp2.w, -1, 1));

      // bisect the rotation from reduced to full control
      temp1 = mkquat(0,
                       0,
                       sinf(alpha * mixing_factor / 2.0f) * (temp2.z < 0 ? -1 : 1), // rotate in the correct direction
                       cosf(alpha * mixing_factor / 2.0f));

      attError = qnormalize(qqmul(attErrorReduced, temp1));
    }

    // ====== COMPUTE CONTROL SIGNALS ======

    // compute the commanded body rates
    control_omega[0] = 2.0f / tau_rp * attError.x;
    control_omega[1] = 2.0f / tau_rp * attError.y;
    control_omega[2] = 2.0f / tau_rp * attError.z + radians(setpoint->attitudeRate.yaw); // due to the mixing, this will behave with time constant tau_yaw

    // apply the rotation heuristic
    if (control_omega[0] * omega[0] < 0 && fabsf(omega[0]) > heuristic_rp) { // desired rotational rate in direction opposite to current rotational rate
      control_omega[0] = omega_rp_max * (omega[0] < 0 ? -1 : 1); // maximum rotational rate in direction of current rotation
    }

    if (control_omega[1] * omega[1] < 0 && fabsf(omega[1]) > heuristic_rp) { // desired rotational rate in direction opposite to current rotational rate
      control_omega[1] = omega_rp_max * (omega[1] < 0 ? -1 : 1); // maximum rotational rate in direction of current rotation
    }

    if (control_omega[2] * omega[2] < 0 && fabsf(omega[2]) > heuristic_yaw) { // desired rotational rate in direction opposite to current rotational rate
      control_omega[2] = omega_rp_max * (omega[2] < 0 ? -1 : 1); // maximum rotational rate in direction of current rotation
    }

    // scale the commands to satisfy rate constraints
    float scaling = 1;
    scaling = fmax(scaling, fabsf(control_omega[0]) / omega_rp_max);
    scaling = fmax(scaling, fabsf(control_omega[1]) / omega_rp_max);
    scaling = fmax(scaling, fabsf(control_omega[2]) / omega_yaw_max);

    control_omega[0] /= scaling;
    control_omega[1] /= scaling;
    control_omega[2] /= scaling;
    control_thrust = collCmd;
  }

  if (setpoint->mode.z == modeDisable) {
    control->thrustSi = 0.0f;
    control->torque[0] =  0.0f;
    control->torque[1] =  0.0f;
    control->torque[2] =  0.0f;
  } else {
    // control the body torques
    struct vec omegaErr = mkvec((control_omega[0] - omega[0])/tau_rp_rate,
                        (control_omega[1] - omega[1])/tau_rp_rate,
                        (control_omega[2] - omega[2])/tau_yaw_rate);

    // update the commanded body torques based on the current error in body rates
    control_torque = mvmul(CRAZYFLIE_INERTIA, omegaErr);

    control->thrustSi = control_thrust * CF_MASS; // force to provide control_thrust
    control->torqueX = control_torque.x;
    control->torqueY = control_torque.y;
    control->torqueZ = control_torque.z;
  }

  control->controlMode = controlModeForceTorque;
}

bool controllerBrescianiniTest(void) {
  return true;
}


PARAM_GROUP_START(ctrlAtt)
PARAM_ADD(PARAM_FLOAT, tau_xy, &tau_xy)
PARAM_ADD(PARAM_FLOAT, zeta_xy, &zeta_xy)
PARAM_ADD(PARAM_FLOAT, tau_z, &tau_z)
PARAM_ADD(PARAM_FLOAT, zeta_z, &zeta_z)
PARAM_ADD(PARAM_FLOAT, tau_rp, &tau_rp)
PARAM_ADD(PARAM_FLOAT, mixing_factor, &mixing_factor)
PARAM_ADD(PARAM_FLOAT, coll_fairness, &thrust_reduction_fairness)
// PARAM_ADD(PARAM_FLOAT, heuristic_rp, &heuristic_rp)
// PARAM_ADD(PARAM_FLOAT, heuristic_yaw, &heuristic_yaw)
// PARAM_ADD(PARAM_FLOAT, tau_rp_rate, &tau_rp_rate)
// PARAM_ADD(PARAM_FLOAT, tau_yaw_rate, &tau_yaw_rate)
// PARAM_ADD(PARAM_FLOAT, coll_min, &coll_min)
// PARAM_ADD(PARAM_FLOAT, coll_max, &coll_max)
// PARAM_ADD(PARAM_FLOAT, omega_rp_max, &omega_rp_max)
// PARAM_ADD(PARAM_FLOAT, omega_yaw_max, &omega_yaw_max)
PARAM_GROUP_STOP(ctrlAtt)
