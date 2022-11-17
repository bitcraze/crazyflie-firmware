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

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "cf_math.h"

#include "controller_non_linear_attitude.h"
#include "log.h"
#include "param.h"
#include "num.h"
#include "cfassert.h"
#include "math3d.h"
#include "physicalConstants.h"

// TODO krri move to math?
#define ARCMINUTE (((float)M_PI)/10800.0f)

// TOOD krri Looks fishy, investigate
#define CONTROLMODE_ACCELERATION(mode) ((0b001 & mode) != 0)
#define CONTROLMODE_VELOCITY(mode)     ((0b010 & mode) != 0)
#define CONTROLMODE_POSITION(mode)     ((0b100 & mode) != 0)

// TODO krri Change for the LED-ring?
static float CRAZYFLIE_INERTIA[3][3] =
    {{16.6e-6f, 0.83e-6f, 0.72e-6f},
     {0.83e-6f, 16.6e-6f, 1.8e-6f},
     {0.72e-6f, 1.8e-6f, 29.3e-6f}};

static arm_matrix_instance_f32 CRAZYFLIE_INERTIA_m = {3, 3, (float*)CRAZYFLIE_INERTIA};

// TODO krri Use math3d lib instead for qaut and vector operations
static float vec_norm(const arm_matrix_instance_f32 *pSrcA) {
  float32_t s = 0;
  configASSERT( pSrcA->numCols == 1 );

  float *a = pSrcA->pData;

  for (int i=0; i<pSrcA->numRows; i++) {
    s += powf(a[0+1*i], 2);
  }

  if (s<=0) { return 0; }

  return arm_sqrt(s);
}

static void vec_normalize(arm_matrix_instance_f32 *pSrcA) {
  float norm = vec_norm(pSrcA);

  if (norm<=0) { return; }

  float *a = pSrcA->pData;
  for (int i=0; i<pSrcA->numRows; i++) {
    a[i] = a[i]/norm;
  }
}

static void vec_negate(arm_matrix_instance_f32 *pSrcA) {
  configASSERT( pSrcA->numCols == 1 );

  float *a = pSrcA->pData;

  for (int i=0; i<pSrcA->numRows; i++) {
    a[i] = -1*a[i];
  }
}

static float vec_dot(const arm_matrix_instance_f32 *pSrcA, const arm_matrix_instance_f32 *pSrcB) {
  float32_t s = 0;
  configASSERT( pSrcA->numCols == 1 );
  configASSERT( pSrcB->numCols == 1 );
  configASSERT( pSrcA->numRows == pSrcB->numRows );

  float *a = pSrcA->pData;
  float *b = pSrcB->pData;

  for (int i=0; i<pSrcA->numRows; i++) {
    s += a[0+1*i] * b[0+1*i];
  }
  return s;
}

static void vec_cross(const arm_matrix_instance_f32 *pSrcA, const arm_matrix_instance_f32 *pSrcB, arm_matrix_instance_f32 *pDst) {
  configASSERT( pSrcA->numCols == 1 );
  configASSERT( pSrcB->numCols == 1 );
  configASSERT( pDst->numCols == 1 );
  configASSERT( pSrcA->numRows == 3 );
  configASSERT( pSrcB->numRows == 3 );
  configASSERT( pDst->numRows == 3 );

  float *a = pSrcA->pData;
  float *b = pSrcB->pData;
  float *c = pDst->pData;

  c[0] = a[1]*b[2] - a[2]*b[1];
  c[1] = a[2]*b[0] - a[0]*b[2];
  c[2] = a[0]*b[1] - a[1]*b[0];
}

static void quaternion_normalize(arm_matrix_instance_f32 *pSrcA) {
  configASSERT( pSrcA->numCols == 1 );
  configASSERT( pSrcA->numRows == 4 ); // vector is a quaternion [w, x, y, z]

  vec_normalize(pSrcA);
  if (pSrcA->pData[0] < 0) {
    vec_negate(pSrcA);
  }
}

static void quaternion_inverse(const arm_matrix_instance_f32 *pSrcA, arm_matrix_instance_f32 *pDst) {
  configASSERT( pSrcA->numCols == 1 );
  configASSERT( pSrcA->numRows == 4 ); // vector is a quaternion [w, x, y, z]
  configASSERT( pDst->numCols == 1 );
  configASSERT( pDst->numRows == 4 ); // vector is a quaternion [w, x, y, z]

  float *a = pSrcA->pData;
  float *c = pDst->pData;
  c[0] = a[0];
  c[1] = -a[1];
  c[2] = -a[2];
  c[3] = -a[3];

  vec_normalize(pDst);
}

static void quaternion_multiply(const arm_matrix_instance_f32 *pSrcA, const arm_matrix_instance_f32 *pSrcB, arm_matrix_instance_f32 *pDst) {
  configASSERT( pSrcA->numCols == 1 );
  configASSERT( pSrcA->numRows == 4 ); // vector is a quaternion [w, x, y, z]
  configASSERT( pSrcB->numCols == 1 );
  configASSERT( pSrcB->numRows == 4 ); // vector is a quaternion [w, x, y, z]
  configASSERT( pDst->numCols == 1 );
  configASSERT( pDst->numRows == 4 ); // vector is a quaternion [w, x, y, z]

  float *a = pSrcA->pData;
  float *b = pSrcB->pData;
  float *c = pDst->pData;

  c[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
  c[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
  c[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
  c[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}



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

void controllerNonLinearAttitudeInit(void) {
  if (isInit) {
    return;
  }

  isInit = true;
}


#define UPDATE_RATE RATE_100_HZ


void controllerNonLinearAttitude(control_t *control,
                        setpoint_t *setpoint,
                        const sensorData_t *sensors,
                        const state_t *state,
                        const uint32_t tick) {

  static float control_omega[3];
  static float control_torque[3];
  static float control_thrust;

  // define this here, since we do body-rate control at 1000Hz below the following if statement
  float omega[3] = {0};
  omega[0] = radians(sensors->gyro.x);
  omega[1] = radians(sensors->gyro.y);
  omega[2] = radians(sensors->gyro.z);

  if (RATE_DO_EXECUTE(UPDATE_RATE, tick)) {
    // desired accelerations
    float accDes[3] = {0};
    // desired thrust
    float collCmd = 0;

    // attitude error as computed by the reduced attitude controller
    float attErrorReduced[4] = {0};
    arm_matrix_instance_f32 attErrorReduced_m = {4, 1, attErrorReduced};

    // desired attitude as computed by the reduced attitude controller
    float attDesiredReduced[4] = {0};
    arm_matrix_instance_f32 attDesiredReduced_m = {4, 1, attDesiredReduced};

    // attitude error as computed by the full attitude controller
    float attErrorFull[4] = {0};
    arm_matrix_instance_f32 attErrorFull_m = {4, 1, attErrorFull};

    // desired attitude as computed by the full attitude controller
    float attDesiredFull[4] = {0};
    arm_matrix_instance_f32 attDesiredFull_m = {4, 1, attDesiredFull};

    // current attitude
    quaternion_t sq = state->attitudeQuaternion;
    float q[4] = {sq.w, sq.x, sq.y, sq.z};
    arm_matrix_instance_f32 attitude_m = {4, 1, q};

    // inverse of current attitude
    float qI[4] = {0};
    arm_matrix_instance_f32 attitudeI_m = {4, 1, qI};
    quaternion_inverse(&attitude_m, &attitudeI_m);

    // body frame -> inertial frame :  vI = R*vB
    float R[3][3] = {0};

    R[0][0] = q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
    R[0][1] = 2 * q[1] * q[2] - 2 * q[0] * q[3];
    R[0][2] = 2 * q[1] * q[3] + 2 * q[0] * q[2];

    R[1][0] = 2 * q[1] * q[2] + 2 * q[0] * q[3];
    R[1][1] = q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3];
    R[1][2] = 2 * q[2] * q[3] - 2 * q[0] * q[1];

    R[2][0] = 2 * q[1] * q[3] - 2 * q[0] * q[2];
    R[2][1] = 2 * q[2] * q[3] + 2 * q[0] * q[1];
    R[2][2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];

    // a few temporary quaternions
    float temp1[4] = {0};
    arm_matrix_instance_f32 temp1_m = {4, 1, temp1};

    float temp2[4] = {0};
    arm_matrix_instance_f32 temp2_m = {4, 1, temp2};

    // compute the position and velocity errors
    float pError[] = {setpoint->position.x - state->position.x,
                      setpoint->position.y - state->position.y,
                      setpoint->position.z - state->position.z};

    float vError[] = {setpoint->velocity.x - state->velocity.x,
                      setpoint->velocity.y - state->velocity.y,
                      setpoint->velocity.z - state->velocity.z};


    // ====== LINEAR CONTROL ======

    // compute desired accelerations in X, Y and Z
    accDes[0] = 0;
    // if (CONTROLMODE_POSITION(setpoint->xmode))
    { accDes[0] += 1.0f / tau_xy / tau_xy * pError[0]; }
    // if (CONTROLMODE_VELOCITY(setpoint->xmode))
    { accDes[0] += 2.0f * zeta_xy / tau_xy * vError[0]; }
    // if (CONTROLMODE_ACCELERATION(setpoint->xmode))
    { accDes[0] += setpoint->acceleration.x; }
    accDes[0] = constrain(accDes[0], -coll_max, coll_max);

    accDes[1] = 0;
    // if (CONTROLMODE_POSITION(setpoint->ymode))
    { accDes[1] += 1.0f / tau_xy / tau_xy * pError[1]; }
    // if (CONTROLMODE_VELOCITY(setpoint->ymode))
    { accDes[1] += 2.0f * zeta_xy / tau_xy * vError[1]; }
    // if (CONTROLMODE_ACCELERATION(setpoint->ymode))
    { accDes[1] += setpoint->acceleration.y; }
    accDes[1] = constrain(accDes[1], -coll_max, coll_max);

    accDes[2] = GRAVITY_MAGNITUDE;
    // if (CONTROLMODE_POSITION(setpoint->zmode))
    { accDes[2] += 1.0f / tau_z / tau_z * pError[2]; }
    // if (CONTROLMODE_VELOCITY(setpoint->zmode))
    { accDes[2] += 2.0f * zeta_z / tau_z * vError[2]; }
    // if (CONTROLMODE_ACCELERATION(setpoint->zmode))
    { accDes[2] += setpoint->acceleration.z; }
    accDes[2] = constrain(accDes[2], -coll_max, coll_max);


    // ====== THRUST CONTROL ======

    // compute commanded thrust required to achieve the z acceleration
    collCmd = accDes[2] / R[2][2];

    if (fabsf(collCmd) > coll_max) {
      // exceeding the thrust threshold
      // we compute a reduction factor r based on fairness f \in [0,1] such that:
      // collMax^2 = (r*x)^2 + (r*y)^2 + (r*f*z + (1-f)z + g)^2
      float x = accDes[0];
      float y = accDes[1];
      float z = accDes[2] - GRAVITY_MAGNITUDE;
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
        configASSERT(sqrtterm>=0);
        r = (-b + arm_sqrt(sqrtterm))/(2.0f*a);
        r = constrain(r,0,1);
      }
      accDes[0] = r*x;
      accDes[1] = r*y;
      accDes[2] = (r*f+(1-f))*z + g;
    }
    collCmd = constrain(accDes[2] / R[2][2], coll_min, coll_max);

    // FYI: this thrust will result in the accelerations
    // xdd = R02*coll
    // ydd = R12*coll

    // a unit vector pointing in the direction of the desired thrust (ie. the direction of body's z axis in the inertial frame)
    float zI_des[3] = {accDes[0], accDes[1], accDes[2]};
    arm_matrix_instance_f32 zI_des_m = {3, 1, zI_des};
    vec_normalize(&zI_des_m);

    // a unit vector pointing in the direction of the current thrust
    float zI_cur[3] = {R[0][2], R[1][2], R[2][2]};
    arm_matrix_instance_f32 zI_cur_m = {3, 1, zI_cur};
    vec_normalize(&zI_cur_m);

    // a unit vector pointing in the direction of the inertial frame z-axis
    float zI[3] = {0, 0, 1};
    arm_matrix_instance_f32 zI_m = {3, 1, zI};



    // ====== REDUCED ATTITUDE CONTROL ======

    // compute the error angle between the current and the desired thrust directions
    float dotProd = vec_dot(&zI_cur_m, &zI_des_m);
    dotProd = constrain(dotProd, -1, 1);
    float alpha = acosf(dotProd);

    // the axis around which this rotation needs to occur in the inertial frame (ie. an axis orthogonal to the two)
    float rotAxisI[3] = {0};
    arm_matrix_instance_f32 rotAxisI_m = {3, 1, rotAxisI};
    if (fabsf(alpha) > 1 * ARCMINUTE)
    {
      vec_cross(&zI_cur_m, &zI_des_m, &rotAxisI_m);
      vec_normalize(&rotAxisI_m);
    }
    else
    {
      rotAxisI[0] = 1;
      rotAxisI[1] = 1;
      rotAxisI[2] = 0;
    }

    // the attitude error quaternion
    attErrorReduced[0] = cosf(alpha / 2.0f);
    attErrorReduced[1] = sinf(alpha / 2.0f) * rotAxisI[0];
    attErrorReduced[2] = sinf(alpha / 2.0f) * rotAxisI[1];
    attErrorReduced[3] = sinf(alpha / 2.0f) * rotAxisI[2];

    // choose the shorter rotation
    if (sinf(alpha / 2.0f) < 0)
    {
      vec_negate(&rotAxisI_m);
    }
    if (cosf(alpha / 2.0f) < 0)
    {
      vec_negate(&rotAxisI_m);
      vec_negate(&attErrorReduced_m);
    }

    quaternion_multiply(&attitude_m, &attErrorReduced_m, &attDesiredReduced_m);
    quaternion_normalize(&attErrorReduced_m);
    quaternion_normalize(&attDesiredReduced_m);


    // ====== FULL ATTITUDE CONTROL ======

    // compute the error angle between the inertial and the desired thrust directions
    dotProd = vec_dot(&zI_m, &zI_des_m);
    dotProd = constrain(dotProd, -1, 1);
    alpha = acosf(dotProd);

    // the axis around which this rotation needs to occur in the inertial frame (ie. an axis orthogonal to the two)
    if (fabsf(alpha) > 1 * ARCMINUTE)
    {
      vec_cross(&zI_m, &zI_des_m, &rotAxisI_m);
      vec_normalize(&rotAxisI_m);
    }
    else
    {
      rotAxisI[0] = 1;
      rotAxisI[1] = 1;
      rotAxisI[2] = 0;
    }

    // the quaternion corresponding to a roll and pitch around this axis
    float attFullReqPitchRoll[4] = {0};
    arm_matrix_instance_f32 attFullReqPitchRoll_m = {4, 1, attFullReqPitchRoll};
    attFullReqPitchRoll[0] = cosf(alpha / 2.0f);
    attFullReqPitchRoll[1] = sinf(alpha / 2.0f) * rotAxisI[0];
    attFullReqPitchRoll[2] = sinf(alpha / 2.0f) * rotAxisI[1];
    attFullReqPitchRoll[3] = sinf(alpha / 2.0f) * rotAxisI[2];

    // the quaternion corresponding to a rotation to the desired yaw
    float attFullReqYaw[4] = {0};
    arm_matrix_instance_f32 attFullReqYaw_m = {4, 1, attFullReqYaw};
    attFullReqYaw[0] = cosf(radians(setpoint->attitude.yaw) / 2.0f);
    attFullReqYaw[1] = 0;
    attFullReqYaw[2] = 0;
    attFullReqYaw[3] = sinf(radians(setpoint->attitude.yaw) / 2.0f);

    // the full rotation (roll & pitch, then yaw)
    quaternion_multiply(&attFullReqPitchRoll_m, &attFullReqYaw_m, &attDesiredFull_m);

    // back transform from the current attitude to get the error between rotations
    quaternion_multiply(&attitudeI_m, &attDesiredFull_m, &attErrorFull_m);

    // correct rotation
    if (attErrorFull[0] < 0)
    {
      vec_negate(&attErrorFull_m);
      quaternion_multiply(&attitude_m, &attErrorFull_m, &attDesiredFull_m);
    }

    quaternion_normalize(&attErrorFull_m);
    quaternion_normalize(&attDesiredFull_m);


    // ====== MIXING FULL & REDUCED CONTROL ======

    float attError[4] = {0};
    arm_matrix_instance_f32 attError_m = {4, 1, attError};

    if (mixing_factor <= 0)
    {
      // 100% reduced control (no yaw control)
      memcpy(attError, attErrorReduced, sizeof(attError));
    }
    else if (mixing_factor >= 1)
    {
      // 100% full control (yaw controlled with same time constant as roll & pitch)
      memcpy(attError, attErrorFull, sizeof(attError));
    }
    else
    {
      // mixture of reduced and full control

      // calculate rotation between the two errors
      quaternion_inverse(&attErrorReduced_m, &temp1_m);
      quaternion_multiply(&temp1_m, &attErrorFull_m, &temp2_m);
      quaternion_normalize(&temp2_m);

      // by defintion this rotation has the form [cos(alpha/2), 0, 0, sin(alpha/2)]
      // where the first element gives the rotation angle, and the last the direction
      alpha = 2.0f * acosf(constrain(temp2[0], -1, 1));

      // bisect the rotation from reduced to full control
      temp1[0] = cosf(alpha * mixing_factor / 2.0f);
      temp1[1] = 0;
      temp1[2] = 0;
      temp1[3] = sinf(alpha * mixing_factor / 2.0f) * (temp2[3] < 0 ? -1 : 1); // rotate in the correct direction

      quaternion_multiply(&attErrorReduced_m, &temp1_m, &attError_m);
      quaternion_normalize(&attError_m);
    }

    // ====== COMPUTE CONTROL SIGNALS ======

    // compute the commanded body rates
    control_omega[0] = 2.0f / tau_rp * attError[1];
    control_omega[1] = 2.0f / tau_rp * attError[2];
    control_omega[2] = 2.0f / tau_rp * attError[3] + radians(setpoint->attitudeRate.yaw); // due to the mixing, this will behave with time constant tau_yaw

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
    float omegaErr[3] = {(control_omega[0] - omega[0])/tau_rp_rate,
                        (control_omega[1] - omega[1])/tau_rp_rate,
                        (control_omega[2] - omega[2])/tau_yaw_rate};

    arm_matrix_instance_f32 omegaErr_m = {3, 1, omegaErr};
    arm_matrix_instance_f32 torques_m = {3, 1, control_torque};

    // update the commanded body torques based on the current error in body rates
    mat_mult(&CRAZYFLIE_INERTIA_m, &omegaErr_m, &torques_m);

    control->thrustSi = control_thrust * CF_MASS; // force to provide control_thrust
    control->torque[0] = control_torque[0];
    control->torque[1] = control_torque[1];
    control->torque[2] = control_torque[2];
  }

  control->controlMode = controlModeForceTorque;
}

bool controllerNonLinearAttitudeTest(void) {
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
