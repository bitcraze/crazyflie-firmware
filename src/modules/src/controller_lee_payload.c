/*
The MIT License (MIT)

Copyright (c) 2022 Khaled Wahba 

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

TODO
*/

#include <math.h>

#include "param.h"
#include "log.h"
#include "math3d.h"
#include "controller_lee_payload.h"

#define GRAVITY_MAGNITUDE (9.81f)

static float g_vehicleMass = 0.034; // TODO: should be CF global for other modules
static float mp = 0.01;
static float l = 1.0;  //length of the cable
static float thrustSI;
// Inertia matrix (diagonal matrix), see
// System Identification of the Crazyflie 2.0 Nano Quadrocopter
// BA theses, Julian Foerster, ETHZ
// https://polybox.ethz.ch/index.php/s/20dde63ee00ffe7085964393a55a91c7
static struct vec J = {16.571710e-6, 16.655602e-6, 29.261652e-6}; // kg m^2

// initValues:
static struct vec qi_prev;
static struct vec qdi_prev;
static struct vec payload_vel_prev;
// Position PID
static struct vec Kpos_P = {5, 5, 5}; // Kp in paper
static float Kpos_P_limit = 100;
static struct vec Kpos_D = {4.5, 4.5, 4.5}; // Kv in paper
static float Kpos_D_limit = 100;
static struct vec Kpos_I = {0, 0, 0}; // not in paper
static float Kpos_I_limit = 2;
static struct vec i_error_pos;

// Cable gains
static struct vec K_q = {28, 28, 20};
static struct vec K_w = {10, 10, 7}; 


static struct vec plp_error;
static struct vec plv_error;
// Attitude PID
static struct vec KR = {0.005, 0.005, 0.005};
static struct vec Komega = {0.0009, 0.0009, 0.0009};

// Logging variables
static struct vec rpy;
static struct vec rpy_des;
static struct vec qr;
static struct mat33 R_des;
static struct quat q;
static struct mat33 R;
static struct vec omega;
static struct vec omega_r;
static struct vec u;

// static uint32_t ticks;

static inline struct vec vclampscl(struct vec value, float min, float max) {
  return mkvec(
    clamp(value.x, min, max),
    clamp(value.y, min, max),
    clamp(value.z, min, max));
}

void controllerLeePayloadReset(void)
{
  i_error_pos = vzero();
  qi_prev = mkvec(0,0,-1);
  payload_vel_prev = mkvec(0,0,0);
  qdi_prev = vzero();
}

void controllerLeePayloadInit(void)
{
  controllerLeePayloadReset();
}

bool controllerLeePayloadTest(void)
{
  return true;
}

// static inline struct vec u_parallel()

void controllerLeePayload(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{


  if (!RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    return;
  }

  // uint64_t startTime = usecTimestamp();

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
    rpy_des = quat2rpy(setpoint_quat);
    desiredYaw = rpy_des.z;
  }

  // Position controller
  if (   setpoint->mode.x == modeAbs
      || setpoint->mode.y == modeAbs
      || setpoint->mode.z == modeAbs) {
    
    struct vec plPos_d = mkvec(setpoint->position.x, setpoint->position.y, setpoint->position.z);
    struct vec plVel_d = mkvec(setpoint->velocity.x, setpoint->velocity.y, setpoint->velocity.z);
    struct vec plAcc_d = mkvec(setpoint->acceleration.x, setpoint->acceleration.y, setpoint->acceleration.z + GRAVITY_MAGNITUDE);
    
    // struct vec stateVel = mkvec(state->velocity.x, state->velocity.y, state->velocity.z);
    struct vec statePos = mkvec(state->position.x, state->position.y, state->position.z);
    struct vec plStPos = mkvec(state->payload_pos.x, state->payload_pos.y, state->payload_pos.z);
    struct vec plStVel = mkvec(state->payload_vel.x, state->payload_vel.y, state->payload_vel.z);
     // errors
    struct vec plpos_e = vclampscl(vsub(plPos_d, plStPos), -Kpos_P_limit, Kpos_P_limit);
    struct vec plvel_e = vclampscl(vsub(plVel_d, plStVel), -Kpos_D_limit, Kpos_D_limit);

    plp_error = plpos_e;
    plv_error = plvel_e;

    struct vec F_d =vscl(mp ,vadd3(
      plAcc_d,
      veltmul(Kpos_P, plpos_e),
      veltmul(Kpos_D, plvel_e)));

    struct vec desVirtInp = F_d;
    //directional unit vector qi and angular velocity wi pointing from UAV to payload
    struct vec qi = vnormalize(vsub(plStPos, statePos));
    struct vec qidot = vdiv(vsub(qi , qi_prev), dt);
    qi_prev = qi;

    struct vec wi = vcross(qi, qidot);

    struct mat33 qiqiT = vecmult(qi);
    struct vec virtualInp = mvmul(qiqiT, desVirtInp);
    // Compute parallel component
    struct vec grav = mkvec(0,0,-GRAVITY_MAGNITUDE);
    struct vec acc0 = vsub(vdiv(vsub(plStVel, payload_vel_prev),dt), grav); 
    payload_vel_prev = plStVel;

    struct vec u_parallel = vadd3(virtualInp, vscl(g_vehicleMass*l*vmag2(wi), qi), vscl(g_vehicleMass, mvmul(qiqiT, acc0)));
    
    // Compute Perpindicular Component
    struct vec qdi = vnormalize(vneg(desVirtInp));
    struct vec eq  = vcross(qdi, qi);
    struct mat33 skewqi = mcrossmat(qi);
    struct mat33 skewqi2 = mmul(skewqi,skewqi);

    struct vec qdidot = vdiv(vsub(qdi, qdi_prev), dt);
    qdi_prev = qdi;
    struct vec wdi = vcross(qdi, qdidot);
    struct vec ew = vadd(wi, mvmul(skewqi2, wdi));

    struct vec u_perpind = vsub(
      vscl(g_vehicleMass*l, mvmul(skewqi, vsub(vneg(veltmul(K_q, eq)), veltmul(K_w, ew)))),
      vscl(g_vehicleMass, mvmul(skewqi2,acc0))
    );

    struct vec u_i = vadd(u_parallel, u_perpind);
    
    control->thrustSI = vmag(u_i);
    control->u_all[0] = u_i.x;
    control->u_all[1] = u_i.y;
    control->u_all[2] = u_i.z;


    thrustSI = control->thrustSI;
  // Reset the accumulated error while on the ground
    if (control->thrustSI < 0.01f) {
      controllerLeePayloadReset();
    }
  
  q = mkquat(state->attitudeQuaternion.x, state->attitudeQuaternion.y, state->attitudeQuaternion.z, state->attitudeQuaternion.w);
  rpy = quat2rpy(q);
  R = quat2rotmat(q);
  struct vec z = vbasis(2);
  // Compute Desired Rotation matrix
    float thrustSI = control->thrustSI;
    struct vec Fd_ = vscl(control->thrustSI, mvmul(R, z));
    struct vec xdes = vbasis(0);
    struct vec ydes = vbasis(1);
    struct vec zdes = vbasis(2);
   
    if (thrustSI > 0) {
      zdes = vnormalize(Fd_);
    } 
    struct vec xcdes = mkvec(cosf(desiredYaw), sinf(desiredYaw), 0); 
    struct vec zcrossx = vcross(zdes, xcdes);
    float normZX = vmag(zcrossx);

    if (normZX > 0) {
      ydes = vnormalize(zcrossx);
    } 
    xdes = vcross(ydes, zdes);
    
    R_des = mcolumns(xdes, ydes, zdes);

  } else {
    if (setpoint->mode.z == modeDisable) {
      if (setpoint->thrust < 1000) {
          control->controlMode = controlModeForceTorque;
          control->thrustSI  = 0;
          control->torque[0] = 0;
          control->torque[1] = 0;
          control->torque[2] = 0;
          controllerLeePayloadReset();
          return;
      }
    }
    // On CF2, thrust is mapped 65536 <==> 4 * 12 grams
    const float max_thrust = 70.0f / 1000.0f * 9.81f; // N
    control->thrustSI = setpoint->thrust / UINT16_MAX * max_thrust;

    qr = mkvec(
      radians(setpoint->attitude.roll),
      -radians(setpoint->attitude.pitch), // This is in the legacy coordinate system where pitch is inverted
      desiredYaw);
  }

  // Attitude controller

  // current rotation [R]
  rpy = quat2rpy(q);
  struct mat33 R = quat2rotmat(q);

  // desired rotation [Rdes]
  struct quat q_des = mat2quat(R_des);
  rpy_des = quat2rpy(q_des);

  // rotation error
  struct mat33 eRM = msub(mmul(mtranspose(R_des), R), mmul(mtranspose(R), R_des));

  struct vec eR = vscl(0.5f, mkvec(eRM.m[2][1], eRM.m[0][2], eRM.m[1][0]));

  // angular velocity
  omega = mkvec(
    radians(sensors->gyro.x),
    radians(sensors->gyro.y),
    radians(sensors->gyro.z));

  // Compute desired omega
  struct vec xdes = mcolumn(R_des, 0);
  struct vec ydes = mcolumn(R_des, 1);
  struct vec zdes = mcolumn(R_des, 2);
  struct vec hw = vzero();
  // Desired Jerk and snap for now are zeros vector
  struct vec desJerk = mkvec(setpoint->jerk.x, setpoint->jerk.y, setpoint->jerk.z);

  if (control->thrustSI != 0) {
    struct vec tmp = vsub(desJerk, vscl(vdot(zdes, desJerk), zdes));
    hw = vscl(g_vehicleMass/control->thrustSI, tmp);
  }

  struct vec omega_des = mkvec(-vdot(hw,ydes), vdot(hw,xdes), 0);
  
  omega_r = mvmul(mmul(mtranspose(R), R_des), omega_des);

  struct vec omega_error = vsub(omega, omega_r);
  


  // compute moments
  // M = -kR eR - kw ew + w x Jw - J(w x wr)
  u = vadd3(
    vneg(veltmul(KR, eR)),
    vneg(veltmul(Komega, omega_error)),
    vcross(omega, veltmul(J, omega)));

  // if (enableNN > 1) {
  //   u = vsub(u, tau_a);
  // }

  control->controlMode = controlModeForceTorque;
  control->torque[0] = u.x;
  control->torque[1] = u.y;
  control->torque[2] = u.z;
  // ticks = usecTimestamp() - startTime;
}

PARAM_GROUP_START(ctrlLeeP)
PARAM_ADD(PARAM_FLOAT, KR_x, &KR.x)
PARAM_ADD(PARAM_FLOAT, KR_y, &KR.y)
PARAM_ADD(PARAM_FLOAT, KR_z, &KR.z)
// Attitude D
PARAM_ADD(PARAM_FLOAT, Kw_x, &Komega.x)
PARAM_ADD(PARAM_FLOAT, Kw_y, &Komega.y)
PARAM_ADD(PARAM_FLOAT, Kw_z, &Komega.z)

// J
PARAM_ADD(PARAM_FLOAT, J_x, &J.x)
PARAM_ADD(PARAM_FLOAT, J_y, &J.y)
PARAM_ADD(PARAM_FLOAT, J_z, &J.z)

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
// Position I
PARAM_ADD(PARAM_FLOAT, Kpos_Ix, &Kpos_I.x)
PARAM_ADD(PARAM_FLOAT, Kpos_Iy, &Kpos_I.y)
PARAM_ADD(PARAM_FLOAT, Kpos_Iz, &Kpos_I.z)
PARAM_ADD(PARAM_FLOAT, Kpos_I_limit, &Kpos_I_limit)

PARAM_ADD(PARAM_FLOAT, mass, &g_vehicleMass)
PARAM_GROUP_STOP(ctrlLeeP)


LOG_GROUP_START(ctrlLeeP)

LOG_ADD(LOG_FLOAT, KR_x, &KR.x)
LOG_ADD(LOG_FLOAT, KR_y, &KR.y)
LOG_ADD(LOG_FLOAT, KR_z, &KR.z)
LOG_ADD(LOG_FLOAT, Kw_x, &Komega.x)
LOG_ADD(LOG_FLOAT, Kw_y, &Komega.y)
LOG_ADD(LOG_FLOAT, Kw_z, &Komega.z)

LOG_ADD(LOG_FLOAT,Kpos_Px, &Kpos_P.x)
LOG_ADD(LOG_FLOAT,Kpos_Py, &Kpos_P.y)
LOG_ADD(LOG_FLOAT,Kpos_Pz, &Kpos_P.z)
LOG_ADD(LOG_FLOAT,Kpos_Dx, &Kpos_D.x)
LOG_ADD(LOG_FLOAT,Kpos_Dy, &Kpos_D.y)
LOG_ADD(LOG_FLOAT,Kpos_Dz, &Kpos_D.z)


LOG_ADD(LOG_FLOAT, thrustSI, &thrustSI)
LOG_ADD(LOG_FLOAT, torquex, &u.x)
LOG_ADD(LOG_FLOAT, torquey, &u.y)
LOG_ADD(LOG_FLOAT, torquez, &u.z)

// current angles
LOG_ADD(LOG_FLOAT, rpyx, &rpy.x)
LOG_ADD(LOG_FLOAT, rpyy, &rpy.y)
LOG_ADD(LOG_FLOAT, rpyz, &rpy.z)

// desired angles
LOG_ADD(LOG_FLOAT, rpydx, &rpy_des.x)
LOG_ADD(LOG_FLOAT, rpydy, &rpy_des.y)
LOG_ADD(LOG_FLOAT, rpydz, &rpy_des.z)

// errors
// LOG_ADD(LOG_FLOAT, error_posx, &p_error.x)
// LOG_ADD(LOG_FLOAT, error_posy, &p_error.y)
// LOG_ADD(LOG_FLOAT, error_posz, &p_error.z)

// LOG_ADD(LOG_FLOAT, error_velx, &v_error.x)
// LOG_ADD(LOG_FLOAT, error_vely, &v_error.y)
// LOG_ADD(LOG_FLOAT, error_velz, &v_error.z)

// omega
LOG_ADD(LOG_FLOAT, omegax, &omega.x)
LOG_ADD(LOG_FLOAT, omegay, &omega.y)
LOG_ADD(LOG_FLOAT, omegaz, &omega.z)

// omega_r
LOG_ADD(LOG_FLOAT, omegarx, &omega_r.x)
LOG_ADD(LOG_FLOAT, omegary, &omega_r.y)
LOG_ADD(LOG_FLOAT, omegarz, &omega_r.z)

// LOG_ADD(LOG_UINT32, ticks, &ticks)

LOG_GROUP_STOP(ctrlLeeP)



               


          