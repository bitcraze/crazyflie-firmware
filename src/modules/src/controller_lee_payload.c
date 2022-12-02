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
#include <string.h>
#include "math3d.h"
#include "controller_lee_payload.h"
#include "stdio.h"
#include "debug.h"
// QP
// #include "workspace_2uav_2hp.h"
#include "osqp.h"
extern OSQPWorkspace workspace_2uav_2hp;

#define GRAVITY_MAGNITUDE (9.81f)


// static inline struct vec computePlaneNormal(struct vec rpy, float yaw) {
// // Compute the normal of a plane, given the extrinsic roll-pitch-yaw of the z-axis 
// // and the yaw representing the x-axis of the plan's frame
//   struct vec x = mkvec(cosf(yaw), sinf(yaw), 0);
//   struct quat q = rpy2quat(rpy);
//   struct mat33 Rq = quat2rotmat(q);
//   struct vec e3 = mkvec(0,0,1);
//   struct vec z = mvmul(Rq, e3);
//   struct vec xcrossz = vcross(x,z);
//   struct vec ni = vnormalize(xcrossz);
//   return ni;
// }
static inline struct vec computePlaneNormal(struct vec pos1, struct vec pos2, struct vec pload, float r) {
// Compute the normal of a plane, given the minimum desired distance r
  struct vec mid = vscl(0.5, vsub(pos2, pos1));
  struct vec rvec = vscl(r, vnormalize(vsub(pos1, pos2)));
  struct vec pr = vadd(pos1, vadd(mid, rvec));

  struct vec p0pr = vsub(pr, pload);
  struct vec prp2 = vsub(pos2, pr);
  struct vec ns = vcross(prp2, p0pr);
  struct vec n_sol = vcross(p0pr, ns);
  return n_sol;
}
static inline struct mat26 Ainequality(float angle_limit) {
  struct vec q1_limit = mkvec(-radians(angle_limit), 0 , 0);
  struct vec q2_limit = mkvec(radians(angle_limit), 0 , 0);

  struct quat q1 = rpy2quat(q1_limit);
  struct quat q2 = rpy2quat(q2_limit);

  struct mat33 R1 = quat2rotmat(q1);
  struct mat33 R2 = quat2rotmat(q2);
  struct vec e3 = mkvec(0,0,-1);
  // Final vectors to rotate
  struct vec q1vec = mvmul(R1, e3);
  struct vec q2vec = mvmul(R2, e3);
  
  struct quat q_rotate = rpy2quat(mkvec(0,0,radians(20.0)));
  struct vec q1_ = qvrot(q_rotate, q1vec); 
  struct vec q2_ = qvrot(q_rotate, q2vec); 

  struct vec n1 = vcross(q1vec, q1_);
  struct vec n2 = vcross(q2vec, q2_);
  
  struct mat26 A_in = addrows(n1,n2);
  return A_in;
}

static controllerLeePayload_t g_self = {
  .mass = 0.034,
  .mp   = 0.01,
  // Inertia matrix (diagonal matrix), see
  // System Identification of the Crazyflie 2.0 Nano Quadrocopter
  // BA theses, Julian Foerster, ETHZ
  // https://polybox.ethz.ch/index.php/s/20dde63ee00ffe7085964393a55a91c7
  .J = {16.571710e-6, 16.655602e-6, 29.261652e-6}, // kg m^2

  // Position PID
  .Kpos_P = {18, 18, 18},
  .Kpos_P_limit = 100,
  .Kpos_D = {15, 15, 15},
  .Kpos_D_limit = 100,
  .Kpos_I ={10, 10, 10},
  .Kpos_I_limit = 0,

  // Cables PD
  .K_q = {25, 25, 25},
  .K_w = {24, 24, 24},

  //Attitude PID 
  .KR = {0.008, 0.008, 0.01},
  .Komega = {0.0013, 0.0013, 0.002},
  .KI = {0.02, 0.02, 0.05},
  // -----------------------FOR QP----------------------------//
  // 0 for UAV 1 and, 1 for UAV 2
  .radius = 0.1,
};

// static inline struct vec vclampscl(struct vec value, float min, float max) {
//   return mkvec(
//     clamp(value.x, min, max),
//     clamp(value.y, min, max),
//     clamp(value.z, min, max));
// }

static struct vec computeDesiredVirtualInput(const state_t *state, struct vec F_d)
{
    struct vec statePos = mkvec(state->position.x, state->position.y, state->position.z);
    struct vec plStPos = mkvec(state->payload_pos.x, state->payload_pos.y, state->payload_pos.z);
    struct vec statePos2 = mkvec(state->position_neighbors[0].x, state->position_neighbors[0].y, state->position_neighbors[0].z);

    float radius = g_self.radius;

    struct vec desVirtInp;

    //------------------------------------------QP------------------------------//
    // The QP will be added here for the desired virtual input (mu_des)
    OSQPWorkspace* workspace = &workspace_2uav_2hp;
    workspace->settings->warm_start = 1;
    struct vec n1 = computePlaneNormal(statePos, statePos2, plStPos, radius);
    struct vec n2 = computePlaneNormal(statePos2, statePos, plStPos, radius);
    c_float Ax_new[12] = {1, n1.x, 1, n1.y, 1, n1.z, 1,  n2.x, 1, n2.y, 1, n2.z};
    // // c_float Px_new[6] = {1, 1, 1, 1, 1, 1};
    
    c_int Ax_new_idx[12] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
    // // c_int Px_new_idx[6] = {0, 1, 2, 3, 4, 5};
    c_int Ax_new_n = 12;
    // // c_int Px_new_n = 6;
     
    c_float l_new[6] =  {F_d.x,	F_d.y,	F_d.z, -INFINITY, -INFINITY,};
    c_float u_new[6] =  {F_d.x,	F_d.y,	F_d.z, 0, 0,};

    osqp_update_A(workspace, Ax_new, Ax_new_idx, Ax_new_n);    
    // osqp_update_P(workspace, Px_new, Px_new_idx, Px_new_n);
    osqp_update_lower_bound(workspace, l_new);
    osqp_update_upper_bound(workspace, u_new);
    
    osqp_solve(workspace);

    if (workspace->info->status_val == OSQP_SOLVED) {
      desVirtInp.x = (workspace)->solution->x[0];
      desVirtInp.y = (workspace)->solution->x[1];
      desVirtInp.z = (workspace)->solution->x[2];
    } else {
      DEBUG_PRINT("QP: %s\n", workspace->info->status);
    }
    // printf("workspace_2uav_2hp status:   %s\n", (workspace)->info->status);
    // printf("tick: %f \n uavID: %d solution: %f %f %f %f %f %f\n", tick, self->value, (workspace)->solution->x[0], (workspace)->solution->x[1], (workspace)->solution->x[2], (workspace)->solution->x[3], (workspace)->solution->x[4], (workspace)->solution->x[5]);
    // printf("tick: %d \n uavID: %f solution: %f %f %f %f %f %f\n", tick, self->value, self->mu1.x, self->mu1.y, self->mu1.z, self->mu2.x, self->mu2.y, self->mu2.z);
    // if (tick % 1000 == 0) {

    //   DEBUG_PRINT("\n value: %f, desVirtInp: %f %f %f\n", (double) self->value, (double)(self->desVirtInp.x),(double)(self->desVirtInp.y),(double)(self->desVirtInp.z));
    //   DEBUG_PRINT("\n state 2 %f %f %f\n", (double)(state->position_neighbors[0].x), (double)(state->position_neighbors[0].y), (double)(state->position_neighbors[0].z));
    //   DEBUG_PRINT("\nn1: %f %f %f\n", (double) (self->n1.x), (double)(self->n1.y),(double)(self->n1.z));
    //   DEBUG_PRINT("\nn2: %f %f %f\n", (double) (self->n2.x), (double)(self->n2.y),(double)(self->n2.z));
    // }
    
    //------------------------------------------QP------------------------------//

    g_self.n1 = n1;
    g_self.n2 = n2;
    return desVirtInp;
}

void controllerLeePayloadReset(controllerLeePayload_t* self)
{
  self->i_error_pos = vzero();
  self->i_error_att = vzero();
  self->qi_prev = mkvec(0,0,-1);
  self->qidot_prev = vzero();
  self->acc_prev   = vzero();
  self->payload_vel_prev = vzero();
  self->qdi_prev = vzero();
}

void controllerLeePayloadInit(controllerLeePayload_t* self)
{
  // copy default values (bindings), or NOP (firmware)
  *self = g_self;

  controllerLeePayloadReset(self);
}

bool controllerLeePayloadTest(controllerLeePayload_t* self)
{
  return true;
}

void controllerLeePayload(controllerLeePayload_t* self, control_t *control, setpoint_t *setpoint,
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
    self->rpy_des = quat2rpy(setpoint_quat);
    desiredYaw = self->rpy_des.z;
  }

  // Position controller
  if (   setpoint->mode.x == modeAbs
      || setpoint->mode.y == modeAbs
      || setpoint->mode.z == modeAbs) {
    
    struct vec plPos_d = mkvec(setpoint->position.x, setpoint->position.y, setpoint->position.z);
    struct vec plVel_d = mkvec(setpoint->velocity.x, setpoint->velocity.y, setpoint->velocity.z);
    struct vec plAcc_d = mkvec(setpoint->acceleration.x, setpoint->acceleration.y, setpoint->acceleration.z + GRAVITY_MAGNITUDE);

    struct vec statePos = mkvec(state->position.x, state->position.y, state->position.z);
    struct vec stateVel = mkvec(state->velocity.x, state->velocity.y, state->velocity.z);
    struct vec plStPos = mkvec(state->payload_pos.x, state->payload_pos.y, state->payload_pos.z);
    struct vec plStVel = mkvec(state->payload_vel.x, state->payload_vel.y, state->payload_vel.z);

    // errors
    struct vec plpos_e = vclampnorm(vsub(plPos_d, plStPos), self->Kpos_P_limit);
    struct vec plvel_e = vclampnorm(vsub(plVel_d, plStVel), self->Kpos_D_limit);
    self->i_error_pos = vclampnorm(vadd(self->i_error_pos, vscl(dt, plpos_e)), self->Kpos_I_limit);

    self->plp_error = plpos_e;
    self->plv_error = plvel_e;

    struct vec F_d =vscl(self->mp ,vadd4(
      plAcc_d,
      veltmul(self->Kpos_P, plpos_e),
      veltmul(self->Kpos_D, plvel_e),
      veltmul(self->Kpos_I, self->i_error_pos)));

    self->desVirtInp = computeDesiredVirtualInput(state, F_d);

    //directional unit vector qi and angular velocity wi pointing from UAV to payload
    struct vec qi = vnormalize(vsub(plStPos, statePos)); 
  
    struct vec qidot = vdiv(vsub(plStVel, stateVel), vmag(vsub(plStPos, statePos)));
    struct vec wi = vcross(qi, qidot);
    struct mat33 qiqiT = vecmult(qi);
    struct vec virtualInp = mvmul(qiqiT,self->desVirtInp);

    // compute cable length (assuming taut cable)
    float l = vmag(vsub(plStPos, statePos));
    
    // Compute parallel component
    struct vec acc_ = plAcc_d; 
    struct vec u_parallel = vadd3(virtualInp, vscl(self->mass*l*vmag2(wi), qi), vscl(self->mass, mvmul(qiqiT, acc_)));
    
    // Compute Perpindicular Component
    struct vec qdi = vneg(vnormalize(self->desVirtInp));
    struct vec eq  = vcross(qdi, qi);
    struct mat33 skewqi = mcrossmat(qi);
    struct mat33 skewqi2 = mmul(skewqi,skewqi);

    struct vec qdidot = vzero(); //vdiv(vsub(qdi, self->qdi_prev), dt);
    self->qdi_prev = qdi;
    struct vec wdi = vcross(qdi, qdidot);
    struct vec ew = vadd(wi, mvmul(skewqi2, wdi));

    struct vec u_perpind = vsub(
      vscl(self->mass*l, mvmul(skewqi, vsub(vsub(vneg(veltmul(self->K_q, eq)), veltmul(self->K_w, ew)), 
      vscl(vdot(qi, wdi), qidot)))),
      vscl(self->mass, mvmul(skewqi2, acc_))
    );

    self->u_i = vadd(u_parallel, u_perpind);
    
    control->thrustSI = vmag(self->u_i);
    control->u_all[0] = self->u_i.x;
    control->u_all[1] = self->u_i.y;
    control->u_all[2] = self->u_i.z;


    self->thrustSI = control->thrustSI;
  //  Reset the accumulated error while on the ground
    if (control->thrustSI < 0.01f) {
      controllerLeePayloadReset(self);
    }
  
  self->q = mkquat(state->attitudeQuaternion.x, state->attitudeQuaternion.y, state->attitudeQuaternion.z, state->attitudeQuaternion.w);
  self->rpy = quat2rpy(self->q);
  self->R = quat2rotmat(self->q);

  // Compute Desired Rotation matrix
    struct vec Fd_ = self->u_i;
    struct vec xdes = vbasis(0);
    struct vec ydes = vbasis(1);
    struct vec zdes = vbasis(2);
   
    if (self->thrustSI > 0) {
      zdes = vnormalize(Fd_);
    } 
    struct vec xcdes = mkvec(cosf(desiredYaw), sinf(desiredYaw), 0); 
    struct vec zcrossx = vcross(zdes, xcdes);
    float normZX = vmag(zcrossx);

    if (normZX > 0) {
      ydes = vnormalize(zcrossx);
    } 
    xdes = vcross(ydes, zdes);
    
    self->R_des = mcolumns(xdes, ydes, zdes);

  } else {
    if (setpoint->mode.z == modeDisable) {
      if (setpoint->thrust < 1000) {
          control->controlMode = controlModeForceTorque;
          control->thrustSI  = 0;
          control->torque[0] = 0;
          control->torque[1] = 0;
          control->torque[2] = 0;
          controllerLeePayloadReset(self);
          return;
      }
    }
    // On CF2, thrust is mapped 65536 <==> 4 * 12 grams
    const float max_thrust = 70.0f / 1000.0f * 9.81f; // N
    control->thrustSI = setpoint->thrust / UINT16_MAX * max_thrust;

    self->qr = mkvec(
      radians(setpoint->attitude.roll),
      -radians(setpoint->attitude.pitch), // This is in the legacy coordinate system where pitch is inverted
      desiredYaw);
  }

  // Attitude controller

  // current rotation [R]
  self->q = mkquat(state->attitudeQuaternion.x, state->attitudeQuaternion.y, state->attitudeQuaternion.z, state->attitudeQuaternion.w);
  self->rpy = quat2rpy(self->q);
  self->R = quat2rotmat(self->q);

  // desired rotation [Rdes]
  struct quat q_des = mat2quat(self->R_des);
  self->rpy_des = quat2rpy(q_des);

  // rotation error
  struct mat33 eRM = msub(mmul(mtranspose(self->R_des), self->R), mmul(mtranspose(self->R), self->R_des));

  struct vec eR = vscl(0.5f, mkvec(eRM.m[2][1], eRM.m[0][2], eRM.m[1][0]));

  // angular velocity
  self->omega = mkvec(
    radians(sensors->gyro.x),
    radians(sensors->gyro.y),
    radians(sensors->gyro.z));

  // Compute desired omega
  struct vec xdes = mcolumn(self->R_des, 0);
  struct vec ydes = mcolumn(self->R_des, 1);
  struct vec zdes = mcolumn(self->R_des, 2);
  struct vec hw = vzero();
  // Desired Jerk and snap for now are zeros vector
  struct vec desJerk = mkvec(setpoint->jerk.x, setpoint->jerk.y, setpoint->jerk.z);

  if (control->thrustSI != 0) {
    struct vec tmp = vsub(desJerk, vscl(vdot(zdes, desJerk), zdes));
    hw = vscl(self->mass/control->thrustSI, tmp);
  }

  struct vec z_w = mkvec(0, 0, 1);
  float desiredYawRate = radians(setpoint->attitudeRate.yaw) * vdot(zdes, z_w);
  struct vec omega_des = mkvec(-vdot(hw,ydes), vdot(hw,xdes), desiredYawRate);
  
  self->omega_r = mvmul(mmul(mtranspose(self->R), self->R_des), omega_des);

  struct vec omega_error = vsub(self->omega, self->omega_r);
  
  // Integral part on angle
  self->i_error_att = vadd(self->i_error_att, vscl(dt, eR));

  // compute moments
  // M = -kR eR - kw ew + w x Jw - J(w x wr)
  self->u = vadd4(
    vneg(veltmul(self->KR, eR)),
    vneg(veltmul(self->Komega, omega_error)),
    vneg(veltmul(self->KI, self->i_error_att)),
    vcross(self->omega, veltmul(self->J, self->omega)));

  // if (enableNN > 1) {
  //   u = vsub(u, tau_a);
  // }

  control->controlMode = controlModeForceTorque;
  control->torque[0] = self->u.x;
  control->torque[1] = self->u.y;
  control->torque[2] = self->u.z;

  // ticks = usecTimestamp() - startTime;
}

#ifdef CRAZYFLIE_FW

#include "param.h"
#include "log.h"

void controllerLeePayloadFirmwareInit(void)
{
  controllerLeePayloadInit(&g_self);
}

bool controllerLeePayloadFirmwareTest(void) 
{
  return true;
}

void controllerLeePayloadFirmware(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
  controllerLeePayload(&g_self, control, setpoint, sensors, state, tick);  
}

PARAM_GROUP_START(ctrlLeeP)
PARAM_ADD(PARAM_FLOAT, KR_x, &g_self.KR.x)
PARAM_ADD(PARAM_FLOAT, KR_y, &g_self.KR.y)
PARAM_ADD(PARAM_FLOAT, KR_z, &g_self.KR.z)
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

// Attitude P
PARAM_ADD(PARAM_FLOAT, KRx, &g_self.KR.x)
PARAM_ADD(PARAM_FLOAT, KRy, &g_self.KR.y)
PARAM_ADD(PARAM_FLOAT, KRz, &g_self.KR.z)

// Attitude D
PARAM_ADD(PARAM_FLOAT, Komx, &g_self.Komega.x)
PARAM_ADD(PARAM_FLOAT, Komy, &g_self.Komega.y)
PARAM_ADD(PARAM_FLOAT, Komz, &g_self.Komega.z)

// Attitude I
PARAM_ADD(PARAM_FLOAT, KI_x, &g_self.KI.x)
PARAM_ADD(PARAM_FLOAT, KI_y, &g_self.KI.y)
PARAM_ADD(PARAM_FLOAT, KI_z, &g_self.KI.z)

// Cable P
PARAM_ADD(PARAM_FLOAT, Kqx, &g_self.K_q.x)
PARAM_ADD(PARAM_FLOAT, Kqy, &g_self.K_q.y)
PARAM_ADD(PARAM_FLOAT, Kqz, &g_self.K_q.z)

// Cable D
PARAM_ADD(PARAM_FLOAT, Kwx, &g_self.K_w.x)
PARAM_ADD(PARAM_FLOAT, Kwy, &g_self.K_w.y)
PARAM_ADD(PARAM_FLOAT, Kwz, &g_self.K_w.z)

PARAM_ADD(PARAM_FLOAT, mass, &g_self.mass)
PARAM_ADD(PARAM_FLOAT, massP, &g_self.mp)

PARAM_ADD(PARAM_FLOAT, radius, &g_self.radius)

PARAM_GROUP_STOP(ctrlLeeP)


LOG_GROUP_START(ctrlLeeP)

LOG_ADD(LOG_FLOAT, thrustSI, &g_self.thrustSI)
LOG_ADD(LOG_FLOAT, torquex, &g_self.u.x)
LOG_ADD(LOG_FLOAT, torquey, &g_self.u.y)
LOG_ADD(LOG_FLOAT, torquez, &g_self.u.z)

// current angles
LOG_ADD(LOG_FLOAT, rpyx, &g_self.rpy.x)
LOG_ADD(LOG_FLOAT, rpyy, &g_self.rpy.y)
LOG_ADD(LOG_FLOAT, rpyz, &g_self.rpy.z)

// desired angles
LOG_ADD(LOG_FLOAT, rpydx, &g_self.rpy_des.x)
LOG_ADD(LOG_FLOAT, rpydy, &g_self.rpy_des.y)
LOG_ADD(LOG_FLOAT, rpydz, &g_self.rpy_des.z)

// omega
LOG_ADD(LOG_FLOAT, omegax, &g_self.omega.x)
LOG_ADD(LOG_FLOAT, omegay, &g_self.omega.y)
LOG_ADD(LOG_FLOAT, omegaz, &g_self.omega.z)

// omega_r
LOG_ADD(LOG_FLOAT, omegarx, &g_self.omega_r.x)
LOG_ADD(LOG_FLOAT, omegary, &g_self.omega_r.y)
LOG_ADD(LOG_FLOAT, omegarz, &g_self.omega_r.z)

LOG_ADD(LOG_FLOAT, ux, &g_self.u_i.x)
LOG_ADD(LOG_FLOAT, uy, &g_self.u_i.y)
LOG_ADD(LOG_FLOAT, uz, &g_self.u_i.z)

// hyperplanes
LOG_ADD(LOG_FLOAT, n1x, &g_self.n1.x)
LOG_ADD(LOG_FLOAT, n1y, &g_self.n1.y)
LOG_ADD(LOG_FLOAT, n1z, &g_self.n1.z)

// computed virtual input
LOG_ADD(LOG_FLOAT, desVirtInpx, &g_self.desVirtInp.x)
LOG_ADD(LOG_FLOAT, desVirtInpy, &g_self.desVirtInp.y)
LOG_ADD(LOG_FLOAT, desVirtInpz, &g_self.desVirtInp.z)

// LOG_ADD(LOG_UINT32, ticks, &ticks)

LOG_GROUP_STOP(ctrlLeeP)

#endif // CRAZYFLIE_FW defined


               


          
