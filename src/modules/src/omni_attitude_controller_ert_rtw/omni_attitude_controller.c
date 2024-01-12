/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: omni_attitude_controller.c
 *
 * Code generated for Simulink model 'omni_attitude_controller'.
 *
 * Model version                  : 8.24
 * Simulink Coder version         : 23.2 (R2023b) 01-Aug-2023
 * C/C++ source code generated on : Tue Dec 26 12:13:00 2023
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objective: Execution efficiency
 * Validation result: Not run
 */

#include "omni_attitude_controller.h"
#include <math.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include <string.h>
#include "math3d.h"

omni_attitude_controller_Gain Omni_gains = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};

/* External inputs (root inport signals with default storage) */
ExtU_omni_attitude_controller_T omni_attitude_controller_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_omni_attitude_controller_T omni_attitude_controller_Y;

static struct mat33 CRAZYFLIE_INERTIA_I =
    {{{16.6e-6f, 0.83e-6f, 1.8e-6f},
      {0.83e-6f, 16.6e-6f, 0.72e-6f},
      {1.8e-6f, 0.72e-6f, 29.3e-6f}}};

static void quatmultiply(const real32_T q[4], const real32_T r[4], real32_T qout[4])
{
  qout[0] = ((q[0] * r[0] - q[1] * r[1]) - q[2] * r[2]) - q[3] * r[3];
  qout[1] = (q[0] * r[1] + r[0] * q[1]) + (q[2] * r[3] - q[3] * r[2]);
  qout[2] = (q[0] * r[2] + r[0] * q[2]) + (q[3] * r[1] - q[1] * r[3]);
  qout[3] = (q[0] * r[3] + r[0] * q[3]) + (q[1] * r[2] - q[2] * r[1]);
}

// make sure the quaternion is normalized
void quatToDCM(float *quat, struct mat33 *RotM)
{
  float q0 = *quat;
  float q1 = *(quat+1);
  float q2 = *(quat+2);
  float q3 = *(quat+3);

  RotM->m[0][0] = q0*q0+q1*q1-q2*q2-q3*q3;
  RotM->m[0][1] = 2.0F*(q1*q2-q0*q3);
  RotM->m[0][2] = 2.0F*(q1*q3+q0*q2);

  RotM->m[1][0] = 2.0F*(q1*q2+q0*q3);
  RotM->m[1][1] = q0*q0-q1*q1+q2*q2-q3*q3;
  RotM->m[1][2] = 2.0F*(q2*q3-q0*q1);

  RotM->m[2][0] = 2.0F*(q1*q3-q0*q2);
  RotM->m[2][1] = 2.0F*(q2*q3+q0*q1);
  RotM->m[2][2] = q0*q0-q1*q1-q2*q2+q3*q3;
}

void omni_attitude_controller_DoAttitudeLoop(void)
{
  // quaternion command to DCM (in i-frame)
  // https://www.mathworks.com/help/aeroblks/quaternionstodirectioncosinematrix.html
  struct mat33 R_r = mzero();
  quatToDCM((float*)&omni_attitude_controller_U.qw_r, &R_r);

  // rotate quaternion fbk to i-frame 
  static const real32_T q45[4] = { 0.707106769F, 0.0F, 0.0F, 0.707106769F };
  static const real32_T q45inv[4] = { -0.707106769F, 0.0F, 0.0F, 0.707106769F };
  real32_T temp[4] = {0.0, 0.0, 0.0, 0.0};
  real32_T q_Qi[4] = {0.0, 0.0, 0.0, 0.0};
  real32_T q_i[4] = {0.0, 0.0, 0.0, 0.0};
  q_Qi[0] = omni_attitude_controller_U.qw_IMU;
  q_Qi[1] = omni_attitude_controller_U.qx_IMU;
  q_Qi[2] = omni_attitude_controller_U.qy_IMU;
  q_Qi[3] = omni_attitude_controller_U.qz_IMU;
  quatmultiply(q_Qi, q45inv, temp);
  quatmultiply(q45, temp, q_i);

  // quaternion fbk to DCM
  struct mat33 R = mzero();
  quatToDCM(q_i, &R);

  // attitude controller
  // 0.5*(R_r_T * R - R_T * R_r), eR = [m32, m12, m21]';
  struct mat33 R_r_T =  mtranspose(R_r);
  struct mat33 R_T =  mtranspose(R);
  struct mat33 E = mscl(0.5f, msub(mmul(R_r_T, R) , mmul(R_T, R_r)));
  
  // eR -> xd - x
  struct vec eR = vzero();
  eR.x = -E.m[2][1];
  eR.y = -E.m[0][2];
  eR.z = -E.m[1][0];

  omni_attitude_controller_Y.eRx = eR.x;
  omni_attitude_controller_Y.eRy = eR.y;
  omni_attitude_controller_Y.eRz = eR.z;

  omni_attitude_controller_Y.wx_r = (real32_T)Omni_gains.krx * eR.x; 
  omni_attitude_controller_Y.wy_r = (real32_T)Omni_gains.kry * eR.y; 
  // omni_attitude_controller_Y.wz_r = (real32_T)Omni_gains.krz * eR.z; 
  omni_attitude_controller_Y.wz_r = omni_attitude_controller_Y.wx_r * tanf(omni_attitude_controller_U.wx_r);
}

void omni_attitude_controller_DoAttitudeRateLoop(float dt)
{
  /* Be aware of the sign of the signals */
  // eW = Omega - R_T * (R_r * agvr);
  // struct vec eW = vsub(Omega, mvmul(R_T, mvmul(R_r, agvr)));
  struct vec Omega = vzero();
  Omega.x = -omni_attitude_controller_U.gyro_y;
  Omega.y = omni_attitude_controller_U.gyro_x;
  Omega.z = omni_attitude_controller_U.gyro_z;

  // eW = agvr - w
  struct vec eW = vzero();
  eW.x = (float)omni_attitude_controller_Y.wx_r - Omega.x;
  eW.y = (float)omni_attitude_controller_Y.wy_r - Omega.y;
  eW.z = (float)omni_attitude_controller_Y.wz_r - Omega.z;

  omni_attitude_controller_Y.eWx = eW.x;
  omni_attitude_controller_Y.eWy = eW.y;
  omni_attitude_controller_Y.eWz = eW.z;

    // Thrust Clamper
  float Thrust;
  if (omni_attitude_controller_U.thrust >
      omni_attitude_controller_P.Saturation_UpperSat) {
    Thrust = omni_attitude_controller_P.Saturation_UpperSat;
  } else if (omni_attitude_controller_U.thrust <
             omni_attitude_controller_P.Saturation_LowerSat) {
    Thrust = omni_attitude_controller_P.Saturation_LowerSat;
  } else {
    Thrust = omni_attitude_controller_U.thrust;
  }

  // eiInt
  if( omni_attitude_controller_Y.IsClamped == 0 && Thrust > 0.000898f )
  {
    omni_attitude_controller_Y.eixInt = omni_attitude_controller_Y.eixInt + eW.x * dt;
    omni_attitude_controller_Y.eiyInt = omni_attitude_controller_Y.eiyInt + eW.y * dt;
    omni_attitude_controller_Y.eizInt = omni_attitude_controller_Y.eizInt + eW.z * dt;
  }

  struct vec eiInt = vzero();
  eiInt.x = omni_attitude_controller_Y.eixInt;
  eiInt.y = omni_attitude_controller_Y.eiyInt;
  eiInt.z = omni_attitude_controller_Y.eizInt;

  // PID Controller M = J * ( Kw*eW + Ki * eiInt ) with unit Nm
  // volatile struct vec KW = vzero();
  // KW.x = omni_attitude_controller_P.Kwx;
  // KW.y = omni_attitude_controller_P.Kwy;
  // KW.z = omni_attitude_controller_P.Kwz;

  // volatile struct vec Ki = vzero();
  // Ki.x = omni_attitude_controller_P.Kix;
  // Ki.y = omni_attitude_controller_P.Kiy;
  // Ki.z = omni_attitude_controller_P.Kiz;

  struct vec uW = vzero();
  struct vec ui = vzero();
  struct vec controlTorque = vzero();

  uW.x = Omni_gains.kwx * eW.x;
  uW.y = Omni_gains.kwy * eW.y;
  uW.z = Omni_gains.kwz * eW.z;

  ui.x = Omni_gains.kix * eiInt.x;
  ui.y = Omni_gains.kiy * eiInt.y;
  ui.z = Omni_gains.kiz * eiInt.z;

  controlTorque = mvmul(CRAZYFLIE_INERTIA_I, vadd(ui,uW));

  float dJzy = CRAZYFLIE_INERTIA_I.m[2][2] - CRAZYFLIE_INERTIA_I.m[1][1];
  float dJzx = CRAZYFLIE_INERTIA_I.m[2][2] - CRAZYFLIE_INERTIA_I.m[0][0];
  omni_attitude_controller_Y.Tau_x = controlTorque.x + Omni_gains.kffx * dJzy * omni_attitude_controller_Y.wy_r * Omega.z;
  omni_attitude_controller_Y.Tau_y = controlTorque.y - Omni_gains.kffy * dJzx * omni_attitude_controller_Y.wx_r * Omega.z;
  omni_attitude_controller_Y.Tau_z = controlTorque.z;
  // omni_attitude_controller_Y.Tau_z = controlTorque.x * tanf(omni_attitude_controller_U.wx_r);

  // power distribution and turn Nm into N
  const float arm = 0.707106781f * 0.046f;
  const float rollPart = 0.25f / arm * omni_attitude_controller_Y.Tau_x;
  const float pitchPart = 0.25f / arm * omni_attitude_controller_Y.Tau_y;
  const float thrustPart = 0.25f * Thrust; // N (per rotor)
  const float yawPart = 0.25f * omni_attitude_controller_Y.Tau_z / 0.005964552f;

  omni_attitude_controller_Y.rollPart = rollPart;
  omni_attitude_controller_Y.pitchPart = pitchPart;
  omni_attitude_controller_Y.thrustPart = thrustPart;
  omni_attitude_controller_Y.yawPart = yawPart;

  // corresponding to i-frame Body coordinate, t_mi 's Unit is Newton
  omni_attitude_controller_Y.t_m1 = thrustPart + rollPart - pitchPart - yawPart;
  omni_attitude_controller_Y.t_m2 = thrustPart - rollPart - pitchPart + yawPart;
  omni_attitude_controller_Y.t_m3 = thrustPart - rollPart + pitchPart - yawPart;
  omni_attitude_controller_Y.t_m4 = thrustPart + rollPart + pitchPart + yawPart;

  omni_attitude_controller_Y.IsClamped = 0;
  if (omni_attitude_controller_Y.t_m1 < 0.0f) 
  {
    omni_attitude_controller_Y.t_m1 = 0.0f;
    omni_attitude_controller_Y.IsClamped = 1;
  } else if (omni_attitude_controller_Y.t_m1 > 0.1472f) {
    omni_attitude_controller_Y.t_m1 = 0.1472f;
    omni_attitude_controller_Y.IsClamped = 1;
  }

  if (omni_attitude_controller_Y.t_m2 < 0.0f) 
  {
    omni_attitude_controller_Y.t_m2 = 0.0f;
    omni_attitude_controller_Y.IsClamped = 1;
  } else if (omni_attitude_controller_Y.t_m2 > 0.1472f) {
    omni_attitude_controller_Y.t_m2 = 0.1472f;
    omni_attitude_controller_Y.IsClamped = 1;
  }

  if (omni_attitude_controller_Y.t_m3 < 0.0f) 
  {
    omni_attitude_controller_Y.t_m3 = 0.0f;
    omni_attitude_controller_Y.IsClamped = 1;
  } else if (omni_attitude_controller_Y.t_m3 > 0.1472f) {
    omni_attitude_controller_Y.t_m3 = 0.1472f;
    omni_attitude_controller_Y.IsClamped = 1;
  }

  if (omni_attitude_controller_Y.t_m4 < 0.0f) 
  {
    omni_attitude_controller_Y.t_m4 = 0.0f;
    omni_attitude_controller_Y.IsClamped = 1;
  } else if (omni_attitude_controller_Y.t_m4 > 0.1472f) {
    omni_attitude_controller_Y.t_m4 = 0.1472f;
    omni_attitude_controller_Y.IsClamped = 1;
  }

  // Turn Newton into percentage and count
  omni_attitude_controller_Y.m1 = omni_attitude_controller_Y.t_m1 / 0.1472f * 65535;
  omni_attitude_controller_Y.m2 = omni_attitude_controller_Y.t_m2 / 0.1472f * 65535;
  omni_attitude_controller_Y.m3 = omni_attitude_controller_Y.t_m3 / 0.1472f * 65535;
  omni_attitude_controller_Y.m4 = omni_attitude_controller_Y.t_m4 / 0.1472f * 65535;
}

void omni_attitude_controller_step_hand(void)
{
  return;
}

/* Model step function */
void omni_attitude_controller_step(void)
{
  int32_T E_tmp_tmp;
  int32_T i;
  int32_T i_0;
  real32_T E[9];
  real32_T E_tmp[9];
  real32_T rtb_VectorConcatenate[9];
  real32_T rtb_VectorConcatenate_f[9];
  real32_T rtb_VectorConcatenate_o[9];
  real32_T rtb_M[3];
  real32_T rtb_VectorConcatenate_f_0[3];
  real32_T fty;
  real32_T ftz;
  real32_T rtb_Product1_a;
  real32_T rtb_Product2_iz;
  real32_T rtb_Product3_k;
  real32_T rtb_Product_i;
  real32_T rtb_VectorConcatenate_tmp;
  real32_T rtb_VectorConcatenate_tmp_0;
  real32_T rtb_VectorConcatenate_tmp_1;
  real32_T rtb_VectorConcatenate_tmp_2;
  real32_T rtb_VectorConcatenate_tmp_3;
  real32_T rtb_motor_com_idx_3;

  /* Sqrt: '<S16>/sqrt' incorporates:
   *  Inport: '<Root>/qw_r'
   *  Inport: '<Root>/qx_r'
   *  Inport: '<Root>/qy_r'
   *  Inport: '<Root>/qz_r'
   *  Product: '<S17>/Product'
   *  Product: '<S17>/Product1'
   *  Product: '<S17>/Product2'
   *  Product: '<S17>/Product3'
   *  Sum: '<S17>/Sum'
   */
  rtb_Product2_iz = sqrtf(((omni_attitude_controller_U.qw_r *
    omni_attitude_controller_U.qw_r + omni_attitude_controller_U.qx_r *
    omni_attitude_controller_U.qx_r) + omni_attitude_controller_U.qy_r *
    omni_attitude_controller_U.qy_r) + omni_attitude_controller_U.qz_r *
    omni_attitude_controller_U.qz_r);

  /* Product: '<S15>/Product' incorporates:
   *  Inport: '<Root>/qw_r'
   */
  rtb_Product1_a = omni_attitude_controller_U.qw_r / rtb_Product2_iz;

  /* Product: '<S15>/Product1' incorporates:
   *  Inport: '<Root>/qx_r'
   */
  rtb_Product_i = omni_attitude_controller_U.qx_r / rtb_Product2_iz;

  /* Product: '<S15>/Product2' incorporates:
   *  Inport: '<Root>/qy_r'
   */
  rtb_Product3_k = omni_attitude_controller_U.qy_r / rtb_Product2_iz;

  /* Product: '<S15>/Product3' incorporates:
   *  Inport: '<Root>/qz_r'
   */
  rtb_Product2_iz = omni_attitude_controller_U.qz_r / rtb_Product2_iz;

  /* Product: '<S5>/Product3' incorporates:
   *  Product: '<S9>/Product3'
   */
  fty = rtb_Product1_a * rtb_Product1_a;

  /* Product: '<S5>/Product2' incorporates:
   *  Product: '<S9>/Product2'
   */
  ftz = rtb_Product_i * rtb_Product_i;

  /* Product: '<S5>/Product1' incorporates:
   *  Product: '<S13>/Product1'
   *  Product: '<S9>/Product1'
   */
  rtb_motor_com_idx_3 = rtb_Product3_k * rtb_Product3_k;

  /* Product: '<S5>/Product' incorporates:
   *  Product: '<S13>/Product'
   *  Product: '<S9>/Product'
   */
  rtb_VectorConcatenate_tmp_1 = rtb_Product2_iz * rtb_Product2_iz;

  /* Sum: '<S5>/Sum' incorporates:
   *  Product: '<S5>/Product'
   *  Product: '<S5>/Product1'
   *  Product: '<S5>/Product2'
   *  Product: '<S5>/Product3'
   */
  rtb_VectorConcatenate[0] = ((fty + ftz) - rtb_motor_com_idx_3) -
    rtb_VectorConcatenate_tmp_1;

  /* Product: '<S8>/Product3' incorporates:
   *  Product: '<S6>/Product3'
   */
  rtb_VectorConcatenate_tmp = rtb_Product2_iz * rtb_Product1_a;

  /* Product: '<S8>/Product2' incorporates:
   *  Product: '<S6>/Product2'
   */
  rtb_VectorConcatenate_tmp_0 = rtb_Product_i * rtb_Product3_k;

  /* Gain: '<S8>/Gain' incorporates:
   *  Product: '<S8>/Product2'
   *  Product: '<S8>/Product3'
   *  Sum: '<S8>/Sum'
   */
  rtb_VectorConcatenate[1] = (rtb_VectorConcatenate_tmp_0 -
    rtb_VectorConcatenate_tmp) * omni_attitude_controller_P.Gain_Gain;

  /* Product: '<S11>/Product2' incorporates:
   *  Product: '<S7>/Product2'
   */
  rtb_VectorConcatenate_tmp_2 = rtb_Product_i * rtb_Product2_iz;

  /* Product: '<S11>/Product1' incorporates:
   *  Product: '<S7>/Product1'
   */
  rtb_VectorConcatenate_tmp_3 = rtb_Product1_a * rtb_Product3_k;

  /* Gain: '<S11>/Gain' incorporates:
   *  Product: '<S11>/Product1'
   *  Product: '<S11>/Product2'
   *  Sum: '<S11>/Sum'
   */
  rtb_VectorConcatenate[2] = (rtb_VectorConcatenate_tmp_3 +
    rtb_VectorConcatenate_tmp_2) * omni_attitude_controller_P.Gain_Gain_l;

  /* Gain: '<S6>/Gain' incorporates:
   *  Sum: '<S6>/Sum'
   */
  rtb_VectorConcatenate[3] = (rtb_VectorConcatenate_tmp +
    rtb_VectorConcatenate_tmp_0) * omni_attitude_controller_P.Gain_Gain_f;

  /* Sum: '<S9>/Sum' incorporates:
   *  Sum: '<S13>/Sum'
   */
  fty -= ftz;
  rtb_VectorConcatenate[4] = (fty + rtb_motor_com_idx_3) -
    rtb_VectorConcatenate_tmp_1;

  /* Product: '<S12>/Product1' incorporates:
   *  Product: '<S10>/Product1'
   */
  ftz = rtb_Product1_a * rtb_Product_i;

  /* Product: '<S12>/Product2' incorporates:
   *  Product: '<S10>/Product2'
   */
  rtb_VectorConcatenate_tmp = rtb_Product3_k * rtb_Product2_iz;

  /* Gain: '<S12>/Gain' incorporates:
   *  Product: '<S12>/Product1'
   *  Product: '<S12>/Product2'
   *  Sum: '<S12>/Sum'
   */
  rtb_VectorConcatenate[5] = (rtb_VectorConcatenate_tmp - ftz) *
    omni_attitude_controller_P.Gain_Gain_d;

  /* Gain: '<S7>/Gain' incorporates:
   *  Sum: '<S7>/Sum'
   */
  rtb_VectorConcatenate[6] = (rtb_VectorConcatenate_tmp_2 -
    rtb_VectorConcatenate_tmp_3) * omni_attitude_controller_P.Gain_Gain_b;

  /* Gain: '<S10>/Gain' incorporates:
   *  Sum: '<S10>/Sum'
   */
  rtb_VectorConcatenate[7] = (ftz + rtb_VectorConcatenate_tmp) *
    omni_attitude_controller_P.Gain_Gain_c;

  /* Sum: '<S13>/Sum' */
  rtb_VectorConcatenate[8] = (fty - rtb_motor_com_idx_3) +
    rtb_VectorConcatenate_tmp_1;

  /* Sqrt: '<S29>/sqrt' incorporates:
   *  Inport: '<Root>/qw_IMU'
   *  Inport: '<Root>/qx_IMU'
   *  Inport: '<Root>/qy_IMU'
   *  Inport: '<Root>/qz_IMU'
   *  Product: '<S30>/Product'
   *  Product: '<S30>/Product1'
   *  Product: '<S30>/Product2'
   *  Product: '<S30>/Product3'
   *  Sum: '<S30>/Sum'
   */
  rtb_Product3_k = sqrtf(((omni_attitude_controller_U.qw_IMU *
    omni_attitude_controller_U.qw_IMU + omni_attitude_controller_U.qx_IMU *
    omni_attitude_controller_U.qx_IMU) + omni_attitude_controller_U.qy_IMU *
    omni_attitude_controller_U.qy_IMU) + omni_attitude_controller_U.qz_IMU *
    omni_attitude_controller_U.qz_IMU);

  /* Product: '<S28>/Product' incorporates:
   *  Inport: '<Root>/qw_IMU'
   */
  rtb_Product_i = omni_attitude_controller_U.qw_IMU / rtb_Product3_k;

  /* Product: '<S28>/Product1' incorporates:
   *  Inport: '<Root>/qx_IMU'
   */
  rtb_Product1_a = omni_attitude_controller_U.qx_IMU / rtb_Product3_k;

  /* Product: '<S28>/Product2' incorporates:
   *  Inport: '<Root>/qy_IMU'
   */
  rtb_Product2_iz = omni_attitude_controller_U.qy_IMU / rtb_Product3_k;

  /* Product: '<S28>/Product3' incorporates:
   *  Inport: '<Root>/qz_IMU'
   */
  rtb_Product3_k = omni_attitude_controller_U.qz_IMU / rtb_Product3_k;

  /* Product: '<S18>/Product3' incorporates:
   *  Product: '<S22>/Product3'
   */
  fty = rtb_Product_i * rtb_Product_i;

  /* Product: '<S18>/Product2' incorporates:
   *  Product: '<S22>/Product2'
   */
  ftz = rtb_Product1_a * rtb_Product1_a;

  /* Product: '<S18>/Product1' incorporates:
   *  Product: '<S22>/Product1'
   *  Product: '<S26>/Product1'
   */
  rtb_motor_com_idx_3 = rtb_Product2_iz * rtb_Product2_iz;

  /* Product: '<S18>/Product' incorporates:
   *  Product: '<S22>/Product'
   *  Product: '<S26>/Product'
   */
  rtb_VectorConcatenate_tmp_1 = rtb_Product3_k * rtb_Product3_k;

  /* Sum: '<S18>/Sum' incorporates:
   *  Product: '<S18>/Product'
   *  Product: '<S18>/Product1'
   *  Product: '<S18>/Product2'
   *  Product: '<S18>/Product3'
   */
  rtb_VectorConcatenate_o[0] = ((fty + ftz) - rtb_motor_com_idx_3) -
    rtb_VectorConcatenate_tmp_1;

  /* Product: '<S21>/Product3' incorporates:
   *  Product: '<S19>/Product3'
   */
  rtb_VectorConcatenate_tmp = rtb_Product3_k * rtb_Product_i;

  /* Product: '<S21>/Product2' incorporates:
   *  Product: '<S19>/Product2'
   */
  rtb_VectorConcatenate_tmp_0 = rtb_Product1_a * rtb_Product2_iz;

  /* Gain: '<S21>/Gain' incorporates:
   *  Product: '<S21>/Product2'
   *  Product: '<S21>/Product3'
   *  Sum: '<S21>/Sum'
   */
  rtb_VectorConcatenate_o[1] = (rtb_VectorConcatenate_tmp_0 -
    rtb_VectorConcatenate_tmp) * omni_attitude_controller_P.Gain_Gain_p;

  /* Product: '<S24>/Product2' incorporates:
   *  Product: '<S20>/Product2'
   */
  rtb_VectorConcatenate_tmp_2 = rtb_Product1_a * rtb_Product3_k;

  /* Product: '<S24>/Product1' incorporates:
   *  Product: '<S20>/Product1'
   */
  rtb_VectorConcatenate_tmp_3 = rtb_Product_i * rtb_Product2_iz;

  /* Gain: '<S24>/Gain' incorporates:
   *  Product: '<S24>/Product1'
   *  Product: '<S24>/Product2'
   *  Sum: '<S24>/Sum'
   */
  rtb_VectorConcatenate_o[2] = (rtb_VectorConcatenate_tmp_3 +
    rtb_VectorConcatenate_tmp_2) * omni_attitude_controller_P.Gain_Gain_i;

  /* Gain: '<S19>/Gain' incorporates:
   *  Sum: '<S19>/Sum'
   */
  rtb_VectorConcatenate_o[3] = (rtb_VectorConcatenate_tmp +
    rtb_VectorConcatenate_tmp_0) * omni_attitude_controller_P.Gain_Gain_fl;

  /* Sum: '<S22>/Sum' incorporates:
   *  Sum: '<S26>/Sum'
   */
  fty -= ftz;
  rtb_VectorConcatenate_o[4] = (fty + rtb_motor_com_idx_3) -
    rtb_VectorConcatenate_tmp_1;

  /* Product: '<S25>/Product1' incorporates:
   *  Product: '<S23>/Product1'
   */
  ftz = rtb_Product_i * rtb_Product1_a;

  /* Product: '<S25>/Product2' incorporates:
   *  Product: '<S23>/Product2'
   */
  rtb_VectorConcatenate_tmp = rtb_Product2_iz * rtb_Product3_k;

  /* Gain: '<S25>/Gain' incorporates:
   *  Product: '<S25>/Product1'
   *  Product: '<S25>/Product2'
   *  Sum: '<S25>/Sum'
   */
  rtb_VectorConcatenate_o[5] = (rtb_VectorConcatenate_tmp - ftz) *
    omni_attitude_controller_P.Gain_Gain_a;

  /* Gain: '<S20>/Gain' incorporates:
   *  Sum: '<S20>/Sum'
   */
  rtb_VectorConcatenate_o[6] = (rtb_VectorConcatenate_tmp_2 -
    rtb_VectorConcatenate_tmp_3) * omni_attitude_controller_P.Gain_Gain_h;

  /* Gain: '<S23>/Gain' incorporates:
   *  Sum: '<S23>/Sum'
   */
  rtb_VectorConcatenate_o[7] = (ftz + rtb_VectorConcatenate_tmp) *
    omni_attitude_controller_P.Gain_Gain_ao;

  /* Sum: '<S26>/Sum' */
  rtb_VectorConcatenate_o[8] = (fty - rtb_motor_com_idx_3) +
    rtb_VectorConcatenate_tmp_1;

  /* MATLAB Function: '<Root>/LLATC' incorporates:
   *  Concatenate: '<S14>/Vector Concatenate'
   *  Concatenate: '<S27>/Vector Concatenate'
   *  Inport: '<Root>/gyro_x'
   *  Inport: '<Root>/gyro_y'
   *  Inport: '<Root>/gyro_z'
   */
  for (i_0 = 0; i_0 < 3; i_0++) {
    rtb_Product3_k = rtb_VectorConcatenate_o[3 * i_0 + 1];
    rtb_Product1_a = rtb_VectorConcatenate_o[3 * i_0];
    rtb_Product_i = rtb_VectorConcatenate_o[3 * i_0 + 2];
    for (i = 0; i < 3; i++) {
      E_tmp_tmp = 3 * i + i_0;
      E_tmp[i + 3 * i_0] = rtb_VectorConcatenate[E_tmp_tmp];
      rtb_VectorConcatenate_f[E_tmp_tmp] = (rtb_VectorConcatenate[3 * i + 1] *
        rtb_Product3_k + rtb_VectorConcatenate[3 * i] * rtb_Product1_a) +
        rtb_VectorConcatenate[3 * i + 2] * rtb_Product_i;
    }
  }

  for (i_0 = 0; i_0 < 3; i_0++) {
    rtb_Product3_k = rtb_VectorConcatenate_o[3 * i_0 + 1];
    rtb_Product1_a = rtb_VectorConcatenate_o[3 * i_0];
    rtb_Product_i = rtb_VectorConcatenate_o[3 * i_0 + 2];
    for (i = 0; i < 3; i++) {
      rtb_VectorConcatenate[i + 3 * i_0] = (E_tmp[i + 3] * rtb_Product3_k +
        rtb_Product1_a * E_tmp[i]) + E_tmp[i + 6] * rtb_Product_i;
    }
  }

  for (i_0 = 0; i_0 < 9; i_0++) {
    E[i_0] = (rtb_VectorConcatenate_f[i_0] - rtb_VectorConcatenate[i_0]) * 0.5F;
  }

  rtb_Product2_iz = E[5];
  fty = E[6];
  ftz = E[1];
  for (i_0 = 0; i_0 < 3; i_0++) {
    rtb_VectorConcatenate_f_0[i_0] = (rtb_VectorConcatenate_o[i_0 + 3] *
      omni_attitude_controller_U.gyro_y + rtb_VectorConcatenate_o[i_0] *
      omni_attitude_controller_U.gyro_x) + rtb_VectorConcatenate_o[i_0 + 6] *
      omni_attitude_controller_U.gyro_z;
  }

  /* SignalConversion generated from: '<S1>/ SFunction ' incorporates:
   *  Inport: '<Root>/wx_r'
   *  Inport: '<Root>/wy_r'
   *  Inport: '<Root>/wz_r'
   *  MATLAB Function: '<Root>/LLATC'
   */
  rtb_M[0] = omni_attitude_controller_U.wx_r;
  rtb_M[1] = omni_attitude_controller_U.wy_r;
  rtb_M[2] = omni_attitude_controller_U.wz_r;

  /* MATLAB Function: '<Root>/LLATC' */
  rtb_Product3_k = rtb_VectorConcatenate_f_0[1];
  rtb_Product1_a = rtb_VectorConcatenate_f_0[0];
  rtb_Product_i = rtb_VectorConcatenate_f_0[2];
  for (i_0 = 0; i_0 < 3; i_0++) {
    rtb_VectorConcatenate_f_0[i_0] = rtb_M[i_0] - ((E_tmp[i_0 + 3] *
      rtb_Product3_k + E_tmp[i_0] * rtb_Product1_a) + E_tmp[i_0 + 6] *
      rtb_Product_i);
  }

  rtb_Product3_k = rtb_VectorConcatenate_f_0[1];
  rtb_Product1_a = rtb_VectorConcatenate_f_0[0];
  rtb_Product_i = rtb_VectorConcatenate_f_0[2];
  // for (i_0 = 0; i_0 < 3; i_0++) {
  //   rtb_VectorConcatenate_f_0[i_0] = ((real32_T)
  //     omni_attitude_controller_P.Kw[i_0 + 3] * rtb_Product3_k + (real32_T)
  //     omni_attitude_controller_P.Kw[i_0] * rtb_Product1_a) + (real32_T)
  //     omni_attitude_controller_P.Kw[i_0 + 6] * rtb_Product_i;
  //   rtb_M[i_0] = ((real32_T)omni_attitude_controller_P.KR[i_0 + 3] * fty +
  //                 (real32_T)omni_attitude_controller_P.KR[i_0] * rtb_Product2_iz)
  //     + (real32_T)omni_attitude_controller_P.KR[i_0 + 6] * ftz;
  // }

  // for (i_0 = 0; i_0 < 9; i_0++) {
  //   rtb_VectorConcatenate_o[i_0] = (real32_T)-omni_attitude_controller_P.Ji[i_0];
  // }

  rtb_Product3_k = rtb_M[0] + rtb_VectorConcatenate_f_0[0];
  rtb_Product2_iz = rtb_M[1] + rtb_VectorConcatenate_f_0[1];
  rtb_Product1_a = rtb_M[2] + rtb_VectorConcatenate_f_0[2];
  for (i_0 = 0; i_0 < 3; i_0++) {
    rtb_M[i_0] = (rtb_VectorConcatenate_o[i_0 + 3] * rtb_Product2_iz +
                  rtb_VectorConcatenate_o[i_0] * rtb_Product3_k) +
      rtb_VectorConcatenate_o[i_0 + 6] * rtb_Product1_a;
  }

  /* Saturate: '<Root>/Saturation' incorporates:
   *  Inport: '<Root>/thrust'
   */
  if (omni_attitude_controller_U.thrust >
      omni_attitude_controller_P.Saturation_UpperSat) {
    rtb_Product3_k = omni_attitude_controller_P.Saturation_UpperSat;
  } else if (omni_attitude_controller_U.thrust <
             omni_attitude_controller_P.Saturation_LowerSat) {
    rtb_Product3_k = omni_attitude_controller_P.Saturation_LowerSat;
  } else {
    rtb_Product3_k = omni_attitude_controller_U.thrust;
  }

  /* MATLAB Function: '<Root>/MATLAB Function1' incorporates:
   *  Saturate: '<Root>/Saturation'
   */
  rtb_Product3_k /= 4.0F;
  rtb_Product1_a = rtb_M[0] / 4.0F / (real32_T)
    omni_attitude_controller_P.a_perpen;
  fty = rtb_M[1] / 4.0F / (real32_T)omni_attitude_controller_P.a_perpen;
  ftz = rtb_M[2] / 4.0F / (real32_T)omni_attitude_controller_P.ThrustToTorque;
  rtb_motor_com_idx_3 = rtb_Product3_k + rtb_Product1_a;
  rtb_Product2_iz = (rtb_motor_com_idx_3 - fty) + ftz;
  rtb_Product3_k -= rtb_Product1_a;
  rtb_Product1_a = (rtb_Product3_k - fty) - ftz;
  rtb_Product_i = (rtb_Product3_k + fty) + ftz;
  rtb_Product3_k = (rtb_motor_com_idx_3 + fty) - ftz;
  if (rtb_Product2_iz < 0.0F) {
    rtb_Product2_iz = 0.0F;
  }

  if (rtb_Product1_a < 0.0F) {
    rtb_Product1_a = 0.0F;
  }

  if (rtb_Product_i < 0.0F) {
    rtb_Product_i = 0.0F;
  }

  if (rtb_Product3_k < 0.0F) {
    rtb_Product3_k = 0.0F;
  }

  rtb_motor_com_idx_3 = (real32_T)(omni_attitude_controller_P.B *
    omni_attitude_controller_P.B);
  rtb_Product2_iz = (sqrtf(rtb_motor_com_idx_3 + (real32_T)(4.0 *
    omni_attitude_controller_P.A) * rtb_Product2_iz) - (real32_T)
                     omni_attitude_controller_P.B) / 2.0F / (real32_T)
    omni_attitude_controller_P.A * 65535.0F;
  fty = (sqrtf(rtb_motor_com_idx_3 + (real32_T)(4.0 *
           omni_attitude_controller_P.A) * rtb_Product1_a) - (real32_T)
         omni_attitude_controller_P.B) / 2.0F / (real32_T)
    omni_attitude_controller_P.A * 65535.0F;
  ftz = (sqrtf(rtb_motor_com_idx_3 + (real32_T)(4.0 *
           omni_attitude_controller_P.A) * rtb_Product_i) - (real32_T)
         omni_attitude_controller_P.B) / 2.0F / (real32_T)
    omni_attitude_controller_P.A * 65535.0F;
  rtb_motor_com_idx_3 = (sqrtf(rtb_motor_com_idx_3 + (real32_T)(4.0 *
    omni_attitude_controller_P.A) * rtb_Product3_k) - (real32_T)
    omni_attitude_controller_P.B) / 2.0F / (real32_T)
    omni_attitude_controller_P.A * 65535.0F;

  /* End of MATLAB Function: '<Root>/MATLAB Function1' */

  /* Saturate: '<Root>/Saturation1' */
  if (rtb_Product2_iz > omni_attitude_controller_P.Saturation1_UpperSat) {
    rtb_Product3_k = omni_attitude_controller_P.Saturation1_UpperSat;
  } else if (rtb_Product2_iz < omni_attitude_controller_P.Saturation1_LowerSat)
  {
    rtb_Product3_k = omni_attitude_controller_P.Saturation1_LowerSat;
  } else {
    rtb_Product3_k = rtb_Product2_iz;
  }

  /* DataTypeConversion: '<Root>/Data Type Conversion2' incorporates:
   *  Saturate: '<Root>/Saturation1'
   */
  rtb_Product3_k = floorf(rtb_Product3_k);
  if (rtIsNaNF(rtb_Product3_k) || rtIsInfF(rtb_Product3_k)) {
    rtb_Product3_k = 0.0F;
  } else {
    rtb_Product3_k = fmodf(rtb_Product3_k, 65536.0F);
  }

  /* Saturate: '<Root>/Saturation1' */
  if (fty > omni_attitude_controller_P.Saturation1_UpperSat) {
    rtb_Product1_a = omni_attitude_controller_P.Saturation1_UpperSat;
  } else if (fty < omni_attitude_controller_P.Saturation1_LowerSat) {
    rtb_Product1_a = omni_attitude_controller_P.Saturation1_LowerSat;
  } else {
    rtb_Product1_a = fty;
  }

  /* DataTypeConversion: '<Root>/Data Type Conversion2' incorporates:
   *  Saturate: '<Root>/Saturation1'
   */
  rtb_Product1_a = floorf(rtb_Product1_a);
  if (rtIsNaNF(rtb_Product1_a) || rtIsInfF(rtb_Product1_a)) {
    rtb_Product1_a = 0.0F;
  } else {
    rtb_Product1_a = fmodf(rtb_Product1_a, 65536.0F);
  }

  /* Saturate: '<Root>/Saturation1' */
  if (ftz > omni_attitude_controller_P.Saturation1_UpperSat) {
    rtb_Product_i = omni_attitude_controller_P.Saturation1_UpperSat;
  } else if (ftz < omni_attitude_controller_P.Saturation1_LowerSat) {
    rtb_Product_i = omni_attitude_controller_P.Saturation1_LowerSat;
  } else {
    rtb_Product_i = ftz;
  }

  /* DataTypeConversion: '<Root>/Data Type Conversion2' incorporates:
   *  Saturate: '<Root>/Saturation1'
   */
  rtb_Product_i = floorf(rtb_Product_i);
  if (rtIsNaNF(rtb_Product_i) || rtIsInfF(rtb_Product_i)) {
    rtb_Product_i = 0.0F;
  } else {
    rtb_Product_i = fmodf(rtb_Product_i, 65536.0F);
  }

  /* Saturate: '<Root>/Saturation1' */
  if (rtb_motor_com_idx_3 > omni_attitude_controller_P.Saturation1_UpperSat) {
    rtb_VectorConcatenate_tmp_1 =
      omni_attitude_controller_P.Saturation1_UpperSat;
  } else if (rtb_motor_com_idx_3 <
             omni_attitude_controller_P.Saturation1_LowerSat) {
    rtb_VectorConcatenate_tmp_1 =
      omni_attitude_controller_P.Saturation1_LowerSat;
  } else {
    rtb_VectorConcatenate_tmp_1 = rtb_motor_com_idx_3;
  }

  /* DataTypeConversion: '<Root>/Data Type Conversion2' incorporates:
   *  Saturate: '<Root>/Saturation1'
   */
  rtb_VectorConcatenate_tmp_1 = floorf(rtb_VectorConcatenate_tmp_1);
  if (rtIsNaNF(rtb_VectorConcatenate_tmp_1) || rtIsInfF
      (rtb_VectorConcatenate_tmp_1)) {
    rtb_VectorConcatenate_tmp_1 = 0.0F;
  } else {
    rtb_VectorConcatenate_tmp_1 = fmodf(rtb_VectorConcatenate_tmp_1, 65536.0F);
  }

  /* Outport: '<Root>/m1' incorporates:
   *  DataTypeConversion: '<Root>/Data Type Conversion2'
   */
  omni_attitude_controller_Y.m1 = (uint16_T)(rtb_Product3_k < 0.0F ? (int32_T)
    (uint16_T)-(int16_T)(uint16_T)-rtb_Product3_k : (int32_T)(uint16_T)
    rtb_Product3_k);

  /* Outport: '<Root>/m2' incorporates:
   *  DataTypeConversion: '<Root>/Data Type Conversion2'
   */
  omni_attitude_controller_Y.m2 = (uint16_T)(rtb_Product1_a < 0.0F ? (int32_T)
    (uint16_T)-(int16_T)(uint16_T)-rtb_Product1_a : (int32_T)(uint16_T)
    rtb_Product1_a);

  /* Outport: '<Root>/m3' incorporates:
   *  DataTypeConversion: '<Root>/Data Type Conversion2'
   */
  omni_attitude_controller_Y.m3 = (uint16_T)(rtb_Product_i < 0.0F ? (int32_T)
    (uint16_T)-(int16_T)(uint16_T)-rtb_Product_i : (int32_T)(uint16_T)
    rtb_Product_i);

  /* Outport: '<Root>/m4' incorporates:
   *  DataTypeConversion: '<Root>/Data Type Conversion2'
   */
  omni_attitude_controller_Y.m4 = (uint16_T)(rtb_VectorConcatenate_tmp_1 < 0.0F ?
    (int32_T)(uint16_T)-(int16_T)(uint16_T)-rtb_VectorConcatenate_tmp_1 :
    (int32_T)(uint16_T)rtb_VectorConcatenate_tmp_1);

  /* Outport: '<Root>/t_m1' */
  omni_attitude_controller_Y.t_m1 = rtb_Product2_iz;

  /* Outport: '<Root>/t_m2' */
  omni_attitude_controller_Y.t_m2 = fty;

  /* Outport: '<Root>/t_m3' */
  omni_attitude_controller_Y.t_m3 = ftz;

  /* Outport: '<Root>/t_m4' */
  omni_attitude_controller_Y.t_m4 = rtb_motor_com_idx_3;

  /* Outport: '<Root>/Tau_x' */
  omni_attitude_controller_Y.Tau_x = rtb_M[0];

  /* Outport: '<Root>/Tau_y' */
  omni_attitude_controller_Y.Tau_y = rtb_M[1];

  /* Outport: '<Root>/Tau_z' */
  omni_attitude_controller_Y.Tau_z = rtb_M[2];
}

/* Model initialize function */
void omni_attitude_controller_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* external inputs */
  (void)memset(&omni_attitude_controller_U, 0, sizeof
               (ExtU_omni_attitude_controller_T));

  /* external outputs */
  (void)memset(&omni_attitude_controller_Y, 0, sizeof
               (ExtY_omni_attitude_controller_T));
}

/* Model terminate function */
void omni_attitude_controller_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
