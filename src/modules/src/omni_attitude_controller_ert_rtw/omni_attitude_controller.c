/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: omni_attitude_controller.c
 *
 * Code generated for Simulink model 'omni_attitude_controller'.
 *
 * Model version                  : 8.19
 * Simulink Coder version         : 23.2 (R2023b) 01-Aug-2023
 * C/C++ source code generated on : Fri Dec 22 16:31:48 2023
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objective: Execution efficiency
 * Validation result: Not run
 */

#include "omni_attitude_controller.h"
#include "rtwtypes.h"
#include <math.h>
#include "rt_nonfinite.h"
#include <string.h>

/* Block states (default storage) */
DW_omni_attitude_controller_T omni_attitude_controller_DW;

/* External inputs (root inport signals with default storage) */
ExtU_omni_attitude_controller_T omni_attitude_controller_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_omni_attitude_controller_T omni_attitude_controller_Y;

/* Forward declaration for local functions */
static void omni_attitude_cont_quatmultiply(const real32_T q[4], const real32_T
  r[4], real32_T qout[4]);

/* Function for MATLAB Function: '<Root>/MATLAB Function2' */
static void omni_attitude_cont_quatmultiply(const real32_T q[4], const real32_T
  r[4], real32_T qout[4])
{
  qout[0] = ((q[0] * r[0] - q[1] * r[1]) - q[2] * r[2]) - q[3] * r[3];
  qout[1] = (q[0] * r[1] + r[0] * q[1]) + (q[2] * r[3] - r[2] * q[3]);
  qout[2] = (q[0] * r[2] + r[0] * q[2]) + (r[1] * q[3] - q[1] * r[3]);
  qout[3] = (q[0] * r[3] + r[0] * q[3]) + (q[1] * r[2] - r[1] * q[2]);
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
  real32_T q_i[4];
  real32_T rtb_VectorConcatenate_f_0[3];
  real32_T rtb_eR[3];
  real32_T rtb_eRiout[3];
  real32_T tmp[3];
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
  real32_T rtb_VectorConcatenate_tmp_4;
  static const real32_T b[4] = { 0.707106769F, 0.0F, 0.0F, 0.707106769F };

  static const real32_T c[4] = { -0.707106769F, 0.0F, 0.0F, 0.707106769F };

  real32_T tmp_0[4];

  /* Sqrt: '<S17>/sqrt' incorporates:
   *  Inport: '<Root>/qw_r'
   *  Inport: '<Root>/qx_r'
   *  Inport: '<Root>/qy_r'
   *  Inport: '<Root>/qz_r'
   *  Product: '<S18>/Product'
   *  Product: '<S18>/Product1'
   *  Product: '<S18>/Product2'
   *  Product: '<S18>/Product3'
   *  Sum: '<S18>/Sum'
   */
  rtb_Product2_iz = sqrtf(((omni_attitude_controller_U.qw_r *
    omni_attitude_controller_U.qw_r + omni_attitude_controller_U.qx_r *
    omni_attitude_controller_U.qx_r) + omni_attitude_controller_U.qy_r *
    omni_attitude_controller_U.qy_r) + omni_attitude_controller_U.qz_r *
    omni_attitude_controller_U.qz_r);

  /* Product: '<S16>/Product' incorporates:
   *  Inport: '<Root>/qw_r'
   */
  rtb_Product1_a = omni_attitude_controller_U.qw_r / rtb_Product2_iz;

  /* Product: '<S16>/Product1' incorporates:
   *  Inport: '<Root>/qx_r'
   */
  rtb_Product_i = omni_attitude_controller_U.qx_r / rtb_Product2_iz;

  /* Product: '<S16>/Product2' incorporates:
   *  Inport: '<Root>/qy_r'
   */
  rtb_Product3_k = omni_attitude_controller_U.qy_r / rtb_Product2_iz;

  /* Product: '<S16>/Product3' incorporates:
   *  Inport: '<Root>/qz_r'
   */
  rtb_Product2_iz = omni_attitude_controller_U.qz_r / rtb_Product2_iz;

  /* Product: '<S6>/Product3' incorporates:
   *  Product: '<S10>/Product3'
   */
  fty = rtb_Product1_a * rtb_Product1_a;

  /* Product: '<S6>/Product2' incorporates:
   *  Product: '<S10>/Product2'
   */
  ftz = rtb_Product_i * rtb_Product_i;

  /* Product: '<S6>/Product1' incorporates:
   *  Product: '<S10>/Product1'
   *  Product: '<S14>/Product1'
   */
  rtb_VectorConcatenate_tmp_1 = rtb_Product3_k * rtb_Product3_k;

  /* Product: '<S6>/Product' incorporates:
   *  Product: '<S10>/Product'
   *  Product: '<S14>/Product'
   */
  rtb_VectorConcatenate_tmp_2 = rtb_Product2_iz * rtb_Product2_iz;

  /* Sum: '<S6>/Sum' incorporates:
   *  Product: '<S6>/Product'
   *  Product: '<S6>/Product1'
   *  Product: '<S6>/Product2'
   *  Product: '<S6>/Product3'
   */
  rtb_VectorConcatenate[0] = ((fty + ftz) - rtb_VectorConcatenate_tmp_1) -
    rtb_VectorConcatenate_tmp_2;

  /* Product: '<S9>/Product3' incorporates:
   *  Product: '<S7>/Product3'
   */
  rtb_VectorConcatenate_tmp = rtb_Product2_iz * rtb_Product1_a;

  /* Product: '<S9>/Product2' incorporates:
   *  Product: '<S7>/Product2'
   */
  rtb_VectorConcatenate_tmp_0 = rtb_Product_i * rtb_Product3_k;

  /* Gain: '<S9>/Gain' incorporates:
   *  Product: '<S9>/Product2'
   *  Product: '<S9>/Product3'
   *  Sum: '<S9>/Sum'
   */
  rtb_VectorConcatenate[1] = (rtb_VectorConcatenate_tmp_0 -
    rtb_VectorConcatenate_tmp) * omni_attitude_controller_P.Gain_Gain;

  /* Product: '<S12>/Product2' incorporates:
   *  Product: '<S8>/Product2'
   */
  rtb_VectorConcatenate_tmp_3 = rtb_Product_i * rtb_Product2_iz;

  /* Product: '<S12>/Product1' incorporates:
   *  Product: '<S8>/Product1'
   */
  rtb_VectorConcatenate_tmp_4 = rtb_Product1_a * rtb_Product3_k;

  /* Gain: '<S12>/Gain' incorporates:
   *  Product: '<S12>/Product1'
   *  Product: '<S12>/Product2'
   *  Sum: '<S12>/Sum'
   */
  rtb_VectorConcatenate[2] = (rtb_VectorConcatenate_tmp_4 +
    rtb_VectorConcatenate_tmp_3) * omni_attitude_controller_P.Gain_Gain_l;

  /* Gain: '<S7>/Gain' incorporates:
   *  Sum: '<S7>/Sum'
   */
  rtb_VectorConcatenate[3] = (rtb_VectorConcatenate_tmp +
    rtb_VectorConcatenate_tmp_0) * omni_attitude_controller_P.Gain_Gain_f;

  /* Sum: '<S10>/Sum' incorporates:
   *  Sum: '<S14>/Sum'
   */
  fty -= ftz;
  rtb_VectorConcatenate[4] = (fty + rtb_VectorConcatenate_tmp_1) -
    rtb_VectorConcatenate_tmp_2;

  /* Product: '<S13>/Product1' incorporates:
   *  Product: '<S11>/Product1'
   */
  ftz = rtb_Product1_a * rtb_Product_i;

  /* Product: '<S13>/Product2' incorporates:
   *  Product: '<S11>/Product2'
   */
  rtb_VectorConcatenate_tmp = rtb_Product3_k * rtb_Product2_iz;

  /* Gain: '<S13>/Gain' incorporates:
   *  Product: '<S13>/Product1'
   *  Product: '<S13>/Product2'
   *  Sum: '<S13>/Sum'
   */
  rtb_VectorConcatenate[5] = (rtb_VectorConcatenate_tmp - ftz) *
    omni_attitude_controller_P.Gain_Gain_d;

  /* Gain: '<S8>/Gain' incorporates:
   *  Sum: '<S8>/Sum'
   */
  rtb_VectorConcatenate[6] = (rtb_VectorConcatenate_tmp_3 -
    rtb_VectorConcatenate_tmp_4) * omni_attitude_controller_P.Gain_Gain_b;

  /* Gain: '<S11>/Gain' incorporates:
   *  Sum: '<S11>/Sum'
   */
  rtb_VectorConcatenate[7] = (ftz + rtb_VectorConcatenate_tmp) *
    omni_attitude_controller_P.Gain_Gain_c;

  /* Sum: '<S14>/Sum' */
  rtb_VectorConcatenate[8] = (fty - rtb_VectorConcatenate_tmp_1) +
    rtb_VectorConcatenate_tmp_2;

  /* MATLAB Function: '<Root>/MATLAB Function2' incorporates:
   *  Inport: '<Root>/qw_IMU'
   *  Inport: '<Root>/qx_IMU'
   *  Inport: '<Root>/qy_IMU'
   *  Inport: '<Root>/qz_IMU'
   */
  q_i[0] = omni_attitude_controller_U.qw_IMU;
  q_i[1] = omni_attitude_controller_U.qx_IMU;
  q_i[2] = omni_attitude_controller_U.qy_IMU;
  q_i[3] = omni_attitude_controller_U.qz_IMU;
  omni_attitude_cont_quatmultiply(q_i, c, tmp_0);
  omni_attitude_cont_quatmultiply(b, tmp_0, q_i);

  /* Sqrt: '<S30>/sqrt' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function2'
   *  Product: '<S31>/Product'
   *  Product: '<S31>/Product1'
   *  Product: '<S31>/Product2'
   *  Product: '<S31>/Product3'
   *  Sum: '<S31>/Sum'
   */
  rtb_Product3_k = sqrtf(((q_i[0] * q_i[0] + q_i[1] * q_i[1]) + q_i[2] * q_i[2])
    + q_i[3] * q_i[3]);

  /* Product: '<S29>/Product' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function2'
   */
  rtb_Product_i = q_i[0] / rtb_Product3_k;

  /* Product: '<S29>/Product1' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function2'
   */
  rtb_Product1_a = q_i[1] / rtb_Product3_k;

  /* Product: '<S29>/Product2' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function2'
   */
  rtb_Product2_iz = q_i[2] / rtb_Product3_k;

  /* Product: '<S29>/Product3' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function2'
   */
  rtb_Product3_k = q_i[3] / rtb_Product3_k;

  /* Product: '<S19>/Product3' incorporates:
   *  Product: '<S23>/Product3'
   */
  fty = rtb_Product_i * rtb_Product_i;

  /* Product: '<S19>/Product2' incorporates:
   *  Product: '<S23>/Product2'
   */
  ftz = rtb_Product1_a * rtb_Product1_a;

  /* Product: '<S19>/Product1' incorporates:
   *  Product: '<S23>/Product1'
   *  Product: '<S27>/Product1'
   */
  rtb_VectorConcatenate_tmp_1 = rtb_Product2_iz * rtb_Product2_iz;

  /* Product: '<S19>/Product' incorporates:
   *  Product: '<S23>/Product'
   *  Product: '<S27>/Product'
   */
  rtb_VectorConcatenate_tmp_2 = rtb_Product3_k * rtb_Product3_k;

  /* Sum: '<S19>/Sum' incorporates:
   *  Product: '<S19>/Product'
   *  Product: '<S19>/Product1'
   *  Product: '<S19>/Product2'
   *  Product: '<S19>/Product3'
   */
  rtb_VectorConcatenate_o[0] = ((fty + ftz) - rtb_VectorConcatenate_tmp_1) -
    rtb_VectorConcatenate_tmp_2;

  /* Product: '<S22>/Product3' incorporates:
   *  Product: '<S20>/Product3'
   */
  rtb_VectorConcatenate_tmp = rtb_Product3_k * rtb_Product_i;

  /* Product: '<S22>/Product2' incorporates:
   *  Product: '<S20>/Product2'
   */
  rtb_VectorConcatenate_tmp_0 = rtb_Product1_a * rtb_Product2_iz;

  /* Gain: '<S22>/Gain' incorporates:
   *  Product: '<S22>/Product2'
   *  Product: '<S22>/Product3'
   *  Sum: '<S22>/Sum'
   */
  rtb_VectorConcatenate_o[1] = (rtb_VectorConcatenate_tmp_0 -
    rtb_VectorConcatenate_tmp) * omni_attitude_controller_P.Gain_Gain_p;

  /* Product: '<S25>/Product2' incorporates:
   *  Product: '<S21>/Product2'
   */
  rtb_VectorConcatenate_tmp_3 = rtb_Product1_a * rtb_Product3_k;

  /* Product: '<S25>/Product1' incorporates:
   *  Product: '<S21>/Product1'
   */
  rtb_VectorConcatenate_tmp_4 = rtb_Product_i * rtb_Product2_iz;

  /* Gain: '<S25>/Gain' incorporates:
   *  Product: '<S25>/Product1'
   *  Product: '<S25>/Product2'
   *  Sum: '<S25>/Sum'
   */
  rtb_VectorConcatenate_o[2] = (rtb_VectorConcatenate_tmp_4 +
    rtb_VectorConcatenate_tmp_3) * omni_attitude_controller_P.Gain_Gain_i;

  /* Gain: '<S20>/Gain' incorporates:
   *  Sum: '<S20>/Sum'
   */
  rtb_VectorConcatenate_o[3] = (rtb_VectorConcatenate_tmp +
    rtb_VectorConcatenate_tmp_0) * omni_attitude_controller_P.Gain_Gain_fl;

  /* Sum: '<S23>/Sum' incorporates:
   *  Sum: '<S27>/Sum'
   */
  fty -= ftz;
  rtb_VectorConcatenate_o[4] = (fty + rtb_VectorConcatenate_tmp_1) -
    rtb_VectorConcatenate_tmp_2;

  /* Product: '<S26>/Product1' incorporates:
   *  Product: '<S24>/Product1'
   */
  ftz = rtb_Product_i * rtb_Product1_a;

  /* Product: '<S26>/Product2' incorporates:
   *  Product: '<S24>/Product2'
   */
  rtb_VectorConcatenate_tmp = rtb_Product2_iz * rtb_Product3_k;

  /* Gain: '<S26>/Gain' incorporates:
   *  Product: '<S26>/Product1'
   *  Product: '<S26>/Product2'
   *  Sum: '<S26>/Sum'
   */
  rtb_VectorConcatenate_o[5] = (rtb_VectorConcatenate_tmp - ftz) *
    omni_attitude_controller_P.Gain_Gain_a;

  /* Gain: '<S21>/Gain' incorporates:
   *  Sum: '<S21>/Sum'
   */
  rtb_VectorConcatenate_o[6] = (rtb_VectorConcatenate_tmp_3 -
    rtb_VectorConcatenate_tmp_4) * omni_attitude_controller_P.Gain_Gain_h;

  /* Gain: '<S24>/Gain' incorporates:
   *  Sum: '<S24>/Sum'
   */
  rtb_VectorConcatenate_o[7] = (ftz + rtb_VectorConcatenate_tmp) *
    omni_attitude_controller_P.Gain_Gain_ao;

  /* Sum: '<S27>/Sum' */
  rtb_VectorConcatenate_o[8] = (fty - rtb_VectorConcatenate_tmp_1) +
    rtb_VectorConcatenate_tmp_2;

  /* MATLAB Function: '<Root>/LLATC' incorporates:
   *  Concatenate: '<S15>/Vector Concatenate'
   *  Concatenate: '<S28>/Vector Concatenate'
   *  Inport: '<Root>/gyro_x'
   *  Inport: '<Root>/gyro_y'
   *  Inport: '<Root>/gyro_z'
   *  Memory: '<Root>/Memory5'
   */
  for (i = 0; i < 3; i++) {
    rtb_Product2_iz = rtb_VectorConcatenate_o[3 * i + 1];
    rtb_Product1_a = rtb_VectorConcatenate_o[3 * i];
    rtb_Product3_k = rtb_VectorConcatenate_o[3 * i + 2];
    for (i_0 = 0; i_0 < 3; i_0++) {
      E_tmp_tmp = 3 * i_0 + i;
      E_tmp[i_0 + 3 * i] = rtb_VectorConcatenate[E_tmp_tmp];
      rtb_VectorConcatenate_f[E_tmp_tmp] = (rtb_VectorConcatenate[3 * i_0 + 1] *
        rtb_Product2_iz + rtb_VectorConcatenate[3 * i_0] * rtb_Product1_a) +
        rtb_VectorConcatenate[3 * i_0 + 2] * rtb_Product3_k;
    }
  }

  for (i = 0; i < 3; i++) {
    rtb_Product2_iz = rtb_VectorConcatenate_o[3 * i + 1];
    rtb_Product1_a = rtb_VectorConcatenate_o[3 * i];
    rtb_Product3_k = rtb_VectorConcatenate_o[3 * i + 2];
    for (i_0 = 0; i_0 < 3; i_0++) {
      rtb_VectorConcatenate[i_0 + 3 * i] = (E_tmp[i_0 + 3] * rtb_Product2_iz +
        rtb_Product1_a * E_tmp[i_0]) + E_tmp[i_0 + 6] * rtb_Product3_k;
    }
  }

  for (i = 0; i < 9; i++) {
    E[i] = (rtb_VectorConcatenate_f[i] - rtb_VectorConcatenate[i]) * 0.5F;
  }

  rtb_eR[0] = E[5];
  rtb_eR[1] = E[6];
  rtb_eR[2] = E[1];
  for (i = 0; i < 3; i++) {
    rtb_eRiout[i] = rtb_eR[i] * 0.01F +
      omni_attitude_controller_DW.Memory5_PreviousInput[i];
    rtb_VectorConcatenate_f_0[i] = (rtb_VectorConcatenate_o[i + 3] *
      omni_attitude_controller_U.gyro_y + rtb_VectorConcatenate_o[i] *
      omni_attitude_controller_U.gyro_x) + rtb_VectorConcatenate_o[i + 6] *
      omni_attitude_controller_U.gyro_z;
  }

  /* SignalConversion generated from: '<S1>/ SFunction ' incorporates:
   *  Inport: '<Root>/wx_r'
   *  Inport: '<Root>/wy_r'
   *  Inport: '<Root>/wz_r'
   *  MATLAB Function: '<Root>/LLATC'
   */
  rtb_eR[0] = omni_attitude_controller_U.wx_r;
  rtb_eR[1] = omni_attitude_controller_U.wy_r;
  rtb_eR[2] = omni_attitude_controller_U.wz_r;

  /* MATLAB Function: '<Root>/LLATC' */
  rtb_Product2_iz = rtb_VectorConcatenate_f_0[1];
  rtb_Product1_a = rtb_VectorConcatenate_f_0[0];
  rtb_Product3_k = rtb_VectorConcatenate_f_0[2];
  for (i = 0; i < 3; i++) {
    rtb_VectorConcatenate_f_0[i] = rtb_eR[i] - ((E_tmp[i + 3] * rtb_Product2_iz
      + E_tmp[i] * rtb_Product1_a) + E_tmp[i + 6] * rtb_Product3_k);
  }

  rtb_Product2_iz = rtb_VectorConcatenate_f_0[1];
  rtb_Product1_a = rtb_VectorConcatenate_f_0[0];
  rtb_Product3_k = rtb_VectorConcatenate_f_0[2];
  rtb_Product_i = E[6];
  fty = E[5];
  ftz = E[1];
  for (i = 0; i < 3; i++) {
    rtb_VectorConcatenate_f_0[i] = ((real32_T)omni_attitude_controller_P.Kw[i +
      3] * rtb_Product2_iz + (real32_T)omni_attitude_controller_P.Kw[i] *
      rtb_Product1_a) + (real32_T)omni_attitude_controller_P.Kw[i + 6] *
      rtb_Product3_k;
    rtb_eR[i] = ((real32_T)omni_attitude_controller_P.KR[i + 3] * rtb_Product_i
                 + (real32_T)omni_attitude_controller_P.KR[i] * fty) + (real32_T)
      omni_attitude_controller_P.KR[i + 6] * ftz;
  }

  for (i = 0; i < 9; i++) {
    rtb_VectorConcatenate_o[i] = (real32_T)-omni_attitude_controller_P.Ji[i];
  }

  rtb_Product2_iz = rtb_eRiout[1];
  rtb_Product1_a = rtb_eRiout[0];
  rtb_Product3_k = rtb_eRiout[2];
  for (i = 0; i < 3; i++) {
    //tmp[i] = (((real32_T)omni_attitude_controller_P.Ki[i + 3] * rtb_Product2_iz
    //           + (real32_T)omni_attitude_controller_P.Ki[i] * rtb_Product1_a) +
    //          (real32_T)omni_attitude_controller_P.Ki[i + 6] * rtb_Product3_k) +
    //  (rtb_eR[i] + rtb_VectorConcatenate_f_0[i]);
    tmp[i] = (rtb_eR[i] + rtb_VectorConcatenate_f_0[i]);
  }

  rtb_Product2_iz = tmp[1];
  rtb_Product1_a = tmp[0];
  rtb_Product3_k = tmp[2];
  for (i = 0; i < 3; i++) {
    rtb_eR[i] = (rtb_VectorConcatenate_o[i + 3] * rtb_Product2_iz +
                 rtb_VectorConcatenate_o[i] * rtb_Product1_a) +
      rtb_VectorConcatenate_o[i + 6] * rtb_Product3_k;
  }

  /* Saturate: '<Root>/Saturation' incorporates:
   *  Inport: '<Root>/thrust'
   */
  if (omni_attitude_controller_U.thrust >
      omni_attitude_controller_P.Saturation_UpperSat) {
    rtb_Product2_iz = omni_attitude_controller_P.Saturation_UpperSat;
  } else if (omni_attitude_controller_U.thrust <
             omni_attitude_controller_P.Saturation_LowerSat) {
    rtb_Product2_iz = omni_attitude_controller_P.Saturation_LowerSat;
  } else {
    rtb_Product2_iz = omni_attitude_controller_U.thrust;
  }

  /* MATLAB Function: '<Root>/MATLAB Function1' incorporates:
   *  Constant: '<Root>/Constant'
   *  Saturate: '<Root>/Saturation'
   */
  rtb_Product3_k = rtb_Product2_iz / 4.0F;
  rtb_Product1_a = rtb_eR[0] / 4.0F / 0.03165F;
  fty = rtb_eR[1] / 4.0F / 0.03165F;
  ftz = rtb_eR[2] / 4.0F / 0.03165F * (real32_T)
    omni_attitude_controller_P.Constant_Value;
  rtb_VectorConcatenate_tmp_1 = rtb_Product3_k + rtb_Product1_a;
  rtb_Product2_iz = (rtb_VectorConcatenate_tmp_1 - fty) - ftz;
  rtb_Product3_k -= rtb_Product1_a;
  rtb_Product1_a = (rtb_Product3_k - fty) + ftz;
  rtb_Product_i = (rtb_Product3_k + fty) - ftz;
  rtb_Product3_k = (rtb_VectorConcatenate_tmp_1 + fty) + ftz;
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

  q_i[0] = rtb_Product2_iz / 0.1472F * 65535.0F;
  q_i[1] = rtb_Product1_a / 0.1472F * 65535.0F;
  q_i[2] = rtb_Product_i / 0.1472F * 65535.0F;
  q_i[3] = rtb_Product3_k / 0.1472F * 65535.0F;

  /* End of MATLAB Function: '<Root>/MATLAB Function1' */

  /* Saturate: '<Root>/Saturation1' */
  if (q_i[0] > omni_attitude_controller_P.Saturation1_UpperSat) {
    rtb_Product2_iz = omni_attitude_controller_P.Saturation1_UpperSat;
  } else if (q_i[0] < omni_attitude_controller_P.Saturation1_LowerSat) {
    rtb_Product2_iz = omni_attitude_controller_P.Saturation1_LowerSat;
  } else {
    rtb_Product2_iz = q_i[0];
  }

  /* DataTypeConversion: '<Root>/Data Type Conversion2' incorporates:
   *  Saturate: '<Root>/Saturation1'
   */
  rtb_Product2_iz = floorf(rtb_Product2_iz);
  if (rtIsNaNF(rtb_Product2_iz) || rtIsInfF(rtb_Product2_iz)) {
    rtb_Product2_iz = 0.0F;
  } else {
    rtb_Product2_iz = fmodf(rtb_Product2_iz, 65536.0F);
  }

  /* Saturate: '<Root>/Saturation1' */
  if (q_i[1] > omni_attitude_controller_P.Saturation1_UpperSat) {
    rtb_Product1_a = omni_attitude_controller_P.Saturation1_UpperSat;
  } else if (q_i[1] < omni_attitude_controller_P.Saturation1_LowerSat) {
    rtb_Product1_a = omni_attitude_controller_P.Saturation1_LowerSat;
  } else {
    rtb_Product1_a = q_i[1];
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
  if (q_i[2] > omni_attitude_controller_P.Saturation1_UpperSat) {
    rtb_Product3_k = omni_attitude_controller_P.Saturation1_UpperSat;
  } else if (q_i[2] < omni_attitude_controller_P.Saturation1_LowerSat) {
    rtb_Product3_k = omni_attitude_controller_P.Saturation1_LowerSat;
  } else {
    rtb_Product3_k = q_i[2];
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
  if (q_i[3] > omni_attitude_controller_P.Saturation1_UpperSat) {
    rtb_Product_i = omni_attitude_controller_P.Saturation1_UpperSat;
  } else if (q_i[3] < omni_attitude_controller_P.Saturation1_LowerSat) {
    rtb_Product_i = omni_attitude_controller_P.Saturation1_LowerSat;
  } else {
    rtb_Product_i = q_i[3];
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

  /* Outport: '<Root>/m1' incorporates:
   *  DataTypeConversion: '<Root>/Data Type Conversion2'
   */
  omni_attitude_controller_Y.m1 = (uint16_T)(rtb_Product2_iz < 0.0F ? (int32_T)
    (uint16_T)-(int16_T)(uint16_T)-rtb_Product2_iz : (int32_T)(uint16_T)
    rtb_Product2_iz);

  /* Outport: '<Root>/m2' incorporates:
   *  DataTypeConversion: '<Root>/Data Type Conversion2'
   */
  omni_attitude_controller_Y.m2 = (uint16_T)(rtb_Product1_a < 0.0F ? (int32_T)
    (uint16_T)-(int16_T)(uint16_T)-rtb_Product1_a : (int32_T)(uint16_T)
    rtb_Product1_a);

  /* Outport: '<Root>/m3' incorporates:
   *  DataTypeConversion: '<Root>/Data Type Conversion2'
   */
  omni_attitude_controller_Y.m3 = (uint16_T)(rtb_Product3_k < 0.0F ? (int32_T)
    (uint16_T)-(int16_T)(uint16_T)-rtb_Product3_k : (int32_T)(uint16_T)
    rtb_Product3_k);

  /* Outport: '<Root>/m4' incorporates:
   *  DataTypeConversion: '<Root>/Data Type Conversion2'
   */
  omni_attitude_controller_Y.m4 = (uint16_T)(rtb_Product_i < 0.0F ? (int32_T)
    (uint16_T)-(int16_T)(uint16_T)-rtb_Product_i : (int32_T)(uint16_T)
    rtb_Product_i);

  /* Outport: '<Root>/t_m1' */
  omni_attitude_controller_Y.t_m1 = q_i[0];

  /* Outport: '<Root>/t_m2' */
  omni_attitude_controller_Y.t_m2 = q_i[1];

  /* Outport: '<Root>/t_m3' */
  omni_attitude_controller_Y.t_m3 = q_i[2];

  /* Outport: '<Root>/t_m4' */
  omni_attitude_controller_Y.t_m4 = q_i[3];

  /* Outport: '<Root>/Tau_x' */
  omni_attitude_controller_Y.Tau_x = rtb_eR[0];

  /* Outport: '<Root>/Tau_y' */
  omni_attitude_controller_Y.Tau_y = rtb_eR[1];

  /* Outport: '<Root>/Tau_z' */
  omni_attitude_controller_Y.Tau_z = rtb_eR[2];

  /* Update for Memory: '<Root>/Memory5' */
  omni_attitude_controller_DW.Memory5_PreviousInput[0] = rtb_eRiout[0];
  omni_attitude_controller_DW.Memory5_PreviousInput[1] = rtb_eRiout[1];
  omni_attitude_controller_DW.Memory5_PreviousInput[2] = rtb_eRiout[2];
}

/* Model initialize function */
void omni_attitude_controller_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* states (dwork) */
  (void) memset((void *)&omni_attitude_controller_DW, 0,
                sizeof(DW_omni_attitude_controller_T));

  /* external inputs */
  (void)memset(&omni_attitude_controller_U, 0, sizeof
               (ExtU_omni_attitude_controller_T));

  /* external outputs */
  (void)memset(&omni_attitude_controller_Y, 0, sizeof
               (ExtY_omni_attitude_controller_T));

  /* InitializeConditions for Memory: '<Root>/Memory5' */
  omni_attitude_controller_DW.Memory5_PreviousInput[0] =
    omni_attitude_controller_P.Memory5_InitialCondition[0];
  omni_attitude_controller_DW.Memory5_PreviousInput[1] =
    omni_attitude_controller_P.Memory5_InitialCondition[1];
  omni_attitude_controller_DW.Memory5_PreviousInput[2] =
    omni_attitude_controller_P.Memory5_InitialCondition[2];
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
