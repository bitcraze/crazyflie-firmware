/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: omni_attitude_controller.c
 *
 * Code generated for Simulink model 'omni_attitude_controller'.
 *
 * Model version                  : 8.7
 * Simulink Coder version         : 23.2 (R2023b) 01-Aug-2023
 * C/C++ source code generated on : Thu Dec 14 15:57:15 2023
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

/* Block states (default storage) */
DW_omni_attitude_controller_T omni_attitude_controller_DW;

/* External inputs (root inport signals with default storage) */
ExtU_omni_attitude_controller_T omni_attitude_controller_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_omni_attitude_controller_T omni_attitude_controller_Y;

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
   *  Memory: '<Root>/Memory5'
   */
  for (i = 0; i < 3; i++) {
    rtb_Product3_k = rtb_VectorConcatenate_o[3 * i + 1];
    rtb_Product2_iz = rtb_VectorConcatenate_o[3 * i];
    rtb_Product1_a = rtb_VectorConcatenate_o[3 * i + 2];
    for (i_0 = 0; i_0 < 3; i_0++) {
      E_tmp_tmp = 3 * i_0 + i;
      E_tmp[i_0 + 3 * i] = rtb_VectorConcatenate[E_tmp_tmp];
      rtb_VectorConcatenate_f[E_tmp_tmp] = (rtb_VectorConcatenate[3 * i_0 + 1] *
        rtb_Product3_k + rtb_VectorConcatenate[3 * i_0] * rtb_Product2_iz) +
        rtb_VectorConcatenate[3 * i_0 + 2] * rtb_Product1_a;
    }
  }

  for (i = 0; i < 3; i++) {
    rtb_Product3_k = rtb_VectorConcatenate_o[3 * i + 1];
    rtb_Product2_iz = rtb_VectorConcatenate_o[3 * i];
    rtb_Product1_a = rtb_VectorConcatenate_o[3 * i + 2];
    for (i_0 = 0; i_0 < 3; i_0++) {
      rtb_VectorConcatenate[i_0 + 3 * i] = (E_tmp[i_0 + 3] * rtb_Product3_k +
        rtb_Product2_iz * E_tmp[i_0]) + E_tmp[i_0 + 6] * rtb_Product1_a;
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
  rtb_Product3_k = rtb_VectorConcatenate_f_0[1];
  rtb_Product2_iz = rtb_VectorConcatenate_f_0[0];
  rtb_Product1_a = rtb_VectorConcatenate_f_0[2];
  for (i = 0; i < 3; i++) {
    rtb_VectorConcatenate_f_0[i] = rtb_eR[i] - ((E_tmp[i + 3] * rtb_Product3_k +
      E_tmp[i] * rtb_Product2_iz) + E_tmp[i + 6] * rtb_Product1_a);
  }

  rtb_Product3_k = rtb_VectorConcatenate_f_0[1];
  rtb_Product2_iz = rtb_VectorConcatenate_f_0[0];
  rtb_Product1_a = rtb_VectorConcatenate_f_0[2];


  rtb_Product_i = E[6];
  fty = E[5];
  ftz = E[1];
  for (i = 0; i < 3; i++) {
    rtb_VectorConcatenate_f_0[i] = ((real32_T)omni_attitude_controller_P.Kw[i +
      3] * rtb_Product3_k + (real32_T)omni_attitude_controller_P.Kw[i] *
      rtb_Product2_iz) + (real32_T)omni_attitude_controller_P.Kw[i + 6] *
      rtb_Product1_a;
    rtb_eR[i] = ((real32_T)omni_attitude_controller_P.KR[i + 3] * rtb_Product_i
                 + (real32_T)omni_attitude_controller_P.KR[i] * fty) + (real32_T)
      omni_attitude_controller_P.KR[i + 6] * ftz;
  }

  omni_attitude_controller_Y.uW_x = rtb_VectorConcatenate_f_0[0];
  omni_attitude_controller_Y.uW_y = rtb_VectorConcatenate_f_0[1];
  omni_attitude_controller_Y.uW_z = rtb_VectorConcatenate_f_0[2];

  omni_attitude_controller_Y.uP_x = rtb_eR[0];
  omni_attitude_controller_Y.uP_y = rtb_eR[1];
  omni_attitude_controller_Y.uP_z = rtb_eR[2];

  for (i = 0; i < 9; i++) {
    rtb_VectorConcatenate_o[i] = (real32_T)-omni_attitude_controller_P.Ji[i];
  }

  rtb_Product3_k = rtb_eRiout[1];
  rtb_Product2_iz = rtb_eRiout[0];
  rtb_Product1_a = rtb_eRiout[2];
  // for (i = 0; i < 3; i++) {
  //   tmp[i] = (((real32_T)omni_attitude_controller_P.Ki[i + 3] * rtb_Product3_k +
  //              (real32_T)omni_attitude_controller_P.Ki[i] * rtb_Product2_iz) +
  //             (real32_T)omni_attitude_controller_P.Ki[i + 6] * rtb_Product1_a) +
  //     (rtb_eR[i] + rtb_VectorConcatenate_f_0[i]);
  //   omni_attitude_controller_Y.debug[i] = tmp[i];
  // }

  for (i = 0; i < 3; i++) {
    tmp[i] = (rtb_eR[i] + rtb_VectorConcatenate_f_0[i]);
    omni_attitude_controller_Y.debug[i] = tmp[i];
  }

  rtb_Product3_k = tmp[1];
  rtb_Product2_iz = tmp[0];
  rtb_Product1_a = tmp[2];
  for (i = 0; i < 3; i++) {
    rtb_eR[i] = (rtb_VectorConcatenate_o[i + 3] * rtb_Product3_k +
                 rtb_VectorConcatenate_o[i] * rtb_Product2_iz) +
      rtb_VectorConcatenate_o[i + 6] * rtb_Product1_a;
  }
  omni_attitude_controller_Y.tau_x = rtb_eR[0];
  omni_attitude_controller_Y.tau_y = rtb_eR[1];
  omni_attitude_controller_Y.tau_z = rtb_eR[2];
  

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
   *  Constant: '<Root>/Constant'
   *  Saturate: '<Root>/Saturation'
   */
  rtb_Product3_k /= 4.0F;
  rtb_Product1_a = rtb_eR[0] / 4.0F / 0.03165F;
  fty = rtb_eR[1] / 4.0F / 0.03165F;
  ftz = rtb_eR[2] / 4.0F / 0.03165F * (real32_T)
    omni_attitude_controller_P.Constant_Value;
  rtb_motor_com_idx_3 = rtb_Product3_k + rtb_Product1_a;
  rtb_Product2_iz = (rtb_motor_com_idx_3 - fty) - ftz;
  rtb_Product3_k -= rtb_Product1_a;
  rtb_Product1_a = (rtb_Product3_k - fty) + ftz;
  rtb_Product_i = (rtb_Product3_k + fty) - ftz;
  rtb_Product3_k = (rtb_motor_com_idx_3 + fty) + ftz;
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

  fty = rtb_Product2_iz / 0.1472F * 65535.0F;
  ftz = rtb_Product1_a / 0.1472F * 65535.0F;
  rtb_Product_i = rtb_Product_i / 0.1472F * 65535.0F;
  rtb_motor_com_idx_3 = rtb_Product3_k / 0.1472F * 65535.0F;

  /* End of MATLAB Function: '<Root>/MATLAB Function1' */

  /* Saturate: '<Root>/Saturation1' */
  if (fty > omni_attitude_controller_P.Saturation1_UpperSat) {
    rtb_Product3_k = omni_attitude_controller_P.Saturation1_UpperSat;
  } else if (fty < omni_attitude_controller_P.Saturation1_LowerSat) {
    rtb_Product3_k = omni_attitude_controller_P.Saturation1_LowerSat;
  } else {
    rtb_Product3_k = fty;
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
  if (ftz > omni_attitude_controller_P.Saturation1_UpperSat) {
    rtb_Product2_iz = omni_attitude_controller_P.Saturation1_UpperSat;
  } else if (ftz < omni_attitude_controller_P.Saturation1_LowerSat) {
    rtb_Product2_iz = omni_attitude_controller_P.Saturation1_LowerSat;
  } else {
    rtb_Product2_iz = ftz;
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
  if (rtb_Product_i > omni_attitude_controller_P.Saturation1_UpperSat) {
    rtb_Product1_a = omni_attitude_controller_P.Saturation1_UpperSat;
  } else if (rtb_Product_i < omni_attitude_controller_P.Saturation1_LowerSat) {
    rtb_Product1_a = omni_attitude_controller_P.Saturation1_LowerSat;
  } else {
    rtb_Product1_a = rtb_Product_i;
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
  omni_attitude_controller_Y.m2 = (uint16_T)(rtb_Product2_iz < 0.0F ? (int32_T)
    (uint16_T)-(int16_T)(uint16_T)-rtb_Product2_iz : (int32_T)(uint16_T)
    rtb_Product2_iz);

  /* Outport: '<Root>/m3' incorporates:
   *  DataTypeConversion: '<Root>/Data Type Conversion2'
   */
  omni_attitude_controller_Y.m3 = (uint16_T)(rtb_Product1_a < 0.0F ? (int32_T)
    (uint16_T)-(int16_T)(uint16_T)-rtb_Product1_a : (int32_T)(uint16_T)
    rtb_Product1_a);

  /* Outport: '<Root>/m4' incorporates:
   *  DataTypeConversion: '<Root>/Data Type Conversion2'
   */
  omni_attitude_controller_Y.m4 = (uint16_T)(rtb_VectorConcatenate_tmp_1 < 0.0F ?
    (int32_T)(uint16_T)-(int16_T)(uint16_T)-rtb_VectorConcatenate_tmp_1 :
    (int32_T)(uint16_T)rtb_VectorConcatenate_tmp_1);

  /* Outport: '<Root>/t_m1' */
  omni_attitude_controller_Y.t_m1 = fty;

  /* Outport: '<Root>/t_m2' */
  omni_attitude_controller_Y.t_m2 = ftz;

  /* Outport: '<Root>/t_m3' */
  omni_attitude_controller_Y.t_m3 = rtb_Product_i;

  /* Outport: '<Root>/t_m4' */
  omni_attitude_controller_Y.t_m4 = rtb_motor_com_idx_3;

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
