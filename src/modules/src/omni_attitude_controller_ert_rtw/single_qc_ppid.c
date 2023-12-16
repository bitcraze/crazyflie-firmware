/*
 * File: single_qc_ppid.c
 *
 * Code generated for Simulink model 'single_qc_ppid'.
 *
 * Model version                  : 1.92
 * Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
 * C/C++ source code generated on : Sun Nov 29 11:18:27 2020
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objective: Execution efficiency
 * Validation result: Not run
 */

#include "single_qc_ppid.h"
#include "single_qc_ppid_private.h"

/* Block states (default storage) */
DW_single_qc_ppid_T single_qc_ppid_DW;

/* External inputs (root inport signals with default storage) */
ExtU_single_qc_ppid_T single_qc_ppid_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_single_qc_ppid_T single_qc_ppid_Y;

/* Forward declaration for local functions */
static void single_qc_ppid_quatmultiply(const real32_T q[4], const real32_T r[4],
  real32_T qout[4]);

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void single_qc_ppid_quatmultiply(const real32_T q[4], const real32_T r[4],
  real32_T qout[4])
{
  qout[0] = ((q[0] * r[0] - q[1] * r[1]) - q[2] * r[2]) - q[3] * r[3];
  qout[1] = (q[0] * r[1] + r[0] * q[1]) + (q[2] * r[3] - q[3] * r[2]);
  qout[2] = (q[0] * r[2] + r[0] * q[2]) + (q[3] * r[1] - q[1] * r[3]);
  qout[3] = (q[0] * r[3] + r[0] * q[3]) + (q[1] * r[2] - q[2] * r[1]);
}

real32_T rt_atan2f_snf(real32_T u0, real32_T u1)
{
  real32_T y;
  int32_T u0_0;
  int32_T u1_0;
  if (rtIsNaNF(u0) || rtIsNaNF(u1)) {
    y = (rtNaNF);
  } else if (rtIsInfF(u0) && rtIsInfF(u1)) {
    if (u0 > 0.0F) {
      u0_0 = 1;
    } else {
      u0_0 = -1;
    }

    if (u1 > 0.0F) {
      u1_0 = 1;
    } else {
      u1_0 = -1;
    }

    y = atan2f((real32_T)u0_0, (real32_T)u1_0);
  } else if (u1 == 0.0F) {
    if (u0 > 0.0F) {
      y = RT_PIF / 2.0F;
    } else if (u0 < 0.0F) {
      y = -(RT_PIF / 2.0F);
    } else {
      y = 0.0F;
    }
  } else {
    y = atan2f(u0, u1);
  }

  return y;
}

/* Model step function */
void single_qc_ppid_step(void)
{
  real32_T acount_prev;
  real32_T bcount_prev;
  real32_T q_Bbi[4];
  real32_T q_bi[4];
  real32_T R_bii[9];
  real32_T diff_a;
  real32_T diff_b;
  real32_T tempR[9];
  int32_T d_k;
  real32_T ftx;
  real32_T fty;
  real32_T ftz;
  real32_T f0;
  real32_T f2;
  real32_T f3;
  real32_T rtb_Sum1_d;
  real32_T rtb_Saturation;
  real32_T rtb_Sum;
  boolean_T rtb_Compare_h;
  real32_T rtb_TSamp;
  real32_T rtb_Sum3;
  real32_T rtb_TSamp_p2;
  real32_T rtb_Sum1;
  real32_T rtb_TSamp_c;
  real32_T rtb_Sum3_b;
  real32_T rtb_TSamp_gn;
  real32_T rtb_Sum1_p;
  real32_T tmp[4];
  int8_T subsa_idx_1;
  real32_T u0;
  real32_T q_Bbi_tmp;
  real32_T q_Bbi_tmp_0;
  int32_T R_bii_tmp;
  static const real32_T b[4] = { 0.707106769F, 0.0F, 0.0F, 0.707106769F };

  static const real32_T c[4] = { -0.707106769F, 0.0F, 0.0F, 0.707106769F };

  real32_T tmp_0[4];

  /* Saturate: '<Root>/Saturation' incorporates:
   *  Inport: '<Root>/thrust'
   */
  if (single_qc_ppid_U.thrust > single_qc_ppid_P.Saturation_UpperSat) {
    rtb_Saturation = single_qc_ppid_P.Saturation_UpperSat;
  } else if (single_qc_ppid_U.thrust < single_qc_ppid_P.Saturation_LowerSat) {
    rtb_Saturation = single_qc_ppid_P.Saturation_LowerSat;
  } else {
    rtb_Saturation = single_qc_ppid_U.thrust;
  }

  /* End of Saturate: '<Root>/Saturation' */

  /* MATLAB Function: '<Root>/MATLAB Function' incorporates:
   *  Constant: '<S1>/Constant'
   *  Constant: '<S2>/Constant'
   *  Inport: '<Root>/index'
   *  Inport: '<Root>/qw_IMU'
   *  Inport: '<Root>/qw_op'
   *  Inport: '<Root>/qx_IMU'
   *  Inport: '<Root>/qx_op'
   *  Inport: '<Root>/qy_IMU'
   *  Inport: '<Root>/qy_op'
   *  Inport: '<Root>/qz_IMU'
   *  Inport: '<Root>/qz_op'
   *  Logic: '<Root>/Logical Operator'
   *  Memory: '<Root>/Memory'
   *  Memory: '<Root>/Memory1'
   *  Memory: '<Root>/Memory2'
   *  Memory: '<Root>/Memory3'
   *  Memory: '<Root>/Memory4'
   *  RelationalOperator: '<S1>/Compare'
   *  RelationalOperator: '<S2>/Compare'
   */
  acount_prev = single_qc_ppid_DW.Memory2_PreviousInput;
  bcount_prev = single_qc_ppid_DW.Memory3_PreviousInput;
  q_Bbi_tmp = -single_qc_ppid_U.index * 3.14159274F / 4.0F;
  q_Bbi_tmp_0 = cosf(q_Bbi_tmp);
  q_Bbi[0] = q_Bbi_tmp_0;
  q_Bbi[1] = 0.0F;
  q_Bbi[2] = 0.0F;
  q_Bbi_tmp = sinf(q_Bbi_tmp);
  q_Bbi[3] = q_Bbi_tmp;
  tmp[0] = single_qc_ppid_U.qw_op;
  tmp[1] = single_qc_ppid_U.qx_op;
  tmp[2] = single_qc_ppid_U.qy_op;
  tmp[3] = single_qc_ppid_U.qz_op;
  q_bi[0] = -q_Bbi_tmp_0;
  q_bi[1] = 0.0F;
  q_bi[2] = 0.0F;
  q_bi[3] = q_Bbi_tmp;
  single_qc_ppid_quatmultiply(tmp, q_bi, tmp_0);
  single_qc_ppid_quatmultiply(q_Bbi, tmp_0, q_bi);
  tmp[0] = single_qc_ppid_U.qw_IMU;
  tmp[1] = single_qc_ppid_U.qx_IMU;
  tmp[2] = single_qc_ppid_U.qy_IMU;
  tmp[3] = single_qc_ppid_U.qz_IMU;
  q_Bbi[0] = -q_bi[0];
  q_Bbi[1] = q_bi[1];
  q_Bbi[2] = q_bi[2];
  q_Bbi[3] = q_bi[3];
  single_qc_ppid_quatmultiply(tmp, c, tmp_0);
  single_qc_ppid_quatmultiply(b, tmp_0, tmp);
  single_qc_ppid_quatmultiply(q_Bbi, tmp, q_bi);
  q_Bbi_tmp = 1.0F / sqrtf(((q_bi[0] * q_bi[0] + q_bi[1] * q_bi[1]) + q_bi[2] *
    q_bi[2]) + q_bi[3] * q_bi[3]);
  q_Bbi[0] = q_bi[0] * q_Bbi_tmp;
  q_Bbi[1] = q_bi[1] * q_Bbi_tmp;
  q_Bbi[2] = q_bi[2] * q_Bbi_tmp;
  q_Bbi[3] = q_bi[3] * q_Bbi_tmp;
  q_Bbi_tmp = q_Bbi[3] * q_Bbi[3];
  q_Bbi_tmp_0 = q_Bbi[2] * q_Bbi[2];
  tempR[0] = 1.0F - (q_Bbi_tmp_0 + q_Bbi_tmp) * 2.0F;
  diff_a = q_Bbi[1] * q_Bbi[2];
  diff_b = q_Bbi[0] * q_Bbi[3];
  tempR[1] = (diff_a - diff_b) * 2.0F;
  rtb_Sum = q_Bbi[1] * q_Bbi[3];
  rtb_TSamp = q_Bbi[0] * q_Bbi[2];
  tempR[2] = (rtb_Sum + rtb_TSamp) * 2.0F;
  tempR[3] = (diff_a + diff_b) * 2.0F;
  diff_a = q_Bbi[1] * q_Bbi[1];
  tempR[4] = 1.0F - (diff_a + q_Bbi_tmp) * 2.0F;
  q_Bbi_tmp = q_Bbi[2] * q_Bbi[3];
  diff_b = q_Bbi[0] * q_Bbi[1];
  tempR[5] = (q_Bbi_tmp - diff_b) * 2.0F;
  tempR[6] = (rtb_Sum - rtb_TSamp) * 2.0F;
  tempR[7] = (q_Bbi_tmp + diff_b) * 2.0F;
  tempR[8] = 1.0F - (diff_a + q_Bbi_tmp_0) * 2.0F;
  for (d_k = 0; d_k < 3; d_k++) {
    R_bii_tmp = (int8_T)(d_k + 1) - 1;
    R_bii[R_bii_tmp] = tempR[R_bii_tmp * 3];
    subsa_idx_1 = (int8_T)(d_k + 1);
    R_bii[subsa_idx_1 + 2] = tempR[(subsa_idx_1 - 1) * 3 + 1];
    subsa_idx_1 = (int8_T)(d_k + 1);
    R_bii[subsa_idx_1 + 5] = tempR[(subsa_idx_1 - 1) * 3 + 2];
  }

  q_Bbi_tmp = rt_atan2f_snf(R_bii[5], R_bii[4]);
  q_Bbi_tmp_0 = rt_atan2f_snf(R_bii[6], R_bii[0]);
  if ((single_qc_ppid_DW.Memory4_PreviousInput <=
       single_qc_ppid_P.Constant_Value_k) && (rtb_Saturation >
       single_qc_ppid_P.Constant_Value)) {
    acount_prev = 0.0F;
    bcount_prev = 0.0F;
  }

  diff_a = single_qc_ppid_DW.Memory_PreviousInput - q_Bbi_tmp;
  if (fabsf(diff_a) > 3.14159274F) {
    if (diff_a < 0.0F) {
      diff_a = -1.0F;
    } else if (diff_a > 0.0F) {
      diff_a = 1.0F;
    } else if (diff_a == 0.0F) {
      diff_a = 0.0F;
    } else {
      diff_a = (rtNaNF);
    }

    acount_prev += diff_a;
  }

  diff_a = 6.28318548F * acount_prev + q_Bbi_tmp;
  diff_b = single_qc_ppid_DW.Memory1_PreviousInput - q_Bbi_tmp_0;
  if (fabsf(diff_b) > 3.14159274F) {
    if (diff_b < 0.0F) {
      diff_b = -1.0F;
    } else if (diff_b > 0.0F) {
      diff_b = 1.0F;
    } else if (diff_b == 0.0F) {
      diff_b = 0.0F;
    } else {
      diff_b = (rtNaNF);
    }

    bcount_prev += diff_b;
  }

  diff_b = 6.28318548F * bcount_prev + q_Bbi_tmp_0;

  /* Sum: '<S6>/Sum' incorporates:
   *  Inport: '<Root>/alpha_desired'
   */
  rtb_Sum = single_qc_ppid_U.alpha_desired - diff_a;

  /* RelationalOperator: '<S9>/Compare' incorporates:
   *  Constant: '<S9>/Constant'
   */
  rtb_Compare_h = (rtb_Saturation > single_qc_ppid_P.Constant_Value_a);

  /* DiscreteIntegrator: '<S7>/Discrete-Time Integrator2' */
  if (rtb_Compare_h && (single_qc_ppid_DW.DiscreteTimeIntegrator2_PrevRes <= 0))
  {
    single_qc_ppid_DW.DiscreteTimeIntegrator2_DSTATE =
      single_qc_ppid_P.DiscreteTimeIntegrator2_IC;
  }

  if (single_qc_ppid_DW.DiscreteTimeIntegrator2_DSTATE >=
      single_qc_ppid_P.DiscreteTimeIntegrator2_UpperSa) {
    single_qc_ppid_DW.DiscreteTimeIntegrator2_DSTATE =
      single_qc_ppid_P.DiscreteTimeIntegrator2_UpperSa;
  } else {
    if (single_qc_ppid_DW.DiscreteTimeIntegrator2_DSTATE <=
        single_qc_ppid_P.DiscreteTimeIntegrator2_LowerSa) {
      single_qc_ppid_DW.DiscreteTimeIntegrator2_DSTATE =
        single_qc_ppid_P.DiscreteTimeIntegrator2_LowerSa;
    }
  }

  /* SampleTimeMath: '<S11>/TSamp'
   *
   * About '<S11>/TSamp':
   *  y = u * K where K = 1 / ( w * Ts )
   */
  rtb_TSamp = rtb_Sum * single_qc_ppid_P.TSamp_WtEt;

  /* MATLAB Function: '<Root>/MATLAB Function2' incorporates:
   *  Trigonometry: '<S6>/Trigonometric Function'
   *  Trigonometry: '<S6>/Trigonometric Function1'
   */
  fty = cosf(diff_b);
  ftz = sinf(diff_b);

  /* Sum: '<S7>/Sum3' incorporates:
   *  DiscreteIntegrator: '<S7>/Discrete-Time Integrator2'
   *  Gain: '<S7>/dgain'
   *  Gain: '<S7>/igain'
   *  Gain: '<S7>/pgain'
   *  Inport: '<Root>/omega_x'
   *  Inport: '<Root>/omega_z'
   *  MATLAB Function: '<Root>/MATLAB Function2'
   *  Sum: '<S11>/Diff'
   *  Sum: '<S7>/Sum2'
   *  UnitDelay: '<S11>/UD'
   *
   * Block description for '<S11>/Diff':
   *
   *  Add in CPU
   *
   * Block description for '<S11>/UD':
   *
   *  Store in Global RAM
   */
  rtb_Sum3 = ((single_qc_ppid_P.pgaina * rtb_Sum + single_qc_ppid_P.igaina *
               single_qc_ppid_DW.DiscreteTimeIntegrator2_DSTATE) + (rtb_TSamp -
    single_qc_ppid_DW.UD_DSTATE) * single_qc_ppid_P.dgaina) - (fty *
    single_qc_ppid_U.omega_x + ftz * single_qc_ppid_U.omega_z);

  /* DiscreteIntegrator: '<S7>/Discrete-Time Integrator1' */
  if (rtb_Compare_h && (single_qc_ppid_DW.DiscreteTimeIntegrator1_PrevRes <= 0))
  {
    single_qc_ppid_DW.DiscreteTimeIntegrator1_DSTATE =
      single_qc_ppid_P.DiscreteTimeIntegrator1_IC;
  }

  if (single_qc_ppid_DW.DiscreteTimeIntegrator1_DSTATE >=
      single_qc_ppid_P.DiscreteTimeIntegrator1_UpperSa) {
    single_qc_ppid_DW.DiscreteTimeIntegrator1_DSTATE =
      single_qc_ppid_P.DiscreteTimeIntegrator1_UpperSa;
  } else {
    if (single_qc_ppid_DW.DiscreteTimeIntegrator1_DSTATE <=
        single_qc_ppid_P.DiscreteTimeIntegrator1_LowerSa) {
      single_qc_ppid_DW.DiscreteTimeIntegrator1_DSTATE =
        single_qc_ppid_P.DiscreteTimeIntegrator1_LowerSa;
    }
  }

  /* SampleTimeMath: '<S10>/TSamp'
   *
   * About '<S10>/TSamp':
   *  y = u * K where K = 1 / ( w * Ts )
   */
  rtb_TSamp_p2 = rtb_Sum3 * single_qc_ppid_P.TSamp_WtEt_k;

  /* Sum: '<S7>/Sum1' incorporates:
   *  DiscreteIntegrator: '<S7>/Discrete-Time Integrator1'
   *  Gain: '<S7>/dgain1'
   *  Gain: '<S7>/igain1'
   *  Gain: '<S7>/pgain1'
   *  Sum: '<S10>/Diff'
   *  UnitDelay: '<S10>/UD'
   *
   * Block description for '<S10>/Diff':
   *
   *  Add in CPU
   *
   * Block description for '<S10>/UD':
   *
   *  Store in Global RAM
   */
  rtb_Sum1_d = (single_qc_ppid_P.pgainas * rtb_Sum3 + single_qc_ppid_P.igainas *
                single_qc_ppid_DW.DiscreteTimeIntegrator1_DSTATE) +
    (rtb_TSamp_p2 - single_qc_ppid_DW.UD_DSTATE_j) * single_qc_ppid_P.dgainas;

  /* Sum: '<S6>/Sum1' incorporates:
   *  Inport: '<Root>/beta_desired'
   */
  rtb_Sum1 = single_qc_ppid_U.beta_desired - diff_b;

  /* DiscreteIntegrator: '<S8>/Discrete-Time Integrator2' */
  if ((rtb_Saturation > 0.0F) &&
      (single_qc_ppid_DW.DiscreteTimeIntegrator2_PrevR_e <= 0)) {
    single_qc_ppid_DW.DiscreteTimeIntegrator2_DSTAT_o =
      single_qc_ppid_P.DiscreteTimeIntegrator2_IC_h;
  }

  if (single_qc_ppid_DW.DiscreteTimeIntegrator2_DSTAT_o >=
      single_qc_ppid_P.DiscreteTimeIntegrator2_Upper_l) {
    single_qc_ppid_DW.DiscreteTimeIntegrator2_DSTAT_o =
      single_qc_ppid_P.DiscreteTimeIntegrator2_Upper_l;
  } else {
    if (single_qc_ppid_DW.DiscreteTimeIntegrator2_DSTAT_o <=
        single_qc_ppid_P.DiscreteTimeIntegrator2_Lower_i) {
      single_qc_ppid_DW.DiscreteTimeIntegrator2_DSTAT_o =
        single_qc_ppid_P.DiscreteTimeIntegrator2_Lower_i;
    }
  }

  /* SampleTimeMath: '<S13>/TSamp'
   *
   * About '<S13>/TSamp':
   *  y = u * K where K = 1 / ( w * Ts )
   */
  rtb_TSamp_c = rtb_Sum1 * single_qc_ppid_P.TSamp_WtEt_i;

  /* Sum: '<S8>/Sum3' incorporates:
   *  DiscreteIntegrator: '<S8>/Discrete-Time Integrator2'
   *  Gain: '<S8>/dgain'
   *  Gain: '<S8>/igain'
   *  Gain: '<S8>/pgain'
   *  Inport: '<Root>/beta_speed'
   *  Sum: '<S13>/Diff'
   *  Sum: '<S8>/Sum2'
   *  UnitDelay: '<S13>/UD'
   *
   * Block description for '<S13>/Diff':
   *
   *  Add in CPU
   *
   * Block description for '<S13>/UD':
   *
   *  Store in Global RAM
   */
  rtb_Sum3_b = ((single_qc_ppid_P.pgainb * rtb_Sum1 + single_qc_ppid_P.igainb *
                 single_qc_ppid_DW.DiscreteTimeIntegrator2_DSTAT_o) +
                (rtb_TSamp_c - single_qc_ppid_DW.UD_DSTATE_p) *
                single_qc_ppid_P.dgainb) - single_qc_ppid_U.beta_speed;

  /* DiscreteIntegrator: '<S8>/Discrete-Time Integrator1' */
  if ((rtb_Saturation > 0.0F) &&
      (single_qc_ppid_DW.DiscreteTimeIntegrator1_PrevR_o <= 0)) {
    single_qc_ppid_DW.DiscreteTimeIntegrator1_DSTAT_p =
      single_qc_ppid_P.DiscreteTimeIntegrator1_IC_h;
  }

  if (single_qc_ppid_DW.DiscreteTimeIntegrator1_DSTAT_p >=
      single_qc_ppid_P.DiscreteTimeIntegrator1_Upper_e) {
    single_qc_ppid_DW.DiscreteTimeIntegrator1_DSTAT_p =
      single_qc_ppid_P.DiscreteTimeIntegrator1_Upper_e;
  } else {
    if (single_qc_ppid_DW.DiscreteTimeIntegrator1_DSTAT_p <=
        single_qc_ppid_P.DiscreteTimeIntegrator1_Lower_f) {
      single_qc_ppid_DW.DiscreteTimeIntegrator1_DSTAT_p =
        single_qc_ppid_P.DiscreteTimeIntegrator1_Lower_f;
    }
  }

  /* SampleTimeMath: '<S12>/TSamp'
   *
   * About '<S12>/TSamp':
   *  y = u * K where K = 1 / ( w * Ts )
   */
  rtb_TSamp_gn = rtb_Sum3_b * single_qc_ppid_P.TSamp_WtEt_b;

  /* Sum: '<S8>/Sum1' incorporates:
   *  DiscreteIntegrator: '<S8>/Discrete-Time Integrator1'
   *  Gain: '<S8>/dgain1'
   *  Gain: '<S8>/igain1'
   *  Gain: '<S8>/pgain1'
   *  Sum: '<S12>/Diff'
   *  UnitDelay: '<S12>/UD'
   *
   * Block description for '<S12>/Diff':
   *
   *  Add in CPU
   *
   * Block description for '<S12>/UD':
   *
   *  Store in Global RAM
   */
  rtb_Sum1_p = (single_qc_ppid_P.pgainbs * rtb_Sum3_b + single_qc_ppid_P.igainbs
                * single_qc_ppid_DW.DiscreteTimeIntegrator1_DSTAT_p) +
    (rtb_TSamp_gn - single_qc_ppid_DW.UD_DSTATE_d) * single_qc_ppid_P.dgainbs;

  /* MATLAB Function: '<Root>/MATLAB Function1' */
  f3 = rtb_Saturation / 4.0F;

  /* Product: '<S6>/Product' */
  u0 = fty * rtb_Sum1_d;

  /* Saturate: '<S6>/Saturation' */
  if (u0 > single_qc_ppid_P.sat_tx) {
    u0 = single_qc_ppid_P.sat_tx;
  } else {
    if (u0 < -single_qc_ppid_P.sat_tx) {
      u0 = -single_qc_ppid_P.sat_tx;
    }
  }

  /* End of Saturate: '<S6>/Saturation' */

  /* MATLAB Function: '<Root>/MATLAB Function1' */
  ftx = u0 / 4.0F / 0.03165F;

  /* Saturate: '<S6>/Saturation1' */
  if (rtb_Sum1_p > single_qc_ppid_P.sat_ty) {
    fty = single_qc_ppid_P.sat_ty;
  } else if (rtb_Sum1_p < -single_qc_ppid_P.sat_ty) {
    fty = -single_qc_ppid_P.sat_ty;
  } else {
    fty = rtb_Sum1_p;
  }

  /* End of Saturate: '<S6>/Saturation1' */

  /* MATLAB Function: '<Root>/MATLAB Function1' */
  fty = fty / 4.0F / 0.03165F;

  /* Product: '<S6>/Product1' */
  u0 = rtb_Sum1_d * ftz;

  /* Saturate: '<S6>/Saturation2' */
  if (u0 > single_qc_ppid_P.sat_tz) {
    u0 = single_qc_ppid_P.sat_tz;
  } else {
    if (u0 < -single_qc_ppid_P.sat_tz) {
      u0 = -single_qc_ppid_P.sat_tz;
    }
  }

  /* End of Saturate: '<S6>/Saturation2' */

  /* MATLAB Function: '<Root>/MATLAB Function1' incorporates:
   *  Constant: '<Root>/Constant'
   */
  ftz = u0 / 4.0F / 0.03165F * single_qc_ppid_P.torque_modifier;
  u0 = f3 + ftx;
  f0 = (u0 - fty) - ftz;
  f3 -= ftx;
  ftx = (f3 - fty) + ftz;
  f2 = (f3 + fty) - ftz;
  f3 = (u0 + fty) + ftz;
  if (f0 < 0.0F) {
    f0 = 0.0F;
  }

  if (ftx < 0.0F) {
    ftx = 0.0F;
  }

  if (f2 < 0.0F) {
    f2 = 0.0F;
  }

  if (f3 < 0.0F) {
    f3 = 0.0F;
  }

  q_Bbi[0] = f0 / 0.1472F * 65535.0F;
  q_Bbi[1] = ftx / 0.1472F * 65535.0F;
  q_Bbi[2] = f2 / 0.1472F * 65535.0F;
  q_Bbi[3] = f3 / 0.1472F * 65535.0F;

  /* Saturate: '<Root>/Saturation1' */
  if (q_Bbi[0] > single_qc_ppid_P.Saturation1_UpperSat) {
    ftz = single_qc_ppid_P.Saturation1_UpperSat;
  } else if (q_Bbi[0] < single_qc_ppid_P.Saturation1_LowerSat) {
    ftz = single_qc_ppid_P.Saturation1_LowerSat;
  } else {
    ftz = q_Bbi[0];
  }

  /* DataTypeConversion: '<Root>/Data Type Conversion2' */
  f3 = floorf(ftz);
  if (rtIsNaNF(f3) || rtIsInfF(f3)) {
    f3 = 0.0F;
  } else {
    f3 = fmodf(f3, 65536.0F);
  }

  /* Saturate: '<Root>/Saturation1' */
  if (q_Bbi[1] > single_qc_ppid_P.Saturation1_UpperSat) {
    ftz = single_qc_ppid_P.Saturation1_UpperSat;
  } else if (q_Bbi[1] < single_qc_ppid_P.Saturation1_LowerSat) {
    ftz = single_qc_ppid_P.Saturation1_LowerSat;
  } else {
    ftz = q_Bbi[1];
  }

  /* DataTypeConversion: '<Root>/Data Type Conversion2' */
  fty = floorf(ftz);
  if (rtIsNaNF(fty) || rtIsInfF(fty)) {
    fty = 0.0F;
  } else {
    fty = fmodf(fty, 65536.0F);
  }

  /* Saturate: '<Root>/Saturation1' */
  if (q_Bbi[2] > single_qc_ppid_P.Saturation1_UpperSat) {
    ftz = single_qc_ppid_P.Saturation1_UpperSat;
  } else if (q_Bbi[2] < single_qc_ppid_P.Saturation1_LowerSat) {
    ftz = single_qc_ppid_P.Saturation1_LowerSat;
  } else {
    ftz = q_Bbi[2];
  }

  /* DataTypeConversion: '<Root>/Data Type Conversion2' */
  ftx = floorf(ftz);
  if (rtIsNaNF(ftx) || rtIsInfF(ftx)) {
    ftx = 0.0F;
  } else {
    ftx = fmodf(ftx, 65536.0F);
  }

  /* Saturate: '<Root>/Saturation1' */
  if (q_Bbi[3] > single_qc_ppid_P.Saturation1_UpperSat) {
    ftz = single_qc_ppid_P.Saturation1_UpperSat;
  } else if (q_Bbi[3] < single_qc_ppid_P.Saturation1_LowerSat) {
    ftz = single_qc_ppid_P.Saturation1_LowerSat;
  } else {
    ftz = q_Bbi[3];
  }

  /* DataTypeConversion: '<Root>/Data Type Conversion2' */
  ftz = floorf(ftz);
  if (rtIsNaNF(ftz) || rtIsInfF(ftz)) {
    ftz = 0.0F;
  } else {
    ftz = fmodf(ftz, 65536.0F);
  }

  /* Outport: '<Root>/m1' incorporates:
   *  DataTypeConversion: '<Root>/Data Type Conversion2'
   */
  single_qc_ppid_Y.m1 = (uint16_T)(f3 < 0.0F ? (int32_T)(uint16_T)-(int16_T)
    (uint16_T)-f3 : (int32_T)(uint16_T)f3);

  /* Outport: '<Root>/m2' incorporates:
   *  DataTypeConversion: '<Root>/Data Type Conversion2'
   */
  single_qc_ppid_Y.m2 = (uint16_T)(fty < 0.0F ? (int32_T)(uint16_T)-(int16_T)
    (uint16_T)-fty : (int32_T)(uint16_T)fty);

  /* Outport: '<Root>/m3' incorporates:
   *  DataTypeConversion: '<Root>/Data Type Conversion2'
   */
  single_qc_ppid_Y.m3 = (uint16_T)(ftx < 0.0F ? (int32_T)(uint16_T)-(int16_T)
    (uint16_T)-ftx : (int32_T)(uint16_T)ftx);

  /* Outport: '<Root>/m4' incorporates:
   *  DataTypeConversion: '<Root>/Data Type Conversion2'
   */
  single_qc_ppid_Y.m4 = (uint16_T)(ftz < 0.0F ? (int32_T)(uint16_T)-(int16_T)
    (uint16_T)-ftz : (int32_T)(uint16_T)ftz);

  /* Outport: '<Root>/t_m1' */
  single_qc_ppid_Y.t_m1 = q_Bbi[0];

  /* Outport: '<Root>/t_m2' */
  single_qc_ppid_Y.t_m2 = q_Bbi[1];

  /* Outport: '<Root>/t_m3' */
  single_qc_ppid_Y.t_m3 = q_Bbi[2];

  /* Outport: '<Root>/t_m4' */
  single_qc_ppid_Y.t_m4 = q_Bbi[3];

  /* Outport: '<Root>/u_beta' */
  single_qc_ppid_Y.u_beta = rtb_Sum1_p;

  /* Outport: '<Root>/error_betas' */
  single_qc_ppid_Y.error_betas = rtb_Sum3_b;

  /* Outport: '<Root>/error_beta' */
  single_qc_ppid_Y.error_beta = rtb_Sum1;

  /* Outport: '<Root>/u_alpha' */
  single_qc_ppid_Y.u_alpha = rtb_Sum1_d;

  /* Outport: '<Root>/error_alphas' */
  single_qc_ppid_Y.error_alphas = rtb_Sum3;

  /* Outport: '<Root>/error_alpha' */
  single_qc_ppid_Y.error_alpha = rtb_Sum;

  /* Outport: '<Root>/t_betae' */
  single_qc_ppid_Y.t_betae = diff_b;

  /* Outport: '<Root>/t_alphae' */
  single_qc_ppid_Y.t_alphae = diff_a;

  /* Outport: '<Root>/t_betain' incorporates:
   *  Inport: '<Root>/beta_desired'
   */
  single_qc_ppid_Y.t_betain = single_qc_ppid_U.beta_desired;

  /* Outport: '<Root>/t_alphain' incorporates:
   *  Inport: '<Root>/alpha_desired'
   */
  single_qc_ppid_Y.t_alphain = single_qc_ppid_U.alpha_desired;

  /* Update for Memory: '<Root>/Memory' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function'
   */
  single_qc_ppid_DW.Memory_PreviousInput = q_Bbi_tmp;

  /* Update for Memory: '<Root>/Memory1' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function'
   */
  single_qc_ppid_DW.Memory1_PreviousInput = q_Bbi_tmp_0;

  /* Update for Memory: '<Root>/Memory2' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function'
   */
  single_qc_ppid_DW.Memory2_PreviousInput = acount_prev;

  /* Update for Memory: '<Root>/Memory3' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function'
   */
  single_qc_ppid_DW.Memory3_PreviousInput = bcount_prev;

  /* Update for Memory: '<Root>/Memory4' */
  single_qc_ppid_DW.Memory4_PreviousInput = rtb_Saturation;

  /* Update for DiscreteIntegrator: '<S7>/Discrete-Time Integrator2' */
  single_qc_ppid_DW.DiscreteTimeIntegrator2_DSTATE +=
    single_qc_ppid_P.DiscreteTimeIntegrator2_gainval * rtb_Sum;
  if (single_qc_ppid_DW.DiscreteTimeIntegrator2_DSTATE >=
      single_qc_ppid_P.DiscreteTimeIntegrator2_UpperSa) {
    single_qc_ppid_DW.DiscreteTimeIntegrator2_DSTATE =
      single_qc_ppid_P.DiscreteTimeIntegrator2_UpperSa;
  } else {
    if (single_qc_ppid_DW.DiscreteTimeIntegrator2_DSTATE <=
        single_qc_ppid_P.DiscreteTimeIntegrator2_LowerSa) {
      single_qc_ppid_DW.DiscreteTimeIntegrator2_DSTATE =
        single_qc_ppid_P.DiscreteTimeIntegrator2_LowerSa;
    }
  }

  single_qc_ppid_DW.DiscreteTimeIntegrator2_PrevRes = (int8_T)rtb_Compare_h;

  /* End of Update for DiscreteIntegrator: '<S7>/Discrete-Time Integrator2' */

  /* Update for UnitDelay: '<S11>/UD'
   *
   * Block description for '<S11>/UD':
   *
   *  Store in Global RAM
   */
  single_qc_ppid_DW.UD_DSTATE = rtb_TSamp;

  /* Update for DiscreteIntegrator: '<S7>/Discrete-Time Integrator1' */
  single_qc_ppid_DW.DiscreteTimeIntegrator1_DSTATE +=
    single_qc_ppid_P.DiscreteTimeIntegrator1_gainval * rtb_Sum3;
  if (single_qc_ppid_DW.DiscreteTimeIntegrator1_DSTATE >=
      single_qc_ppid_P.DiscreteTimeIntegrator1_UpperSa) {
    single_qc_ppid_DW.DiscreteTimeIntegrator1_DSTATE =
      single_qc_ppid_P.DiscreteTimeIntegrator1_UpperSa;
  } else {
    if (single_qc_ppid_DW.DiscreteTimeIntegrator1_DSTATE <=
        single_qc_ppid_P.DiscreteTimeIntegrator1_LowerSa) {
      single_qc_ppid_DW.DiscreteTimeIntegrator1_DSTATE =
        single_qc_ppid_P.DiscreteTimeIntegrator1_LowerSa;
    }
  }

  single_qc_ppid_DW.DiscreteTimeIntegrator1_PrevRes = (int8_T)rtb_Compare_h;

  /* End of Update for DiscreteIntegrator: '<S7>/Discrete-Time Integrator1' */

  /* Update for UnitDelay: '<S10>/UD'
   *
   * Block description for '<S10>/UD':
   *
   *  Store in Global RAM
   */
  single_qc_ppid_DW.UD_DSTATE_j = rtb_TSamp_p2;

  /* Update for DiscreteIntegrator: '<S8>/Discrete-Time Integrator2' */
  single_qc_ppid_DW.DiscreteTimeIntegrator2_DSTAT_o +=
    single_qc_ppid_P.DiscreteTimeIntegrator2_gainv_f * rtb_Sum1;
  if (single_qc_ppid_DW.DiscreteTimeIntegrator2_DSTAT_o >=
      single_qc_ppid_P.DiscreteTimeIntegrator2_Upper_l) {
    single_qc_ppid_DW.DiscreteTimeIntegrator2_DSTAT_o =
      single_qc_ppid_P.DiscreteTimeIntegrator2_Upper_l;
  } else {
    if (single_qc_ppid_DW.DiscreteTimeIntegrator2_DSTAT_o <=
        single_qc_ppid_P.DiscreteTimeIntegrator2_Lower_i) {
      single_qc_ppid_DW.DiscreteTimeIntegrator2_DSTAT_o =
        single_qc_ppid_P.DiscreteTimeIntegrator2_Lower_i;
    }
  }

  if (rtb_Saturation > 0.0F) {
    single_qc_ppid_DW.DiscreteTimeIntegrator2_PrevR_e = 1;
  } else if (rtb_Saturation < 0.0F) {
    single_qc_ppid_DW.DiscreteTimeIntegrator2_PrevR_e = -1;
  } else if (rtb_Saturation == 0.0F) {
    single_qc_ppid_DW.DiscreteTimeIntegrator2_PrevR_e = 0;
  } else {
    single_qc_ppid_DW.DiscreteTimeIntegrator2_PrevR_e = 2;
  }

  /* End of Update for DiscreteIntegrator: '<S8>/Discrete-Time Integrator2' */

  /* Update for UnitDelay: '<S13>/UD'
   *
   * Block description for '<S13>/UD':
   *
   *  Store in Global RAM
   */
  single_qc_ppid_DW.UD_DSTATE_p = rtb_TSamp_c;

  /* Update for DiscreteIntegrator: '<S8>/Discrete-Time Integrator1' */
  single_qc_ppid_DW.DiscreteTimeIntegrator1_DSTAT_p +=
    single_qc_ppid_P.DiscreteTimeIntegrator1_gainv_c * rtb_Sum3_b;
  if (single_qc_ppid_DW.DiscreteTimeIntegrator1_DSTAT_p >=
      single_qc_ppid_P.DiscreteTimeIntegrator1_Upper_e) {
    single_qc_ppid_DW.DiscreteTimeIntegrator1_DSTAT_p =
      single_qc_ppid_P.DiscreteTimeIntegrator1_Upper_e;
  } else {
    if (single_qc_ppid_DW.DiscreteTimeIntegrator1_DSTAT_p <=
        single_qc_ppid_P.DiscreteTimeIntegrator1_Lower_f) {
      single_qc_ppid_DW.DiscreteTimeIntegrator1_DSTAT_p =
        single_qc_ppid_P.DiscreteTimeIntegrator1_Lower_f;
    }
  }

  if (rtb_Saturation > 0.0F) {
    single_qc_ppid_DW.DiscreteTimeIntegrator1_PrevR_o = 1;
  } else if (rtb_Saturation < 0.0F) {
    single_qc_ppid_DW.DiscreteTimeIntegrator1_PrevR_o = -1;
  } else if (rtb_Saturation == 0.0F) {
    single_qc_ppid_DW.DiscreteTimeIntegrator1_PrevR_o = 0;
  } else {
    single_qc_ppid_DW.DiscreteTimeIntegrator1_PrevR_o = 2;
  }

  /* End of Update for DiscreteIntegrator: '<S8>/Discrete-Time Integrator1' */

  /* Update for UnitDelay: '<S12>/UD'
   *
   * Block description for '<S12>/UD':
   *
   *  Store in Global RAM
   */
  single_qc_ppid_DW.UD_DSTATE_d = rtb_TSamp_gn;
}

/* Model initialize function */
void single_qc_ppid_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* states (dwork) */
  (void) memset((void *)&single_qc_ppid_DW, 0,
                sizeof(DW_single_qc_ppid_T));

  /* external inputs */
  (void)memset(&single_qc_ppid_U, 0, sizeof(ExtU_single_qc_ppid_T));

  /* external outputs */
  (void) memset((void *)&single_qc_ppid_Y, 0,
                sizeof(ExtY_single_qc_ppid_T));

  /* InitializeConditions for Memory: '<Root>/Memory' */
  single_qc_ppid_DW.Memory_PreviousInput =
    single_qc_ppid_P.Memory_InitialCondition;

  /* InitializeConditions for Memory: '<Root>/Memory1' */
  single_qc_ppid_DW.Memory1_PreviousInput =
    single_qc_ppid_P.Memory1_InitialCondition;

  /* InitializeConditions for Memory: '<Root>/Memory2' */
  single_qc_ppid_DW.Memory2_PreviousInput =
    single_qc_ppid_P.Memory2_InitialCondition;

  /* InitializeConditions for Memory: '<Root>/Memory3' */
  single_qc_ppid_DW.Memory3_PreviousInput =
    single_qc_ppid_P.Memory3_InitialCondition;

  /* InitializeConditions for Memory: '<Root>/Memory4' */
  single_qc_ppid_DW.Memory4_PreviousInput =
    single_qc_ppid_P.Memory4_InitialCondition;

  /* InitializeConditions for DiscreteIntegrator: '<S7>/Discrete-Time Integrator2' */
  single_qc_ppid_DW.DiscreteTimeIntegrator2_DSTATE =
    single_qc_ppid_P.DiscreteTimeIntegrator2_IC;
  single_qc_ppid_DW.DiscreteTimeIntegrator2_PrevRes = 2;

  /* InitializeConditions for UnitDelay: '<S11>/UD'
   *
   * Block description for '<S11>/UD':
   *
   *  Store in Global RAM
   */
  single_qc_ppid_DW.UD_DSTATE = single_qc_ppid_P.DiscreteDerivative2_ICPrevScale;

  /* InitializeConditions for DiscreteIntegrator: '<S7>/Discrete-Time Integrator1' */
  single_qc_ppid_DW.DiscreteTimeIntegrator1_DSTATE =
    single_qc_ppid_P.DiscreteTimeIntegrator1_IC;
  single_qc_ppid_DW.DiscreteTimeIntegrator1_PrevRes = 2;

  /* InitializeConditions for UnitDelay: '<S10>/UD'
   *
   * Block description for '<S10>/UD':
   *
   *  Store in Global RAM
   */
  single_qc_ppid_DW.UD_DSTATE_j =
    single_qc_ppid_P.DiscreteDerivative1_ICPrevScale;

  /* InitializeConditions for DiscreteIntegrator: '<S8>/Discrete-Time Integrator2' */
  single_qc_ppid_DW.DiscreteTimeIntegrator2_DSTAT_o =
    single_qc_ppid_P.DiscreteTimeIntegrator2_IC_h;
  single_qc_ppid_DW.DiscreteTimeIntegrator2_PrevR_e = 2;

  /* InitializeConditions for UnitDelay: '<S13>/UD'
   *
   * Block description for '<S13>/UD':
   *
   *  Store in Global RAM
   */
  single_qc_ppid_DW.UD_DSTATE_p =
    single_qc_ppid_P.DiscreteDerivative2_ICPrevSca_g;

  /* InitializeConditions for DiscreteIntegrator: '<S8>/Discrete-Time Integrator1' */
  single_qc_ppid_DW.DiscreteTimeIntegrator1_DSTAT_p =
    single_qc_ppid_P.DiscreteTimeIntegrator1_IC_h;
  single_qc_ppid_DW.DiscreteTimeIntegrator1_PrevR_o = 2;

  /* InitializeConditions for UnitDelay: '<S12>/UD'
   *
   * Block description for '<S12>/UD':
   *
   *  Store in Global RAM
   */
  single_qc_ppid_DW.UD_DSTATE_d =
    single_qc_ppid_P.DiscreteDerivative1_ICPrevSca_h;
}

/* Model terminate function */
void single_qc_ppid_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
