/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: omni_attitude_controller_data.c
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

/* Block parameters (default storage) */
P_omni_attitude_controller_T omni_attitude_controller_P = {
  /* Variable: A
   * Referenced by: '<Root>/MATLAB Function1'
   */
  0.091492681,

  /* Variable: B
   * Referenced by: '<Root>/MATLAB Function1'
   */
  0.067673604,

  /* Variable: Ji
   * Referenced by: '<Root>/LLATC'
   */
  { 1.2E-5, 0.0, 0.0, 0.0, 1.2E-5, 0.0, 0.0, 0.0, 2.2E-5 },

  /* Variable: KR
   * Referenced by: '<Root>/LLATC'
   */
  { 333.0, 0.0, 0.0, 0.0, 333.0, 0.0, 0.0, 0.0, 272.0 },

  /* Variable: Kw
   * Referenced by: '<Root>/LLATC'
   */
  { 62.5, 0.0, 0.0, 0.0, 62.5, 0.0, 0.0, 0.0, 45.0 },

  /* Ki */
  { 1.67, 1.67, 1.25 },

  /* Variable: ThrustToTorque
   * Referenced by: '<Root>/MATLAB Function1'
   */
  0.005964552,

  /* Variable: a_perpen
   * Referenced by: '<Root>/MATLAB Function1'
   */
  0.032521999999999995,

  /* Computed Parameter: Saturation_UpperSat
   * Referenced by: '<Root>/Saturation'
   */
  0.7F,

  /* Computed Parameter: Saturation_LowerSat
   * Referenced by: '<Root>/Saturation'
   */
  0.0F,

  /* Computed Parameter: Gain_Gain
   * Referenced by: '<S8>/Gain'
   */
  2.0F,

  /* Computed Parameter: Gain_Gain_l
   * Referenced by: '<S11>/Gain'
   */
  2.0F,

  /* Computed Parameter: Gain_Gain_f
   * Referenced by: '<S6>/Gain'
   */
  2.0F,

  /* Computed Parameter: Gain_Gain_d
   * Referenced by: '<S12>/Gain'
   */
  2.0F,

  /* Computed Parameter: Gain_Gain_b
   * Referenced by: '<S7>/Gain'
   */
  2.0F,

  /* Computed Parameter: Gain_Gain_c
   * Referenced by: '<S10>/Gain'
   */
  2.0F,

  /* Computed Parameter: Gain_Gain_p
   * Referenced by: '<S21>/Gain'
   */
  2.0F,

  /* Computed Parameter: Gain_Gain_i
   * Referenced by: '<S24>/Gain'
   */
  2.0F,

  /* Computed Parameter: Gain_Gain_fl
   * Referenced by: '<S19>/Gain'
   */
  2.0F,

  /* Computed Parameter: Gain_Gain_a
   * Referenced by: '<S25>/Gain'
   */
  2.0F,

  /* Computed Parameter: Gain_Gain_h
   * Referenced by: '<S20>/Gain'
   */
  2.0F,

  /* Computed Parameter: Gain_Gain_ao
   * Referenced by: '<S23>/Gain'
   */
  2.0F,

  /* Computed Parameter: Saturation1_UpperSat
   * Referenced by: '<Root>/Saturation1'
   */
  65535.0F,

  /* Computed Parameter: Saturation1_LowerSat
   * Referenced by: '<Root>/Saturation1'
   */
  0.0F
};

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
