/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: omni_attitude_controller.h
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

#ifndef RTW_HEADER_omni_attitude_controller_h_
#define RTW_HEADER_omni_attitude_controller_h_
#ifndef omni_attitude_controller_COMMON_INCLUDES_
#define omni_attitude_controller_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                           /* omni_attitude_controller_COMMON_INCLUDES_ */

#include "omni_attitude_controller_types.h"
#include <string.h>
#include "rt_nonfinite.h"

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real32_T Memory5_PreviousInput[3];   /* '<Root>/Memory5' */
} DW_omni_attitude_controller_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real32_T thrust;                     /* '<Root>/thrust' */
  real32_T qw_r;                       /* '<Root>/qw_r' */
  real32_T qx_r;                       /* '<Root>/qx_r' */
  real32_T qy_r;                       /* '<Root>/qy_r' */
  real32_T qz_r;                       /* '<Root>/qz_r' */
  real32_T qw_IMU;                     /* '<Root>/qw_IMU' */
  real32_T qx_IMU;                     /* '<Root>/qx_IMU' */
  real32_T qy_IMU;                     /* '<Root>/qy_IMU' */
  real32_T qz_IMU;                     /* '<Root>/qz_IMU' */
  real32_T wx_r;                       /* '<Root>/wx_r' */
  real32_T wy_r;                       /* '<Root>/wy_r' */
  real32_T wz_r;                       /* '<Root>/wz_r' */
  real32_T gyro_x;                     /* '<Root>/gyro_x' */
  real32_T gyro_y;                     /* '<Root>/gyro_y' */
  real32_T gyro_z;                     /* '<Root>/gyro_z' */
} ExtU_omni_attitude_controller_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  uint16_T m1;                         /* '<Root>/m1' */
  uint16_T m2;                         /* '<Root>/m2' */
  uint16_T m3;                         /* '<Root>/m3' */
  uint16_T m4;                         /* '<Root>/m4' */
  real32_T t_m1;                       /* '<Root>/t_m1' */
  real32_T t_m2;                       /* '<Root>/t_m2' */
  real32_T t_m3;                       /* '<Root>/t_m3' */
  real32_T t_m4;                       /* '<Root>/t_m4' */
  float uW_x;
  float uW_y;
  float uW_z;
  float uP_x;
  float uP_y;
  float uP_z;
  float tau_x;
  float tau_y;
  float tau_z;
  float debug[4];
} ExtY_omni_attitude_controller_T;

/* Parameters (default storage) */
struct P_omni_attitude_controller_T_ {
  real_T Ji[9];                        /* Variable: Ji
                                        * Referenced by: '<Root>/LLATC'
                                        */
  real_T KR[9];                        /* Variable: KR
                                        * Referenced by: '<Root>/LLATC'
                                        */
  real_T Ki[9];                        /* Variable: Ki
                                        * Referenced by: '<Root>/LLATC'
                                        */
  real_T Kw[9];                        /* Variable: Kw
                                        * Referenced by: '<Root>/LLATC'
                                        */
  real_T Constant_Value;               /* Expression: (31.65e-3)/(5.964552e-3)
                                        * Referenced by: '<Root>/Constant'
                                        */
  real32_T Saturation_UpperSat;       /* Computed Parameter: Saturation_UpperSat
                                       * Referenced by: '<Root>/Saturation'
                                       */
  real32_T Saturation_LowerSat;       /* Computed Parameter: Saturation_LowerSat
                                       * Referenced by: '<Root>/Saturation'
                                       */
  real32_T Gain_Gain;                  /* Computed Parameter: Gain_Gain
                                        * Referenced by: '<S8>/Gain'
                                        */
  real32_T Gain_Gain_l;                /* Computed Parameter: Gain_Gain_l
                                        * Referenced by: '<S11>/Gain'
                                        */
  real32_T Gain_Gain_f;                /* Computed Parameter: Gain_Gain_f
                                        * Referenced by: '<S6>/Gain'
                                        */
  real32_T Gain_Gain_d;                /* Computed Parameter: Gain_Gain_d
                                        * Referenced by: '<S12>/Gain'
                                        */
  real32_T Gain_Gain_b;                /* Computed Parameter: Gain_Gain_b
                                        * Referenced by: '<S7>/Gain'
                                        */
  real32_T Gain_Gain_c;                /* Computed Parameter: Gain_Gain_c
                                        * Referenced by: '<S10>/Gain'
                                        */
  real32_T Gain_Gain_p;                /* Computed Parameter: Gain_Gain_p
                                        * Referenced by: '<S21>/Gain'
                                        */
  real32_T Gain_Gain_i;                /* Computed Parameter: Gain_Gain_i
                                        * Referenced by: '<S24>/Gain'
                                        */
  real32_T Gain_Gain_fl;               /* Computed Parameter: Gain_Gain_fl
                                        * Referenced by: '<S19>/Gain'
                                        */
  real32_T Gain_Gain_a;                /* Computed Parameter: Gain_Gain_a
                                        * Referenced by: '<S25>/Gain'
                                        */
  real32_T Gain_Gain_h;                /* Computed Parameter: Gain_Gain_h
                                        * Referenced by: '<S20>/Gain'
                                        */
  real32_T Gain_Gain_ao;               /* Computed Parameter: Gain_Gain_ao
                                        * Referenced by: '<S23>/Gain'
                                        */
  real32_T Memory5_InitialCondition[3];
                                 /* Computed Parameter: Memory5_InitialCondition
                                  * Referenced by: '<Root>/Memory5'
                                  */
  real32_T Saturation1_UpperSat;     /* Computed Parameter: Saturation1_UpperSat
                                      * Referenced by: '<Root>/Saturation1'
                                      */
  real32_T Saturation1_LowerSat;     /* Computed Parameter: Saturation1_LowerSat
                                      * Referenced by: '<Root>/Saturation1'
                                      */
};

/* Block parameters (default storage) */
extern P_omni_attitude_controller_T omni_attitude_controller_P;

/* Block states (default storage) */
extern DW_omni_attitude_controller_T omni_attitude_controller_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_omni_attitude_controller_T omni_attitude_controller_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_omni_attitude_controller_T omni_attitude_controller_Y;

/* Model entry point functions */
extern void omni_attitude_controller_initialize(void);
extern void omni_attitude_controller_step(void);
extern void omni_attitude_controller_terminate(void);

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S14>/Reshape (9) to [3x3] column-major' : Reshape block reduction
 * Block '<S27>/Reshape (9) to [3x3] column-major' : Reshape block reduction
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'omni_attitude_controller'
 * '<S1>'   : 'omni_attitude_controller/LLATC'
 * '<S2>'   : 'omni_attitude_controller/MATLAB Function1'
 * '<S3>'   : 'omni_attitude_controller/Quaternions to  Direction Cosine Matrix'
 * '<S4>'   : 'omni_attitude_controller/Quaternions to  Direction Cosine Matrix1'
 * '<S5>'   : 'omni_attitude_controller/Quaternions to  Direction Cosine Matrix/A11'
 * '<S6>'   : 'omni_attitude_controller/Quaternions to  Direction Cosine Matrix/A12'
 * '<S7>'   : 'omni_attitude_controller/Quaternions to  Direction Cosine Matrix/A13'
 * '<S8>'   : 'omni_attitude_controller/Quaternions to  Direction Cosine Matrix/A21'
 * '<S9>'   : 'omni_attitude_controller/Quaternions to  Direction Cosine Matrix/A22'
 * '<S10>'  : 'omni_attitude_controller/Quaternions to  Direction Cosine Matrix/A23'
 * '<S11>'  : 'omni_attitude_controller/Quaternions to  Direction Cosine Matrix/A31'
 * '<S12>'  : 'omni_attitude_controller/Quaternions to  Direction Cosine Matrix/A32'
 * '<S13>'  : 'omni_attitude_controller/Quaternions to  Direction Cosine Matrix/A33'
 * '<S14>'  : 'omni_attitude_controller/Quaternions to  Direction Cosine Matrix/Create 3x3 Matrix'
 * '<S15>'  : 'omni_attitude_controller/Quaternions to  Direction Cosine Matrix/Quaternion Normalize'
 * '<S16>'  : 'omni_attitude_controller/Quaternions to  Direction Cosine Matrix/Quaternion Normalize/Quaternion Modulus'
 * '<S17>'  : 'omni_attitude_controller/Quaternions to  Direction Cosine Matrix/Quaternion Normalize/Quaternion Modulus/Quaternion Norm'
 * '<S18>'  : 'omni_attitude_controller/Quaternions to  Direction Cosine Matrix1/A11'
 * '<S19>'  : 'omni_attitude_controller/Quaternions to  Direction Cosine Matrix1/A12'
 * '<S20>'  : 'omni_attitude_controller/Quaternions to  Direction Cosine Matrix1/A13'
 * '<S21>'  : 'omni_attitude_controller/Quaternions to  Direction Cosine Matrix1/A21'
 * '<S22>'  : 'omni_attitude_controller/Quaternions to  Direction Cosine Matrix1/A22'
 * '<S23>'  : 'omni_attitude_controller/Quaternions to  Direction Cosine Matrix1/A23'
 * '<S24>'  : 'omni_attitude_controller/Quaternions to  Direction Cosine Matrix1/A31'
 * '<S25>'  : 'omni_attitude_controller/Quaternions to  Direction Cosine Matrix1/A32'
 * '<S26>'  : 'omni_attitude_controller/Quaternions to  Direction Cosine Matrix1/A33'
 * '<S27>'  : 'omni_attitude_controller/Quaternions to  Direction Cosine Matrix1/Create 3x3 Matrix'
 * '<S28>'  : 'omni_attitude_controller/Quaternions to  Direction Cosine Matrix1/Quaternion Normalize'
 * '<S29>'  : 'omni_attitude_controller/Quaternions to  Direction Cosine Matrix1/Quaternion Normalize/Quaternion Modulus'
 * '<S30>'  : 'omni_attitude_controller/Quaternions to  Direction Cosine Matrix1/Quaternion Normalize/Quaternion Modulus/Quaternion Norm'
 */
#endif                              /* RTW_HEADER_omni_attitude_controller_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
