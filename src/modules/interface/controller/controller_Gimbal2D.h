/**
 * Authored by Mike Wen, 2024.March
 *
 * ============================================================================
 */

#pragma once

#include "stabilizer_types.h"
#include "pid.h"

enum CONTROL_MODE
{
  GIMBAL2D_CONTROLMODE_PID = 0,
  GIMBAL2D_CONTROLMODE_PID_JALPHA = 1,

};

typedef struct {
  float qw_Base;                      /* '<Root>/qw_op' */
  float qx_Base;                      /* '<Root>/qx_op' */
  float qy_Base;                      /* '<Root>/qy_op' */
  float qz_Base;                      /* '<Root>/qz_op' */
  float index;                      /* '<Root>/index' */
  float thrust;                     /* '<Root>/thrust' */
  float LastThrust;
  float ClampedThrust;
  float alpha_desired;              /* '<Root>/alpha_desired' */
  float beta_desired;               /* '<Root>/beta_desired' */
  float qw_IMU;                     /* '<Root>/qw_IMU' */
  float qx_IMU;                     /* '<Root>/qx_IMU' */
  float qy_IMU;                     /* '<Root>/qy_IMU' */
  float qz_IMU;                     /* '<Root>/qz_IMU' */
  float omega_x;                    /* '<Root>/omega_x' */
  float beta_speed;                 /* '<Root>/beta_speed' */
  float omega_z;                    /* '<Root>/omega_z' */
} Gimbal2D_U_Type;

typedef struct {
  unsigned short IsClamped;
  unsigned short Treset;
  unsigned short m1;                         /* '<Root>/m1' */
  unsigned short m2;                         /* '<Root>/m2' */
  unsigned short m3;                         /* '<Root>/m3' */
  unsigned short m4;                         /* '<Root>/m4' */
  float acount_prev;
  float bcount_prev;
  float alpha_prev;
  float beta_prev;
  float alpha_e;
  float beta_e;
  float alpha_speed_e;
  float beta_speed_e;
  float error_alpha;                /* '<Root>/error_alpha' */
  float error_beta;                 /* '<Root>/error_beta' */
  float u_alpha;                    /* '<Root>/u_alpha' */
  float u_beta;                     /* '<Root>/u_beta' */
  float t_m1;                       /* '<Root>/t_m1' */
  float t_m2;                       /* '<Root>/t_m2' */
  float t_m3;                       /* '<Root>/t_m3' */
  float t_m4;                       /* '<Root>/t_m4' */
  float error_alphas;               /* '<Root>/error_alphas' */
  float error_betas;                /* '<Root>/error_betas' */
  float rollPart;
  float pitchPart;
  float yawPart;
  float thrustPart;
  float Tau_x;                      /* '<Root>/Tau_x' */
  float Tau_y;                      /* '<Root>/Tau_y' */
  float Tau_z;                      /* '<Root>/Tau_z' */
  PidObject alphaPID;
  PidObject betaPID;
  PidObject alphasPID;
  PidObject betasPID;
} Gimbal2D_Y_Type;

typedef struct {
    unsigned short ControlMode;
    float Kp;
    float ThrustUpperBound;
    float ThrustLowerBound;
} Gimbal2D_P_Type;

extern Gimbal2D_P_Type Gimbal2D_P;
extern Gimbal2D_U_Type Gimbal2D_U;
extern Gimbal2D_Y_Type Gimbal2D_Y;

void controllerGimbal2DInit(void);
bool controllerGimbal2DTest(void);
void controllerGimbal2D(control_t *control,
                        const setpoint_t *setpoint,
                        const sensorData_t *sensors,
                        const state_t *state,
                        const stabilizerStep_t stabilizerStep);
