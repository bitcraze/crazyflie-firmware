/**
 * Authored by Mike Wen, 2024.March
 *
 * ============================================================================
 */

#include "controller_Gimbal2D.h"
#include "log.h"
#include "param.h"
#include "num.h"
#include "math3d.h"
#include "physicalConstants.h"

#include "stabilizer.h"
#include "stabilizer_types.h"
#include "debug.h"
#include "motors.h"

#define MOTOR_MAX_THRUST_N        (0.1472f)
#define GIMBAL2D_ATTITUDE_UPDATE_DT    (float)(1.0f / ATTITUDE_RATE)

// static struct mat33 CRAZYFLIE_INERTIA =
//     {{{16.6e-6f, 0.83e-6f, 0.72e-6f},
//       {0.83e-6f, 16.6e-6f, 1.8e-6f},
//       {0.72e-6f, 1.8e-6f, 29.3e-6f}}};

static bool isInit = false;

Gimbal2D_P_Type Gimbal2D_P = {
  .ControlMode = GIMBAL2D_CONTROLMODE_PID,
  .Kp = 1.0f,
  .ThrustUpperBound = 4.0f * MOTOR_MAX_THRUST_N,
  .ThrustLowerBound = 0.0f,
};

/**Instance of Input structure*/
Gimbal2D_U_Type Gimbal2D_U = {
        .qw_Base = 0.0,
        .qx_Base = 0.0,
        .qy_Base = 0.0,
        .qz_Base = 0.0,
        .index = 0.0,
        .thrust = 0.0, 
        .LastThrust = 0.0,
        .ClampedThrust = 0.0,
        .alpha_desired = 0.0,
        .beta_desired = 0.0,
        .qw_IMU = 0.0,
        .qx_IMU = 0.0,
        .qy_IMU = 0.0,
        .qz_IMU = 0.0,
        .omega_x = 0.0,
        .beta_speed = 0.0,
        .omega_z = 0.0,
        };

/**Instance of Output structure*/
Gimbal2D_Y_Type Gimbal2D_Y = {
        .IsClamped = 0,
        .Treset = 0,
        .m1 = 0,
        .m2 = 0,
        .m3 = 0,
        .m4 = 0,
        .acount_prev = 0.0,
        .bcount_prev = 0.0,
        .alpha_prev = 0.0,
        .beta_prev = 0.0,
        .alpha_e = 0.0,
        .beta_e = 0.0,
        .alpha_speed_e = 0.0,
        .beta_speed_e = 0.0,
        .error_alpha = 0.0,
        .error_beta = 0.0,
        .u_alpha = 0.0,
        .u_beta = 0.0,
        .t_m1 = 0.0,
        .t_m2 = 0.0,
        .t_m3 = 0.0,
        .t_m4 = 0.0,
        .error_alphas = 0.0,
        .error_betas = 0.0,
        .rollPart = 0.0,
        .pitchPart = 0.0,
        .yawPart = 0.0,
        .thrustPart = 0.0,
        .Tau_x = 0.0, 
        .Tau_y = 0.0,
        .Tau_z = 0.0,
        .alphaPID = {
            .kp = 900.0f,
            .ki = 10.0f,
            .kd = 0,
            .kff = 0,
        },
        .alphasPID = {
            .kp = 0.00007f/3.0f,
            .ki = 0.00006f/3.0f,
            .kd = 0.00003f/3.0f,
            .kff = 0,
        },
        .betaPID = {
            .kp = 900.0f,
            .ki = 10.0f,
            .kd = 0,
            .kff = 0,
        },
        .betasPID = {
            .kp = 0.00004f,
            .ki = 0.00004f,
            .kd = 0.00002f,
            .kff = 0,
        },
        };

static void Gimbal2D_quatmultiply(const float q[4], const float r[4],
  float qout[4])
{
  qout[0] = ((q[0] * r[0] - q[1] * r[1]) - q[2] * r[2]) - q[3] * r[3];
  qout[1] = (q[0] * r[1] + r[0] * q[1]) + (q[2] * r[3] - q[3] * r[2]);
  qout[2] = (q[0] * r[2] + r[0] * q[2]) + (q[3] * r[1] - q[1] * r[3]);
  qout[3] = (q[0] * r[3] + r[0] * q[3]) + (q[1] * r[2] - q[2] * r[1]);
}

void Gimbal2D_quatToDCM(float *quat, struct mat33 *RotM)
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

typedef struct {
  union {
    float wordLreal;
    unsigned int wordLuint;
  } wordL;
} IEEESingle;

unsigned char IsNaNF(float value)
{
  IEEESingle tmp;
  tmp.wordL.wordLreal = value;
  return (unsigned char)( (tmp.wordL.wordLuint & 0x7F800000) == 0x7F800000 &&
                     (tmp.wordL.wordLuint & 0x007FFFFF) != 0 );
}

unsigned char IsInfF(float value)
{
  IEEESingle infF;
  infF.wordL.wordLuint = 0x7F800000U;
  IEEESingle minfF;
  minfF.wordL.wordLuint = 0xFF800000U;
  float rtInfF = infF.wordL.wordLreal;
  float rtMinusInfF = minfF.wordL.wordLuint;
  return (unsigned char)(((value)==rtInfF || (value)==rtMinusInfF) ? 1U : 0U);
}

float atan2f_snf(float u0, float u1)
{
  float y;
  int u0_0;
  int u1_0;
  if (IsNaNF(u0) || IsNaNF(u1)) {
    y = (float)0xFFC00000U;
  } else if (IsInfF(u0) && IsInfF(u1)) {
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

    y = atan2f((float)u0_0, (float)u1_0);
  } else if (u1 == 0.0F) {
    if (u0 > 0.0F) {
      y = 3.1415927F / 2.0F;
    } else if (u0 < 0.0F) {
      y = -(3.1415927F / 2.0F);
    } else {
      y = 0.0F;
    }
  } else {
    y = atan2f(u0, u1);
  }

  return y;
}

void controllerGimbal2DInit(void) {
  if (isInit) {
    return;
  }

  pidInit(&Gimbal2D_Y.alphaPID,  0, Gimbal2D_Y.alphaPID.kp,  Gimbal2D_Y.alphaPID.ki,  Gimbal2D_Y.alphaPID.kd,
      Gimbal2D_Y.alphaPID.kff,  GIMBAL2D_ATTITUDE_UPDATE_DT, ATTITUDE_RATE, 0, 0);
  pidInit(&Gimbal2D_Y.alphasPID,  0, Gimbal2D_Y.alphasPID.kp,  Gimbal2D_Y.alphasPID.ki,  Gimbal2D_Y.alphasPID.kd,
      Gimbal2D_Y.alphasPID.kff,  GIMBAL2D_ATTITUDE_UPDATE_DT, ATTITUDE_RATE, 0, 0);
  pidInit(&Gimbal2D_Y.betaPID,  0, Gimbal2D_Y.betaPID.kp,  Gimbal2D_Y.betaPID.ki,  Gimbal2D_Y.betaPID.kd,
      Gimbal2D_Y.betaPID.kff,  GIMBAL2D_ATTITUDE_UPDATE_DT, ATTITUDE_RATE, 0, 0);
  pidInit(&Gimbal2D_Y.betasPID,  0, Gimbal2D_Y.betasPID.kp,  Gimbal2D_Y.betasPID.ki,  Gimbal2D_Y.betasPID.kd,
      Gimbal2D_Y.betasPID.kff,  GIMBAL2D_ATTITUDE_UPDATE_DT, ATTITUDE_RATE, 0, 0);
      
  isInit = true;
}

void Gimbal2D_AlphaBetaEstimator()
{
  // rotation about z for 90 deg
  static const float qt[4] = { 0.707106769F, 0.0F, 0.0F, 0.707106769F };
  static const float qt_inv[4] = { -0.707106769F, 0.0F, 0.0F, 0.707106769F };
  float q_tmp[4], q_i[4], q_Bbi[4], q_Bbi_inv[4], q_bi[4], q_bi_inv[4], q_bii[4];
  Gimbal2D_quatmultiply((float*)&Gimbal2D_U.qw_IMU, qt_inv, q_tmp);
  Gimbal2D_quatmultiply(qt, q_tmp, q_i);
  
  // change of basis of base quat on ith qc
  float q_Bbi_tmp = -Gimbal2D_U.index * 3.14159274F / 4.0F;
  q_Bbi[0] = cosf(q_Bbi_tmp);
  q_Bbi[1] = 0.0F;
  q_Bbi[2] = 0.0F;
  q_Bbi[3] = sinf(q_Bbi_tmp);

  q_Bbi_inv[0] = -q_Bbi[0];
  q_Bbi_inv[1] = q_Bbi[1];
  q_Bbi_inv[2] = q_Bbi[2];
  q_Bbi_inv[3] = q_Bbi[3];

  // ith base quat
  Gimbal2D_quatmultiply((float*)&Gimbal2D_U.qw_Base, q_Bbi_inv, q_tmp);
  Gimbal2D_quatmultiply(q_Bbi, q_tmp, q_bi);
  
  // inv of ith base quat
  q_bi_inv[0] = -q_bi[0];
  q_bi_inv[1] = q_bi[1];
  q_bi_inv[2] = q_bi[2];
  q_bi_inv[3] = q_bi[3];

  // rotation from ith base to qc
  Gimbal2D_quatmultiply(q_bi_inv, q_i, q_bii);

  // change to R3x3
  struct mat33 RotM;
  Gimbal2D_quatToDCM(q_bii, &RotM);
  RotM = mtranspose(RotM);

  // incremental alpha-beta estimator
  float alpha0_e = atan2f_snf(RotM.m[1][2], RotM.m[1][1]);
  float beta0_e = atan2f_snf(RotM.m[2][0], RotM.m[0][0]);

  // Thrust Clamper
  if (Gimbal2D_U.thrust >
      Gimbal2D_P.ThrustUpperBound) {
    Gimbal2D_U.ClampedThrust = Gimbal2D_P.ThrustUpperBound;
  } else if (Gimbal2D_U.thrust <
             Gimbal2D_P.ThrustLowerBound) {
    Gimbal2D_U.ClampedThrust = Gimbal2D_P.ThrustLowerBound;
  } else {
    Gimbal2D_U.ClampedThrust = Gimbal2D_U.thrust;
  }

  if(Gimbal2D_U.ClampedThrust >= 0.000898f && Gimbal2D_U.LastThrust <= 0.000898f)
  {
    Gimbal2D_Y.acount_prev = 0.0F;
    Gimbal2D_Y.bcount_prev = 0.0F;
  }

  float diff_a = Gimbal2D_Y.alpha_prev - alpha0_e;
  float acount = Gimbal2D_Y.acount_prev;
  
  if (fabsf(diff_a) > 3.14159274F) {
    if (diff_a < 0.0F) {
      diff_a = -1.0F;
    } else if (diff_a > 0.0F) {
      diff_a = 1.0F;
    } else if (diff_a == 0.0F) {
      diff_a = 0.0F;
    }
    acount += diff_a;
  }
  Gimbal2D_Y.alpha_e = alpha0_e + 6.28318548F * acount;

  float diff_b = Gimbal2D_Y.beta_prev - beta0_e;
  float bcount = Gimbal2D_Y.bcount_prev;
  
  if (fabsf(diff_b) > 3.14159274F) {
    if (diff_b < 0.0F) {
      diff_b = -1.0F;
    } else if (diff_b > 0.0F) {
      diff_b = 1.0F;
    } else if (diff_b == 0.0F) {
      diff_b = 0.0F;
    }
    acount += diff_b;
  }
  Gimbal2D_Y.beta_e = beta0_e + 6.28318548F * bcount;

  // speed estimator
  Gimbal2D_Y.alpha_speed_e = Gimbal2D_U.omega_x * cosf(Gimbal2D_Y.beta_e) + Gimbal2D_U.omega_z * sinf(Gimbal2D_Y.beta_e);
  Gimbal2D_Y.beta_speed_e = Gimbal2D_U.beta_speed;

  // Last
  Gimbal2D_U.LastThrust = Gimbal2D_U.ClampedThrust;
  Gimbal2D_Y.acount_prev = acount;
  Gimbal2D_Y.bcount_prev = bcount;
  Gimbal2D_Y.alpha_prev = Gimbal2D_Y.alpha_e;
  Gimbal2D_Y.beta_prev = Gimbal2D_Y.beta_e; 
}

void Gimbal2D_controller()
{
    // Update your control law here
  float alphas_desired;
  float betas_desired;

  // float J_alpha = 1.66e-5f*cosf(Gimbal2D_Y.beta_e) + 2.93e-5f*sinf(Gimbal2D_Y.beta_e);
  float Jx = 1.66e-5f, Jy = 1.66e-5f, Jz = 2.93e-5f;
  float J_alpha, J_beta;

  if(Gimbal2D_P.ControlMode==GIMBAL2D_CONTROLMODE_PID_JALPHA)
  {
    float temp = (Jz-Jx)*sinf(Gimbal2D_Y.beta_e)*sinf(Gimbal2D_Y.beta_e);
    J_alpha = Jz*(temp - Jx)/(2.0f*temp - Jz);
  } else {
    J_alpha = Jx;
  }

  J_beta = Jy;

  Gimbal2D_Y.error_alpha = Gimbal2D_U.alpha_desired - Gimbal2D_Y.alpha_e;
  Gimbal2D_Y.error_beta = Gimbal2D_U.beta_desired - Gimbal2D_Y.beta_e;

  pidSetError(&Gimbal2D_Y.alphaPID, Gimbal2D_Y.error_alpha);
  alphas_desired = pidUpdate(&Gimbal2D_Y.alphaPID, Gimbal2D_Y.alpha_e, false);

  pidSetError(&Gimbal2D_Y.betaPID, Gimbal2D_Y.error_beta);
  betas_desired = pidUpdate(&Gimbal2D_Y.betaPID, Gimbal2D_Y.beta_e, false);

  pidSetDesired(&Gimbal2D_Y.alphasPID, alphas_desired);
  Gimbal2D_Y.u_alpha = J_alpha * pidUpdate(&Gimbal2D_Y.alphasPID, Gimbal2D_Y.alpha_speed_e, true);

  pidSetDesired(&Gimbal2D_Y.betasPID, betas_desired);
  Gimbal2D_Y.u_beta = J_beta * pidUpdate(&Gimbal2D_Y.betasPID, Gimbal2D_Y.beta_speed_e, true);
}

void Gimbal2D_PowerDistribution()
{
  Gimbal2D_Y.Tau_x = Gimbal2D_Y.u_alpha * cosf(Gimbal2D_Y.beta_e);
  Gimbal2D_Y.Tau_y = Gimbal2D_Y.u_beta;
  Gimbal2D_Y.Tau_z = Gimbal2D_Y.u_alpha * sinf(Gimbal2D_Y.beta_e);

    // power distribution and turn Nm into N
  const float arm = 0.707106781f * 0.046f;
  const float rollPart = 0.25f / arm * Gimbal2D_Y.Tau_x;
  const float pitchPart = 0.25f / arm * Gimbal2D_Y.Tau_y;
  const float thrustPart = 0.25f * Gimbal2D_U.ClampedThrust; // N (per rotor)
  const float yawPart = 0.25f * Gimbal2D_Y.Tau_z / 0.005964552f;

  Gimbal2D_Y.rollPart = rollPart;
  Gimbal2D_Y.pitchPart = pitchPart;
  Gimbal2D_Y.thrustPart = thrustPart;
  Gimbal2D_Y.yawPart = yawPart;

  // corresponding to i-frame Body coordinate, t_mi 's Unit is Newton
  Gimbal2D_Y.t_m1 = thrustPart + rollPart - pitchPart - yawPart;
  Gimbal2D_Y.t_m2 = thrustPart - rollPart - pitchPart + yawPart;
  Gimbal2D_Y.t_m3 = thrustPart - rollPart + pitchPart - yawPart;
  Gimbal2D_Y.t_m4 = thrustPart + rollPart + pitchPart + yawPart;

  Gimbal2D_Y.IsClamped = 0;
  if (Gimbal2D_Y.t_m1 < 0.0f) 
  {
    Gimbal2D_Y.t_m1 = 0.0f;
    Gimbal2D_Y.IsClamped = 1;
  } else if (Gimbal2D_Y.t_m1 > MOTOR_MAX_THRUST_N) {
    Gimbal2D_Y.t_m1 = MOTOR_MAX_THRUST_N;
    Gimbal2D_Y.IsClamped = 1;
  }

  if (Gimbal2D_Y.t_m2 < 0.0f) 
  {
    Gimbal2D_Y.t_m2 = 0.0f;
    Gimbal2D_Y.IsClamped = 1;
  } else if (Gimbal2D_Y.t_m2 > MOTOR_MAX_THRUST_N) {
    Gimbal2D_Y.t_m2 = MOTOR_MAX_THRUST_N;
    Gimbal2D_Y.IsClamped = 1;
  }

  if (Gimbal2D_Y.t_m3 < 0.0f) 
  {
    Gimbal2D_Y.t_m3 = 0.0f;
    Gimbal2D_Y.IsClamped = 1;
  } else if (Gimbal2D_Y.t_m3 > MOTOR_MAX_THRUST_N) {
    Gimbal2D_Y.t_m3 = MOTOR_MAX_THRUST_N;
    Gimbal2D_Y.IsClamped = 1;
  }

  if (Gimbal2D_Y.t_m4 < 0.0f) 
  {
    Gimbal2D_Y.t_m4 = 0.0f;
    Gimbal2D_Y.IsClamped = 1;
  } else if (Gimbal2D_Y.t_m4 > MOTOR_MAX_THRUST_N) {
    Gimbal2D_Y.t_m4 = MOTOR_MAX_THRUST_N;
    Gimbal2D_Y.IsClamped = 1;
  }

  // Turn Newton into percentage and count
  Gimbal2D_Y.m1 = Gimbal2D_Y.t_m1 / MOTOR_MAX_THRUST_N * 65535;
  Gimbal2D_Y.m2 = Gimbal2D_Y.t_m2 / MOTOR_MAX_THRUST_N * 65535;
  Gimbal2D_Y.m3 = Gimbal2D_Y.t_m3 / MOTOR_MAX_THRUST_N * 65535;
  Gimbal2D_Y.m4 = Gimbal2D_Y.t_m4 / MOTOR_MAX_THRUST_N * 65535;
}

void controllerGimbal2D(control_t *control,
                                 const setpoint_t *setpoint,
                                 const sensorData_t *sensors,
                                 const state_t *state,
                                 const stabilizerStep_t stabilizerStep) 
{
  if (RATE_DO_EXECUTE(ATTITUDE_RATE, stabilizerStep)) {

    // Update Command (Send2D)
    Gimbal2D_U.index = setpoint->attitude.roll;

    Gimbal2D_U.qw_Base = setpoint->attitudeQuaternion.w;
    Gimbal2D_U.qx_Base = setpoint->attitudeQuaternion.x;
    Gimbal2D_U.qy_Base = setpoint->attitudeQuaternion.y;
    Gimbal2D_U.qz_Base = setpoint->attitudeQuaternion.z;

    Gimbal2D_U.alpha_desired = setpoint->attitude.pitch;
    Gimbal2D_U.beta_desired = setpoint->attitude.yaw;
    Gimbal2D_U.thrust = setpoint->thrust;

    // Update Feedback
    Gimbal2D_U.qw_IMU = state->attitudeQuaternion.w;
    Gimbal2D_U.qx_IMU = state->attitudeQuaternion.x;
    Gimbal2D_U.qy_IMU = state->attitudeQuaternion.y;
    Gimbal2D_U.qz_IMU = state->attitudeQuaternion.z;

    Gimbal2D_U.omega_x = -radians(sensors->gyro.y);
    Gimbal2D_U.beta_speed = radians(sensors->gyro.x);
    Gimbal2D_U.omega_z = radians(sensors->gyro.z);

    // deg /s 
    // Gimbal2D_U.omega_x = -sensors->gyro.y;
    // Gimbal2D_U.beta_speed = sensors->gyro.x;
    // Gimbal2D_U.omega_z = sensors->gyro.z;

    Gimbal2D_AlphaBetaEstimator();
    Gimbal2D_controller();
    Gimbal2D_PowerDistribution();

    // Update the output
    if (setpoint->thrust < 0.000898f)
    {
      motorsSetRatio(0, 0);
      motorsSetRatio(1, 0);
      motorsSetRatio(2, 0);
      motorsSetRatio(3, 0);    
    }
    else
    {
      motorsSetRatio(0, Gimbal2D_Y.m1);
      motorsSetRatio(1, Gimbal2D_Y.m2);
      motorsSetRatio(2, Gimbal2D_Y.m3);
      motorsSetRatio(3, Gimbal2D_Y.m4);
    }
  }
}

bool controllerGimbal2DTest(void) {
  return true;
}

// Update your parameter here
// The name of the variable cannot be too long
PARAM_GROUP_START(sparam_Gimbal2D)
PARAM_ADD(PARAM_FLOAT, pgaina, &Gimbal2D_Y.alphaPID.kp)
PARAM_ADD(PARAM_FLOAT, igaina, &Gimbal2D_Y.alphaPID.ki)
PARAM_ADD(PARAM_FLOAT, dgaina, &Gimbal2D_Y.alphaPID.kd)

PARAM_ADD(PARAM_FLOAT, pgainb, &Gimbal2D_Y.betaPID.kp)
PARAM_ADD(PARAM_FLOAT, igainb, &Gimbal2D_Y.betaPID.ki)
PARAM_ADD(PARAM_FLOAT, dgainb, &Gimbal2D_Y.betaPID.kd)

PARAM_ADD(PARAM_FLOAT, pgainas, &Gimbal2D_Y.alphasPID.kp)
PARAM_ADD(PARAM_FLOAT, igainas, &Gimbal2D_Y.alphasPID.ki)
PARAM_ADD(PARAM_FLOAT, dgainas, &Gimbal2D_Y.alphasPID.kd)

PARAM_ADD(PARAM_FLOAT, pgainbs, &Gimbal2D_Y.betasPID.kp)
PARAM_ADD(PARAM_FLOAT, igainbs, &Gimbal2D_Y.betasPID.ki)
PARAM_ADD(PARAM_FLOAT, dgainbs, &Gimbal2D_Y.betasPID.kd)
PARAM_ADD(PARAM_FLOAT, cmode, &Gimbal2D_P.ControlMode)

PARAM_GROUP_STOP(sparam_Gimbal2D)

/**
 * Update your debug scope here
 * Logging variables for the command and reference signals for the
 * Gimbal2D controller
 */
LOG_GROUP_START(sctrl_Gimbal2D)
LOG_ADD(LOG_FLOAT, alpha, &Gimbal2D_Y.alpha_e)
LOG_ADD(LOG_FLOAT, alphas, &Gimbal2D_Y.alpha_speed_e)
LOG_ADD(LOG_FLOAT, beta, &Gimbal2D_Y.beta_e)
LOG_ADD(LOG_FLOAT, betas, &Gimbal2D_Y.beta_speed_e)

LOG_ADD(LOG_FLOAT, u_alpha, &Gimbal2D_Y.u_alpha)
LOG_ADD(LOG_FLOAT, u_beta, &Gimbal2D_Y.u_beta)

LOG_ADD(LOG_FLOAT, alphain, &Gimbal2D_U.alpha_desired)
LOG_ADD(LOG_FLOAT, betain, &Gimbal2D_U.beta_desired)

LOG_ADD(LOG_FLOAT, t_m1, &Gimbal2D_Y.t_m1)
LOG_ADD(LOG_FLOAT, t_m2, &Gimbal2D_Y.t_m2)
LOG_ADD(LOG_FLOAT, t_m3, &Gimbal2D_Y.t_m3)
LOG_ADD(LOG_FLOAT, t_m4, &Gimbal2D_Y.t_m4)
LOG_GROUP_STOP(sctrl_Gimbal2D)