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

static struct mat33 CRAZYFLIE_INERTIA =
    {{{16.6e-6f, 0.83e-6f, 0.72e-6f},
      {0.83e-6f, 16.6e-6f, 1.8e-6f},
      {0.72e-6f, 1.8e-6f, 29.3e-6f}}};

static bool isInit = false;

Gimbal2D_P_Type Gimbal2D_P = {
  .Kp = 1.0f,
};

/**Instance of Input structure*/
Gimbal2D_U_Type Gimbal2D_U = {
        .qw_Base = 0.0,
        .qx_Base = 0.0,
        .qy_Base = 0.0,
        .qz_Base = 0.0,
        .index = 0.0,
        .thrust = 0.0, 
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
        .m1 = 0,
        .m2 = 0,
        .m3 = 0,
        .m4 = 0,
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
        };

void controllerGimbal2DInit(void) {
  if (isInit) {
    return;
  }

  isInit = true;
}

#define GIMBAL2D_ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)

void Gimbal2D_step()
{
  // Update your control law here
  Gimbal2D_Y.m1 = 0;
  Gimbal2D_Y.m2 = 0;
  Gimbal2D_Y.m3 = 0;
  Gimbal2D_Y.m4 = 0;
}

void controllerGimbal2D(control_t *control,
                                 const setpoint_t *setpoint,
                                 const sensorData_t *sensors,
                                 const state_t *state,
                                 const stabilizerStep_t stabilizerStep) 
{
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

  Gimbal2D_step();

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

bool controllerGimbal2DTest(void) {
  return true;
}

// Update your parameter here
PARAM_GROUP_START(ctrlGimbal2D)
PARAM_ADD(PARAM_FLOAT, kp, &Gimbal2D_P.Kp)
PARAM_GROUP_STOP(ctrlGimbal2D)


/**
 * Update your debug scope here
 * Logging variables for the command and reference signals for the
 * Gimbal2D controller
 */
LOG_GROUP_START(ctrlGimbal2D)
LOG_GROUP_STOP(ctrlGimbal2D)