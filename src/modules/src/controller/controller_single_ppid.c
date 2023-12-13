#define DEBUG_MODULE "SinglePPID"

#include "stabilizer.h"
#include "stabilizer_types.h"

// #include "attitude_controller.h"
#include "sensfusion6.h"
// #include "position_controller.h"
#include "controller_single_ppid.h"
#include "single_qc_ppid.h"
#include "debug.h"
#include "motors.h"

#include "log.h"
#include "param.h"

#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)


void controllerSinglePPIDInit(void)
{
  single_qc_ppid_initialize();
}

bool controllerSinglePPIDTest(void)
{
  return true;
}

void controllerSinglePPID(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const stabilizerStep_t stabilizerStep)
{
  single_qc_ppid_U.index = setpoint->attitude.roll;

  single_qc_ppid_U.qw_op = setpoint->attitudeQuaternion.w;
  single_qc_ppid_U.qx_op = setpoint->attitudeQuaternion.x;
  single_qc_ppid_U.qy_op = setpoint->attitudeQuaternion.y;
  single_qc_ppid_U.qz_op = setpoint->attitudeQuaternion.z;

  single_qc_ppid_U.qw_IMU = state->attitudeQuaternion.w;
  single_qc_ppid_U.qx_IMU = state->attitudeQuaternion.x;
  single_qc_ppid_U.qy_IMU = state->attitudeQuaternion.y;
  single_qc_ppid_U.qz_IMU = state->attitudeQuaternion.z;

  single_qc_ppid_U.alpha_desired = setpoint->attitude.pitch;
  single_qc_ppid_U.beta_desired = setpoint->attitude.yaw;

  single_qc_ppid_U.omega_x = -sensors->gyro.y;
  single_qc_ppid_U.beta_speed = sensors->gyro.x;
  single_qc_ppid_U.omega_z = sensors->gyro.z;

  single_qc_ppid_U.thrust = setpoint->thrust;

  single_qc_ppid_step();

  if (setpoint->thrust < 0.000898f)
  {
    motorsSetRatio(0, 0);
    motorsSetRatio(1, 0);
    motorsSetRatio(2, 0);
    motorsSetRatio(3, 0);    
  }
  else
  {
    motorsSetRatio(0, single_qc_ppid_Y.m1);
    motorsSetRatio(1, single_qc_ppid_Y.m2);
    motorsSetRatio(2, single_qc_ppid_Y.m3);
    motorsSetRatio(3, single_qc_ppid_Y.m4);
  }
}


// log/param name can't be too long, otherwise error
// only one log group is allowed

LOG_GROUP_START(sctrl_ppid)

LOG_ADD(LOG_FLOAT, e_alpha, &single_qc_ppid_Y.error_alpha)
LOG_ADD(LOG_FLOAT, e_beta, &single_qc_ppid_Y.error_beta)
LOG_ADD(LOG_FLOAT, e_alphas, &single_qc_ppid_Y.error_alphas)
LOG_ADD(LOG_FLOAT, e_betas, &single_qc_ppid_Y.error_betas)
LOG_ADD(LOG_FLOAT, u_alpha, &single_qc_ppid_Y.u_alpha)
LOG_ADD(LOG_FLOAT, u_beta, &single_qc_ppid_Y.u_beta)

LOG_ADD(LOG_FLOAT, t_be, &single_qc_ppid_Y.t_betae)
LOG_ADD(LOG_FLOAT, t_bin, &single_qc_ppid_Y.t_betain)
LOG_ADD(LOG_FLOAT, t_ae, &single_qc_ppid_Y.t_alphae)
LOG_ADD(LOG_FLOAT, t_ain, &single_qc_ppid_Y.t_alphain)

LOG_ADD(LOG_FLOAT, x_gyro, &single_qc_ppid_U.omega_x)
LOG_ADD(LOG_FLOAT, b_gyro, &single_qc_ppid_U.beta_speed)
LOG_ADD(LOG_FLOAT, z_gyro, &single_qc_ppid_U.omega_z)

LOG_ADD(LOG_FLOAT, t_m1, &single_qc_ppid_Y.t_m1)
LOG_ADD(LOG_FLOAT, t_m2, &single_qc_ppid_Y.t_m2)
LOG_ADD(LOG_FLOAT, t_m3, &single_qc_ppid_Y.t_m3)
LOG_ADD(LOG_FLOAT, t_m4, &single_qc_ppid_Y.t_m4)

LOG_GROUP_STOP(sctrl_ppid)




PARAM_GROUP_START(sparam_ppid)
PARAM_ADD(PARAM_FLOAT, pgaina, &single_qc_ppid_P.pgaina)
PARAM_ADD(PARAM_FLOAT, igaina, &single_qc_ppid_P.igaina)
PARAM_ADD(PARAM_FLOAT, dgaina, &single_qc_ppid_P.dgaina)

PARAM_ADD(PARAM_FLOAT, pgainb, &single_qc_ppid_P.pgainb)
PARAM_ADD(PARAM_FLOAT, igainb, &single_qc_ppid_P.igainb)
PARAM_ADD(PARAM_FLOAT, dgainb, &single_qc_ppid_P.dgainb)

PARAM_ADD(PARAM_FLOAT, pgainas, &single_qc_ppid_P.pgainas)
PARAM_ADD(PARAM_FLOAT, igainas, &single_qc_ppid_P.igainas)
PARAM_ADD(PARAM_FLOAT, dgainas, &single_qc_ppid_P.dgainas)

PARAM_ADD(PARAM_FLOAT, pgainbs, &single_qc_ppid_P.pgainbs)
PARAM_ADD(PARAM_FLOAT, igainbs, &single_qc_ppid_P.igainbs)
PARAM_ADD(PARAM_FLOAT, dgainbs, &single_qc_ppid_P.dgainbs)

PARAM_ADD(PARAM_FLOAT, t_mod, &single_qc_ppid_P.torque_modifier)

PARAM_ADD(PARAM_FLOAT, s_tx, &single_qc_ppid_P.sat_tx)
PARAM_ADD(PARAM_FLOAT, s_ty, &single_qc_ppid_P.sat_ty)
PARAM_ADD(PARAM_FLOAT, s_tz, &single_qc_ppid_P.sat_tz)
PARAM_GROUP_STOP(sparam_ppid)

