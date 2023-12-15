#define DEBUG_MODULE "SinglePPID"

#include "stabilizer.h"
#include "stabilizer_types.h"

// #include "attitude_controller.h"
#include "sensfusion6.h"
// #include "position_controller.h"
#include "controller_omni_att.h"
#include "omni_attitude_controller.h"
#include "debug.h"
#include "motors.h"

#include "log.h"
#include "param.h"

#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)


void controllerOmniAttInit(void)
{
  omni_attitude_controller_initialize();
}

bool controllerOmniAttTest(void)
{
  return true;
}

void controllerOmniAtt(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const stabilizerStep_t stabilizerStep)
{
  omni_attitude_controller_U.thrust = setpoint->thrust;

  omni_attitude_controller_U.qw_r = setpoint->attitudeQuaternion.w;
  omni_attitude_controller_U.qx_r = setpoint->attitudeQuaternion.x;
  omni_attitude_controller_U.qy_r = setpoint->attitudeQuaternion.y;
  omni_attitude_controller_U.qz_r = setpoint->attitudeQuaternion.z;

  omni_attitude_controller_U.wx_r = setpoint->attitude.roll;
  omni_attitude_controller_U.wy_r = setpoint->attitude.pitch;
  omni_attitude_controller_U.wz_r = setpoint->attitude.yaw;

  omni_attitude_controller_U.qw_IMU = state->attitudeQuaternion.w;
  omni_attitude_controller_U.qx_IMU = state->attitudeQuaternion.x;
  omni_attitude_controller_U.qy_IMU = state->attitudeQuaternion.y;
  omni_attitude_controller_U.qz_IMU = state->attitudeQuaternion.z;

  omni_attitude_controller_U.gyro_x = sensors->gyro.x;
  omni_attitude_controller_U.gyro_y = sensors->gyro.y;
  omni_attitude_controller_U.gyro_z = sensors->gyro.z;

  omni_attitude_controller_step();

  if (setpoint->thrust < 0.000898f)
  {
    motorsSetRatio(0, 0);
    motorsSetRatio(1, 0);
    motorsSetRatio(2, 0);
    motorsSetRatio(3, 0);    
  }
  else
  {
    motorsSetRatio(0, omni_attitude_controller_Y.m1);
    motorsSetRatio(1, omni_attitude_controller_Y.m2);
    motorsSetRatio(2, omni_attitude_controller_Y.m3);
    motorsSetRatio(3, omni_attitude_controller_Y.m4);
  }
}


// log/param name can't be too long, otherwise error
// only one log group is allowed

// LOG_GROUP_START(sctrl_ppid)

// LOG_ADD(LOG_FLOAT, e_alpha, &single_qc_ppid_Y.error_alpha)
// LOG_ADD(LOG_FLOAT, e_beta, &single_qc_ppid_Y.error_beta)
// LOG_ADD(LOG_FLOAT, e_alphas, &single_qc_ppid_Y.error_alphas)
// LOG_ADD(LOG_FLOAT, e_betas, &single_qc_ppid_Y.error_betas)
// LOG_ADD(LOG_FLOAT, u_alpha, &single_qc_ppid_Y.u_alpha)
// LOG_ADD(LOG_FLOAT, u_beta, &single_qc_ppid_Y.u_beta)

// LOG_ADD(LOG_FLOAT, t_be, &single_qc_ppid_Y.t_betae)
// LOG_ADD(LOG_FLOAT, t_bin, &single_qc_ppid_Y.t_betain)
// LOG_ADD(LOG_FLOAT, t_ae, &single_qc_ppid_Y.t_alphae)
// LOG_ADD(LOG_FLOAT, t_ain, &single_qc_ppid_Y.t_alphain)

// LOG_ADD(LOG_FLOAT, x_gyro, &single_qc_ppid_U.omega_x)
// LOG_ADD(LOG_FLOAT, b_gyro, &single_qc_ppid_U.beta_speed)
// LOG_ADD(LOG_FLOAT, z_gyro, &single_qc_ppid_U.omega_z)

// LOG_ADD(LOG_FLOAT, t_m1, &single_qc_ppid_Y.t_m1)
// LOG_ADD(LOG_FLOAT, t_m2, &single_qc_ppid_Y.t_m2)
// LOG_ADD(LOG_FLOAT, t_m3, &single_qc_ppid_Y.t_m3)
// LOG_ADD(LOG_FLOAT, t_m4, &single_qc_ppid_Y.t_m4)

// LOG_GROUP_STOP(sctrl_ppid)




// PARAM_GROUP_START(sparam_ppid)
// PARAM_ADD(PARAM_FLOAT, pgaina, &single_qc_ppid_P.pgaina)
// PARAM_ADD(PARAM_FLOAT, igaina, &single_qc_ppid_P.igaina)
// PARAM_ADD(PARAM_FLOAT, dgaina, &single_qc_ppid_P.dgaina)

// PARAM_ADD(PARAM_FLOAT, pgainb, &single_qc_ppid_P.pgainb)
// PARAM_ADD(PARAM_FLOAT, igainb, &single_qc_ppid_P.igainb)
// PARAM_ADD(PARAM_FLOAT, dgainb, &single_qc_ppid_P.dgainb)

// PARAM_ADD(PARAM_FLOAT, pgainas, &single_qc_ppid_P.pgainas)
// PARAM_ADD(PARAM_FLOAT, igainas, &single_qc_ppid_P.igainas)
// PARAM_ADD(PARAM_FLOAT, dgainas, &single_qc_ppid_P.dgainas)

// PARAM_ADD(PARAM_FLOAT, pgainbs, &single_qc_ppid_P.pgainbs)
// PARAM_ADD(PARAM_FLOAT, igainbs, &single_qc_ppid_P.igainbs)
// PARAM_ADD(PARAM_FLOAT, dgainbs, &single_qc_ppid_P.dgainbs)

// PARAM_ADD(PARAM_FLOAT, t_mod, &single_qc_ppid_P.torque_modifier)

// PARAM_ADD(PARAM_FLOAT, s_tx, &single_qc_ppid_P.sat_tx)
// PARAM_ADD(PARAM_FLOAT, s_ty, &single_qc_ppid_P.sat_ty)
// PARAM_ADD(PARAM_FLOAT, s_tz, &single_qc_ppid_P.sat_tz)
// PARAM_GROUP_STOP(sparam_ppid)

