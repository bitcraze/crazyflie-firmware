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

LOG_GROUP_START(sctrl_omni)

LOG_ADD(LOG_FLOAT, qw_r, &omni_attitude_controller_U.qw_r)
LOG_ADD(LOG_FLOAT, qx_r, &omni_attitude_controller_U.qx_r)
LOG_ADD(LOG_FLOAT, qy_r, &omni_attitude_controller_U.qy_r)
LOG_ADD(LOG_FLOAT, qz_r, &omni_attitude_controller_U.qz_r)

LOG_ADD(LOG_FLOAT, wx_r, &omni_attitude_controller_U.wx_r)
LOG_ADD(LOG_FLOAT, wy_r, &omni_attitude_controller_U.wy_r)
LOG_ADD(LOG_FLOAT, wz_r, &omni_attitude_controller_U.wz_r)

LOG_ADD(LOG_FLOAT, qw_IMU, &omni_attitude_controller_U.qw_IMU)
LOG_ADD(LOG_FLOAT, qx_IMU, &omni_attitude_controller_U.qx_IMU)
LOG_ADD(LOG_FLOAT, qy_IMU, &omni_attitude_controller_U.qy_IMU)
LOG_ADD(LOG_FLOAT, qz_IMU, &omni_attitude_controller_U.qz_IMU)

LOG_ADD(LOG_FLOAT, gyro_x, &omni_attitude_controller_U.gyro_x)
LOG_ADD(LOG_FLOAT, gyro_y, &omni_attitude_controller_U.gyro_y)
LOG_ADD(LOG_FLOAT, gyro_z, &omni_attitude_controller_U.gyro_z)

LOG_ADD(LOG_FLOAT, t_m1, &omni_attitude_controller_Y.t_m1)
LOG_ADD(LOG_FLOAT, t_m2, &omni_attitude_controller_Y.t_m2)
LOG_ADD(LOG_FLOAT, t_m3, &omni_attitude_controller_Y.t_m3)
LOG_ADD(LOG_FLOAT, t_m4, &omni_attitude_controller_Y.t_m4)

LOG_GROUP_STOP(sctrl_omni)

PARAM_GROUP_START(sparam_omni)
PARAM_ADD(PARAM_FLOAT, KRx, &omni_attitude_controller_P.KR[0])
PARAM_ADD(PARAM_FLOAT, KRy, &omni_attitude_controller_P.KR[4])
PARAM_ADD(PARAM_FLOAT, KRz, &omni_attitude_controller_P.KR[8])

PARAM_ADD(PARAM_FLOAT, Kix, &omni_attitude_controller_P.Ki[0])
PARAM_ADD(PARAM_FLOAT, Kiy, &omni_attitude_controller_P.Ki[4])
PARAM_ADD(PARAM_FLOAT, Kiz, &omni_attitude_controller_P.Ki[8])

PARAM_ADD(PARAM_FLOAT, Kwx, &omni_attitude_controller_P.Kw[0])
PARAM_ADD(PARAM_FLOAT, Kwy, &omni_attitude_controller_P.Kw[4])
PARAM_ADD(PARAM_FLOAT, Kwz, &omni_attitude_controller_P.Kw[8])

PARAM_ADD(PARAM_FLOAT, t_mod, &omni_attitude_controller_P.Constant_Value)

PARAM_GROUP_STOP(sparam_omni)

