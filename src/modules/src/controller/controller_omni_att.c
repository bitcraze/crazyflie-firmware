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
#include "math3d.h"

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
  if (RATE_DO_EXECUTE(ATTITUDE_RATE, stabilizerStep)) {
      omni_attitude_controller_U.thrust = setpoint->thrust;

      omni_attitude_controller_U.qw_r = setpoint->attitudeQuaternion.q0;
      omni_attitude_controller_U.qx_r = setpoint->attitudeQuaternion.q1;
      omni_attitude_controller_U.qy_r = setpoint->attitudeQuaternion.q2;
      omni_attitude_controller_U.qz_r = setpoint->attitudeQuaternion.q3;

      omni_attitude_controller_U.wx_r = setpoint->attitude.roll;
      // omni_attitude_controller_U.wy_r = setpoint->attitude.pitch;
      // omni_attitude_controller_U.wz_r = setpoint->attitude.yaw;

      omni_attitude_controller_U.qw_IMU = state->attitudeQuaternion.w;
      omni_attitude_controller_U.qx_IMU = state->attitudeQuaternion.x;
      omni_attitude_controller_U.qy_IMU = state->attitudeQuaternion.y;
      omni_attitude_controller_U.qz_IMU = state->attitudeQuaternion.z;

      omni_attitude_controller_U.gyro_x = radians(sensors->gyro.x);
      omni_attitude_controller_U.gyro_y = radians(sensors->gyro.y);
      omni_attitude_controller_U.gyro_z = radians(sensors->gyro.z);

      omni_attitude_controller_DoAttitudeLoop();
      omni_attitude_controller_DoAttitudeRateLoop();
  }

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
// Angular position loop
LOG_ADD(LOG_FLOAT, KRx, &omni_attitude_controller_P.KRx)
LOG_ADD(LOG_FLOAT, KWx, &omni_attitude_controller_P.Kwx)

LOG_ADD(LOG_FLOAT, qw_r, &omni_attitude_controller_U.qw_r)
LOG_ADD(LOG_FLOAT, qx_r, &omni_attitude_controller_U.qx_r)
LOG_ADD(LOG_FLOAT, qy_r, &omni_attitude_controller_U.qy_r)
LOG_ADD(LOG_FLOAT, qz_r, &omni_attitude_controller_U.qz_r)

LOG_ADD(LOG_FLOAT, qw_IMU, &omni_attitude_controller_U.qw_IMU)
LOG_ADD(LOG_FLOAT, qx_IMU, &omni_attitude_controller_U.qx_IMU)
LOG_ADD(LOG_FLOAT, qy_IMU, &omni_attitude_controller_U.qy_IMU)
LOG_ADD(LOG_FLOAT, qz_IMU, &omni_attitude_controller_U.qz_IMU)

LOG_ADD(LOG_FLOAT, eRx, &omni_attitude_controller_Y.eRx)
LOG_ADD(LOG_FLOAT, eRy, &omni_attitude_controller_Y.eRy)
LOG_ADD(LOG_FLOAT, eRz, &omni_attitude_controller_Y.eRz)

LOG_ADD(LOG_FLOAT, wx_r, &omni_attitude_controller_Y.wx_r)
LOG_ADD(LOG_FLOAT, wy_r, &omni_attitude_controller_Y.wy_r)
LOG_ADD(LOG_FLOAT, wz_r, &omni_attitude_controller_Y.wz_r)

// Angular speed loop
LOG_ADD(LOG_FLOAT, eWx, &omni_attitude_controller_Y.eWx)
LOG_ADD(LOG_FLOAT, eWy, &omni_attitude_controller_Y.eWy)
LOG_ADD(LOG_FLOAT, eWz, &omni_attitude_controller_Y.eWz)

LOG_ADD(LOG_FLOAT, eixInt, &omni_attitude_controller_Y.eixInt)
LOG_ADD(LOG_FLOAT, eiyInt, &omni_attitude_controller_Y.eiyInt)
LOG_ADD(LOG_FLOAT, eizInt, &omni_attitude_controller_Y.eizInt)

LOG_ADD(LOG_FLOAT, Tau_x, &omni_attitude_controller_Y.Tau_x)
LOG_ADD(LOG_FLOAT, Tau_y, &omni_attitude_controller_Y.Tau_y)
LOG_ADD(LOG_FLOAT, Tau_z, &omni_attitude_controller_Y.Tau_z)

// Power Distribution
LOG_ADD(LOG_FLOAT, thrust, &omni_attitude_controller_U.thrust)

LOG_ADD(LOG_FLOAT, rollPart, &omni_attitude_controller_Y.rollPart)
LOG_ADD(LOG_FLOAT, pitchPart, &omni_attitude_controller_Y.pitchPart)
LOG_ADD(LOG_FLOAT, thrustPart, &omni_attitude_controller_Y.thrustPart)
LOG_ADD(LOG_FLOAT, yawPart, &omni_attitude_controller_Y.yawPart)

LOG_ADD(LOG_FLOAT, t_m1, &omni_attitude_controller_Y.t_m1)
LOG_ADD(LOG_FLOAT, t_m2, &omni_attitude_controller_Y.t_m2)
LOG_ADD(LOG_FLOAT, t_m3, &omni_attitude_controller_Y.t_m3)
LOG_ADD(LOG_FLOAT, t_m4, &omni_attitude_controller_Y.t_m4)


LOG_GROUP_STOP(sctrl_omni)

PARAM_GROUP_START(sparam_omni)
PARAM_ADD(PARAM_FLOAT, KRx, &omni_attitude_controller_P.KRx)
PARAM_ADD(PARAM_FLOAT, KRy, &omni_attitude_controller_P.KRy)
PARAM_ADD(PARAM_FLOAT, KRz, &omni_attitude_controller_P.KRz)

PARAM_ADD(PARAM_FLOAT, Kwx, &omni_attitude_controller_P.Kwx)
PARAM_ADD(PARAM_FLOAT, Kwy, &omni_attitude_controller_P.Kwy)
PARAM_ADD(PARAM_FLOAT, Kwz, &omni_attitude_controller_P.Kwz)

PARAM_ADD(PARAM_FLOAT, Kix, &omni_attitude_controller_P.Kix)
PARAM_ADD(PARAM_FLOAT, Kiy, &omni_attitude_controller_P.Kiy)
PARAM_ADD(PARAM_FLOAT, Kiz, &omni_attitude_controller_P.Kiz)

PARAM_GROUP_STOP(sparam_omni)

