
#include "stabilizer.h"
#include "stabilizer_types.h"

#include "attitude_controller.h"
#include "sensfusion6.h"
#include "position_controller.h"

#include "log.h"

static float accMAG;
static attitude_t attitudeDesired;
static attitude_t rateDesired;
static float yawRateAngle;
static mode_t prevZMode = modeDisable;
static setpointZ_t setpointZ;
uint16_t actuatorThrust;

void stateController(control_t *control, const sensorData_t *sensors,
                                         const state_t *state,
                                         const setpoint_t *setpoint)
{
  if (!RATE_SKIP_500HZ()) {
    // Rate-controled YAW is moving YAW angle setpoint
    if (setpoint->mode.yaw == modeVelocity) {
       yawRateAngle -= setpoint->attitudeRate.yaw/500.0;
      while (yawRateAngle > 180.0)
        yawRateAngle -= 360.0;
      while (yawRateAngle < -180.0)
        yawRateAngle += 360.0;

      attitudeDesired.yaw = yawRateAngle;
    } else {
      attitudeDesired.yaw = setpoint->attitude.yaw;
    }
  }

  if (!RATE_SKIP_500HZ()) {
    accMAG = (sensors->acc.x*sensors->acc.x) +
             (sensors->acc.y*sensors->acc.y) +
             (sensors->acc.z*sensors->acc.z);

    attitudeControllerCorrectAttitudePID(state->attitude.roll, state->attitude.pitch, state->attitude.yaw,
                                setpoint->attitude.roll, setpoint->attitude.pitch, attitudeDesired.yaw,
                                &rateDesired.roll, &rateDesired.pitch, &rateDesired.yaw);
  }

  if (!RATE_SKIP_500HZ()) {
    if (setpoint->mode.roll == modeVelocity)
    {
      rateDesired.roll = setpoint->attitudeRate.roll;
    }
    if (setpoint->mode.pitch == modeVelocity)
    {
      rateDesired.pitch = setpoint->attitudeRate.pitch;
    }

    // TODO: Investigate possibility to subtract gyro drift.
    attitudeControllerCorrectRatePID(sensors->gyro.x, -sensors->gyro.y, sensors->gyro.z,
                             rateDesired.roll, rateDesired.pitch, rateDesired.yaw);

    attitudeControllerGetActuatorOutput(&control->roll,
                                        &control->pitch,
                                        &control->yaw);

    control->yaw = -control->yaw;
  }

  if (!RATE_SKIP_100HZ()) {
    if (setpoint->mode.z == modeDisable) {
     actuatorThrust = setpoint->thrust;
    } else {
      if (prevZMode == modeDisable) {
        setpointZ.isUpdate = false;
        if (setpoint->mode.z == modeVelocity) {
          setpointZ.z = state->position.z;
        }
      } else {
        setpointZ.isUpdate = true;
      }

      if (setpoint->mode.z == modeVelocity) {
        setpointZ.z += setpoint->velocity.z * 0.01;
      } else if (setpoint->mode.z == modeAbs) {
        setpointZ.z = setpoint->position.z;
      }

      positionControllerSetZTarget(&setpointZ, 0.01);
      positionControllerUpdate(&actuatorThrust, state->position.z, 0.01);
    }
    prevZMode = setpoint->mode.z;
  }

  control->thrust = actuatorThrust / sensfusion6GetInvThrustCompensationForTilt();

  if (control->thrust == 0)
  {
    control->thrust = 0;
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;

    attitudeControllerResetAllPID();

    // Reset the calculated YAW angle for rate control
    yawRateAngle = state->attitude.yaw;
  }
}


LOG_GROUP_START(controller)
LOG_ADD(LOG_FLOAT, sp_yaw, &attitudeDesired.yaw)
LOG_ADD(LOG_FLOAT, sp_alt, &setpointZ.z)
LOG_ADD(LOG_UINT16, actuatorThrust, &actuatorThrust)
LOG_ADD(LOG_UINT8, zmode, &prevZMode)
LOG_GROUP_STOP(controller)
