/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 */
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "config.h"
#include "system.h"
#include "stabilizer.h"
#include "commander.h"
#include "attitude_controller.h"
#include "sensfusion6.h"
#include "imu.h"
#include "motors.h"
#include "log.h"
#include "param.h"
#include "sitaw.h"
#ifdef PLATFORM_CF1
  #include "ms5611.h"
#else
  #include "lps25h.h"
#endif
#include "num.h"

static struct {
  uint32_t m1;
  uint32_t m2;
  uint32_t m3;
  uint32_t m4;
} motorPower;

static bool isInit;

static uint16_t limitThrust(int32_t value);
static void stabilizerTask(void* param);

void stabilizerInit(void)
{
  if(isInit)
    return;

  motorsInit(motorMapDefaultBrushed);
  imu6Init();
  sensfusion6Init();
  attitudeControllerInit();
#if defined(SITAW_ENABLED)
  sitAwInit();
#endif

  // rollRateDesired = 0;
  // pitchRateDesired = 0;
  // yawRateDesired = 0;

  xTaskCreate(stabilizerTask, STABILIZER_TASK_NAME,
              STABILIZER_TASK_STACKSIZE, NULL, STABILIZER_TASK_PRI, NULL);

  isInit = true;
}

bool stabilizerTest(void)
{
  bool pass = true;

  pass &= motorsTest();
  pass &= imu6Test();
  pass &= sensfusion6Test();
  pass &= attitudeControllerTest();

  return pass;
}

//
// static void stabilizerTask(void* param)
// {
//   RPYType rollType;
//   RPYType pitchType;
//   RPYType yawType;
//   uint32_t attitudeCounter = 0;
//   uint32_t altHoldCounter = 0;
//   uint32_t lastWakeTime;
//   float yawRateAngle = 0;
//
//   vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR);
//
//   //Wait for the system to be fully started to start stabilization loop
//   systemWaitStart();
//
//   lastWakeTime = xTaskGetTickCount ();
//
//   while(1)
//   {
//     vTaskDelayUntil(&lastWakeTime, F2T(IMU_UPDATE_FREQ)); // 500Hz
//
//     // Magnetometer not yet used more then for logging.
//     imu9Read(&gyro, &acc, &mag);
//
//     if (imu6IsCalibrated())
//     {
//       commanderGetRPY(&eulerRollDesired, &eulerPitchDesired, &eulerYawDesired);
//       commanderGetRPYType(&rollType, &pitchType, &yawType);
//
//       // Rate-controled YAW is moving YAW angle setpoint
//       if (yawType == RATE) {
//         yawRateAngle -= eulerYawDesired/500.0;
//         while (yawRateAngle > 180.0)
//           yawRateAngle -= 360.0;
//         while (yawRateAngle < -180.0)
//           yawRateAngle += 360.0;
//
//         eulerYawDesired = -yawRateAngle;
//       }
//
//       // 250HZ
//       if (++attitudeCounter >= ATTITUDE_UPDATE_RATE_DIVIDER)
//       {
//         sensfusion6UpdateQ(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z, ATTITUDE_UPDATE_DT);
//         sensfusion6GetEulerRPY(&eulerRollActual, &eulerPitchActual, &eulerYawActual);
//
//         accWZ = sensfusion6GetAccZWithoutGravity(acc.x, acc.y, acc.z);
//         accMAG = (acc.x*acc.x) + (acc.y*acc.y) + (acc.z*acc.z);
//
//         positionUpdateVelocity(accWZ, ATTITUDE_UPDATE_DT);
//
//         // Adjust yaw if configured to do so
//         stabilizerYawModeUpdate();
//
//         attitudeControllerCorrectAttitudePID(eulerRollActual, eulerPitchActual, eulerYawActual,
//                                      eulerRollDesired, eulerPitchDesired, -eulerYawDesired,
//                                      &rollRateDesired, &pitchRateDesired, &yawRateDesired);
//         attitudeCounter = 0;
//
//         /* Call out after performing attitude updates, if any functions would like to use the calculated values. */
//         stabilizerPostAttitudeUpdateCallOut();
//       }
//
//       if (rollType == RATE)
//       {
//         rollRateDesired = eulerRollDesired;
//       }
//       if (pitchType == RATE)
//       {
//         pitchRateDesired = eulerPitchDesired;
//       }
//
//       // TODO: Investigate possibility to subtract gyro drift.
//       attitudeControllerCorrectRatePID(gyro.x, -gyro.y, gyro.z,
//                                rollRateDesired, pitchRateDesired, yawRateDesired);
//
//       attitudeControllerGetActuatorOutput(&actuatorRoll, &actuatorPitch, &actuatorYaw);
//
//       // 100HZ
//       if (++altHoldCounter >= ALTHOLD_UPDATE_RATE_DIVIDER)
//       {
//         if (imuHasBarometer()) {
//           readBarometerData(&pressure, &temperature, &asl);
//         }
//         positionEstimate(&estimatedPosition, asl, ALTHOLD_UPDATE_DT);
//
//         if (altHoldIsActive()) {
//           setpointZ_t setpoint;
//           altHoldGetNewSetPoint(&setpoint, &estimatedPosition);
//           positionControllerSetZTarget(&setpoint, ALTHOLD_UPDATE_DT);
//
//           positionControllerUpdate(&actuatorThrust, &estimatedPosition, ALTHOLD_UPDATE_DT);
//         } else {
//           commanderGetThrust(&actuatorThrust);
//         }
//
//         altHoldCounter = 0;
//       }
//
//       /* Call out before performing thrust updates, if any functions would like to influence the thrust. */
//       stabilizerPreThrustUpdateCallOut();
//
//       actuatorThrust /= sensfusion6GetInvThrustCompensationForTilt();
//
//       if (actuatorThrust > 0)
//       {
// #if defined(TUNE_ROLL)
//         distributePower(actuatorThrust, actuatorRoll, 0, 0);
// #elif defined(TUNE_PITCH)
//         distributePower(actuatorThrust, 0, actuatorPitch, 0);
// #elif defined(TUNE_YAW)
//         distributePower(actuatorThrust, 0, 0, -actuatorYaw);
// #else
//         distributePower(actuatorThrust, actuatorRoll, actuatorPitch, -actuatorYaw);
// #endif
//       }
//       else
//       {
//         distributePower(0, 0, 0, 0);
//         attitudeControllerResetAllPID();
//
//         // Reset the calculated YAW angle for rate control
//         yawRateAngle = eulerYawActual;
//       }
//     }
//   }
// }
//
//
//

//


static uint16_t limitThrust(int32_t value)
{
  return limitUint16(value);
}

static void readBarometerData(baro_t *baro) {
#ifdef PLATFORM_CF1
  ms5611GetData(&baro->pressure, &baro->temperature, &baro->asl);
#else
  lps25hGetData(&baro->pressure, &baro->temperature, &baro->asl);
#endif
}

static bool acquireSensors(sensorData_t *sensors)
{
  if (RATE_SKIP_500HZ()) {
    return imu6IsCalibrated();
  }

  imu9Read(&sensors->gyro, &sensors->acc, &sensors->mag);
  if (imuHasBarometer()) {
    readBarometerData(&sensors->baro);
  }
  // Get the position

  return imu6IsCalibrated();
}

void distributePower(const control_t *control)
{
#ifdef QUAD_FORMATION_X
  int16_t r = control->roll / 2.0f;
  int16_t p = control->pitch / 2.0f;
  motorPower.m1 = limitThrust(control->thrust - r + p + control->yaw);
  motorPower.m2 = limitThrust(control->thrust - r - p - control->yaw);
  motorPower.m3 =  limitThrust(control->thrust + r - p + control->yaw);
  motorPower.m4 =  limitThrust(control->thrust + r + p - control->yaw);
#else // QUAD_FORMATION_NORMAL
  motorPower.m1 = limitThrust(control->thrust + control->pitch +
                             control->yaw);
  motorPower.m2 = limitThrust(control->thrust - control->roll -
                             control->yaw);
  motorPower.m3 =  limitThrust(control->thrust - control->pitch +
                             control->yaw);
  motorPower.m4 =  limitThrust(control->thrust + control->roll -
                             control->yaw);
#endif

  motorsSetRatio(MOTOR_M1, motorPower.m1);
  motorsSetRatio(MOTOR_M2, motorPower.m2);
  motorsSetRatio(MOTOR_M3, motorPower.m3);
  motorsSetRatio(MOTOR_M4, motorPower.m4);
}

// State variables for the stabilizer
static setpoint_t setpoint;
static sensorData_t sensorData;
static state_t state;
static control_t control;

/* The stabilizer loop runs at 1KHz. It is the responsability or the different
 * functions to run slower and returns cached data
 */
static void stabilizerTask(void* param)
{
  uint32_t lastWakeTime;
  vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR);

  //Wait for the system to be fully started to start stabilization loop
  systemWaitStart();

  lastWakeTime = xTaskGetTickCount ();

  while(1)
  {
    vTaskDelayUntil(&lastWakeTime, F2T(1000));

    if (acquireSensors(&sensorData))
    {
      stateEstimator(&state, &sensorData);
      commanderGetSetpoint(&setpoint, &state);

      sitAwUpdateSetpoint(&setpoint, &sensorData, &state);

      stateController(&control, &sensorData, &state, &setpoint);
      distributePower(&control);
    }
  }
}


LOG_GROUP_START(ctrltarget)
LOG_ADD(LOG_FLOAT, roll, &setpoint.attitude.roll)
LOG_ADD(LOG_FLOAT, pitch, &setpoint.attitude.pitch)
LOG_ADD(LOG_FLOAT, yaw, &setpoint.attitudeRate.yaw)
LOG_GROUP_STOP(ctrltarget)

LOG_GROUP_START(stabilizer)
LOG_ADD(LOG_FLOAT, roll, &state.attitude.roll)
LOG_ADD(LOG_FLOAT, pitch, &state.attitude.pitch)
LOG_ADD(LOG_FLOAT, yaw, &state.attitude.yaw)
LOG_ADD(LOG_UINT16, thrust, &control.thrust)
LOG_GROUP_STOP(stabilizer)

LOG_GROUP_START(acc)
LOG_ADD(LOG_FLOAT, x, &sensorData.acc.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.acc.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.acc.z)
LOG_GROUP_STOP(acc)

LOG_GROUP_START(baro)
LOG_ADD(LOG_FLOAT, asl, &sensorData.baro.asl)
LOG_GROUP_STOP(baro)

LOG_GROUP_START(gyro)
LOG_ADD(LOG_FLOAT, x, &sensorData.gyro.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.gyro.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.gyro.z)
LOG_GROUP_STOP(gyro)

LOG_GROUP_START(mag)
LOG_ADD(LOG_FLOAT, x, &sensorData.mag.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.mag.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.mag.z)
LOG_GROUP_STOP(mag)

LOG_GROUP_START(motor)
LOG_ADD(LOG_INT32, m4, &motorPower.m4)
LOG_ADD(LOG_INT32, m1, &motorPower.m1)
LOG_ADD(LOG_INT32, m2, &motorPower.m2)
LOG_ADD(LOG_INT32, m3, &motorPower.m3)
LOG_GROUP_STOP(motor)

LOG_GROUP_START(controller)
LOG_ADD(LOG_INT16, ctr_yaw, &control.yaw)
LOG_GROUP_STOP(controller)
