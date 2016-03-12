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
#include "pm.h"
#include "stabilizer.h"
#include "commander.h"
#include "controller.h"
#include "sensfusion6.h"
#include "imu.h"
#include "motors.h"
#include "log.h"
#include "pid.h"
#include "param.h"
#include "sitaw.h"
#ifdef PLATFORM_CF1
  #include "ms5611.h"
#else
  #include "lps25h.h"
#endif
#include "num.h"
#include "altitudehold.h"


/**
 * Defines in what divided update rate should the attitude
 * control loop run relative the rate control loop.
 */
#define ATTITUDE_UPDATE_RATE_DIVIDER  2
#define FUSION_UPDATE_DT  (float)(1.0 / (IMU_UPDATE_FREQ / ATTITUDE_UPDATE_RATE_DIVIDER)) // 250hz

#define ALTHOLD_UPDATE_RATE_DIVIDER  5
#define ALTHOLD_UPDATE_DT  (float)(1.0 / (IMU_UPDATE_FREQ / ALTHOLD_UPDATE_RATE_DIVIDER))   // 100hz

#define G 9.81;

// Barometer/ Altitude hold stuff
static float accWZ     = 0.0; // Acceleration Without gravity along Z axis (G).
static float accMAG    = 0.0; // Acceleration magnitude
static float velocityZ = 0.0; // Vertical speed (world frame) integrated from vertical acceleration (m/s)

static float vAccDeadband = 0.04; // Vertical acceleration deadband
static float velZAlpha = 0.995;   // Blending factor to avoid vertical speed to accumulate error


static Axis3f gyro; // Gyro axis data in deg/s
static Axis3f acc;  // Accelerometer axis data in mG
static Axis3f mag;  // Magnetometer axis data in testla

static float eulerRollActual;   // Measured roll angle in deg
static float eulerPitchActual;  // Measured pitch angle in deg
static float eulerYawActual;    // Measured yaw angle in deg
static float eulerRollDesired;  // Desired roll angle in deg
static float eulerPitchDesired; // Desired ptich angle in deg
static float eulerYawDesired;   // Desired yaw angle in deg
static float rollRateDesired;   // Desired roll rate in deg/s
static float pitchRateDesired;  // Desired pitch rate in deg/s
static float yawRateDesired;    // Desired yaw rate in deg/s


static float carefreeFrontAngle = 0; // carefree front angle that is set

uint16_t actuatorThrust;  // Actuator output for thrust base
int16_t  actuatorRoll;    // Actuator output roll compensation
int16_t  actuatorPitch;   // Actuator output pitch compensation
int16_t  actuatorYaw;     // Actuator output yaw compensation

uint32_t motorPowerM1;  // Motor 1 power output (16bit value used: 0 - 65535)
uint32_t motorPowerM2;  // Motor 2 power output (16bit value used: 0 - 65535)
uint32_t motorPowerM3;  // Motor 3 power output (16bit value used: 0 - 65535)
uint32_t motorPowerM4;  // Motor 4 power output (16bit value used: 0 - 65535)

static bool isInit;


static void stabilizerRotateYaw(float yawRad);
static void stabilizerRotateYawCarefree(bool reset);
static void stabilizerYawModeUpdate(void);
static void distributePower(const uint16_t thrust, const int16_t roll,
                            const int16_t pitch, const int16_t yaw);
static uint16_t limitThrust(int32_t value);
static void stabilizerTask(void* param);
static void readBarometerData(float* pressure, float* temperature, float* asl);

// Baro variables
static float temperature; // temp from barometer in celcius
static float pressure;    // pressure from barometer in bar
static float asl;         // raw Altitude over Sea Level from pressure sensor, in meters. Has an offset.

void stabilizerInit(void)
{
  if(isInit)
    return;

  motorsInit(motorMapDefaultBrushed);
  imu6Init();
  sensfusion6Init();
  controllerInit();
#if defined(SITAW_ENABLED)
  sitAwInit();
#endif

  rollRateDesired = 0;
  pitchRateDesired = 0;
  yawRateDesired = 0;

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
  pass &= controllerTest();

  return pass;
}

static void stabilizerPostAttitudeUpdateCallOut(void)
{
  /* Code that shall run AFTER each attitude update, should be placed here. */

#if defined(SITAW_ENABLED)
  /* Test values for Free Fall detection. */
  sitAwFFTest(accWZ, accMAG);

  /* Test values for Tumbled detection. */
  sitAwTuTest(eulerRollActual, eulerPitchActual);

  /* Test values for At Rest detection. */
  sitAwARTest(acc.x, acc.y, acc.z);

  /* Enable altHold mode if free fall is detected. */
  if(sitAwFFDetected() && !sitAwTuDetected()) {
    commanderSetAltHoldMode(true);
  }

  /* Disable altHold mode if a Tumbled situation is detected. */
  if(sitAwTuDetected()) {
    commanderSetAltHoldMode(false);
  }
#endif
}

static void stabilizerPreThrustUpdateCallOut(void)
{
  /* Code that shall run BEFORE each thrust distribution update, should be placed here. */

#if defined(SITAW_ENABLED)
      if(sitAwTuDetected()) {
        /* Kill the thrust to the motors if a Tumbled situation is detected. */
        actuatorThrust = 0;
      }
#endif
}

static void stabilizerTask(void* param)
{
  RPYType rollType;
  RPYType pitchType;
  RPYType yawType;
  uint32_t attitudeCounter = 0;
  uint32_t altHoldCounter = 0;
  uint32_t lastWakeTime;
  float yawRateAngle = 0;

  vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR);

  //Wait for the system to be fully started to start stabilization loop
  systemWaitStart();

  lastWakeTime = xTaskGetTickCount ();

  while(1)
  {
    vTaskDelayUntil(&lastWakeTime, F2T(IMU_UPDATE_FREQ)); // 500Hz

    // Magnetometer not yet used more then for logging.
    imu9Read(&gyro, &acc, &mag);

    if (imu6IsCalibrated())
    {
      commanderGetRPY(&eulerRollDesired, &eulerPitchDesired, &eulerYawDesired);
      commanderGetRPYType(&rollType, &pitchType, &yawType);

      // Rate-controled YAW is moving YAW angle setpoint
      if (yawType == RATE) {
        yawRateAngle -= eulerYawDesired/500.0;
        while (yawRateAngle > 180.0)
          yawRateAngle -= 360.0;
        while (yawRateAngle < -180.0)
          yawRateAngle += 360.0;

        eulerYawDesired = -yawRateAngle;
      }

      // 250HZ
      if (++attitudeCounter >= ATTITUDE_UPDATE_RATE_DIVIDER)
      {
        sensfusion6UpdateQ(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z, FUSION_UPDATE_DT);
        sensfusion6GetEulerRPY(&eulerRollActual, &eulerPitchActual, &eulerYawActual);

        accWZ = sensfusion6GetAccZWithoutGravity(acc.x, acc.y, acc.z);
        accMAG = (acc.x*acc.x) + (acc.y*acc.y) + (acc.z*acc.z);

        // Estimate speed from acc (drifts)
        velocityZ += deadband(accWZ, vAccDeadband) * FUSION_UPDATE_DT * G;
        velocityZ *= velZAlpha;

        // Adjust yaw if configured to do so
        stabilizerYawModeUpdate();

        controllerCorrectAttitudePID(eulerRollActual, eulerPitchActual, eulerYawActual,
                                     eulerRollDesired, eulerPitchDesired, -eulerYawDesired,
                                     &rollRateDesired, &pitchRateDesired, &yawRateDesired);
        attitudeCounter = 0;

        /* Call out after performing attitude updates, if any functions would like to use the calculated values. */
        stabilizerPostAttitudeUpdateCallOut();
      }

      if (rollType == RATE)
      {
        rollRateDesired = eulerRollDesired;
      }
      if (pitchType == RATE)
      {
        pitchRateDesired = eulerPitchDesired;
      }

      // TODO: Investigate possibility to subtract gyro drift.
      controllerCorrectRatePID(gyro.x, -gyro.y, gyro.z,
                               rollRateDesired, pitchRateDesired, yawRateDesired);

      controllerGetActuatorOutput(&actuatorRoll, &actuatorPitch, &actuatorYaw);

      // 100HZ
      if (++altHoldCounter >= ALTHOLD_UPDATE_RATE_DIVIDER)
      {
        if (imuHasBarometer()) {
          readBarometerData(&pressure, &temperature, &asl);
          altHoldUpdate(&actuatorThrust, asl, velocityZ, ALTHOLD_UPDATE_DT);
        }
        altHoldCounter = 0;
      }

      if (!commanderGetAltHoldMode() || !imuHasBarometer())
      {
        commanderGetThrust(&actuatorThrust);
      }

      /* Call out before performing thrust updates, if any functions would like to influence the thrust. */
      stabilizerPreThrustUpdateCallOut();

      if (actuatorThrust > 0)
      {
#if defined(TUNE_ROLL)
        distributePower(actuatorThrust, actuatorRoll, 0, 0);
#elif defined(TUNE_PITCH)
        distributePower(actuatorThrust, 0, actuatorPitch, 0);
#elif defined(TUNE_YAW)
        distributePower(actuatorThrust, 0, 0, -actuatorYaw);
#else
        distributePower(actuatorThrust, actuatorRoll, actuatorPitch, -actuatorYaw);
#endif
      }
      else
      {
        distributePower(0, 0, 0, 0);
        controllerResetAllPID();

        // Reset the calculated YAW angle for rate control
        yawRateAngle = eulerYawActual;
      }
    }
  }
}


/**
 * Rotate Yaw so that the Crazyflie will change what is considered front.
 *
 * @param yawRad Amount of radians to rotate yaw.
 */
static void stabilizerRotateYaw(float yawRad)
{
  float cosy;
  float siny;
  float originalRoll = eulerRollDesired;
  float originalPitch = eulerPitchDesired;

  cosy = cosf(yawRad);
  siny = sinf(yawRad);
  eulerRollDesired = originalRoll * cosy - originalPitch * siny;
  eulerPitchDesired = originalPitch * cosy + originalRoll * siny;
}

/**
 * Yaw carefree mode means yaw will stay in world coordinates. So even though
 * the Crazyflie rotates around the yaw, front will stay the same as when it started.
 * This makes makes it a bit easier for beginners
 */
static void stabilizerRotateYawCarefree(bool reset)
{
  float yawRad;
  float cosy;
  float siny;
  float originalRoll = eulerRollDesired;

  if (reset)
  {
    carefreeFrontAngle = eulerYawActual;
  }

  yawRad = (eulerYawActual - carefreeFrontAngle) * (float)M_PI / 180;
  cosy = cosf(yawRad);
  siny = sinf(yawRad);
  eulerRollDesired = eulerRollDesired * cosy - eulerPitchDesired * siny;
  eulerPitchDesired = eulerPitchDesired * cosy + originalRoll * siny;
}

/**
 * Update Yaw according to current setting
 */
#ifdef PLATFORM_CF1
static void stabilizerYawModeUpdate(void)
{
  switch (commanderGetYawMode())
  {
    case CAREFREE:
      stabilizerRotateYawCarefree(commanderGetYawModeCarefreeResetFront());
      break;
    case PLUSMODE:
      // Default in plus mode. Do nothing
      break;
    case XMODE: // Fall though
    default:
      stabilizerRotateYaw(-45 * M_PI / 180);
      break;
  }
}
#else
static void stabilizerYawModeUpdate(void)
{
  switch (commanderGetYawMode())
  {
    case CAREFREE:
      stabilizerRotateYawCarefree(commanderGetYawModeCarefreeResetFront());
      break;
    case PLUSMODE:
      stabilizerRotateYaw(45 * M_PI / 180);
      break;
    case XMODE: // Fall though
    default:
      // Default in x-mode. Do nothing
      break;
  }
}
#endif

static void distributePower(const uint16_t thrust, const int16_t roll,
                            const int16_t pitch, const int16_t yaw)
{
#ifdef QUAD_FORMATION_X
  int16_t r = roll >> 1;
  int16_t p = pitch >> 1;
  motorPowerM1 = limitThrust(thrust - r + p + yaw);
  motorPowerM2 = limitThrust(thrust - r - p - yaw);
  motorPowerM3 =  limitThrust(thrust + r - p + yaw);
  motorPowerM4 =  limitThrust(thrust + r + p - yaw);
#else // QUAD_FORMATION_NORMAL
  motorPowerM1 = limitThrust(thrust + pitch + yaw);
  motorPowerM2 = limitThrust(thrust - roll - yaw);
  motorPowerM3 =  limitThrust(thrust - pitch + yaw);
  motorPowerM4 =  limitThrust(thrust + roll - yaw);
#endif

  motorsSetRatio(MOTOR_M1, motorPowerM1);
  motorsSetRatio(MOTOR_M2, motorPowerM2);
  motorsSetRatio(MOTOR_M3, motorPowerM3);
  motorsSetRatio(MOTOR_M4, motorPowerM4);
}

static uint16_t limitThrust(int32_t value)
{
  return limitUint16(value);
}

static void readBarometerData(float* pressure, float* temperature, float* asl) {
#ifdef PLATFORM_CF1
  ms5611GetData(pressure, temperature, asl);
#else
  lps25hGetData(pressure, temperature, asl);
#endif
}


LOG_GROUP_START(ctrltarget)
LOG_ADD(LOG_FLOAT, roll, &eulerRollDesired)
LOG_ADD(LOG_FLOAT, pitch, &eulerPitchDesired)
LOG_ADD(LOG_FLOAT, yaw, &eulerYawDesired)
LOG_GROUP_STOP(ctrltarget)

LOG_GROUP_START(stabilizer)
LOG_ADD(LOG_FLOAT, roll, &eulerRollActual)
LOG_ADD(LOG_FLOAT, pitch, &eulerPitchActual)
LOG_ADD(LOG_FLOAT, yaw, &eulerYawActual)
LOG_ADD(LOG_UINT16, thrust, &actuatorThrust)
LOG_GROUP_STOP(stabilizer)

LOG_GROUP_START(acc)
LOG_ADD(LOG_FLOAT, x, &acc.x)
LOG_ADD(LOG_FLOAT, y, &acc.y)
LOG_ADD(LOG_FLOAT, z, &acc.z)
LOG_ADD(LOG_FLOAT, zw, &accWZ)
LOG_ADD(LOG_FLOAT, mag2, &accMAG)
LOG_ADD(LOG_FLOAT, velocityZ, &velocityZ)
LOG_GROUP_STOP(acc)

LOG_GROUP_START(baro)
LOG_ADD(LOG_FLOAT, asl, &asl)
LOG_ADD(LOG_FLOAT, temp, &temperature)
LOG_ADD(LOG_FLOAT, pressure, &pressure)
LOG_GROUP_STOP(baro)

LOG_GROUP_START(gyro)
LOG_ADD(LOG_FLOAT, x, &gyro.x)
LOG_ADD(LOG_FLOAT, y, &gyro.y)
LOG_ADD(LOG_FLOAT, z, &gyro.z)
LOG_GROUP_STOP(gyro)

LOG_GROUP_START(mag)
LOG_ADD(LOG_FLOAT, x, &mag.x)
LOG_ADD(LOG_FLOAT, y, &mag.y)
LOG_ADD(LOG_FLOAT, z, &mag.z)
LOG_GROUP_STOP(mag)

LOG_GROUP_START(motor)
LOG_ADD(LOG_INT32, m4, &motorPowerM4)
LOG_ADD(LOG_INT32, m1, &motorPowerM1)
LOG_ADD(LOG_INT32, m2, &motorPowerM2)
LOG_ADD(LOG_INT32, m3, &motorPowerM3)
LOG_GROUP_STOP(motor)

PARAM_GROUP_START(acc)
PARAM_ADD(PARAM_FLOAT, velZAlpha, &velZAlpha)
PARAM_ADD(PARAM_FLOAT, vAccDeadband, &vAccDeadband)
PARAM_GROUP_STOP(acc)

