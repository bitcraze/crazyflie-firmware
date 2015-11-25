/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
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


#undef max
#define max(a,b) ((a) > (b) ? (a) : (b))
#undef min
#define min(a,b) ((a) < (b) ? (a) : (b))

/**
 * Defines in what divided update rate should the attitude
 * control loop run relative the rate control loop.
 */
#define ATTITUDE_UPDATE_RATE_DIVIDER  2
#define FUSION_UPDATE_DT  (float)(1.0 / (IMU_UPDATE_FREQ / ATTITUDE_UPDATE_RATE_DIVIDER)) // 250hz

// Barometer/ Altitude hold stuff
#define ALTHOLD_UPDATE_RATE_DIVIDER  5 // 500hz/5 = 100hz for barometer measurements
#define ALTHOLD_UPDATE_DT  (float)(1.0 / (IMU_UPDATE_FREQ / ALTHOLD_UPDATE_RATE_DIVIDER))   // 500hz

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

// Baro variables
static float temperature; // temp from barometer in celcius
static float pressure;    // pressure from barometer in bar
static float asl;         // smoothed asl
static float aslRaw;      // raw asl
static float aslLong;     // long term asl
static float aslRef;      // asl reference (ie. offset)

// Altitude hold variables
static PidObject altHoldPID;  // Used for altitute hold mode. I gets reset when the bat status changes
bool altHold = false;         // Currently in altitude hold mode
bool setAltHold = false;      // Hover mode has just been activated
static float accWZ     = 0.0; // Acceleration Without gravity along Z axis.
static float accMAG    = 0.0; // Acceleration magnitude
static float vSpeedASL = 0.0; // Vertical speed (world frame) derived from barometer ASL
static float vSpeedAcc = 0.0; // Vertical speed (world frame) integrated from vertical acceleration
static float vSpeed    = 0.0; // Vertical speed (world frame) integrated from vertical acceleration
static float altHoldPIDVal;   // Output of the PID controller
static float altHoldErr;      // Different between target and current altitude

// Altitude hold & Baro Params
static float altHoldKp              = 0.5;  // PID gain constants, used everytime we reinitialise the PID controller
static float altHoldKi              = 0.18;
static float altHoldKd              = 0.0;
static float altHoldChange          = 0;     // Change in target altitude
static float altHoldTarget          = -1;    // Target altitude
static float altHoldErrMax          = 1.0;   // max cap on current estimated altitude vs target altitude in meters
static float altHoldChange_SENS     = 200;   // sensitivity of target altitude change (thrust input control) while hovering. Lower = more sensitive & faster changes
static float pidAslFac              = 13000; // relates meters asl to thrust
static float pidAlpha               = 0.8;   // PID Smoothing //TODO: shouldnt need to do this
static float vSpeedASLFac           = 0;    // multiplier
static float vSpeedAccFac           = -48;  // multiplier
static float vAccDeadband           = 0.05;  // Vertical acceleration deadband
static float vSpeedASLDeadband      = 0.005; // Vertical speed based on barometer readings deadband
static float vSpeedLimit            = 0.05;  // used to constrain vertical velocity
static float errDeadband            = 0.00;  // error (target - altitude) deadband
static float vBiasAlpha             = 0.98; // Blending factor we use to fuse vSpeedASL and vSpeedAcc
static float aslAlpha               = 0.92; // Short term smoothing
static float aslAlphaLong           = 0.93; // Long term smoothing
static uint16_t altHoldMinThrust    = 00000; // minimum hover thrust - not used yet
static uint16_t altHoldBaseThrust   = 43000; // approximate throttle needed when in perfect hover. More weight/older battery can use a higher value
static uint16_t altHoldMaxThrust    = 60000; // max altitude hold thrust

#if defined(SITAW_ENABLED)
// Automatic take-off variables
static bool autoTOActive           = false; // Flag indicating if automatic take-off is active / deactive.
static float autoTOAltBase         = 0.0f;  // Base altitude for the automatic take-off. Set to altHoldTarget when automatic take-off is activated.
static float autoTOAltCurrent      = 0.0f;  // Current target altitude adjustment. Equals 0 when function is activated, increases to autoTOThresh when function is deactivated.
// Automatic take-off parameters
static float autoTOAlpha           = 0.98f; // Smoothing factor when adjusting the altHoldTarget altitude.
static float autoTOTargetAdjust    = 1.5f;  // Meters to add to altHoldTarget to reach auto take-off altitude.
static float autoTOThresh          = 0.97f; // Threshold for when to deactivate auto Take-Off. A value of 0.97 means 97% of the target altitude adjustment.
#endif

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


static void stabilizerAltHoldUpdate(void);
static void stabilizerRotateYaw(float yawRad);
static void stabilizerRotateYawCarefree(bool reset);
static void stabilizerYawModeUpdate(void);
static void distributePower(const uint16_t thrust, const int16_t roll,
                            const int16_t pitch, const int16_t yaw);
static uint16_t limitThrust(int32_t value);
static void stabilizerTask(void* param);
static float constrain(float value, const float minVal, const float maxVal);
static float deadband(float value, const float threshold);

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
        vSpeed += deadband(accWZ, vAccDeadband) * FUSION_UPDATE_DT;

        // Adjust yaw if configured to do so
        stabilizerYawModeUpdate();

        controllerCorrectAttitudePID(eulerRollActual, eulerPitchActual, eulerYawActual,
                                     eulerRollDesired, eulerPitchDesired, -eulerYawDesired,
                                     &rollRateDesired, &pitchRateDesired, &yawRateDesired);
        attitudeCounter = 0;

        /* Call out after performing attitude updates, if any functions would like to use the calculated values. */
        stabilizerPostAttitudeUpdateCallOut();
      }

      // 100HZ
      if (imuHasBarometer() && (++altHoldCounter >= ALTHOLD_UPDATE_RATE_DIVIDER))
      {
        stabilizerAltHoldUpdate();
        altHoldCounter = 0;
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

      if (!altHold || !imuHasBarometer())
      {
        // Use thrust from controller if not in altitude hold mode
        commanderGetThrust(&actuatorThrust);
      }
      else
      {
        // Added so thrust can be set to 0 while in altitude hold mode after disconnect
        commanderWatchdog();
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

static void stabilizerPreAltHoldComputeThrustCallOut(void)
{
  /* Code that shall run BEFORE each altHold thrust computation, should be placed here. */

#if defined(SITAW_ENABLED)
  /*
   * The number of variables used for automatic Take-Off could be reduced, however that would
   * cause debugging and tuning to become more difficult. The variables currently used ensure
   * that tuning can easily be done through the LOG and PARAM frameworks.
   *
   * Note that while the automatic take-off function is active, it will overrule any other
   * changes to altHoldTarget by the user.
   *
   * The automatic take-off function will automatically deactivate once the take-off has been
   * conducted.
   */
  if(!autoTOActive){
    /*
     * Enabling automatic take-off: When At Rest, Not Tumbled, and the user pressing the AltHold button
     */
    if(sitAwARDetected() && !sitAwTuDetected() && setAltHold) {
      /* Enable automatic take-off. */
      autoTOActive = true;
      autoTOAltBase = altHoldTarget;
      autoTOAltCurrent = 0.0f;
    }
  }

  if(autoTOActive) {
    /*
     * Automatic take-off is quite simple: Slowly increase altHoldTarget until reaching the target altitude.
     */

    /* Calculate the new current setpoint for altHoldTarget. autoTOAltCurrent is normalized to values from 0 to 1. */
    autoTOAltCurrent = autoTOAltCurrent * autoTOAlpha + (1 - autoTOAlpha);

    /* Update the altHoldTarget variable. */
    altHoldTarget = autoTOAltBase + autoTOAltCurrent * autoTOTargetAdjust;

    if((autoTOAltCurrent >= autoTOThresh)) {
      /* Disable the automatic take-off mode if target altitude has been reached. */
      autoTOActive = false;
      autoTOAltBase = 0.0f;
      autoTOAltCurrent = 0.0f;
    }
  }
#endif
}

static void stabilizerAltHoldUpdate(void)
{
  // Get altitude hold commands from pilot
  commanderGetAltHold(&altHold, &setAltHold, &altHoldChange);

  // Get barometer height estimates
  //TODO do the smoothing within getData
#ifdef PLATFORM_CF1
  ms5611GetData(&pressure, &temperature, &aslRaw);
#else
  lps25hGetData(&pressure, &temperature, &aslRaw);
#endif

  aslRaw -= aslRef;

  asl = asl * aslAlpha + aslRaw * (1 - aslAlpha);
  aslLong = aslLong * aslAlphaLong + aslRaw * (1 - aslAlphaLong);

  // Estimate vertical speed based on successive barometer readings. This is ugly :)
  vSpeedASL = deadband(asl - aslLong, vSpeedASLDeadband);

  // Estimate vertical speed based on Acc - fused with baro to reduce drift
  vSpeed = constrain(vSpeed, -vSpeedLimit, vSpeedLimit);
  vSpeed = vSpeed * vBiasAlpha + vSpeedASL * (1.f - vBiasAlpha);
  vSpeedAcc = vSpeed;

  // Reset Integral gain of PID controller if being charged
  if (!pmIsDischarging())
  {
    altHoldPID.integ = 0.0;
  }

  // Altitude hold mode just activated, set target altitude as current altitude. Reuse previous integral term as a starting point
  if (setAltHold)
  {
    // Set to current altitude
    altHoldTarget = asl;

    // Cache last integral term for reuse after pid init
    const float pre_integral = altHoldPID.integ;

    // Reset PID controller
    pidInit(&altHoldPID, asl, altHoldKp, altHoldKi, altHoldKd,
            ALTHOLD_UPDATE_DT);
    // TODO set low and high limits depending on voltage
    // TODO for now just use previous I value and manually set limits for whole voltage range
    //                    pidSetIntegralLimit(&altHoldPID, 12345);
    //                    pidSetIntegralLimitLow(&altHoldPID, 12345);              /

    altHoldPID.integ = pre_integral;

    // Reset altHoldPID
    altHoldPIDVal = pidUpdate(&altHoldPID, asl, false);
  }

  /* Call out before performing altHold thrust regulation. */
  stabilizerPreAltHoldComputeThrustCallOut();

  // In altitude hold mode
  if (altHold)
  {
    // Update target altitude from joy controller input
    altHoldTarget += altHoldChange / altHoldChange_SENS;
    pidSetDesired(&altHoldPID, altHoldTarget);

    // Compute error (current - target), limit the error
    altHoldErr = constrain(deadband(asl - altHoldTarget, errDeadband),
                           -altHoldErrMax, altHoldErrMax);
    pidSetError(&altHoldPID, -altHoldErr);

    // Get control from PID controller, dont update the error (done above)
    // Smooth it and include barometer vspeed
    // TODO same as smoothing the error??
    altHoldPIDVal = (pidAlpha) * altHoldPIDVal + (1.f - pidAlpha) * ((vSpeedAcc * vSpeedAccFac) +
                    (vSpeedASL * vSpeedASLFac) + pidUpdate(&altHoldPID, asl, false));

    // compute new thrust
    actuatorThrust =  max(altHoldMinThrust, min(altHoldMaxThrust,
                          limitThrust( altHoldBaseThrust + (int32_t)(altHoldPIDVal*pidAslFac))));

    // i part should compensate for voltage drop

  }
  else
  {
    altHoldTarget = 0.0;
    altHoldErr = 0.0;
    altHoldPIDVal = 0.0;
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
  if(value > UINT16_MAX)
  {
    value = UINT16_MAX;
  }
  else if(value < 0)
  {
    value = 0;
  }

  return (uint16_t)value;
}

// Constrain value between min and max
static float constrain(float value, const float minVal, const float maxVal)
{
  return min(maxVal, max(minVal,value));
}

// Deadzone
static float deadband(float value, const float threshold)
{
  if (fabs(value) < threshold)
  {
    value = 0;
  }
  else if (value > 0)
  {
    value -= threshold;
  }
  else if (value < 0)
  {
    value += threshold;
  }
  return value;
}

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
LOG_GROUP_STOP(acc)

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

// LOG altitude hold PID controller states
LOG_GROUP_START(vpid)
LOG_ADD(LOG_FLOAT, pid, &altHoldPID)
LOG_ADD(LOG_FLOAT, p, &altHoldPID.outP)
LOG_ADD(LOG_FLOAT, i, &altHoldPID.outI)
LOG_ADD(LOG_FLOAT, d, &altHoldPID.outD)
LOG_GROUP_STOP(vpid)

LOG_GROUP_START(baro)
LOG_ADD(LOG_FLOAT, asl, &asl)
LOG_ADD(LOG_FLOAT, aslRaw, &aslRaw)
LOG_ADD(LOG_FLOAT, aslLong, &aslLong)
LOG_ADD(LOG_FLOAT, temp, &temperature)
LOG_ADD(LOG_FLOAT, pressure, &pressure)
LOG_GROUP_STOP(baro)

LOG_GROUP_START(altHold)
LOG_ADD(LOG_FLOAT, err, &altHoldErr)
LOG_ADD(LOG_FLOAT, target, &altHoldTarget)
LOG_ADD(LOG_FLOAT, zSpeed, &vSpeed)
LOG_ADD(LOG_FLOAT, vSpeed, &vSpeed)
LOG_ADD(LOG_FLOAT, vSpeedASL, &vSpeedASL)
LOG_ADD(LOG_FLOAT, vSpeedAcc, &vSpeedAcc)
LOG_GROUP_STOP(altHold)

#if defined(SITAW_ENABLED)
// Automatic take-off parameters
LOG_GROUP_START(autoTO)
LOG_ADD(LOG_UINT8, Active, &autoTOActive)
LOG_ADD(LOG_FLOAT, AltBase, &autoTOAltBase)
LOG_ADD(LOG_FLOAT, AltCurrent, &autoTOAltCurrent)
LOG_GROUP_STOP(autoTO)
#endif

// Params for altitude hold
PARAM_GROUP_START(altHold)
PARAM_ADD(PARAM_FLOAT, aslAlpha, &aslAlpha)
PARAM_ADD(PARAM_FLOAT, aslAlphaLong, &aslAlphaLong)
PARAM_ADD(PARAM_FLOAT, aslRef, &aslRef)
PARAM_ADD(PARAM_FLOAT, errDeadband, &errDeadband)
PARAM_ADD(PARAM_FLOAT, altHoldChangeSens, &altHoldChange_SENS)
PARAM_ADD(PARAM_FLOAT, altHoldErrMax, &altHoldErrMax)
PARAM_ADD(PARAM_FLOAT, kd, &altHoldKd)
PARAM_ADD(PARAM_FLOAT, ki, &altHoldKi)
PARAM_ADD(PARAM_FLOAT, kp, &altHoldKp)
PARAM_ADD(PARAM_FLOAT, pidAlpha, &pidAlpha)
PARAM_ADD(PARAM_FLOAT, pidAslFac, &pidAslFac)
PARAM_ADD(PARAM_FLOAT, vAccDeadband, &vAccDeadband)
PARAM_ADD(PARAM_FLOAT, vBiasAlpha, &vBiasAlpha)
PARAM_ADD(PARAM_FLOAT, vSpeedAccFac, &vSpeedAccFac)
PARAM_ADD(PARAM_FLOAT, vSpeedASLDeadband, &vSpeedASLDeadband)
PARAM_ADD(PARAM_FLOAT, vSpeedASLFac, &vSpeedASLFac)
PARAM_ADD(PARAM_FLOAT, vSpeedLimit, &vSpeedLimit)
PARAM_ADD(PARAM_UINT16, baseThrust, &altHoldBaseThrust)
PARAM_ADD(PARAM_UINT16, maxThrust, &altHoldMaxThrust)
PARAM_ADD(PARAM_UINT16, minThrust, &altHoldMinThrust)
PARAM_GROUP_STOP(altHold)

#if defined(SITAW_ENABLED)
// Automatic take-off parameters
PARAM_GROUP_START(autoTO)
PARAM_ADD(PARAM_FLOAT, TargetAdjust, &autoTOTargetAdjust)
PARAM_ADD(PARAM_FLOAT, Thresh, &autoTOThresh)
PARAM_ADD(PARAM_FLOAT, Alpha, &autoTOAlpha)
PARAM_GROUP_STOP(autoTO)
#endif
