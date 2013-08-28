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
#include "stm32f10x_conf.h"
#include "FreeRTOS.h"
#include "task.h"

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
#include "ledseq.h"
#include "param.h"
#include "ms5611.h"
#include "global.h"
#include "math.h"



/**
 * Defines in what divided update rate should the attitude
 * control loop run relative the rate control loop.
 */
#define ATTITUDE_UPDATE_RATE_DIVIDER  2
#define FUSION_UPDATE_DT  (float)(1.0 / (IMU_UPDATE_FREQ / ATTITUDE_UPDATE_RATE_DIVIDER)) // 250hz


// Barometer/ Hover stuff
#define HOVER_UPDATE_RATE_DIVIDER  5 // 500hz/5 = 100hz for barometer measurements
#define HOVER_UPDATE_DT  (float)(1.0 / (IMU_UPDATE_FREQ / HOVER_UPDATE_RATE_DIVIDER))   // 500hz


#define LOGGING_ENABLED
#ifdef LOGGING_ENABLED
  #define PRIVATE
#else
  #define PRIVATE static
#endif

PRIVATE Axis3f gyro; // Gyro axis data in deg/s
PRIVATE Axis3f acc;  // Accelerometer axis data in mG


PRIVATE float eulerRollActual;
PRIVATE float eulerPitchActual;
PRIVATE float eulerYawActual;
PRIVATE float eulerRollDesired;
PRIVATE float eulerPitchDesired;
PRIVATE float eulerYawDesired;
PRIVATE float rollRateDesired;
PRIVATE float pitchRateDesired;
PRIVATE float yawRateDesired;
PRIVATE float fusionDt;




//



// Baro variables
PRIVATE float temperature; // temp from barometer
PRIVATE float pressure;    // pressure from barometer
PRIVATE float asl;     // smoothed asl
PRIVATE float aslRaw;  // raw asl
PRIVATE float aslLong; // long term asl

// Hover variables
PRIVATE PidObject hoverPID; // Used for hover mode. I gets reset when the bat status changes
bool hover = false;          // Currently in hover mode
bool set_hover = false;      // Hover mode has just been activated
PRIVATE float accWZ     = 0.0;
PRIVATE float vSpeedASL = 0.0;
PRIVATE float vSpeedAcc = 0.0;
PRIVATE float vSpeed    = 0.0; // Vertical speed (world frame) integrated from vertical acceleration
PRIVATE float hoverPIDVal;                    // Output of the PID controller
PRIVATE float hoverErr;                       // Different between target and current altitude

// Hover & Baro Params
PRIVATE float hoverKp                = 0.5;  // PID gain constants, used everytime we reinitialise the PID controller
PRIVATE float hoverKi                = 0.18;
PRIVATE float hoverKd                = 0.0;
PRIVATE float hoverChange            = 0;     // Change in target altitude
PRIVATE float hoverTarget            = -1;    // Target altitude
PRIVATE float hoverErrMax            = 1.0;   // max cap on current estimated altitude vs target altitude in meters
PRIVATE float hoverChange_SENS       = 200;   // sensitivity of target altitude change (thrust input control) while hovering. Higher = more sensitive & faster changes
PRIVATE float pidAslFac              = 13000; // relates meters asl to thrust
PRIVATE float pidAlpha               = 0.8;   // PID Smoothing //TODO: shouldnt need to do this
PRIVATE float vSpeedASLFac           = 0;    // multiplier
PRIVATE float vSpeedAccFac           = -48;  // multiplier
PRIVATE float vAccDeadband           = 0.05;  // Vertical acceleration deadband
PRIVATE float vSpeedASLDeadband      = 0.005; // Vertical speed based on barometer readings deadband
PRIVATE float vSpeedLimit            = 0.05;  // used to constrain vertical velocity
PRIVATE float errDeadband            = 0.00;  // error (target - altitude) deadband
PRIVATE float vBiasAlpha             = 0.91; // Blending factor we use to fuse vSpeedASL and vSpeedAcc
PRIVATE float aslAlpha               = 0.92; // Short term smoothing
PRIVATE float aslAlphaLong           = 0.93; // Long term smoothing
PRIVATE uint16_t hoverMinThrust      = 00000; // minimum hover thrust - not used yet
PRIVATE uint16_t hoverBaseThrust     = 43000; // approximate throttle needed when in perfect hover. More weight/older battery can use a higher value
PRIVATE uint16_t hoverMaxThrust      = 60000; // max hover thrust



//

RPYType rollType;
RPYType pitchType;
RPYType yawType;

uint16_t actuatorThrust;
int16_t  actuatorRoll;
int16_t  actuatorPitch;
int16_t  actuatorYaw;

uint32_t motorPowerM4;
uint32_t motorPowerM2;
uint32_t motorPowerM1;
uint32_t motorPowerM3;

LOG_GROUP_START(stabilizer)
LOG_ADD(LOG_FLOAT, roll, &eulerRollActual)
LOG_ADD(LOG_FLOAT, pitch, &eulerPitchActual)
LOG_ADD(LOG_FLOAT, yaw, &eulerYawActual)
LOG_ADD(LOG_UINT16, thrust, &actuatorThrust)
LOG_GROUP_STOP(stabilizer)

LOG_GROUP_START(motor)
LOG_ADD(LOG_INT32, m4, &motorPowerM4) 
LOG_ADD(LOG_INT32, m1, &motorPowerM1) 
LOG_ADD(LOG_INT32, m2, &motorPowerM2) 
LOG_ADD(LOG_INT32, m3, &motorPowerM3) 
LOG_GROUP_STOP(motor)

LOG_GROUP_START(acc)
LOG_ADD(LOG_FLOAT, x, &acc.x)
LOG_ADD(LOG_FLOAT, y, &acc.y)
LOG_ADD(LOG_FLOAT, z, &acc.z)
LOG_ADD(LOG_FLOAT, zw, &accWZ)
LOG_GROUP_STOP(acc)

LOG_GROUP_START(gyro)
LOG_ADD(LOG_FLOAT, x, &gyro.x)
LOG_ADD(LOG_FLOAT, y, &gyro.y)
LOG_ADD(LOG_FLOAT, z, &gyro.z)
LOG_GROUP_STOP(gyro)



// LOG Hovering PID controller states
LOG_GROUP_START(vpid)
LOG_ADD(LOG_FLOAT, pid, &hoverPID)
LOG_ADD(LOG_FLOAT, p, &hoverPID.outP)
LOG_ADD(LOG_FLOAT, i, &hoverPID.outI)
LOG_ADD(LOG_FLOAT, d, &hoverPID.outD)
LOG_GROUP_STOP(vpid)

LOG_GROUP_START(baro)
LOG_ADD(LOG_FLOAT, asl, &asl)
LOG_ADD(LOG_FLOAT, aslRaw, &aslRaw)
LOG_ADD(LOG_FLOAT, aslLong, &aslLong)
LOG_ADD(LOG_FLOAT, temp, &temperature)
LOG_ADD(LOG_FLOAT, pressure, &pressure)
LOG_GROUP_STOP(baro)

LOG_GROUP_START(hover)
LOG_ADD(LOG_FLOAT, err, &hoverErr)
LOG_ADD(LOG_FLOAT, target, &hoverTarget)
LOG_ADD(LOG_FLOAT, zSpeed, &vSpeed)
LOG_ADD(LOG_FLOAT, vSpeed, &vSpeed)
LOG_ADD(LOG_FLOAT, vSpeedASL, &vSpeedASL)
LOG_ADD(LOG_FLOAT, vSpeedAcc, &vSpeedAcc)
LOG_GROUP_STOP(hover)

//// Params for hovering
PARAM_GROUP_START(hover)
PARAM_ADD(PARAM_FLOAT, aslAlpha, &aslAlpha)
PARAM_ADD(PARAM_FLOAT, aslAlphaLong, &aslAlphaLong)
PARAM_ADD(PARAM_FLOAT, errDeadband, &errDeadband)
PARAM_ADD(PARAM_FLOAT, hoverChangeSens, &hoverChange_SENS)
PARAM_ADD(PARAM_FLOAT, hoverErrMax, &hoverErrMax)
PARAM_ADD(PARAM_FLOAT, kd, &hoverKd)
PARAM_ADD(PARAM_FLOAT, ki, &hoverKi)
PARAM_ADD(PARAM_FLOAT, kp, &hoverKp)
PARAM_ADD(PARAM_FLOAT, pidAlpha, &pidAlpha)
PARAM_ADD(PARAM_FLOAT, pidAslFac, &pidAslFac)
PARAM_ADD(PARAM_FLOAT, vAccDeadband, &vAccDeadband)
PARAM_ADD(PARAM_FLOAT, vBiasAlpha, &vBiasAlpha)
PARAM_ADD(PARAM_FLOAT, vSpeedAccFac, &vSpeedAccFac)
PARAM_ADD(PARAM_FLOAT, vSpeedASLDeadband, &vSpeedASLDeadband)
PARAM_ADD(PARAM_FLOAT, vSpeedASLFac, &vSpeedASLFac)
PARAM_ADD(PARAM_FLOAT, vSpeedLimit, &vSpeedLimit)
PARAM_ADD(PARAM_UINT16, baseThrust, &hoverBaseThrust)
PARAM_ADD(PARAM_UINT16, maxThrust, &hoverMaxThrust)
PARAM_ADD(PARAM_UINT16, minThrust, &hoverMinThrust)
PARAM_GROUP_STOP(hover)


static bool isInit;

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

  motorsInit();
  imu6Init();
  sensfusion6Init();
  controllerInit();

  rollRateDesired = 0;
  pitchRateDesired = 0;
  yawRateDesired = 0;

  xTaskCreate(stabilizerTask, (const signed char * const)"STABILIZER",
              2*configMINIMAL_STACK_SIZE, NULL, /*Piority*/2, NULL);

  isInit = TRUE;
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

static void stabilizerTask(void* param)
{
  uint32_t attitudeCounter = 0;
  uint32_t hoverCounter = 0;
  uint32_t lastWakeTime;

  vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR);

  //Wait for the system to be fully started to start stabilization loop
  systemWaitStart();

  lastWakeTime = xTaskGetTickCount ();

  while(1)
  {
    vTaskDelayUntil(&lastWakeTime, F2T(IMU_UPDATE_FREQ));

    imu6Read(&gyro, &acc);

    if (imu6IsCalibrated())
    {
      commanderGetRPY(&eulerRollDesired, &eulerPitchDesired, &eulerYawDesired);
      commanderGetRPYType(&rollType, &pitchType, &yawType);

      // 100HZ
      if (++hoverCounter >= HOVER_UPDATE_RATE_DIVIDER) {
          hoverCounter = 0;

          // Get hover commands from pilot
          commanderGetHover(&hover, &set_hover, &hoverChange);


          // Get barometer height estimates
          //TODO do the smoothing within getData
          ms5611GetData(&pressure, &temperature, &aslRaw);
          asl     = asl     * aslAlpha     + aslRaw * (1-aslAlpha);
          aslLong = aslLong * aslAlphaLong + aslRaw * (1-aslAlphaLong);


          // Estimate vertical speed based on successive barometer readings. This is ugly :)
          vSpeedASL = deadband(asl-aslLong, vSpeedASLDeadband);

          // Estimate vertical speed based on Acc - fused with baro to reduce drift
          vSpeed  = constrain(vSpeed, -vSpeedLimit, vSpeedLimit);
          vSpeed = vSpeed*vBiasAlpha + vSpeedASL*(1.f - vBiasAlpha);
          vSpeedAcc = vSpeed;

          // Reset Integral gain of PID controller if being charged
          if (!pmIsDischarging()){
              hoverPID.integ = 0.0;
          }

          // Hover mode just activated, set target altitude as current altitude. Reuse previous integral term as a starting point
          if (set_hover){
              // Set to current altitude
              hoverTarget = asl;

              // Cache last integral term for reuse after pid init
              const float pre_integral = hoverPID.integ;

              // Reset PID controller
              pidInit(&hoverPID, asl, hoverKp, hoverKi, hoverKd, HOVER_UPDATE_DT );
              // TODO set low and high limits depending on voltage
              // TODO for now just use previous I value and manually set limits for whole voltage range
              //                    pidSetIntegralLimit(&hoverPID, 12345);
              //                    pidSetIntegralLimitLow(&hoverPID, 12345);              /

              hoverPID.integ = pre_integral;

              // Reset hoverPID
              hoverPIDVal = pidUpdate(&hoverPID, asl, false);
          }

          // In hover mode
          if (hover){
              // Update target altitude from joy controller input
              hoverTarget += hoverChange/hoverChange_SENS;
              pidSetDesired(&hoverPID, hoverTarget);

              // LED on to show hover mode active
              if (hoverCounter==0){
                  ledseqRun(LED_RED, seq_hover);

              }

              // Compute error (current - target), limit the error
              hoverErr =  constrain(deadband(asl-hoverTarget, errDeadband), -hoverErrMax, hoverErrMax);
              pidSetError(&hoverPID, -hoverErr);

              // Get control from PID controller, dont update the error (done above)
              // Smooth it and include barometer vspeed
              // TODO same as smoothing the error??
              hoverPIDVal =   (pidAlpha    ) * hoverPIDVal
                      + (1.f-pidAlpha) * ((vSpeedAcc * vSpeedAccFac) + (vSpeedASL * vSpeedASLFac) + pidUpdate(&hoverPID, asl, false));

              // compute new thrust
              actuatorThrust = max( hoverMinThrust,
                      min(hoverMaxThrust,
                              limitThrust( hoverBaseThrust + (int32_t)(hoverPIDVal*pidAslFac))
                      )
              );

              // i part should compensate for voltage drop

          } else {
              hoverTarget = 0.0;
              hoverErr = 0.0;
              hoverPIDVal = 0.0;
          }
      }


      // 250HZ
      if (++attitudeCounter >= ATTITUDE_UPDATE_RATE_DIVIDER)
      {
        sensfusion6UpdateQ(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z, FUSION_UPDATE_DT);
        sensfusion6GetEulerRPY(&eulerRollActual, &eulerPitchActual, &eulerYawActual, acc.z, &accWZ);



        // Estimate speed from acc (drifts)
        vSpeed += deadband(accWZ, vAccDeadband) * FUSION_UPDATE_DT;


        controllerCorrectAttitudePID(eulerRollActual, eulerPitchActual, eulerYawActual,
                                     eulerRollDesired, eulerPitchDesired, -eulerYawDesired,
                                     &rollRateDesired, &pitchRateDesired, &yawRateDesired);
        attitudeCounter = 0;
      }

      if (rollType == RATE)
      {
        rollRateDesired = eulerRollDesired;
      }
      if (pitchType == RATE)
      {
        pitchRateDesired = eulerPitchDesired;
      }
      if (yawType == RATE)
      {
        yawRateDesired = -eulerYawDesired;
      }

      // TODO: Investigate possibility to subtract gyro drift.
      controllerCorrectRatePID(gyro.x, -gyro.y, gyro.z,
                               rollRateDesired, pitchRateDesired, yawRateDesired);

      controllerGetActuatorOutput(&actuatorRoll, &actuatorPitch, &actuatorYaw);

      if (!hover){
          // Use thrust from controller if not in hover mode
          commanderGetThrust(&actuatorThrust);
      }
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
      }
#if 0
     static int i = 0;
      if (++i > 19)
      {
        uartPrintf("%i, %i, %i\n",
            (int32_t)(eulerRollActual*100),
            (int32_t)(eulerPitchActual*100),
            (int32_t)(eulerYawActual*100));
        i = 0;
      }
#endif
    }
  }
}

static void distributePower(const uint16_t thrust, const int16_t roll,
                            const int16_t pitch, const int16_t yaw)
{
#ifdef QUAD_FORMATION_X
  roll = roll >> 1;
  pitch = pitch >> 1;
  motorPowerM1 = limitThrust(thrust - roll + pitch + yaw);
  motorPowerM2 = limitThrust(thrust - roll - pitch - yaw);
  motorPowerM3 =  limitThrust(thrust + roll - pitch + yaw);
  motorPowerM4 =  limitThrust(thrust + roll + pitch - yaw);
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
static float constrain(float value, const float minVal, const float maxVal){
    return min(maxVal,max(minVal,value));
}


// Deadzone
static float deadband(float value, const float threshold){
    if(fabs(value) < threshold) {
      value = 0;
    } else if(value > 0){
      value -= threshold;
    } else if(value < 0){
      value += threshold;
    }
    return value;
}

