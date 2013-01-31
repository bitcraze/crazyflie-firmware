/**
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
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
 * multilog.c : multiple source log task
 */

#include <string.h>

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"

/* Project includes */
#include "imu.h"
#include "pid.h"
#include "crtp.h"
#include "multilog.h"
#include "pm.h"
#include "motors.h"
#include "commander.h"

//#define MULTILOG_SEND_DUAL_IMU_DATA

#define NORMAL_DATA

/* Availale log packets */
typedef enum {
  logReserved = 0,
  logAcc      = 1,
  logGyro     = 2,
  logImu      = 3,
  logPidRP    = 4,
  logPidY     = 5,
  logFlight   = 6,
} logId;

/* Local functions */
void multilogTask(void *param);

/* Implementation */

//Multilog task launcher
void multilogLaunch(int taskId) {
  xTaskCreate(multilogTask, (const signed char * const)"Logging",
              configMINIMAL_STACK_SIZE, (void*) taskId, /*priority*/2, NULL);
}

// Multilog task
void multilogTask(void *param) {
  int taskId = (int)param;
  CRTPPacket p;
  int ret;
  /* Parameter of the task */
  unsigned short bitField = 0;
  unsigned short period = 0;

  //Register this task to receive CRTP Packets
  crtpInitTaskQueue(taskId);
  
  while(1)
  {
    // Wait for a new config packet. Will continue every 'period' milli seconds 
    //if no packet received and the logging is activated
    if (period>0)
      ret = crtpReceivePacketWait(taskId, &p, period);
    else
      ret = crtpReceivePacketBlock(taskId, &p);
    
    // If a packet is received from CRTP the configuration contained in it is
    // extracted
    if (ret == pdTRUE)
    {
      memcpy((uint8_t*)&period, &p.data[0], sizeof(unsigned short));
      memcpy((uint8_t*)&bitField, &p.data[2], sizeof(unsigned short));
    }
    
    //Send the activated log packets
    if (bitField & (1<<logAcc))
    {
      extern Axis3f acc;

      p.port = taskId;
      p.data[0] = logAcc;
      memcpy(&p.data[1+(0)], (uint8_t*)&acc.x, sizeof(float));
      memcpy(&p.data[1+(sizeof(float))], (uint8_t*)&acc.y, sizeof(float));
      memcpy(&p.data[1+(2*sizeof(float))], (uint8_t*)&acc.z, sizeof(float));
      p.size = 1+(3*sizeof(float));
      crtpSendPacket(&p);
    }
    
    if (bitField & (1<<logGyro))
    {
      extern Axis3f gyro; // Gyro axis data in deg/s
      
      p.port = taskId;
      p.data[0] = logGyro;
      memcpy(&p.data[1+(0)], (uint8_t*)&gyro.x, sizeof(float));
      memcpy(&p.data[1+(sizeof(float))], (uint8_t*)&gyro.y, sizeof(float));
      memcpy(&p.data[1+(2*sizeof(float))], (uint8_t*)&gyro.z, sizeof(float));
      p.size = 1+(3*sizeof(float));
      crtpSendPacket(&p);
    }
    
    if (bitField & (1<<logImu))
    {
      extern float eulerRollActual;
      extern float eulerPitchActual;
      extern float eulerYawActual;
      p.port = taskId;
      p.data[0] = logImu;
      memcpy(&p.data[1+(0)], (uint8_t*)&eulerRollActual, sizeof(float));
      memcpy(&p.data[1+(sizeof(float))], (uint8_t*)&eulerPitchActual, sizeof(float));
      memcpy(&p.data[1+(2*sizeof(float))], (uint8_t*)&eulerYawActual, sizeof(float));
      p.size = 1+(3*sizeof(float));
      crtpSendPacket(&p);
    }

    if (bitField & (1<<logPidRP))
    {
      extern PidObject pidPitch;
      extern PidObject pidRoll;

      p.port = taskId;
      p.data[0] = logPidRP;
      memcpy(&p.data[1+(0)], (uint8_t*)&pidPitch.outP, sizeof(int16_t));
      memcpy(&p.data[1+(1*sizeof(int16_t))], (uint8_t*)&pidPitch.outI, sizeof(int16_t));
      memcpy(&p.data[1+(2*sizeof(int16_t))], (uint8_t*)&pidPitch.outD, sizeof(int16_t));

      memcpy(&p.data[1+(3*sizeof(int16_t))], (uint8_t*)&pidRoll.outP, sizeof(int16_t));
      memcpy(&p.data[1+(4*sizeof(int16_t))], (uint8_t*)&pidRoll.outI, sizeof(int16_t));
      memcpy(&p.data[1+(5*sizeof(int16_t))], (uint8_t*)&pidRoll.outD, sizeof(int16_t));

      p.size = 1+(6*sizeof(int16_t));
      crtpSendPacket(&p);
    }

    if (bitField & (1<<logPidY))
    {
      extern PidObject pidYaw;

      p.port = taskId;
      p.data[0] = logPidY;
      memcpy(&p.data[1+(0)], (uint8_t*)&pidYaw.error, sizeof(float));
      memcpy(&p.data[1+(1*sizeof(float))], (uint8_t*)&pidYaw.integ, sizeof(float));
      memcpy(&p.data[1+(2*sizeof(float))], (uint8_t*)&pidYaw.deriv, sizeof(float));

      p.size = 1+(3*sizeof(float));
      crtpSendPacket(&p);
    }

    if (bitField & (1<<logFlight))
    {
      extern float eulerRollActual;
      extern float eulerPitchActual;
      extern float eulerYawActual;
      extern PidObject pidPitch;
      extern PidObject pidRoll;
      extern PidObject pidYaw;
#ifdef NORMAL_DATA
      uint8_t mthrust;
      uint16_t thrust;
#else
      extern Axis3i16  gyroMpu;
      extern Axis3i16  accelMpu;
#endif

      int16_t roll;
      int16_t pitch;
      int16_t yaw;
      uint16_t battery;
      uint32_t timestamp = xTaskGetTickCount();

      p.port = taskId;
      p.data[0] = logFlight;

      // Timestamp: 4 bytes
      memcpy(&p.data[1+(0)], (uint8_t*)&timestamp, sizeof(uint32_t));
      p.size = 1 + sizeof(uint32_t);
      // IMU Roll&Pitch&Yaw: 6 bytes centi-degrees
      roll = (int16_t)(eulerRollActual * 100);
      pitch = (int16_t)(eulerPitchActual * 100);
      yaw = (int16_t)(eulerYawActual * 100);
      memcpy(&p.data[p.size+0], (uint8_t*)&roll, sizeof(int16_t));
      memcpy(&p.data[p.size+sizeof(int16_t)], (uint8_t*)&pitch, sizeof(int16_t));
      memcpy(&p.data[p.size+2*sizeof(int16_t)], (uint8_t*)&yaw, sizeof(int16_t));
      // Battery 2 bytes mV
      battery = pmGetBatteryVoltage() * 1000;
      memcpy(&p.data[p.size+3*sizeof(uint16_t)], (uint8_t*)&battery, sizeof(uint16_t));
      p.size += 4*sizeof(uint16_t);
#ifdef NORMAL_DATA
      // Motor thrusts: 4 bytes
      mthrust = (uint8_t)(motorsGetRatio(MOTOR_LEFT) >> 8);
      memcpy(&p.data[p.size+(0*sizeof(uint8_t))], (uint8_t*)&mthrust, sizeof(uint8_t));
      mthrust = (uint8_t)(motorsGetRatio(MOTOR_RIGHT) >> 8);
      memcpy(&p.data[p.size+(1*sizeof(uint8_t))], (uint8_t*)&mthrust, sizeof(uint8_t));
      mthrust = (uint8_t)(motorsGetRatio(MOTOR_FRONT) >> 8);
      memcpy(&p.data[p.size+(2*sizeof(uint8_t))], (uint8_t*)&mthrust, sizeof(uint8_t));
      mthrust = (uint8_t)(motorsGetRatio(MOTOR_REAR) >> 8);
      memcpy(&p.data[p.size+(3*sizeof(uint8_t))], (uint8_t*)&mthrust, sizeof(uint8_t));
      p.size += 4*sizeof(uint8_t);

      // Set values: 7 bytes
      commanderGetTrust(&thrust);
      roll = (int16_t)(pidRoll.desired * 100);
      pitch = (int16_t)(pidPitch.desired * 100);
      yaw = (int16_t)(pidYaw.desired * 100);
      memcpy(&p.data[p.size+(0)], (uint8_t*)&roll, sizeof(int16_t));
      memcpy(&p.data[p.size+(sizeof(int16_t))], (uint8_t*)&pitch, sizeof(int16_t));
      memcpy(&p.data[p.size+(2*sizeof(int16_t))], (uint8_t*)&yaw, sizeof(int16_t));
      memcpy(&p.data[p.size+(3*sizeof(uint16_t))], (uint8_t*)&thrust, sizeof(uint8_t));
      p.size += 3*sizeof(uint16_t) + sizeof(uint8_t);
#else
      // mpu6050 values: 12 bytes
      memcpy(&p.data[p.size+0*sizeof(int16_t)], (uint8_t*)&gyroMpu.x, sizeof(int16_t));
      memcpy(&p.data[p.size+1*sizeof(int16_t)], (uint8_t*)&gyroMpu.y, sizeof(int16_t));
      memcpy(&p.data[p.size+2*sizeof(int16_t)], (uint8_t*)&gyroMpu.z, sizeof(int16_t));
      memcpy(&p.data[p.size+3*sizeof(int16_t)], (uint8_t*)&accelMpu.x, sizeof(int16_t));
      memcpy(&p.data[p.size+4*sizeof(int16_t)], (uint8_t*)&accelMpu.y, sizeof(int16_t));
      memcpy(&p.data[p.size+5*sizeof(int16_t)], (uint8_t*)&accelMpu.z, sizeof(int16_t));
      p.size += 6*sizeof(uint16_t);
#endif
      crtpSendPacket(&p);
    }
  }
}

