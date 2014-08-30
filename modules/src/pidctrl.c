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
 * pidctrl.c - Used to receive/answer requests from client and to receive updated PID values from client
 */

/* FreeRtos includes */
//#include "FreeRTOS.h"
//#include "task.h"

//#include "crtp.h"
#include "pidctrl.h"
#include <time.h>
//#include "pid.h"

/*
typedef enum {
	pidCtrlValues = 0x00,
} PIDCrtlNbr;

void pidCrtlTask(void *param);

void pidCtrlInit() {
	xTaskCreate(pidCrtlTask, (const signed char * const )"PIDCrtl", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
	crtpInitTaskQueue(6);
}

void pidCrtlTask(void *param) {
	CRTPPacket p;
	extern PidObject pidRollRate;
	extern PidObject pidPitchRate;
	extern PidObject pidYawRate;
	extern PidObject pidRoll;
	extern PidObject pidPitch;
	extern PidObject pidYaw;
	struct pidValues {
		uint16_t rateKpRP;
		uint16_t rateKiRP;
		uint16_t rateKdRP;
		uint16_t attKpRP;
		uint16_t attKiRP;
		uint16_t attKdRP;
		uint16_t rateKpY;
		uint16_t rateKiY;
		uint16_t rateKdY;
		uint16_t attKpY;
		uint16_t attKiY;
		uint16_t attKdY;
	}__attribute__((packed));
	struct pidValues *pPid;

	while (TRUE) {
		if (crtpReceivePacketBlock(6, &p) == pdTRUE) {
			PIDCrtlNbr pidNbr = p.channel;

			switch (pidNbr) {
			case pidCtrlValues:
				pPid = (struct pidValues *) p.data;
				{
					pidSetKp(&pidRollRate, (float) pPid->rateKpRP / 100.0);
					pidSetKi(&pidRollRate, (float) pPid->rateKiRP / 100.0);
					pidSetKd(&pidRollRate, (float) pPid->rateKdRP / 100.0);
					pidSetKp(&pidRoll, (float) pPid->attKpRP / 100.0);
					pidSetKi(&pidRoll, (float) pPid->attKiRP / 100.0);
					pidSetKd(&pidRoll, (float) pPid->attKdRP / 100.0);
					pidSetKp(&pidPitchRate, (float) pPid->rateKpRP / 100.0);
					pidSetKi(&pidPitchRate, (float) pPid->rateKiRP / 100.0);
					pidSetKd(&pidPitchRate, (float) pPid->rateKdRP / 100.0);
					pidSetKp(&pidPitch, (float) pPid->attKpRP / 100.0);
					pidSetKi(&pidPitch, (float) pPid->attKiRP / 100.0);
					pidSetKd(&pidPitch, (float) pPid->attKdRP / 100.0);
					pidSetKp(&pidYawRate, (float) pPid->rateKpY / 100.0);
					pidSetKi(&pidYawRate, (float) pPid->rateKiY / 100.0);
					pidSetKd(&pidYawRate, (float) pPid->rateKdY / 100.0);
					pidSetKp(&pidYaw, (float) pPid->attKpY / 100.0);
					pidSetKi(&pidYaw, (float) pPid->attKiY / 100.0);
					pidSetKd(&pidYaw, (float) pPid->attKdY / 100.0);
				}
				break;
			default:
				break;
			}
		}
	}
}
*/

void Initialize(PID_AT *pid);
/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
void pidCtrlInit(PID_AT *pid, float Input, float Kp, float Ki, float Kd, int ControllerDirection)
{
	//float Output, Setpoint;
    //pid->myOutput = Output;
    pid->myInput = Input;
    //pid->mySetpoint = Setpoint;
    pid->inAuto = false;

	SetOutputLimits(pid, 0, 255);				//default output limit corresponds to
											//the arduino pwm limits

	pid->SampleTime = 2;//ms				//default Controller Sample Time is 500Hz

    SetControllerDirection(pid, ControllerDirection);
    SetTunings(pid, Kp, Ki, Kd);

    pid->lastTime = time(NULL)*1000-pid->SampleTime;
    Initialize(pid);
}


/* Compute() **********************************************************************
 *   This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/
bool Compute(PID_AT *pid)
{
   if(!pid->inAuto) return false;
   unsigned long now = time(NULL)*1000;
   unsigned long timeChange = (now - pid->lastTime);
   if(timeChange>=pid->SampleTime)
   {
      /*Compute all the working error variables*/
	  float input = pid->myInput;
      float error = pid->mySetpoint - input;
      pid->ITerm += (pid->ki * error);
      if(pid->ITerm > pid->outMax) pid->ITerm = pid->outMax;
      else if(pid->ITerm < pid->outMin) pid->ITerm = pid->outMin;
      float dInput = (input - pid->lastInput);

      /*Compute PID Output*/
      float output = pid->kp * error + pid->ITerm- pid->kd * dInput;

	  if(output > pid->outMax) output = pid->outMax;
      else if(output < pid->outMin) output = pid->outMin;
	  pid->myOutput = output;

      /*Remember some variables for next time*/
	  pid->lastInput = input;
	  pid->lastTime = now;
	  return true;
   }
   else return false;
}


/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
void SetTunings(PID_AT *pid, float Kp, float Ki, float Kd)
{
   if (Kp<0 || Ki<0 || Kd<0) return;

   float SampleTimeInSec = ((float)pid->SampleTime)/1000;
   pid->kp = Kp;
   pid->ki = Ki * SampleTimeInSec;
   pid->kd = Kd / SampleTimeInSec;

  if(pid->controllerDirection == REVERSE)
   {
	  pid->kp = (0 - pid->kp);
	  pid->ki = (0 - pid->ki);
	  pid->kd = (0 - pid->kd);
   }
}

/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed
 ******************************************************************************/
void SetSampleTime(PID_AT *pid, int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      float ratio  = (float)NewSampleTime/(float)pid->SampleTime;
      pid->ki *= ratio;
      pid->kd /= ratio;
      pid->SampleTime = (unsigned long)NewSampleTime;
   }
}

/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void SetOutputLimits(PID_AT *pid, float Min, float Max)
{
   if(Min >= Max) return;
   pid->outMin = Min;
   pid->outMax = Max;

   if(pid->inAuto)
   {
	   if(pid->myOutput > pid->outMax) pid->myOutput = pid->outMax;
	   else if(pid->myOutput < pid->outMin) pid->myOutput = pid->outMin;

	   if(pid->ITerm > pid->outMax) pid->ITerm = pid->outMax;
	   else if(pid->ITerm < pid->outMin) pid->ITerm = pid->outMin;
   }
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
void SetMode(PID_AT *pid, int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto == !pid->inAuto)
    {  /*we just went from manual to auto*/
        Initialize(pid);
    }
    pid->inAuto = newAuto;
}

/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
void Initialize(PID_AT *pid)
{
	pid->ITerm = pid->myOutput;
	pid->lastInput = pid->myInput;
	if(pid->ITerm > pid->outMax) pid->ITerm = pid->outMax;
	else if(pid->ITerm < pid->outMin) pid->ITerm = pid->outMin;
}

/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void SetControllerDirection(PID_AT *pid, int Direction)
{
   if(pid->inAuto && Direction != pid->controllerDirection)
   {
	   pid->kp = (0 - pid->kp);
	   pid->ki = (0 - pid->ki);
	   pid->kd = (0 - pid->kd);
   }
   pid->controllerDirection = Direction;
}

void SetSetpoint(PID_AT *pid, float setPoint){
	pid->mySetpoint = setPoint;
}
