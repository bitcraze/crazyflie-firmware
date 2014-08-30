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
 * pidtctrl.h - Used to receive/answer requests from client and to receive updated PID values from client
 */
#include "crtp.h"

//Constants used in some of the functions below
#define AUTOMATIC	1
#define MANUAL	0
#define DIRECT  0
#define REVERSE  1

typedef struct PidAutoTune {
	float kp;          // * (P)roportional Tuning Parameter
	float ki;          // * (I)ntegral Tuning Parameter
	float kd;          // * (D)erivative Tuning Parameter

	int controllerDirection;

	float myInput;      // * Pointers to the Input, Output, and Setpoint variables
	float myOutput;     //   This creates a hard link between the variables and the
	float mySetpoint;   //   PID, freeing the user from having to constantly tell us
						//   what these values are.  with pointers we'll just know.

	unsigned long lastTime;
	float ITerm, lastInput;

	unsigned long SampleTime;
	float outMin, outMax;
	bool inAuto;
} PID_AT;//PID Auto Tuning

/**
 * Initialize the PID control task
 * PID(float*, float*, float*,// * constructor.  links the PID to the Input, Output, and
 * float, float, float, int); //   Setpoint.  Initial tuning parameters are also set here
 */
void pidCtrlInit(PID_AT*, float, float, float, float, int);

void SetMode(PID_AT*, int);  // * sets PID to either Manual (0) or Auto (non-0)

bool Compute(PID_AT*); // * performs the PID calculation.  it should be
				//   called every time loop() cycles. ON/OFF and
				//   calculation frequency can be set using SetMode
				//   SetSampleTime respectively

void SetOutputLimits(PID_AT*, float, float); //clamps the output to a specific range. 0-255 by default, but
									  //it's likely the user will want to change this depending on
									  //the application

//available but not commonly used functions ********************************************************
void SetTunings(PID_AT*, float, float, float); // * While most users will set the tunings once in the
		       	   	   	   	   	   	   	 //   constructor, this function gives the user the option
										 //   of changing tunings during runtime for Adaptive control
void SetControllerDirection(PID_AT*, int);	// * Sets the Direction, or "Action" of the controller. DIRECT
									//   means the output will increase when error is positive. REVERSE
									//   means the opposite.  it's very unlikely that this will be needed
									//   once it is set in the constructor.
void SetSampleTime(PID_AT*, int);   // * sets the frequency, in Milliseconds, with which
						   //   the PID calculation is performed.  default is 100
void SetSetpoint(PID_AT*, float);   // * sets the frequency, in Milliseconds, with which

