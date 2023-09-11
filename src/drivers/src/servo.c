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
 * servo.c - Servo driver
 *
 * This code mainly interfacing the PWM peripheral lib of ST.
 * Author: Eric Ewing
 */

#include <stdbool.h>

/* ST includes */
#include "stm32fxxx.h"
#include "param.h"


//FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include <string.h>
#include <inttypes.h>
#include <motors.h>


// TODO Verify PWM settings
#define SERVO_PWM_BITS      (0)
#define SERVO_PWM_PERIOD    1000  // 84mhz -> 50hz with 100 as a prescaler
#define SERVO_PWM_PRESCALE  (uint16_t) (1680 * 2) //84mhz to khz
#define SERVO_BASE_FREQ     (0) // should be calculated

#include "servo.h"

static bool isInit = false;
// we use the "servoMapMOSI" struct to initialize PWM
extern const MotorPerifDef* servoMapMOSI;

/* Public functions */

void servoInit()
{
  if (isInit){
    DEBUG_PRINT("Already Initialized!");
    return;
  }
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  //clock the servo pin and the timers
  RCC_AHB1PeriphClockCmd(servoMapMOSI->gpioPerif, ENABLE);
  RCC_APB1PeriphClockCmd(servoMapMOSI->timPerif, ENABLE);

  //configure gpio for timer out
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; // TODO: verify this
  GPIO_InitStructure.GPIO_Pin = servoMapMOSI->gpioPin;
  GPIO_Init(servoMapMOSI->gpioPort, &GPIO_InitStructure);

  //map timer to alternate function
  GPIO_PinAFConfig(servoMapMOSI->gpioPort, servoMapMOSI->gpioPin, servoMapMOSI->gpioAF);

  //Timer configuration
  TIM_TimeBaseStructure.TIM_Period = SERVO_PWM_PERIOD;
  TIM_TimeBaseStructure.TIM_Prescaler = SERVO_PWM_PRESCALE;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(servoMapMOSI->tim, &TIM_TimeBaseStructure);

  // PWM channels configuration
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

  // Configure OC3
  servoMapMOSI->ocInit(servoMapMOSI->tim, &TIM_OCInitStructure);
  servoMapMOSI->preloadConfig(servoMapMOSI->tim, TIM_OCPreload_Enable);


  //Enable the timer PWM outputs
  TIM_CtrlPWMOutputs(servoMapMOSI->tim, ENABLE);
  servoMapMOSI->setCompare(servoMapMOSI->tim, 0x00);

  //Enable the timer
  TIM_Cmd(servoMapMOSI->tim, ENABLE);

  servoSetAngle(90);
  isInit = true;
}

bool servoTest(void)
{
  return isInit;
}

void servoSetAngle(uint8_t angle)
{
  // Convert angle to between .025 and .125 and then to ratio of signal
  double angle_d = (double) angle;
  uint32_t ratio = ((angle_d / 180.)* SERVO_PWM_PERIOD) / 10. + 100;
  servoMapMOSI->setCompare(servoMapMOSI->tim, ratio);
  // TIM_SetCompare4(SERVO_TIM, ratio);
  DEBUG_PRINT("Set Angle: %u, set ratio: %" PRId32 "\n", angle, ratio);
}

PARAM_GROUP_START(servo_controller)

// PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, servoInit, &isInit)

PARAM_GROUP_STOP(servo_controller)
