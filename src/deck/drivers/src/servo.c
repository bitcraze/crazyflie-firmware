/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2024 Bitcraze AB
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
 * Using contributions from: Eric Ewing, Will Wu
 */
#define DEBUG_MODULE "SERVO"

#include <stdbool.h>

/* ST includes */
#include "stm32fxxx.h"

//FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include <string.h>
#include <inttypes.h>
#include "motors.h"
#include "param.h"
#include "deck.h"

static uint16_t servo_MIN_us = 1000;
static uint16_t servo_MAX_us = 2000;

#include "servo.h"

// #define DEBUG_SERVO

static bool isInit = false;

const MotorPerifDef* servoMap;
extern const MotorPerifDef* servoMapIO1;
extern const MotorPerifDef* servoMapIO2;
extern const MotorPerifDef* servoMapIO3;
extern const MotorPerifDef* servoMapRX2;
extern const MotorPerifDef* servoMapTX2;
extern const MotorPerifDef* servoMapMOSI;

/* Public functions */
static uint8_t servo_idle = 90;
static uint8_t s_servo_angle;
static uint8_t servo_range = 180; // in degrees

void servoMapInit(const MotorPerifDef* servoMapSelect)
{
  servoMap = servoMapSelect;

  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  //clock the servo pin and the timers
  RCC_AHB1PeriphClockCmd(servoMap->gpioPerif, ENABLE);
  RCC_APB1PeriphClockCmd(servoMap->timPerif, ENABLE);

  //configure gpio for timer out
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Pin = servoMap->gpioPin;
  GPIO_Init(servoMap->gpioPort, &GPIO_InitStructure);

  //map timer to alternate function
  GPIO_PinAFConfig(servoMap->gpioPort, servoMap->gpioPinSource, servoMap->gpioAF);

  //Timer configuration
  TIM_TimeBaseStructure.TIM_Period = SERVO_PWM_PERIOD;
  TIM_TimeBaseStructure.TIM_Prescaler = SERVO_PWM_PRESCALE;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(servoMap->tim, &TIM_TimeBaseStructure);

  // PWM channels configuration
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

  // Configure OC1
  servoMap->ocInit(servoMap->tim, &TIM_OCInitStructure);
  servoMap->preloadConfig(servoMap->tim, TIM_OCPreload_Enable);


  //Enable the timer PWM outputs
  TIM_CtrlPWMOutputs(servoMap->tim, ENABLE);
  servoMap->setCompare(servoMap->tim, 0x00);

  //Enable the timer
  TIM_Cmd(servoMap->tim, ENABLE);
}

void servoInit()
{
  if (isInit){
    return;
  }

  #ifdef CONFIG_DECK_SERVO_USE_IO1
    servoMapInit(servoMapIO1);
    DEBUG_PRINT("Init on IO1 [OK]\n");
  #elif CONFIG_DECK_SERVO_USE_IO2
    servoMapInit(servoMapIO2);
    DEBUG_PRINT("Init on IO2 [OK]\n");
  #elif CONFIG_DECK_SERVO_USE_IO3
    servoMapInit(servoMapIO3);
    DEBUG_PRINT("Init on IO3 [OK]\n");
  #elif CONFIG_DECK_SERVO_USE_RX2
    servoMapInit(servoMapRX2);
    DEBUG_PRINT("Init on RX2 [OK]\n"); // not working on Bolt 1.1...
  #elif CONFIG_DECK_SERVO_USE_MOSI
    servoMapInit(servoMapMOSI);
    DEBUG_PRINT("Init on MOSI [OK]\n");
  #elif CONFIG_DECK_SERVO_USE_TX2
    servoMapInit(servoMapTX2);
    DEBUG_PRINT("Init on TX2 [OK]\n"); // not working on Bolt 1.1...
  #else
    isInit = false
    DEBUG_PRINT("Failed to configure servo pin!\n");
    return;
  #endif
  
  servoSetAngle(saturateAngle(servo_idle));

  s_servo_angle = servo_idle;

  isInit = true;
}

bool servoTest(void)
{
  return isInit;
}

void servoSetAngle(uint8_t angle)
{
  // set CCR register
  // Duty% = CCR/ARR*100, so CCR = Duty%/100 * ARR

  double pulse_length_us = (double)(angle) / servo_range * (servo_MAX_us - servo_MIN_us) + servo_MIN_us;
  double pulse_length_s = pulse_length_us / 1000000;
  const uint32_t ccr_val = (uint32_t)(pulse_length_s * SERVO_PWM_PERIOD * SERVO_PWM_FREQUENCY_HZ);
  servoMap->setCompare(servoMap->tim, ccr_val);
  
  #ifdef DEBUG_SERVO
    DEBUG_PRINT("Set Angle: %u deg, pulse width: %f us \n", angle, pulse_length_us);
  #endif
}

uint8_t saturateAngle(uint8_t angle)
{
  if (angle > servo_range) {
    return servo_range;
  }
  else if (angle < 0) {
    return 0;
  }
  else {
    return angle;
  }

}

void servoAngleCallBack(void)
{
  servoSetAngle(saturateAngle(s_servo_angle));
}

static const DeckDriver servo_deck = {
  .vid = 0x00,
  .pid = 0x00,
  .name = "bcServo",

  #ifdef CONFIG_DECK_SERVO_USE_IO1
    .usedPeriph = DECK_USING_TIMER4,
    .usedGpio = DECK_USING_IO_1,
  #elif CONFIG_DECK_SERVO_USE_IO2
    .usedPeriph = DECK_USING_TIMER3,
    .usedGpio = DECK_USING_IO_2,
  #elif CONFIG_DECK_SERVO_USE_IO3
    .usedPeriph = DECK_USING_TIMER3,
    .usedGpio = DECK_USING_IO_3,
  #elif CONFIG_DECK_SERVO_USE_RX2
    .usedPeriph = DECK_USING_TIMER5,
    .usedGpio = DECK_USING_PA3,
  #elif CONFIG_DECK_SERVO_USE_MOSI
    .usedPeriph = DECK_USING_TIMER14,
    .usedGpio = DECK_USING_PA7,
  #elif CONFIG_DECK_SERVO_USE_TX2
    .usedPeriph = DECK_USING_TIMER5,
    .usedGpio = DECK_USING_PA2,
  #else
    .usedPeriph = 0,
    .usedGpio = 0,
  #endif
  
  .init = servoInit,
  .test = servoTest,
};

DECK_DRIVER(servo_deck);

PARAM_GROUP_START(deck)

PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcServo, &isInit)
PARAM_GROUP_STOP(deck)

/**
 * "Servo" deck parameters
 */
PARAM_GROUP_START(servo)

/**
 * @brief PWM pulse width for minimal servo position (in microseconds)
 */
PARAM_ADD(PARAM_UINT16 | PARAM_PERSISTENT, servoMINus, &servo_MIN_us)
/**
 * @brief PWM pulse width for maximal servo position (in microseconds)
 */
PARAM_ADD(PARAM_UINT16 | PARAM_PERSISTENT, servoMAXus, &servo_MAX_us)
/**
 * @brief Servo range, i.e. angle between the min and max positions (in degrees)
 */
PARAM_ADD(PARAM_UINT8 | PARAM_PERSISTENT, servoRange, &servo_range)
/**
 * @brief Servo idle (startup) angular position (in degrees, min = 0, max = servoRange)
 */
PARAM_ADD(PARAM_UINT8 | PARAM_PERSISTENT, servoIdle, &servo_idle)
/**
 * @brief Servo angular position (in degrees, min = 0, max = servoRange)
 */
PARAM_ADD_WITH_CALLBACK(PARAM_UINT8 , servoAngle, &s_servo_angle, &servoAngleCallBack)

PARAM_GROUP_STOP(servo)
