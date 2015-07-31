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
 * motors.c - Motor driver
 *
 * This code mainly interfacing the PWM peripheral lib of ST.
 */

#include <stdbool.h>

/* ST includes */
#include "stm32fxxx.h"

#include "motors.h"
#include "pm.h"

//FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"

typedef struct
{
  uint32_t      gpioPerif;
  GPIO_TypeDef* gpioPort;
  uint32_t      gpioPin;
  uint32_t      gpioPinSource;
  uint32_t      gpioAF;
  uint32_t      timPerif;
  TIM_TypeDef*  tim;
  uint16_t      timPolarity;
  uint32_t      timDbgStop;
  uint32_t      timPeriod;
  uint16_t      timPrescaler;
  /* Function pointers */
  void (*setCompare)(TIM_TypeDef* TIMx, uint32_t Compare2);
  uint32_t (*getCompare)(TIM_TypeDef* TIMx);
  void (*ocInit)(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
  void (*preloadConfig)(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
} MotorPerifDef;

static const MotorPerifDef DECK_TX2 =
{
    .gpioPerif     = RCC_AHB1Periph_GPIOA,
    .gpioPort      = GPIOA,
    .gpioPin       = GPIO_Pin_2,
    .gpioPinSource = GPIO_PinSource2,
    .gpioAF        = GPIO_AF_TIM2,
    .timPerif      = RCC_APB1Periph_TIM2,
    .tim           = TIM2,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM2_STOP,
    .timPeriod     = MOTORS_PWM_PERIOD,
    .timPrescaler  = MOTORS_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare3,
    .getCompare    = TIM_GetCapture3,
    .ocInit        = TIM_OC3Init,
    .preloadConfig = TIM_OC3PreloadConfig,
};

static const MotorPerifDef DECK_RX2 =
{
    .gpioPerif     = RCC_AHB1Periph_GPIOA,
    .gpioPort      = GPIOA,
    .gpioPin       = GPIO_Pin_3,
    .gpioPinSource = GPIO_PinSource3,
    .gpioAF        = GPIO_AF_TIM2,
    .timPerif      = RCC_APB1Periph_TIM2,
    .tim           = TIM2,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM2_STOP,
    .timPeriod     = MOTORS_PWM_PERIOD,
    .timPrescaler  = MOTORS_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare4,
    .getCompare    = TIM_GetCapture4,
    .ocInit        = TIM_OC4Init,
    .preloadConfig = TIM_OC4PreloadConfig,
};

static const MotorPerifDef DECK_IO2 =
{
    .gpioPerif     = RCC_AHB1Periph_GPIOB,
    .gpioPort      = GPIOB,
    .gpioPin       = GPIO_Pin_5,
    .gpioPinSource = GPIO_PinSource5,
    .gpioAF        = GPIO_AF_TIM3,
    .timPerif      = RCC_APB1Periph_TIM3,
    .tim           = TIM3,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM3_STOP,
    .timPeriod     = MOTORS_PWM_PERIOD,
    .timPrescaler  = MOTORS_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare2,
    .getCompare    = TIM_GetCapture2,
    .ocInit        = TIM_OC2Init,
    .preloadConfig = TIM_OC2PreloadConfig,
};

static const MotorPerifDef DECK_IO3 =
{
    .gpioPerif     = RCC_AHB1Periph_GPIOB,
    .gpioPort      = GPIOB,
    .gpioPin       = GPIO_Pin_4,
    .gpioPinSource = GPIO_PinSource4,
    .gpioAF        = GPIO_AF_TIM3,
    .timPerif      = RCC_APB1Periph_TIM3,
    .tim           = TIM3,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM3_STOP,
    .timPeriod     = MOTORS_PWM_PERIOD,
    .timPrescaler  = MOTORS_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare1,
    .getCompare    = TIM_GetCapture1,
    .ocInit        = TIM_OC1Init,
    .preloadConfig = TIM_OC1PreloadConfig,
};

static const MotorPerifDef CONN_M1 =
{
    .gpioPerif     = RCC_AHB1Periph_GPIOA,
    .gpioPort      = GPIOA,
    .gpioPin       = GPIO_Pin_1,
    .gpioPinSource = GPIO_PinSource1,
    .gpioAF        = GPIO_AF_TIM2,
    .timPerif      = RCC_APB1Periph_TIM2,
    .tim           = TIM2,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM2_STOP,
    .timPeriod     = MOTORS_PWM_PERIOD,
    .timPrescaler  = MOTORS_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare2,
    .getCompare    = TIM_GetCapture2,
    .ocInit        = TIM_OC2Init,
    .preloadConfig = TIM_OC2PreloadConfig,
};

static const MotorPerifDef CONN_M2 =
{
    .gpioPerif     = RCC_AHB1Periph_GPIOB,
    .gpioPort      = GPIOB,
    .gpioPin       = GPIO_Pin_11,
    .gpioPinSource = GPIO_PinSource11,
    .gpioAF        = GPIO_AF_TIM2,
    .timPerif      = RCC_APB1Periph_TIM2,
    .tim           = TIM2,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM2_STOP,
    .timPeriod     = MOTORS_PWM_PERIOD,
    .timPrescaler  = MOTORS_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare4,
    .getCompare    = TIM_GetCapture4,
    .ocInit        = TIM_OC4Init,
    .preloadConfig = TIM_OC4PreloadConfig,
};

static const MotorPerifDef CONN_M3 =
{
    .gpioPerif     = RCC_AHB1Periph_GPIOA,
    .gpioPort      = GPIOA,
    .gpioPin       = GPIO_Pin_15,
    .gpioPinSource = GPIO_PinSource15,
    .gpioAF        = GPIO_AF_TIM2,
    .timPerif      = RCC_APB1Periph_TIM2,
    .tim           = TIM2,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM2_STOP,
    .timPeriod     = MOTORS_PWM_PERIOD,
    .timPrescaler  = MOTORS_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare1,
    .getCompare    = TIM_GetCapture1,
    .ocInit        = TIM_OC1Init,
    .preloadConfig = TIM_OC1PreloadConfig,
};

static const MotorPerifDef CONN_M4 =
{
    .gpioPerif     = RCC_AHB1Periph_GPIOB,
    .gpioPort      = GPIOB,
    .gpioPin       = GPIO_Pin_9,
    .gpioPinSource = GPIO_PinSource9,
    .gpioAF        = GPIO_AF_TIM4,
    .timPerif      = RCC_APB1Periph_TIM4,
    .tim           = TIM4,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM4_STOP,
    .timPeriod     = MOTORS_PWM_PERIOD,
    .timPrescaler  = MOTORS_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare4,
    .getCompare    = TIM_GetCapture4,
    .ocInit        = TIM_OC4Init,
    .preloadConfig = TIM_OC4PreloadConfig,
};


static const MotorPerifDef* motorMap[NBR_OF_MOTORS] = {&CONN_M1, &CONN_M2, &CONN_M3, &CONN_M4};

/* Utils Conversion macro */
#ifdef BRUSHLESS_MOTORCONTROLLER
#define C_BITS_TO_16(X) (0xFFFF * (X - MOTORS_PWM_CNT_FOR_1MS) / MOTORS_PWM_CNT_FOR_1MS)
#define C_16_TO_BITS(X) (MOTORS_PWM_CNT_FOR_1MS + ((X * MOTORS_PWM_CNT_FOR_1MS) / 0xFFFF))
#else
#define C_BITS_TO_16(X) ((X)<<(16-MOTORS_PWM_BITS))
#define C_16_TO_BITS(X) ((X)>>(16-MOTORS_PWM_BITS)&((1<<MOTORS_PWM_BITS)-1))
#endif

const int MOTORS[] = { MOTOR_M1, MOTOR_M2, MOTOR_M3, MOTOR_M4 };
static bool isInit = false;

/* Public functions */

//Initialization. Will set all motors ratio to 0%
void motorsInit()
{
  int i;

  if (isInit)
    return;

  //Init structures
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  for (i = 0; i < NBR_OF_MOTORS; i++)
  {
    //Clock the gpio and the timers
    RCC_AHB1PeriphClockCmd(motorMap[i]->gpioPerif, ENABLE);
    RCC_APB1PeriphClockCmd(motorMap[i]->timPerif, ENABLE);

    // Configure the GPIO for the timer output
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_Pin = motorMap[i]->gpioPin;
    GPIO_Init(motorMap[i]->gpioPort, &GPIO_InitStructure);

    //Map timers to alternate functions
    GPIO_PinAFConfig(motorMap[i]->gpioPort, motorMap[i]->gpioPinSource, motorMap[i]->gpioAF);

    //Timer configuration
    TIM_TimeBaseStructure.TIM_Period = motorMap[i]->timPeriod;
    TIM_TimeBaseStructure.TIM_Prescaler = motorMap[i]->timPrescaler;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(motorMap[i]->tim, &TIM_TimeBaseStructure);

    // PWM channels configuration (All identical!)
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = motorMap[i]->timPolarity;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

    // Configure Output Compare for PWM
    motorMap[i]->ocInit(motorMap[i]->tim, &TIM_OCInitStructure);
    motorMap[i]->preloadConfig(motorMap[i]->tim, TIM_OCPreload_Enable);

    // Halt timer during debug halt.
    DBGMCU_APB2PeriphConfig(motorMap[i]->timDbgStop, ENABLE);
    //Enable the timer PWM outputs
    TIM_CtrlPWMOutputs(motorMap[i]->tim, ENABLE);
  }

  // Start the timers
  for (i = 0; i < NBR_OF_MOTORS; i++)
  {
    TIM_Cmd(motorMap[i]->tim, ENABLE);
  }

  isInit = true;
}

bool motorsTest(void)
{
 #ifndef BRUSHLESS_MOTORCONTROLLER
  int i;
  for (i = 0; i < sizeof(MOTORS) / sizeof(*MOTORS); i++)
  {
    motorsSetRatio(MOTORS[i], MOTORS_TEST_RATIO);
    vTaskDelay(M2T(MOTORS_TEST_ON_TIME_MS));
    motorsSetRatio(MOTORS[i], 0);
    vTaskDelay(M2T(MOTORS_TEST_DELAY_TIME_MS));
  }
 #endif
  return isInit;
}

#ifdef ENABLE_THRUST_BAT_COMPENSATED
// Ithrust is thrust mapped for 65536 <==> 60g
void motorsSetRatio(int id, uint16_t ithrust)
{
  ASSERT(id < NBR_OF_MOTORS);

  float thrust = ((float)ithrust / 65536.0f) * 60;
  float volts = -0.0006239f * thrust * thrust + 0.088f * thrust + 0.069f;
  float supply_voltage = pmGetBatteryVoltage();
  float percentage = volts / supply_voltage;
  percentage = percentage > 1.0f ? 1.0f : percentage;
  uint16_t ratio = percentage * UINT16_MAX;

  motorMap[id]->setCompare(motorMap[id]->tim, C_16_TO_BITS(ratio));
}
#else
void motorsSetRatio(int id, uint16_t ratio)
{
  ASSERT(id < NBR_OF_MOTORS);
  motorMap[id]->setCompare(motorMap[id]->tim, C_16_TO_BITS(ratio));
}
#endif

int motorsGetRatio(int id)
{
  ASSERT(id < NBR_OF_MOTORS);
  return C_BITS_TO_16(motorMap[id]->getCompare(motorMap[id]->tim));
}
