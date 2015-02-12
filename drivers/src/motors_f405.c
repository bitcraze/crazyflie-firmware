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

//FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"

// HW defines
#define MOTORS_TIM_M1_PERIF       RCC_APB1Periph_TIM2
#define MOTORS_TIM_M1             TIM2
#define MOTORS_TIM_M1_DBG         DBGMCU_TIM2_STOP
#define M1_TIM_SETCOMPARE         TIM_SetCompare2
#define M1_TIM_GETCAPTURE         TIM_GetCapture2

#define MOTORS_TIM_M2_PERIF       RCC_APB1Periph_TIM2
#define MOTORS_TIM_M2             TIM2
#define MOTORS_TIM_M2_DBG         DBGMCU_TIM2_STOP
#define M2_TIM_SETCOMPARE         TIM_SetCompare4
#define M2_TIM_GETCAPTURE         TIM_GetCapture4

#define MOTORS_TIM_M3_PERIF       RCC_APB1Periph_TIM2
#define MOTORS_TIM_M3             TIM2
#define MOTORS_TIM_M3_DBG         DBGMCU_TIM2_STOP
#define M3_TIM_SETCOMPARE         TIM_SetCompare1
#define M3_TIM_GETCAPTURE         TIM_GetCapture1

#define MOTORS_TIM_M4_PERIF       RCC_APB1Periph_TIM4
#define MOTORS_TIM_M4             TIM4
#define MOTORS_TIM_M4_DBG         DBGMCU_TIM4_STOP
#define M4_TIM_SETCOMPARE         TIM_SetCompare4
#define M4_TIM_GETCAPTURE         TIM_GetCapture4

#define MOTORS_GPIO_M1_PERIF         RCC_AHB1Periph_GPIOB
#define MOTORS_GPIO_M1_PORT          GPIOA
#define MOTORS_GPIO_M1_PIN           GPIO_Pin_1 // TIM2_CH2
#define MOTORS_GPIO_AF_M1_PIN        GPIO_PinSource1
#define MOTORS_GPIO_AF_M1            GPIO_AF_TIM2

#define MOTORS_GPIO_M2_PERIF         RCC_AHB1Periph_GPIOB
#define MOTORS_GPIO_M2_PORT          GPIOB
#define MOTORS_GPIO_M2_PIN           GPIO_Pin_11 // TIM2_CH4
#define MOTORS_GPIO_AF_M2_PIN        GPIO_PinSource11
#define MOTORS_GPIO_AF_M2            GPIO_AF_TIM2

#define MOTORS_GPIO_M3_PERIF         RCC_AHB1Periph_GPIOA
#define MOTORS_GPIO_M3_PORT          GPIOA
#define MOTORS_GPIO_M3_PIN           GPIO_Pin_15 // TIM2_CH1
#define MOTORS_GPIO_AF_M3_PIN        GPIO_PinSource15
#define MOTORS_GPIO_AF_M3            GPIO_AF_TIM2

#define MOTORS_GPIO_M4_PERIF         RCC_AHB1Periph_GPIOB
#define MOTORS_GPIO_M4_PORT          GPIOB
#define MOTORS_GPIO_M4_PIN           GPIO_Pin_9 // TIM4_CH4
#define MOTORS_GPIO_AF_M4_PIN        GPIO_PinSource9
#define MOTORS_GPIO_AF_M4            GPIO_AF_TIM4

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
  int tempCR1_M1_2_3;
  int tempCR1_M4;

  if (isInit)
    return;

  //Init structures
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  //Clock the gpio and the timers
  RCC_AHB1PeriphClockCmd(MOTORS_GPIO_M1_PERIF | MOTORS_GPIO_M2_PERIF |
                         MOTORS_GPIO_M3_PERIF | MOTORS_GPIO_M4_PERIF, ENABLE);
  RCC_APB1PeriphClockCmd(MOTORS_TIM_M1_PERIF | MOTORS_TIM_M2_PERIF |
                         MOTORS_TIM_M3_PERIF | MOTORS_TIM_M4_PERIF, ENABLE);

  // Configure the GPIO for the timer output
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_Pin = MOTORS_GPIO_M1_PIN;
  GPIO_Init(MOTORS_GPIO_M1_PORT, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = MOTORS_GPIO_M2_PIN;
  GPIO_Init(MOTORS_GPIO_M2_PORT, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = MOTORS_GPIO_M3_PIN;
  GPIO_Init(MOTORS_GPIO_M3_PORT, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = MOTORS_GPIO_M4_PIN;
  GPIO_Init(MOTORS_GPIO_M4_PORT, &GPIO_InitStructure);

  //Map timers to alternate functions
  GPIO_PinAFConfig(MOTORS_GPIO_M1_PORT, MOTORS_GPIO_AF_M1_PIN, MOTORS_GPIO_AF_M1);
  GPIO_PinAFConfig(MOTORS_GPIO_M2_PORT, MOTORS_GPIO_AF_M2_PIN, MOTORS_GPIO_AF_M2);
  GPIO_PinAFConfig(MOTORS_GPIO_M3_PORT, MOTORS_GPIO_AF_M3_PIN, MOTORS_GPIO_AF_M3);
  GPIO_PinAFConfig(MOTORS_GPIO_M4_PORT, MOTORS_GPIO_AF_M4_PIN, MOTORS_GPIO_AF_M4);

  //Timer configuration
  TIM_TimeBaseStructure.TIM_Period = MOTORS_PWM_PERIOD;
  TIM_TimeBaseStructure.TIM_Prescaler = MOTORS_PWM_PRESCALE;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(MOTORS_TIM_M1, &TIM_TimeBaseStructure);
  TIM_TimeBaseInit(MOTORS_TIM_M2, &TIM_TimeBaseStructure);
  TIM_TimeBaseInit(MOTORS_TIM_M3, &TIM_TimeBaseStructure);
  TIM_TimeBaseInit(MOTORS_TIM_M4, &TIM_TimeBaseStructure);

  // PWM channels configuration (All identical!)
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
#ifdef BRUSHLESS_MOTORCONTROLLER
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
#else
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
#endif
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

  //M1:TIM2_CH2
  TIM_OC2Init(MOTORS_TIM_M1, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(MOTORS_TIM_M1, TIM_OCPreload_Enable);
  //M2:TIM2_CH4
  TIM_OC4Init(MOTORS_TIM_M2, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(MOTORS_TIM_M2, TIM_OCPreload_Enable);
  //M3:TIM2_CH1
  TIM_OC1Init(MOTORS_TIM_M3, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(MOTORS_TIM_M3, TIM_OCPreload_Enable);
  //M4:TIM4_CH4
  TIM_OC4Init(MOTORS_TIM_M4, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(MOTORS_TIM_M4, TIM_OCPreload_Enable);

  // Try to sync counters...
  tempCR1_M1_2_3 = MOTORS_TIM_M1->CR1 | TIM_CR1_CEN;
  tempCR1_M4 = MOTORS_TIM_M4->CR1 | TIM_CR1_CEN;
  //Enable the timer
  portDISABLE_INTERRUPTS();
  MOTORS_TIM_M1->CR1 = tempCR1_M1_2_3;
  MOTORS_TIM_M4->CR1 = tempCR1_M4;
  portENABLE_INTERRUPTS();

  //Enable the timer PWM outputs
  TIM_CtrlPWMOutputs(MOTORS_TIM_M1, ENABLE);
  TIM_CtrlPWMOutputs(MOTORS_TIM_M2, ENABLE);
  TIM_CtrlPWMOutputs(MOTORS_TIM_M3, ENABLE);
  TIM_CtrlPWMOutputs(MOTORS_TIM_M4, ENABLE);
  // Halt timer during debug halt.
  DBGMCU_APB2PeriphConfig(MOTORS_TIM_M1_DBG, ENABLE);
  DBGMCU_APB2PeriphConfig(MOTORS_TIM_M2_DBG, ENABLE);
  DBGMCU_APB2PeriphConfig(MOTORS_TIM_M3_DBG, ENABLE);
  DBGMCU_APB2PeriphConfig(MOTORS_TIM_M4_DBG, ENABLE);

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


void motorsSetRatio(int id, uint16_t ratio)
{
  switch(id)
  {
    case MOTOR_M1:
      M1_TIM_SETCOMPARE(MOTORS_TIM_M1, C_16_TO_BITS(ratio));
      break;
    case MOTOR_M2:
      M2_TIM_SETCOMPARE(MOTORS_TIM_M2, C_16_TO_BITS(ratio));
      break;
    case MOTOR_M3:
      M3_TIM_SETCOMPARE(MOTORS_TIM_M3, C_16_TO_BITS(ratio));
      break;
    case MOTOR_M4:
      M4_TIM_SETCOMPARE(MOTORS_TIM_M4, C_16_TO_BITS(ratio));
      break;
  }
}

int motorsGetRatio(int id)
{
  switch(id)
  {
    case MOTOR_M1:
      return C_BITS_TO_16(M1_TIM_GETCAPTURE(MOTORS_TIM_M1));
    case MOTOR_M2:
      return C_BITS_TO_16(M2_TIM_GETCAPTURE(MOTORS_TIM_M2));
    case MOTOR_M3:
      return C_BITS_TO_16(M3_TIM_GETCAPTURE(MOTORS_TIM_M3));
    case MOTOR_M4:
      return C_BITS_TO_16(M4_TIM_GETCAPTURE(MOTORS_TIM_M4));
  }

  return -1;
}

#ifdef MOTOR_RAMPUP_TEST
// FreeRTOS Task to test the Motors driver with a rampup of each motor alone.
void motorsTestTask(void* params)
{
  int step=0;
  float rampup = 0.01;

  motorsSetupMinMaxPos();
  motorsSetRatio(MOTOR_M4, 1*(1<<16) * 0.0);
  motorsSetRatio(MOTOR_M3, 1*(1<<16) * 0.0);
  motorsSetRatio(MOTOR_M2, 1*(1<<16) * 0.0);
  motorsSetRatio(MOTOR_M1, 1*(1<<16) * 0.0);
  vTaskDelay(M2T(1000));

  while(1)
  {
    vTaskDelay(M2T(100));

    motorsSetRatio(MOTOR_M4, 1*(1<<16) * rampup);
    motorsSetRatio(MOTOR_M3, 1*(1<<16) * rampup);
    motorsSetRatio(MOTOR_M2, 1*(1<<16) * rampup);
    motorsSetRatio(MOTOR_M1, 1*(1<<16) * rampup);

    rampup += 0.001;
    if (rampup >= 0.1)
    {
      if(++step>3) step=0;
      rampup = 0.01;
    }
  }
}
#else
// FreeRTOS Task to test the Motors driver
void motorsTestTask(void* params)
{
  static const int sequence[] = {0.1*(1<<16), 0.15*(1<<16), 0.2*(1<<16), 0.25*(1<<16)};
  int step=0;

  //Wait 3 seconds before starting the motors
  vTaskDelay(M2T(3000));

  while(1)
  {
    motorsSetRatio(MOTOR_M4, sequence[step%4]);
    motorsSetRatio(MOTOR_M3, sequence[(step+1)%4]);
    motorsSetRatio(MOTOR_M2, sequence[(step+2)%4]);
    motorsSetRatio(MOTOR_M1, sequence[(step+3)%4]);

    if(++step>3) step=0;

    vTaskDelay(M2T(1000));
  }
}
#endif
