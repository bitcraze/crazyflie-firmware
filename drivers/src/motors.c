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
#include "task.h"

static uint16_t motorsBLConvBitsTo16(uint16_t bits);
static uint16_t motorsBLConv16ToBits(uint16_t bits);
static uint16_t motorsConvBitsTo16(uint16_t bits);
static uint16_t motorsConv16ToBits(uint16_t bits);
void motorsPlayTone(uint16_t frequency, uint16_t duration_msec);
void motorsPlayMelody(uint16_t *notes);
void motorsBeep(int id, bool enable, uint16_t frequency, uint16_t ratio);

#ifdef PLATFORM_CF1
#include "motors_def_cf1.c"
#else
#include "motors_def_cf2.c"
#endif

const MotorPerifDef** motorMap;  /* Current map configuration */

const uint32_t MOTORS[] = { MOTOR_M1, MOTOR_M2, MOTOR_M3, MOTOR_M4 };

static const uint16_t testsound[NBR_OF_MOTORS] = {A4, A5, F5, D5 };

static bool isInit = false;

/* Private functions */

static uint16_t motorsBLConvBitsTo16(uint16_t bits)
{
  return (0xFFFF * (bits - MOTORS_BL_PWM_CNT_FOR_1MS) / MOTORS_BL_PWM_CNT_FOR_1MS);
}

static uint16_t motorsBLConv16ToBits(uint16_t bits)
{
  return (MOTORS_BL_PWM_CNT_FOR_1MS + ((bits * MOTORS_BL_PWM_CNT_FOR_1MS) / 0xFFFF));
}

static uint16_t motorsConvBitsTo16(uint16_t bits)
{
  return ((bits) << (16 - MOTORS_PWM_BITS));
}

static uint16_t motorsConv16ToBits(uint16_t bits)
{
  return ((bits) >> (16 - MOTORS_PWM_BITS) & ((1 << MOTORS_PWM_BITS) - 1));
}

/* Public functions */

//Initialization. Will set all motors ratio to 0%
void motorsInit(const MotorPerifDef** motorMapSelect)
{
  int i;
  //Init structures
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  if (isInit)
  {
    motorsDeInit(motorMap);
  }

  motorMap = motorMapSelect;

  for (i = 0; i < NBR_OF_MOTORS; i++)
  {
    //Clock the gpio and the timers
    MOTORS_RCC_GPIO_CMD(motorMap[i]->gpioPerif, ENABLE);
    MOTORS_RCC_TIM_CMD(motorMap[i]->timPerif, ENABLE);

    // Configure the GPIO for the timer output
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Mode = MOTORS_GPIO_MODE;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
#ifdef PLATFORM_CF2
    GPIO_InitStructure.GPIO_OType = motorMap[i]->gpioOType;
#endif
    GPIO_InitStructure.GPIO_Pin = motorMap[i]->gpioPin;
    GPIO_Init(motorMap[i]->gpioPort, &GPIO_InitStructure);

    //Map timers to alternate functions
    MOTORS_GPIO_AF_CFG(motorMap[i]->gpioPort, motorMap[i]->gpioPinSource, motorMap[i]->gpioAF);

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

    MOTORS_TIM_DBG_CFG(motorMap[i]->timDbgStop, ENABLE);
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

void motorsDeInit(const MotorPerifDef** motorMapSelect)
{
  int i;
  GPIO_InitTypeDef GPIO_InitStructure;

  for (i = 0; i < NBR_OF_MOTORS; i++)
  {
    // Configure default
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = motorMap[i]->gpioPin;
    GPIO_Init(motorMap[i]->gpioPort, &GPIO_InitStructure);

#ifdef PLATFORM_CF1
    //Map timers to alternate functions
    GPIO_PinRemapConfig(motorMap[i]->gpioAF , DISABLE);
#else
    //Map timers to alternate functions
    GPIO_PinAFConfig(motorMap[i]->gpioPort, motorMap[i]->gpioPinSource, 0x00);
#endif

    //Deinit timer
    TIM_DeInit(motorMap[i]->tim);
  }
}

bool motorsTest(void)
{
  int i;

  for (i = 0; i < sizeof(MOTORS) / sizeof(*MOTORS); i++)
  {
    if (motorMap[i]->drvType == BRUSHED)
    {
#ifdef ACTIVATE_STARTUP_SOUND
      motorsBeep(MOTORS[i], true, testsound[i], (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / A4)/ 20);
      vTaskDelay(M2T(MOTORS_TEST_ON_TIME_MS));
      motorsBeep(MOTORS[i], false, 0, 0);
      vTaskDelay(M2T(MOTORS_TEST_DELAY_TIME_MS));
#else
      motorsSetRatio(MOTORS[i], MOTORS_TEST_RATIO);
      vTaskDelay(M2T(MOTORS_TEST_ON_TIME_MS));
      motorsSetRatio(MOTORS[i], 0);
      vTaskDelay(M2T(MOTORS_TEST_DELAY_TIME_MS));
#endif
    }
  }

  return isInit;
}

// Ithrust is thrust mapped for 65536 <==> 60g
void motorsSetRatio(uint32_t id, uint16_t ithrust)
{
  uint16_t ratio;

  ASSERT(id < NBR_OF_MOTORS);

  ratio = ithrust;

#ifdef ENABLE_THRUST_BAT_COMPENSATED
  if (motorMap[id]->drvType == BRUSHED)
  {
    float thrust = ((float)ithrust / 65536.0f) * 60;
    float volts = -0.0006239 * thrust * thrust + 0.088 * thrust;
    float supply_voltage = pmGetBatteryVoltage();
    float percentage = volts / supply_voltage;
    percentage = percentage > 1.0 ? 1.0 : percentage;
    ratio = percentage * UINT16_MAX;
  }
#endif
  if (motorMap[id]->drvType == BRUSHLESS)
  {
    motorMap[id]->setCompare(motorMap[id]->tim, motorsBLConv16ToBits(ratio));
  }
  else
  {
    motorMap[id]->setCompare(motorMap[id]->tim, motorsConv16ToBits(ratio));
  }
}

int motorsGetRatio(uint32_t id)
{
  int ratio;

  ASSERT(id < NBR_OF_MOTORS);
  if (motorMap[id]->drvType == BRUSHLESS)
  {
    ratio = motorsBLConvBitsTo16(motorMap[id]->getCompare(motorMap[id]->tim));
  }
  else
  {
    ratio = motorsConvBitsTo16(motorMap[id]->getCompare(motorMap[id]->tim));
  }

  return ratio;
}

/* Set PWM frequency for motor controller
 * This function will set all motors into a "beep"-mode,
 * each of the motor will turned on with a given ratio and frequency.
 * The higher the ratio the higher the given power to the motors.
 * ATTENTION: To much ratio can push your crazyflie into the air and hurt you!
 * Example:
 *     motorsBeep(true, 1000, (uint16_t)(72000000L / frequency)/ 20);
 *     motorsBeep(false, 0, 0); *
 * */
void motorsBeep(int id, bool enable, uint16_t frequency, uint16_t ratio)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

  ASSERT(id < NBR_OF_MOTORS);

  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

  if (enable)
  {
    TIM_TimeBaseStructure.TIM_Prescaler = (5 - 1);
    TIM_TimeBaseStructure.TIM_Period = (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / frequency);
  }
  else
  {
    TIM_TimeBaseStructure.TIM_Period = motorMap[id]->timPeriod;
    TIM_TimeBaseStructure.TIM_Prescaler = motorMap[id]->timPrescaler;
  }

  // Timer configuration
  TIM_TimeBaseInit(motorMap[id]->tim, &TIM_TimeBaseStructure);
  motorMap[id]->setCompare(motorMap[id]->tim, ratio);
}


// Play a tone with a given frequency and a specific duration in milliseconds (ms)
void motorsPlayTone(uint16_t frequency, uint16_t duration_msec)
{
  motorsBeep(MOTOR_M1, true, frequency, (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / frequency)/ 20);
  motorsBeep(MOTOR_M2, true, frequency, (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / frequency)/ 20);
  motorsBeep(MOTOR_M3, true, frequency, (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / frequency)/ 20);
  motorsBeep(MOTOR_M4, true, frequency, (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / frequency)/ 20);
  vTaskDelay(M2T(duration_msec));
  motorsBeep(MOTOR_M1, false, frequency, 0);
  motorsBeep(MOTOR_M2, false, frequency, 0);
  motorsBeep(MOTOR_M3, false, frequency, 0);
  motorsBeep(MOTOR_M4, false, frequency, 0);
}

// Plays a melody from a note array
void motorsPlayMelody(uint16_t *notes)
{
  int i = 0;
  uint16_t note;      // Note in hz
  uint16_t duration;  // Duration in ms

  do
  {
    note = notes[i++];
    duration = notes[i++];
    motorsPlayTone(note, duration);
  } while (duration != 0);
}
