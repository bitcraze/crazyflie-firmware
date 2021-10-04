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
#define DEBUG_MODULE "MTR-DRV"

#include <stdbool.h>

/* ST includes */
#include "stm32fxxx.h"

#include "motors.h"
#include "pm.h"
#include "debug.h"

//FreeRTOS includes
#include "task.h"

//Logging includes
#include "log.h"

static uint16_t motorsBLConvBitsTo16(uint16_t bits);
static uint16_t motorsBLConv16ToBits(uint16_t bits);
static uint16_t motorsConvBitsTo16(uint16_t bits);
static uint16_t motorsConv16ToBits(uint16_t bits);

uint32_t motor_ratios[] = {0, 0, 0, 0};

void motorsPlayTone(uint16_t frequency, uint16_t duration_msec);
void motorsPlayMelody(uint16_t *notes);
void motorsBeep(int id, bool enable, uint16_t frequency, uint16_t ratio);

#include "motors_def_cf2.c"

const MotorPerifDef** motorMap;  /* Current map configuration */

const uint32_t MOTORS[] = { MOTOR_M1, MOTOR_M2, MOTOR_M3, MOTOR_M4 };

const uint16_t testsound[NBR_OF_MOTORS] = {A4, A5, F5, D5 };

const MotorHealthTestDef brushedMotorHealthTestSettings = {
  /* onPeriodMsec = */ 50,
  /* offPeriodMsec = */ 950,
  /* varianceMeasurementStartMsec = */ 0,
  /* onPeriodPWMRatio = */ 0xFFFF,
};

const MotorHealthTestDef brushlessMotorHealthTestSettings = {
  /* onPeriodMsec = */ 2000,
  /* offPeriodMsec = */ 1000,
  /* varianceMeasurementStartMsec = */ 1000,
  /* onPeriodPWMRatio = */ 0 /* user must set health.propTestPWMRatio explicitly */
};

const MotorHealthTestDef unknownMotorHealthTestSettings = {
  /* onPeriodMsec = */ 0,
  /* offPeriodMseec = */ 0,
  /* varianceMeasurementStartMsec = */ 0,
  /* onPeriodPWMRatio = */ 0
};

static bool isInit = false;

/* Private functions */

static uint16_t motorsBLConvBitsTo16(uint16_t bits)
{
  return (0xFFFF * (bits - MOTORS_BL_PWM_CNT_FOR_HIGH) / MOTORS_BL_PWM_CNT_FOR_HIGH);
}

static uint16_t motorsBLConv16ToBits(uint16_t bits)
{
  return (MOTORS_BL_PWM_CNT_FOR_HIGH + ((bits * MOTORS_BL_PWM_CNT_FOR_HIGH) / 0xFFFF));
}

static uint16_t motorsConvBitsTo16(uint16_t bits)
{
  return ((bits) << (16 - MOTORS_PWM_BITS));
}

static uint16_t motorsConv16ToBits(uint16_t bits)
{
  return ((bits) >> (16 - MOTORS_PWM_BITS) & ((1 << MOTORS_PWM_BITS) - 1));
}

// We have data that maps PWM to thrust at different supply voltage levels.
// However, it is not the PWM that drives the motors but the voltage and
// amps (= power). With the PWM it is possible to simulate different
// voltage levels. The assumption is that the voltage used will be an
// procentage of the supply voltage, we assume that 50% PWM will result in
// 50% voltage.
//
//  Thrust (g)    Supply Voltage    PWM (%)     Voltage needed
//  0.0           4.01              0           0
//  1.6           3.98              6.25        0.24875
//  4.8           3.95              12.25       0.49375
//  7.9           3.82              18.75       0.735
//  10.9          3.88              25          0.97
//  13.9          3.84              31.25       1.2
//  17.3          3.80              37.5        1.425
//  21.0          3.76              43.25       1.6262
//  24.4          3.71              50          1.855
//  28.6          3.67              56.25       2.06438
//  32.8          3.65              62.5        2.28125
//  37.3          3.62              68.75       2.48875
//  41.7          3.56              75          2.67
//  46.0          3.48              81.25       2.8275
//  51.9          3.40              87.5        2.975
//  57.9          3.30              93.75       3.09375
//
// To get Voltage needed from wanted thrust we can get the quadratic
// polyfit coefficients using GNU octave:
//
// thrust = [0.0 1.6 4.8 7.9 10.9 13.9 17.3 21.0 ...
//           24.4 28.6 32.8 37.3 41.7 46.0 51.9 57.9]
//
// volts  = [0.0 0.24875 0.49375 0.735 0.97 1.2 1.425 1.6262 1.855 ...
//           2.064375 2.28125 2.48875 2.67 2.8275 2.975 3.09375]
//
// p = polyfit(thrust, volts, 2)
//
// => p = -0.00062390   0.08835522   0.06865956
//
// We will not use the contant term, since we want zero thrust to equal
// zero PWM.
//
// And to get the PWM as a percentage we would need to divide the
// Voltage needed with the Supply voltage.
static uint16_t motorsCompensateBatteryVoltage(uint16_t ithrust)
{
  float supply_voltage = pmGetBatteryVoltage();
  /*
   * A LiPo battery is supposed to be 4.2V charged, 3.7V mid-charge and 3V
   * discharged.
   * 
   * A suiteble sanity check for disabiling the voltage compensation would be
   * under 2V. That would suggest a damaged battery. This protects against
   * rushing the motors on bugs and invalid voltage levels.
   */
  if (supply_voltage < 2.0f)
  {
    return ithrust;
  }

  float thrust = ((float) ithrust / 65536.0f) * 60;
  float volts = -0.0006239f * thrust * thrust + 0.088f * thrust;
  float percentage = volts / supply_voltage;
  percentage = percentage > 1.0f ? 1.0f : percentage;
  return percentage * UINT16_MAX;
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
    // First to init will configure it
    return;
  }

  motorMap = motorMapSelect;

  DEBUG_PRINT("Using %s motor driver\n", motorMap[0]->drvType == BRUSHED ? "brushed" : "brushless");

  for (i = 0; i < NBR_OF_MOTORS; i++)
  {
    //Clock the gpio and the timers
    MOTORS_RCC_GPIO_CMD(motorMap[i]->gpioPerif, ENABLE);
    MOTORS_RCC_GPIO_CMD(motorMap[i]->gpioPowerswitchPerif, ENABLE);
    MOTORS_RCC_TIM_CMD(motorMap[i]->timPerif, ENABLE);

    // If there is a power switch, as on Bolt, enable power to ESC by
    // switching on mosfet.
    if (motorMap[i]->gpioPowerswitchPin != 0)
    {
      GPIO_StructInit(&GPIO_InitStructure);
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
      GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
      GPIO_InitStructure.GPIO_Pin = motorMap[i]->gpioPowerswitchPin;
      GPIO_Init(motorMap[i]->gpioPowerswitchPort, &GPIO_InitStructure);
      GPIO_WriteBit(motorMap[i]->gpioPowerswitchPort, motorMap[i]->gpioPowerswitchPin, 1);
    }

    // Configure the GPIO for the timer output
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Mode = MOTORS_GPIO_MODE;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = motorMap[i]->gpioOType;
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
  }

  // Start the timers
  for (i = 0; i < NBR_OF_MOTORS; i++)
  {
    TIM_Cmd(motorMap[i]->tim, ENABLE);
  }

  isInit = true;

  // Output zero power
  motorsSetRatio(MOTOR_M1, 0);
  motorsSetRatio(MOTOR_M2, 0);
  motorsSetRatio(MOTOR_M3, 0);
  motorsSetRatio(MOTOR_M4, 0);
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

    //Map timers to alternate functions
    GPIO_PinAFConfig(motorMap[i]->gpioPort, motorMap[i]->gpioPinSource, 0x00);

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

// Ithrust is thrust mapped for 65536 <==> 60 grams
void motorsSetRatio(uint32_t id, uint16_t ithrust)
{
  if (isInit) {
    uint16_t ratio;

    ASSERT(id < NBR_OF_MOTORS);

    ratio = ithrust;

#ifdef ENABLE_THRUST_BAT_COMPENSATED
    if (motorMap[id]->drvType == BRUSHED)
    {
      // To make sure we provide the correct PWM given current supply voltage
      // from the battery, we do calculations based on measurements of PWM,
      // voltage and thrust. See comment at function definition for details.
      ratio = motorsCompensateBatteryVoltage(ithrust);
      motor_ratios[id] = ratio;
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

const MotorHealthTestDef* motorsGetHealthTestSettings(uint32_t id)
{
  if (id >= NBR_OF_MOTORS)
  {
    return &unknownMotorHealthTestSettings;
  }
  else if (motorMap[id]->drvType == BRUSHLESS)
  {
    return &brushlessMotorHealthTestSettings;
  }
  else if (motorMap[id]->drvType == BRUSHED)
  {
    return &brushedMotorHealthTestSettings;
  }
  else
  {
    return &unknownMotorHealthTestSettings;
  }
}

/**
 * Logging variables of the motors PWM output
 */
LOG_GROUP_START(pwm)
/**
 * @brief Current motor 1 PWM output
 */ 
LOG_ADD(LOG_UINT32, m1_pwm, &motor_ratios[0])
/**
 * @brief Current motor 2 PWM output
 */ 
LOG_ADD(LOG_UINT32, m2_pwm, &motor_ratios[1])
/**
 * @brief Current motor 3 PWM output
 */ 
LOG_ADD(LOG_UINT32, m3_pwm, &motor_ratios[2])
/**
 * @brief Current motor 4 PWM output
 */ 
LOG_ADD(LOG_UINT32, m4_pwm, &motor_ratios[3])
LOG_GROUP_STOP(pwm)
