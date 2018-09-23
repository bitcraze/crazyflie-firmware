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
 * Motors.h - Motor driver header file
 *
 * The motors PWM ratio is a value on 16 bits (from 0 to 0xFFFF)
 * the functions of the driver will make the conversions to the actual PWM
 * precision (ie. if the precision is 8bits 0xFFFF and 0xFF00 are equivalents).
 *
 * The precision is set in number of bits by the define MOTORS_PWM_BITS
 * The timer prescaler is set by MOTORS_PWM_PRESCALE
 */
#ifndef __MOTORS_H__
#define __MOTORS_H__

#include <stdint.h>
#include <stdbool.h>
#include "config.h"
/* ST includes */
#include "stm32fxxx.h"

/******** Defines ********/

// The following defines gives a PWM of 8 bits at ~328KHz for a sysclock of 168MHz
// CF2 PWM ripple is filtered better at 328kHz. At 168kHz the NCP702 regulator is affected.
#define TIM_CLOCK_HZ 84000000
#define MOTORS_PWM_BITS           8
#define MOTORS_PWM_PERIOD         ((1<<MOTORS_PWM_BITS) - 1)
#define MOTORS_PWM_PRESCALE       0
#define MOTORS_TIM_BEEP_CLK_FREQ  (84000000L / 5)
#define MOTORS_POLARITY           TIM_OCPolarity_High

// Abstraction of ST lib functions
#define MOTORS_GPIO_MODE          GPIO_Mode_AF
#define MOTORS_RCC_GPIO_CMD       RCC_AHB1PeriphClockCmd
#define MOTORS_RCC_TIM_CMD        RCC_APB1PeriphClockCmd
#define MOTORS_TIM_DBG_CFG        DBGMCU_APB2PeriphConfig
#define MOTORS_GPIO_AF_CFG(a,b,c) GPIO_PinAFConfig(a,b,c)

// Compensate thrust depending on battery voltage so it will produce about the same
// amount of thrust independent of the battery voltage. Based on thrust measurement.
#define ENABLE_THRUST_BAT_COMPENSATED

//#define ENABLE_ONESHOT125

#ifdef ENABLE_ONESHOT125
/**
 * *VARNING* Make sure the brushless driver is configured correctly as on the Crazyflie with normal
 * brushed motors connected they can turn on at full speed when it is powered on!
 *
 * Generates a PWM wave (50 - 400 Hz update rate with 1-2 ms high pulse) using the timer. That way we can use the same
 * base as for the regular PWM driver. This means it will be a PWM with a period of the update rate configured to be high
 * only in the 1-2 ms range.
 */
  #define BLMC_PERIOD 0.0005   // 0.5ms = 2000Hz
  #define MOTORS_BL_PWM_PRESCALE_RAW   (uint32_t)((TIM_CLOCK_HZ/0xFFFF) * BLMC_PERIOD + 1) // +1 is to not end up above 0xFFFF in the end
  #define MOTORS_BL_PWM_CNT_FOR_PERIOD (uint32_t)(TIM_CLOCK_HZ * BLMC_PERIOD / MOTORS_BL_PWM_PRESCALE_RAW)
  #define MOTORS_BL_PWM_CNT_FOR_HIGH   (uint32_t)(TIM_CLOCK_HZ * 0.000125 / MOTORS_BL_PWM_PRESCALE_RAW)
  #define MOTORS_BL_PWM_PERIOD         MOTORS_BL_PWM_CNT_FOR_PERIOD
  #define MOTORS_BL_PWM_PRESCALE       (uint16_t)(MOTORS_BL_PWM_PRESCALE_RAW - 1)
  #define MOTORS_BL_POLARITY           TIM_OCPolarity_Low
#else
/**
 * *VARNING* Make sure the brushless driver is configured correctly as on the Crazyflie with normal
 * brushed motors connected they can turn on at full speed when it is powered on!
 *
 * Generates a PWM wave (50 - 400 Hz update rate with 1-2 ms high pulse) using the timer. That way we can use the same
 * base as for the regular PWM driver. This means it will be a PWM with a period of the update rate configured to be high
 * only in the 1-2 ms range.
 */
  #define BLMC_PERIOD 0.0025   // 2.5ms = 400Hz
  #define MOTORS_BL_PWM_PRESCALE_RAW   (uint32_t)((TIM_CLOCK_HZ/0xFFFF) * BLMC_PERIOD + 1) // +1 is to not end up above 0xFFFF in the end
  #define MOTORS_BL_PWM_CNT_FOR_PERIOD (uint32_t)(TIM_CLOCK_HZ * BLMC_PERIOD / MOTORS_BL_PWM_PRESCALE_RAW)
  #define MOTORS_BL_PWM_CNT_FOR_HIGH    (uint32_t)(TIM_CLOCK_HZ * 0.001 / MOTORS_BL_PWM_PRESCALE_RAW)
  #define MOTORS_BL_PWM_PERIOD         MOTORS_BL_PWM_CNT_FOR_PERIOD
  #define MOTORS_BL_PWM_PRESCALE       (uint16_t)(MOTORS_BL_PWM_PRESCALE_RAW - 1)
  #define MOTORS_BL_POLARITY           TIM_OCPolarity_Low
#endif

#define NBR_OF_MOTORS 4
// Motors IDs define
#define MOTOR_M1  0
#define MOTOR_M2  1
#define MOTOR_M3  2
#define MOTOR_M4  3

// Test defines
#define MOTORS_TEST_RATIO         (uint16_t)(0.2*(1<<16))
#define MOTORS_TEST_ON_TIME_MS    50
#define MOTORS_TEST_DELAY_TIME_MS 150

// Sound defines
#define C4    262
#define DES4  277
#define D4    294
#define ES4   311
#define E4    330
#define F4    349
#define GES4  370
#define G4    392
#define AS4   415
#define A4    440
#define B4    466
#define H4    493
#define C5    523
#define DES5  554
#define D5    587
#define ES5   622
#define E5    659
#define F5    698
#define GES5  740
#define G5    783
#define AS5   830
#define A5    880
#define B5    932
#define H5    987
#define C6    1046
#define DES6  1108
#define D6    1174
#define ES6   1244
#define E6    1318
#define F6    1396
#define GES6  1479
#define G6    1567
#define AS6   1661
#define A6    1760
#define B6    1864
#define H6    1975
#define C7    2093
#define DES7  2217
#define D7    2349
#define ES7   2489
#define E7    2637
#define F7    2793
#define GES7  2959
#define G7    3135
#define AS7   3322
#define A7    3520
#define H7    3729
#define B7    3951

// Sound duration defines
#define EIGHTS 125
#define QUAD 250
#define HALF 500
#define FULL 1000
#define STOP 0

typedef enum
{
  BRUSHED,
  BRUSHLESS
} motorsDrvType;

typedef struct
{
  motorsDrvType drvType;
  uint32_t      gpioPerif;
  GPIO_TypeDef* gpioPort;
  uint32_t      gpioPin;
  uint32_t      gpioPinSource;
  uint32_t      gpioOType;
  uint32_t      gpioAF;
  uint32_t      timPerif;
  TIM_TypeDef*  tim;
  uint16_t      timPolarity;
  uint32_t      timDbgStop;
  uint32_t      timPeriod;
  uint16_t      timPrescaler;
  /* Function pointers */
  void (*setCompare)(TIM_TypeDef* TIMx, uint32_t Compare);
  uint32_t (*getCompare)(TIM_TypeDef* TIMx);
  void (*ocInit)(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
  void (*preloadConfig)(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
} MotorPerifDef;

/**
 * Motor mapping configurations
 */
extern const MotorPerifDef* motorMapDefaultBrushed[NBR_OF_MOTORS];
extern const MotorPerifDef* motorMapDefaltConBrushless[NBR_OF_MOTORS];
extern const MotorPerifDef* motorMapBigQuadDeck[NBR_OF_MOTORS];
extern const MotorPerifDef* motorMapRZRBrushless[NBR_OF_MOTORS];

/**
 * Test sound tones
 */
extern const uint16_t testsound[NBR_OF_MOTORS];
/*** Public interface ***/

/**
 * Initialisation. Will set all motors ratio to 0%
 */
void motorsInit(const MotorPerifDef** motorMapSelect);

/**
 * DeInitialisation. Reset to default
 */
void motorsDeInit(const MotorPerifDef** motorMapSelect);

/**
 * Test of the motor modules. The test will spin each motor very short in
 * the sequence M1 to M4.
 */
bool motorsTest(void);

/**
 * Set the PWM ratio of the motor 'id'
 */
void motorsSetRatio(uint32_t id, uint16_t ratio);

/**
 * Get the PWM ratio of the motor 'id'. Return -1 if wrong ID.
 */
int motorsGetRatio(uint32_t id);

/**
 * FreeRTOS Task to test the Motors driver
 */
void motorsTestTask(void* params);

/* Set PWM frequency for motor controller
 * This function will set all motors into a "beep"-mode,
 * each of the motor will turned on with a given ratio and frequency.
 * The higher the ratio the higher the given power to the motors.
 * ATTENTION: To much ratio can push your crazyflie into the air and hurt you!
 * Example:
 *     motorsBeep(true, 1000, (uint16_t)(72000000L / frequency)/ 20);
 *     motorsBeep(false, 0, 0); *
 * */
void motorsBeep(int id, bool enable, uint16_t frequency, uint16_t ratio);

#endif /* __MOTORS_H__ */

