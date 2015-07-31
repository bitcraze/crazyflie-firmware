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

/******** Defines ********/
/**
 * *VARNING* Flashing the brushless driver on the Crazyflie with normal brushed motors connected
 *  will turn it on at full speed when it is powered on!
 *
 * Generates a PWM wave (50 - 200 Hz update rate with 1-2 ms high pulse) using the timer. That way we can use the same
 * base as for the regular PWM driver. This means it will be a PWM with a period of the update rate configured to be high
 * only in the 1-2 ms range.
 * The BLMC input signal are meant to be connected to the Crazyflie round motor solder pad (open-drain output). A resistor
 * around 470 ohm needs to pull the signal high to the voltage level of the BLMC (normally 5V).
 */
#ifdef BRUSHLESS_MOTORCONTROLLER //Crazyflie2
  #define BLMC_PERIOD 0.0025   // 2.5ms = 400Hz
  #define TIM_CLOCK_HZ 84000000
  #define MOTORS_PWM_PRESCALE_RAW   (uint32_t)((TIM_CLOCK_HZ/0xFFFF) * BLMC_PERIOD + 1) // +1 is to not end up above 0xFFFF in the end
  #define MOTORS_PWM_CNT_FOR_PERIOD (uint32_t)(TIM_CLOCK_HZ * BLMC_PERIOD / MOTORS_PWM_PRESCALE_RAW)
  #define MOTORS_PWM_CNT_FOR_1MS    (uint32_t)(TIM_CLOCK_HZ * 0.001 / MOTORS_PWM_PRESCALE_RAW)
  #define MOTORS_PWM_PERIOD         MOTORS_PWM_CNT_FOR_PERIOD
  #define MOTORS_PWM_BITS           11  // Only for compatibiliy
  #define MOTORS_PWM_PRESCALE       (uint16_t)(MOTORS_PWM_PRESCALE_RAW - 1)
  #define MOTORS_POLARITY           TIM_OCPolarity_Low
#else
  #ifdef PLATFORM_CF1
    // The following defines gives a PWM of 9 bits at ~140KHz for a sysclock of 72MHz
    #define MOTORS_PWM_BITS     9
    #define MOTORS_PWM_PERIOD   ((1<<MOTORS_PWM_BITS) - 1)
    #define MOTORS_PWM_PRESCALE 0
    #define MOTORS_POLARITY           TIM_OCPolarity_High
  #else
    // The following defines gives a PWM of 8 bits at ~328KHz for a sysclock of 168MHz
    // CF2 PWM ripple is filtered better at 328kHz. At 168kHz the NCP702 regulator is affected.
    #define MOTORS_PWM_BITS     8
    #define MOTORS_PWM_PERIOD   ((1<<MOTORS_PWM_BITS) - 1)
    #define MOTORS_PWM_PRESCALE 0
    #define MOTORS_POLARITY           TIM_OCPolarity_High
    // Compensate thrust depending on battery voltage so it will produce about the same
    // amount of thrust independent of the battery voltage. Based on thrust measurement.
    #define ENABLE_THRUST_BAT_COMPENSATED
  #endif
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
#define QUAD 250
#define HALF 500
#define FULL 1000
#define STOP 0

#ifdef BRUSHLESS_PROTO_DECK_MAPPING
// HW defines for prototype brushless deck
// PIN7-LEFT  -> PB5: TIM3_CH2, connect as M1
// PIN1-RIGHT -> PA2: TIM2_CH3, connect as M2
// PIN2-RIGHT -> PA3: TIM2_CH4, connect as M3
// PIN8-LEFT  -> PB4: TIM3_CH1, connect as M4

#define MOTORS_TIM_M1_PERIF       RCC_APB1Periph_TIM3 // TIM3_CH2
#define MOTORS_TIM_M1             TIM3
#define MOTORS_TIM_M1_DBG         DBGMCU_TIM3_STOP
#define M1_TIM_SETCOMPARE         TIM_SetCompare2
#define M1_TIM_GETCAPTURE         TIM_GetCapture2
#define M1_TIM_OC_INIT            TIM_OC2Init
#define M1_TIM_OC_PRE_CFG         TIM_OC2PreloadConfig

#define MOTORS_TIM_M2_PERIF       RCC_APB1Periph_TIM2 // TIM2_CH3
#define MOTORS_TIM_M2             TIM2
#define MOTORS_TIM_M2_DBG         DBGMCU_TIM2_STOP
#define M2_TIM_SETCOMPARE         TIM_SetCompare3
#define M2_TIM_GETCAPTURE         TIM_GetCapture3
#define M2_TIM_OC_INIT            TIM_OC3Init
#define M2_TIM_OC_PRE_CFG         TIM_OC3PreloadConfig

#define MOTORS_TIM_M3_PERIF       RCC_APB1Periph_TIM2 // TIM2_CH4
#define MOTORS_TIM_M3             TIM2
#define MOTORS_TIM_M3_DBG         DBGMCU_TIM2_STOP
#define M3_TIM_SETCOMPARE         TIM_SetCompare4
#define M3_TIM_GETCAPTURE         TIM_GetCapture4
#define M3_TIM_OC_INIT            TIM_OC4Init
#define M3_TIM_OC_PRE_CFG         TIM_OC4PreloadConfig

#define MOTORS_TIM_M4_PERIF       RCC_APB1Periph_TIM3 // TIM3_CH1
#define MOTORS_TIM_M4             TIM3
#define MOTORS_TIM_M4_DBG         DBGMCU_TIM3_STOP
#define M4_TIM_SETCOMPARE         TIM_SetCompare1
#define M4_TIM_GETCAPTURE         TIM_GetCapture1
#define M4_TIM_OC_INIT            TIM_OC1Init
#define M4_TIM_OC_PRE_CFG         TIM_OC1PreloadConfig

#define MOTORS_GPIO_M1_PERIF         RCC_AHB1Periph_GPIOB // PB5
#define MOTORS_GPIO_M1_PORT          GPIOB
#define MOTORS_GPIO_M1_PIN           GPIO_Pin_5
#define MOTORS_GPIO_AF_M1_PIN        GPIO_PinSource5
#define MOTORS_GPIO_AF_M1            GPIO_AF_TIM3

#define MOTORS_GPIO_M2_PERIF         RCC_AHB1Periph_GPIOA // PA2
#define MOTORS_GPIO_M2_PORT          GPIOA
#define MOTORS_GPIO_M2_PIN           GPIO_Pin_2
#define MOTORS_GPIO_AF_M2_PIN        GPIO_PinSource2
#define MOTORS_GPIO_AF_M2            GPIO_AF_TIM2

#define MOTORS_GPIO_M3_PERIF         RCC_AHB1Periph_GPIOA // PA3
#define MOTORS_GPIO_M3_PORT          GPIOA
#define MOTORS_GPIO_M3_PIN           GPIO_Pin_3
#define MOTORS_GPIO_AF_M3_PIN        GPIO_PinSource3
#define MOTORS_GPIO_AF_M3            GPIO_AF_TIM2

#define MOTORS_GPIO_M4_PERIF         RCC_AHB1Periph_GPIOB // PB4
#define MOTORS_GPIO_M4_PORT          GPIOB
#define MOTORS_GPIO_M4_PIN           GPIO_Pin_4
#define MOTORS_GPIO_AF_M4_PIN        GPIO_PinSource4
#define MOTORS_GPIO_AF_M4            GPIO_AF_TIM3

#elif defined(BRUSHLESS_DECK_MAPPING)
// HW defines for prototype brushless deck
// PIN7-LEFT  -> PB5: TIM3_CH2, connect as M1
// PIN1-RIGHT -> PA2: TIM2_CH3, connect as M2
// PIN2-RIGHT -> PA3: TIM2_CH4, connect as M3
// PIN8-LEFT  -> PB4: TIM3_CH1, connect as M4

#define MOTORS_TIM_M1_PERIF       RCC_APB1Periph_TIM3 // TIM3_CH1
#define MOTORS_TIM_M1             TIM3
#define MOTORS_TIM_M1_DBG         DBGMCU_TIM3_STOP
#define M1_TIM_SETCOMPARE         TIM_SetCompare1
#define M1_TIM_GETCAPTURE         TIM_GetCapture1
#define M1_TIM_OC_INIT            TIM_OC1Init
#define M1_TIM_OC_PRE_CFG         TIM_OC1PreloadConfig

#define MOTORS_TIM_M2_PERIF       RCC_APB1Periph_TIM2 // TIM2_CH3
#define MOTORS_TIM_M2             TIM2
#define MOTORS_TIM_M2_DBG         DBGMCU_TIM2_STOP
#define M2_TIM_SETCOMPARE         TIM_SetCompare3
#define M2_TIM_GETCAPTURE         TIM_GetCapture3
#define M2_TIM_OC_INIT            TIM_OC3Init
#define M2_TIM_OC_PRE_CFG         TIM_OC3PreloadConfig

#define MOTORS_TIM_M3_PERIF       RCC_APB1Periph_TIM2 // TIM2_CH4
#define MOTORS_TIM_M3             TIM2
#define MOTORS_TIM_M3_DBG         DBGMCU_TIM2_STOP
#define M3_TIM_SETCOMPARE         TIM_SetCompare4
#define M3_TIM_GETCAPTURE         TIM_GetCapture4
#define M3_TIM_OC_INIT            TIM_OC4Init
#define M3_TIM_OC_PRE_CFG         TIM_OC4PreloadConfig

#define MOTORS_TIM_M4_PERIF       RCC_APB1Periph_TIM3 // TIM3_CH2
#define MOTORS_TIM_M4             TIM3
#define MOTORS_TIM_M4_DBG         DBGMCU_TIM3_STOP
#define M4_TIM_SETCOMPARE         TIM_SetCompare2
#define M4_TIM_GETCAPTURE         TIM_GetCapture2
#define M4_TIM_OC_INIT            TIM_OC2Init
#define M4_TIM_OC_PRE_CFG         TIM_OC2PreloadConfig

#define MOTORS_GPIO_M1_PERIF         RCC_AHB1Periph_GPIOB // PB4
#define MOTORS_GPIO_M1_PORT          GPIOB
#define MOTORS_GPIO_M1_PIN           GPIO_Pin_4
#define MOTORS_GPIO_AF_M1_PIN        GPIO_PinSource4
#define MOTORS_GPIO_AF_M1            GPIO_AF_TIM3

#define MOTORS_GPIO_M2_PERIF         RCC_AHB1Periph_GPIOA // PA2
#define MOTORS_GPIO_M2_PORT          GPIOA
#define MOTORS_GPIO_M2_PIN           GPIO_Pin_2
#define MOTORS_GPIO_AF_M2_PIN        GPIO_PinSource2
#define MOTORS_GPIO_AF_M2            GPIO_AF_TIM2

#define MOTORS_GPIO_M3_PERIF         RCC_AHB1Periph_GPIOA // PA3
#define MOTORS_GPIO_M3_PORT          GPIOA
#define MOTORS_GPIO_M3_PIN           GPIO_Pin_3
#define MOTORS_GPIO_AF_M3_PIN        GPIO_PinSource3
#define MOTORS_GPIO_AF_M3            GPIO_AF_TIM2

#define MOTORS_GPIO_M4_PERIF         RCC_AHB1Periph_GPIOB // PB5
#define MOTORS_GPIO_M4_PORT          GPIOB
#define MOTORS_GPIO_M4_PIN           GPIO_Pin_5
#define MOTORS_GPIO_AF_M4_PIN        GPIO_PinSource5
#define MOTORS_GPIO_AF_M4            GPIO_AF_TIM3
#else
// Mapping of brushed controller timers. Brushless controller can still be activated using
// the same mapping, PWM then needs to be inverted.

#ifdef BRUSHLESS_MOTORCONTROLLER
  // The brushed motor drivers (pull-down mosfet) inverses the output. Compensate for that.
  #define BRUSHLESS_INVERSED_POLARITY
#endif

#define MOTORS_TIM_M1_PERIF       RCC_APB1Periph_TIM2
#define MOTORS_TIM_M1             TIM2
#define MOTORS_TIM_M1_DBG         DBGMCU_TIM2_STOP
#define M1_TIM_SETCOMPARE         TIM_SetCompare2
#define M1_TIM_GETCAPTURE         TIM_GetCapture2
#define M1_TIM_OC_INIT            TIM_OC2Init
#define M1_TIM_OC_PRE_CFG         TIM_OC2PreloadConfig

#define MOTORS_TIM_M2_PERIF       RCC_APB1Periph_TIM2
#define MOTORS_TIM_M2             TIM2
#define MOTORS_TIM_M2_DBG         DBGMCU_TIM2_STOP
#define M2_TIM_SETCOMPARE         TIM_SetCompare4
#define M2_TIM_GETCAPTURE         TIM_GetCapture4
#define M2_TIM_OC_INIT            TIM_OC4Init
#define M2_TIM_OC_PRE_CFG         TIM_OC4PreloadConfig

#define MOTORS_TIM_M3_PERIF       RCC_APB1Periph_TIM2
#define MOTORS_TIM_M3             TIM2
#define MOTORS_TIM_M3_DBG         DBGMCU_TIM2_STOP
#define M3_TIM_SETCOMPARE         TIM_SetCompare1
#define M3_TIM_GETCAPTURE         TIM_GetCapture1
#define M3_TIM_OC_INIT            TIM_OC1Init
#define M3_TIM_OC_PRE_CFG         TIM_OC1PreloadConfig

#define MOTORS_TIM_M4_PERIF       RCC_APB1Periph_TIM4
#define MOTORS_TIM_M4             TIM4
#define MOTORS_TIM_M4_DBG         DBGMCU_TIM4_STOP
#define M4_TIM_SETCOMPARE         TIM_SetCompare4
#define M4_TIM_GETCAPTURE         TIM_GetCapture4
#define M4_TIM_OC_INIT            TIM_OC4Init
#define M4_TIM_OC_PRE_CFG         TIM_OC4PreloadConfig

#define MOTORS_GPIO_M1_PERIF         RCC_AHB1Periph_GPIOA
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

#endif


/*** Public interface ***/

/**
 * Initialisation. Will set all motors ratio to 0%
 */
void motorsInit();

/**
 * Test of the motor modules. The test will spin each motor very short in
 * the sequence M1 to M4.
 */
bool motorsTest(void);

/**
 * Set the PWM ratio of the motor 'id'
 */
void motorsSetRatio(int id, uint16_t ratio);

/**
 * Get the PWM ratio of the motor 'id'. Return -1 if wrong ID.
 */
int motorsGetRatio(int id);

/**
 * FreeRTOS Task to test the Motors driver
 */
void motorsTestTask(void* params);

#endif /* __MOTORS_H__ */

