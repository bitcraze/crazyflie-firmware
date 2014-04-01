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
#ifdef BRUSHLESS_MOTORCONTROLLER

//#define BLMC_PERIOD 0.02   // 20ms = 50Hz
//#define BLMC_PERIOD 0.01   // 10ms = 100Hz
#define BLMC_PERIOD 0.005   // 5ms = 200Hz

#define MOTORS_PWM_PRESCALE_RAW   (uint32_t)((72000000/0xFFFF) * BLMC_PERIOD + 1) // +1 is to not end up above 0xFFFF in the end
#define MOTORS_PWM_CNT_FOR_PERIOD (uint32_t)(72000000 * BLMC_PERIOD / MOTORS_PWM_PRESCALE_RAW)
#define MOTORS_PWM_CNT_FOR_1MS    (uint32_t)(72000000 * 0.001 / MOTORS_PWM_PRESCALE_RAW)
#define MOTORS_PWM_PERIOD         MOTORS_PWM_CNT_FOR_PERIOD
#define MOTORS_PWM_BITS           11  // Only for compatibiliy
#define MOTORS_PWM_PRESCALE       (uint16_t)(MOTORS_PWM_PRESCALE_RAW - 1)
#define MOTORS_POLARITY           TIM_OCPolarity_Low
#else
//The following defines gives a PWM of 9 bits at ~140KHz for a sysclock of 72MHz
#define MOTORS_PWM_BITS     9
#define MOTORS_PWM_PERIOD   ((1<<MOTORS_PWM_BITS) - 1)
#define MOTORS_PWM_PRESCALE 0
#define MOTORS_POLARITY           TIM_OCPolarity_High
#endif


// Motors IDs define
#define MOTOR_M1  0
#define MOTOR_M2  1
#define MOTOR_M3  2
#define MOTOR_M4  3

// Test defines
#define MOTORS_TEST_RATIO         (uint16_t)(0.5*(1<<16))
#define MOTORS_TEST_ON_TIME_MS    10
#define MOTORS_TEST_DELAY_TIME_MS 50

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

