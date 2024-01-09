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
 * servo.h - servo header file
 *
 * The motors PWM ratio is a value on 16 bits (from 0 to 0xFFFF)
 * the functions of the driver will make the conversions to the actual PWM
 * precision (ie. if the precision is 8bits 0xFFFF and 0xFF00 are equivalents).
 *
 * The precision is set in number of bits by the define MOTORS_PWM_BITS
 * The timer prescaler is set by MOTORS_PWM_PRESCALE
 */
#ifndef __SERVO_H__
#define __SERVO_H__

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

/******** Defines ********/
#define SERVO_PWM_PERIOD         1000  // ARR register content
#define SERVO_PWM_FREQUENCY_HZ   50 // target servo pwm frequency
#define SERVO_PWM_PRESCALE       (uint16_t) (1680) // 84mhz / (50hz * ARR)

/**
 * Servo Initialization
 */
void servoInit();

bool servoTest(void);

/**
 *
 * @brief Set servo angle.
 * @param: angle: desired servo angle in degrees
 */
void servoSetAngle(uint8_t angle);

/**
 * Servo angle parameter callback. When servo angle is changed, call
 * function to change servo angle automatically
 *  */
void servoAngleCallBack(void);

/**
 * Saturate servo angle. Min is 0, max is defined by parameter 'servoRange'
 * @param angle: pointer to desired angle
 * */
uint8_t saturateAngle(uint8_t angle);

#endif /* __SERVO_H__ */
