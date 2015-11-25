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
 * buzzer.c - Functions for interfacing with decks with buzzers
 */
#ifndef __BUZZER_H__
#define __BUZZER_H__
 
#include <stdint.h>
#include <stdbool.h>

struct buzzerControl
{
  void (*off)();
  void (*on)(uint32_t freq);
};

void buzzerInit();
bool buzzerTest();
void buzzerOff();
void buzzerOn(uint32_t freq);
void buzzerSetControl(struct buzzerControl * bc);

#endif //__BUZZER_H__

