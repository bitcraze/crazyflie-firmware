/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2026 Bitcraze AB
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
 * ledseq.c - Stub of ledseq.h for the Simmyflie.
 *
 * The mainline ledseq.c plays timed sequences on the LEDs. The sim has no LEDs
 * and shows no LED state, so every function does nothing. The seq_* contexts
 * are defined because mainline code passes their addresses, as in
 * ledseqRun(&seq_alive).
 */

#include "ledseq.h"

#include <stddef.h>

ledseqContext_t seq_calibrated              = { .sequence = NULL, .nextContext = NULL, .state = 0, .led = LED_BLUE_L };
ledseqContext_t seq_alive                   = { .sequence = NULL, .nextContext = NULL, .state = 0, .led = LED_BLUE_L };
ledseqContext_t seq_lowbat                  = { .sequence = NULL, .nextContext = NULL, .state = 0, .led = LED_BLUE_L };
ledseqContext_t seq_linkUp                  = { .sequence = NULL, .nextContext = NULL, .state = 0, .led = LED_GREEN_L };
ledseqContext_t seq_linkDown                = { .sequence = NULL, .nextContext = NULL, .state = 0, .led = LED_RED_L };
ledseqContext_t seq_charged                 = { .sequence = NULL, .nextContext = NULL, .state = 0, .led = LED_BLUE_L };
ledseqContext_t seq_charging                = { .sequence = NULL, .nextContext = NULL, .state = 0, .led = LED_BLUE_L };
ledseqContext_t seq_testPassed              = { .sequence = NULL, .nextContext = NULL, .state = 0, .led = LED_GREEN_R };
ledseqContext_t seq_testFailed              = { .sequence = NULL, .nextContext = NULL, .state = 0, .led = LED_RED_R };
ledseqContext_t seq_user_notification_success = { .sequence = NULL, .nextContext = NULL, .state = 0, .led = LED_GREEN_R };
ledseqContext_t seq_user_notification_fail    = { .sequence = NULL, .nextContext = NULL, .state = 0, .led = LED_RED_R };

void ledseqInit(void)
{
}

bool ledseqTest(void)
{
  return true;
}

void ledseqEnable(bool enable)
{
  (void)enable;
}

void ledseqRegisterSequence(ledseqContext_t* context)
{
  (void)context;
}

bool ledseqRun(ledseqContext_t* context)
{
  (void)context;
  return true;
}

void ledseqRunBlocking(ledseqContext_t* context)
{
  (void)context;
}

bool ledseqStop(ledseqContext_t* context)
{
  (void)context;
  return true;
}

void ledseqStopBlocking(ledseqContext_t* context)
{
  (void)context;
}

void ledseqSetChargeLevel(const float chargeLevel)
{
  (void)chargeLevel;
}
