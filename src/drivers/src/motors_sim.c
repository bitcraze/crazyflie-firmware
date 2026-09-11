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
 * motors_sim.c - motors.h backend for CONFIG_PLATFORM_SIM (Simmyflie),
 * Phase 4.7.
 *
 * EXPLICITLY TEMPORARY (see implementation-plan-phase-4.md's 4.7 section):
 * motorsSetRatio() just records the requested ratio, no forward to a Physics
 * model -- that model doesn't exist yet (Phase 5's job). Phase 5 replaces
 * this file wholesale rather than extending it, same spirit as
 * sensors_sim.c.
 *
 * Not "motors.h": like pm.h/platform.h before it, that header pulls in the
 * STM32 hardware chain directly (#include "stm32fxxx.h", plus
 * MotorPerifDef's ST peripheral-register fields) -- see platform_sim.c's
 * header comment for the same reasoning. The five functions below are
 * declared locally instead, with mainline-matching signatures except
 * motorsInit()'s motor-map argument, narrowed from `const MotorPerifDef**`
 * (a real, ST-register-laden struct) to `const void**` since this stub never
 * dereferences it -- same narrowing precedent as i2cdevInit()'s `void*` in
 * Phase 4.0. This is link-compatible with real callers (stabilizer.c, Phase
 * 4.8): C linking matches symbol names only, not per-TU parameter types, and
 * a pointer is a pointer regardless of declared pointee type.
 *
 * motorsSetRatio(id, ratio)'s signature (not CrazySim's diverged
 * motorsSetRatio(const motors_thrust_pwm_t*)) matches current mainline
 * exactly -- see stabilizer.c's setMotorRatios(), which calls it four times,
 * once per motor. Reusing the exact mainline signature is what lets 4.8's
 * stabilizer.c call this stub completely unmodified.
 *
 * The "motor" LOG_GROUP (m1-m4 PWM ratios) matches real motors.c's own,
 * narrower only in that it skips the DSHOT-bidirectional RPM telemetry
 * entries (m1_rpm.. -- CONFIG_MOTORS_ESC_PROTOCOL_DSHOT_BIDIRECTIONAL has no
 * meaning in sim, there's no real ESC to report RPM from).
 */

#include <stdbool.h>
#include <stdint.h>

#include "log.h"

#define NBR_OF_MOTORS 4
#define MOTOR_M1 0
#define MOTOR_M2 1
#define MOTOR_M3 2
#define MOTOR_M4 3

static bool isInit = false;
static uint16_t motor_ratios[NBR_OF_MOTORS] = {0, 0, 0, 0};

void motorsInit(const void **motorMapSelect)
{
  (void)motorMapSelect;

  for (int i = 0; i < NBR_OF_MOTORS; i++) {
    motor_ratios[i] = 0;
  }
  isInit = true;
}

bool motorsTest(void)
{
  return isInit;
}

void motorsSetRatio(uint32_t id, uint16_t ratio)
{
  if (id < NBR_OF_MOTORS) {
    motor_ratios[id] = ratio;
  }
}

uint16_t motorsGetRatio(uint32_t id)
{
  if (id < NBR_OF_MOTORS) {
    return motor_ratios[id];
  }
  return 0;
}

void motorsStop(void)
{
  for (int i = 0; i < NBR_OF_MOTORS; i++) {
    motor_ratios[i] = 0;
  }
}

/**
 * Motor output related log variables.
 */
LOG_GROUP_START(motor)
/**
 * @brief Motor power (PWM value) for M1 [0 - UINT16_MAX]
 */
LOG_ADD_CORE(LOG_UINT16, m1, &motor_ratios[MOTOR_M1])
/**
 * @brief Motor power (PWM value) for M2 [0 - UINT16_MAX]
 */
LOG_ADD_CORE(LOG_UINT16, m2, &motor_ratios[MOTOR_M2])
/**
 * @brief Motor power (PWM value) for M3 [0 - UINT16_MAX]
 */
LOG_ADD_CORE(LOG_UINT16, m3, &motor_ratios[MOTOR_M3])
/**
 * @brief Motor power (PWM value) for M4 [0 - UINT16_MAX]
 */
LOG_ADD_CORE(LOG_UINT16, m4, &motor_ratios[MOTOR_M4])
LOG_GROUP_STOP(motor)
