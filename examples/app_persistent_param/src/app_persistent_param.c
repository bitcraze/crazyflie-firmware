/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
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
 *
 * app_persistent_param.c - App layer example of persisting a parameter
 *  value from app code using paramPersistentStoreByVarId.
 *
 * Sets health.propTestThreshold to TARGET_THRESHOLD and persists it to EEPROM.
 * Reboot to confirm the value is restored.
 */

#include "app.h"
#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include "param_logic.h"

#define DEBUG_MODULE "PERSIST_PARAM"

#define TARGET_THRESHOLD 25.0f

void appMain()
{
  vTaskDelay(M2T(2000));
  paramVarId_t idThreshold = paramGetVarId("health", "propTestThreshold");

  if (!PARAM_VARID_IS_VALID(idThreshold)) {
    DEBUG_PRINT("Parameter health.propTestThreshold not found\n");
    return;
  }

  DEBUG_PRINT("health.propTestThreshold = %.1f\n", (double)paramGetFloat(idThreshold));
  vTaskDelay(M2T(2000));

  paramSetFloat(idThreshold, TARGET_THRESHOLD);

  if (paramPersistentStoreByVarId(idThreshold)) {
    DEBUG_PRINT("health.propTestThreshold set to %.1f and persisted\n", (double)TARGET_THRESHOLD);
  } else {
    DEBUG_PRINT("health.propTestThreshold set to %.1f but persistent store failed\n", (double)TARGET_THRESHOLD);
  }

  DEBUG_PRINT("health.propTestThreshold = %.1f\n", (double)paramGetFloat(idThreshold));
}
