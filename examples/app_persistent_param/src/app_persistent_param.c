/**
 * Example: persist a parameter value from app code using paramPersistentStoreByVarId.
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
  DEBUG_PRINT("health.propTestThreshold = %.1f\n", (double)paramGetFloat(idThreshold));
  vTaskDelay(M2T(2000));

  if (!PARAM_VARID_IS_VALID(idThreshold)) {
    DEBUG_PRINT("Parameter health.propTestThreshold not found\n");
    return;
  }

  paramSetFloat(idThreshold, TARGET_THRESHOLD);

  if (paramPersistentStoreByVarId(idThreshold)) {
    DEBUG_PRINT("health.propTestThreshold set to %.1f and persisted\n", (double)TARGET_THRESHOLD);
  } else {
    DEBUG_PRINT("health.propTestThreshold set to %.1f but persistent store failed\n", (double)TARGET_THRESHOLD);
  }

  DEBUG_PRINT("health.propTestThreshold = %.1f\n", (double)paramGetFloat(idThreshold));
}
