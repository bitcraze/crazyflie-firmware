#include "clockCorrectionEngine.h"
#include "clockCorrectionFunctions.h"

static uint64_t mask = 0xFFFFFFFFFF; // Default to 40 bits

/**
 Initializes the engine
 */
static void init(const uint16_t sizeOfTimestamps) {
  uint64_t one = 1;
  mask = (one << sizeOfTimestamps) - 1;
}

/**
 Calculates the clock correction using the mask configured at init
 */
static double engine_calculateClockCorrection(const uint64_t new_t_in_cl_reference, const uint64_t old_t_in_cl_reference, const uint64_t new_t_in_cl_x, const uint64_t old_t_in_cl_x) {
  return calculateClockCorrection(new_t_in_cl_reference, old_t_in_cl_reference, new_t_in_cl_x, old_t_in_cl_x, mask);
}

clockCorrectionEngine_t clockCorrectionEngine = {
  .init = init,
  .getClockCorrection = getClockCorrection,
  .calculateClockCorrection = engine_calculateClockCorrection,
  .updateClockCorrection = updateClockCorrection
};


