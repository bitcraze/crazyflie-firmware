#include "clockCorrectionEngine.h"
#include "clockCorrectionFunctions.h"

clockCorrectionEngine_t clockCorrectionEngine = {
  .getClockCorrection = getClockCorrection,
  .calculateClockCorrection = calculateClockCorrection,
  .updateClockCorrection = updateClockCorrection
};


