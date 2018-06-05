#include "clockCorrectionFunctions.h"

#define MAX_CLOCK_DEVIATION_SPEC 10e-6
#define CLOCK_CORRECTION_SPEC_MIN (1.0 - MAX_CLOCK_DEVIATION_SPEC * 2)
#define CLOCK_CORRECTION_SPEC_MAX (1.0 + MAX_CLOCK_DEVIATION_SPEC * 2)

#define CLOCK_CORRECTION_ACCEPTED_NOISE 0.03e-6
#define CLOCK_CORRECTION_FILTER 0.1
#define CLOCK_CORRECTION_BUCKET_MAX 4

/**
 Obtains the clock correction from a clockCorrectionStorage_t object. This is the recommended public API to obtan the clock correction, instead of getting it directly from the storage object.
 */
double getClockCorrection(clockCorrectionStorage_t* storage) {
  return storage->clockCorrection;
}

/**
 Truncates a timestamp to 40 bits, which is the number of bits used to timestamp events in the DW1000 chip. This truncation ensures that the value returned is a valid time event, even if the DW1000 wrapped around at some point.
 */
uint64_t truncateTimeStampFromDW1000(uint64_t timeStamp) {
  uint64_t mask = 0xFFFFFFFFFF; // 40 bits
  return timeStamp & mask;
}

/**
 Implementation of the leaky bucket algorithm. See: https://en.wikipedia.org/wiki/Leaky_bucket
 */
void fillClockCorrectionBucket(clockCorrectionStorage_t* storage) {
  if (storage->clockCorrectionBucket < CLOCK_CORRECTION_BUCKET_MAX) {
    storage->clockCorrectionBucket++;
  }
}

/**
 Implementation of the leaky bucket algorithm. See: https://en.wikipedia.org/wiki/Leaky_bucket
 */
bool emptyClockCorrectionBucket(clockCorrectionStorage_t* storage) {
  if (storage->clockCorrectionBucket > 0) {
    storage->clockCorrectionBucket--;
    return false;
  }

  return true;
}

/**
 Calculates the clock correction between a reference clock and another clock (x).

 @param new_t_in_cl_reference The newest time of occurrence for an event (t), meassured by the clock of reference
 @param old_t_in_cl_reference An older time of occurrence for an event (t), meassured by the clock of reference
 @param new_t_in_cl_x The newest time of occurrence for an event (t), meassured by clock x
 @param old_t_in_cl_x The previous time of occurrence for an event (t), meassured by clock x
 @return The necessary clock correction to apply to timestamps meassured by clock x, in order to obtain their value like if the meassurement was done by the reference clock. Or -1 if it was not possible to perform the computation. Example: timestamp_in_cl_reference = clockCorrection * timestamp_in_cl_x
 */
double calculateClockCorrection(const int64_t new_t_in_cl_reference, const int64_t old_t_in_cl_reference, const int64_t new_t_in_cl_x, const int64_t old_t_in_cl_x) {
  const uint64_t tickCount_in_cl_reference = truncateTimeStampFromDW1000(new_t_in_cl_reference - old_t_in_cl_reference);
  const uint64_t tickCount_in_cl_x = truncateTimeStampFromDW1000(new_t_in_cl_x - old_t_in_cl_x);

  if (tickCount_in_cl_x == 0) {
    return -1;
  }

  return (double)tickCount_in_cl_reference / (double)tickCount_in_cl_x;
}

/**
 Updates the clock correction only if the provided value follows certain conditions. This is used to discard wrong clock correction meassurements.
 @return True if the provided clock correction was valid and accepted, false otherwise.
 */
bool updateClockCorrection(clockCorrectionStorage_t* storage, const double clockCorrectionCandidate) {
  bool sampleIsAccepted = false;

  if (CLOCK_CORRECTION_SPEC_MIN < clockCorrectionCandidate && clockCorrectionCandidate < CLOCK_CORRECTION_SPEC_MAX) {
    const double currentClockCorrection = storage->clockCorrection;
    const double difference = clockCorrectionCandidate - currentClockCorrection;

    if (-CLOCK_CORRECTION_ACCEPTED_NOISE < difference && difference < CLOCK_CORRECTION_ACCEPTED_NOISE) {
      // Simple low pass filter
      const double newClockCorrection = currentClockCorrection * CLOCK_CORRECTION_FILTER + clockCorrectionCandidate * (1.0 - CLOCK_CORRECTION_FILTER);

      fillClockCorrectionBucket(storage);

      storage->clockCorrection = newClockCorrection;
      sampleIsAccepted = true;
    } else {
      const bool shouldAcceptANewClockReference = emptyClockCorrectionBucket(storage);
      if (shouldAcceptANewClockReference) {
        fillClockCorrectionBucket(storage);
        storage->clockCorrection = clockCorrectionCandidate;
        sampleIsAccepted = true;
      }
    }
  }

  return sampleIsAccepted;
}
