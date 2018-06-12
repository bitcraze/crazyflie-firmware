#include "clockCorrectionFunctions.h"

#define MAX_CLOCK_DEVIATION_SPEC 10e-6
#define CLOCK_CORRECTION_SPEC_MIN (1.0 - MAX_CLOCK_DEVIATION_SPEC * 2)
#define CLOCK_CORRECTION_SPEC_MAX (1.0 + MAX_CLOCK_DEVIATION_SPEC * 2)

#define CLOCK_CORRECTION_ACCEPTED_NOISE 0.03e-6
#define CLOCK_CORRECTION_FILTER 0.1
#define CLOCK_CORRECTION_BUCKET_MAX 4

/**
 Obtains the clock correction from a clockCorrectionStorage_t object. This is the recommended public API to obtain the clock correction, instead of getting it directly from the storage object.
 */
double getClockCorrection(clockCorrectionStorage_t* storage) {
  return storage->clockCorrection;
}

/**
 Truncates a timestamp to the number of bits of the mask. This truncation ensures that the value returned is a valid time event, even if the time counter wrapped around.
 */
uint64_t truncateTimeStamp(uint64_t timeStamp, uint64_t mask) {
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
 @param mask A mask as long as the number of bits used to represent the timestamps. Used to calculate a valid timestamp, even if wrapped arounds of the time counter happened at some point
 @return The necessary clock correction to apply to timestamps meassured by clock x, in order to obtain their value like if the meassurement was done by the reference clock. Or -1 if it was not possible to perform the computation. Example: timestamp_in_cl_reference = clockCorrection * timestamp_in_cl_x
 */
double calculateClockCorrection(const uint64_t new_t_in_cl_reference, const uint64_t old_t_in_cl_reference, const uint64_t new_t_in_cl_x, const uint64_t old_t_in_cl_x, const uint64_t mask) {
  const uint64_t tickCount_in_cl_reference = truncateTimeStamp(new_t_in_cl_reference - old_t_in_cl_reference, mask);
  const uint64_t tickCount_in_cl_x = truncateTimeStamp(new_t_in_cl_x - old_t_in_cl_x, mask);

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
