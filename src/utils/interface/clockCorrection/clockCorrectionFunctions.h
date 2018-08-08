/**
Exposes all the functions used for clock correction, for testing purposes.
 */

#ifndef clockCorrectionFunctions_h
#define clockCorrectionFunctions_h

#include <stdbool.h>
#include "clockCorrectionStorage.h"

/* Public */
double getClockCorrection(const clockCorrectionStorage_t* storage);
double calculateClockCorrection(const uint64_t new_t_in_cl_reference, const uint64_t old_t_in_cl_reference, const uint64_t new_t_in_cl_x, const uint64_t old_t_in_cl_x, const uint64_t mask);
bool updateClockCorrection(clockCorrectionStorage_t* storage, const double clockCorrectionCandidate);

/* Private */
uint64_t truncateTimeStamp(const uint64_t timeStamp, const uint64_t mask);
void fillClockCorrectionBucket(clockCorrectionStorage_t* storage);
bool emptyClockCorrectionBucket(clockCorrectionStorage_t* storage);

#endif /* clockCorrectionFunctions_h */
