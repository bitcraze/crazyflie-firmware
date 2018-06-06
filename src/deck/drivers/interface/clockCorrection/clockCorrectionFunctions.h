/**
Exposes all the functions used for clock correction, for testing purposes.
 */

#ifndef clockCorrectionFunctions_h
#define clockCorrectionFunctions_h

#include <stdbool.h>
#include "clockCorrectionStorage.h"

double getClockCorrection(clockCorrectionStorage_t* storage);
uint64_t truncateTimeStampFromDW1000(uint64_t timeStamp);
void fillClockCorrectionBucket(clockCorrectionStorage_t* storage);
bool emptyClockCorrectionBucket(clockCorrectionStorage_t* storage);
double calculateClockCorrection(const uint64_t new_t_in_cl_reference, const uint64_t old_t_in_cl_reference, const uint64_t new_t_in_cl_x, const uint64_t old_t_in_cl_x);
bool updateClockCorrection(clockCorrectionStorage_t* storage, const double clockCorrectionCandidate);

#endif /* clockCorrectionFunctions_h */
