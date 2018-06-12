/**
 Exposes the storage required by the clock correction engine.
 */

#ifndef clockCorrectionStorage_h
#define clockCorrectionStorage_h

#include <stdint.h>

typedef struct {
  double clockCorrection;
  unsigned int clockCorrectionBucket;
} clockCorrectionStorage_t;


#endif /* clockCorrectionStorage_h */
