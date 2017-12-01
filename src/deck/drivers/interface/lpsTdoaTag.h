#ifndef __LPS_TDOA_TAG_H__
#define __LPS_TDOA_TAG_H__

#include "locodeck.h"
#include "libdw1000.h"

#include "mac.h"

extern uwbAlgorithm_t uwbTdoaTagAlgorithm;

#define LOCODECK_NR_OF_TDOA2_ANCHORS 8

typedef struct rangePacket_s {
  uint8_t type;
  uint8_t sequenceNrs[LOCODECK_NR_OF_TDOA2_ANCHORS];
  uint32_t timestamps[LOCODECK_NR_OF_TDOA2_ANCHORS];
  uint16_t distances[LOCODECK_NR_OF_TDOA2_ANCHORS];
} __attribute__((packed)) rangePacket_t;

#endif // __LPS_TDOA_TAG_H__
