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

// Positions in payload for received LPP packets
#define LPS_TDOA_LPP_HEADER (sizeof(rangePacket_t))
#define LPS_TDOA_LPP_TYPE (sizeof(rangePacket_t) + 1)
#define LPS_TDOA_LPP_PAYLOAD (sizeof(rangePacket_t) + 2)

// Positions for sent LPP packets
#define LPS_TDOA2_TYPE 0
#define LPS_TDOA2_SEND_LPP_PAYLOAD 1

#define TDOA2_LPP_PACKET_SEND_TIMEOUT (LOCODECK_NR_OF_ANCHORS * 5)

#define TDOA_RECEIVE_TIMEOUT 10000

#endif // __LPS_TDOA_TAG_H__
