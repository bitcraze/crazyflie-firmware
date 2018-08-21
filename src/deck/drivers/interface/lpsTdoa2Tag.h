#ifndef __LPS_TDOA2_TAG_H__
#define __LPS_TDOA2_TAG_H__

#include "locodeck.h"
#include "libdw1000.h"

#include "mac.h"

extern uwbAlgorithm_t uwbTdoa2TagAlgorithm;

#ifdef LOCODECK_NR_OF_ANCHORS
#define LOCODECK_NR_OF_TDOA2_ANCHORS LOCODECK_NR_OF_ANCHORS
#else
#define LOCODECK_NR_OF_TDOA2_ANCHORS 8
#endif

typedef struct {
  const locoAddress_t anchorAddress[LOCODECK_NR_OF_TDOA2_ANCHORS];

  point_t anchorPosition[LOCODECK_NR_OF_TDOA2_ANCHORS];
  bool combinedAnchorPositionOk;
} lpsTdoa2AlgoOptions_t;


typedef struct {
  uint8_t type;
  uint8_t sequenceNrs[LOCODECK_NR_OF_TDOA2_ANCHORS];
  uint32_t timestamps[LOCODECK_NR_OF_TDOA2_ANCHORS];
  uint16_t distances[LOCODECK_NR_OF_TDOA2_ANCHORS];
} __attribute__((packed)) rangePacket2_t;


// Protocol version
#define PACKET_TYPE_TDOA2 0x22

// Positions in payload for received LPP packets
#define LPS_TDOA2_LPP_HEADER (sizeof(rangePacket2_t))
#define LPS_TDOA2_LPP_TYPE (sizeof(rangePacket2_t) + 1)
#define LPS_TDOA2_LPP_PAYLOAD (sizeof(rangePacket2_t) + 2)

// Positions for sent LPP packets
#define LPS_TDOA2_TYPE_INDEX 0
#define LPS_TDOA2_SEND_LPP_PAYLOAD_INDEX 1

#define TDOA2_LPP_PACKET_SEND_TIMEOUT (LOCODECK_NR_OF_TDOA2_ANCHORS * 5)

#define TDOA2_RECEIVE_TIMEOUT 10000

void lpsTdoa2TagSetOptions(lpsTdoa2AlgoOptions_t* newOptions);

#endif // __LPS_TDOA2_TAG_H__
