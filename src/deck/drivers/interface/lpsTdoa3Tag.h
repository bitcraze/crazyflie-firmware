#ifndef __LPS_TDOA3_TAG_H__
#define __LPS_TDOA3_TAG_H__

#include "locodeck.h"
#include "libdw1000.h"

#include "mac.h"

extern uwbAlgorithm_t uwbTdoa3TagAlgorithm;

#define LOCODECK_NR_OF_TDOA3_ANCHORS 8

typedef struct {
  uint8_t type;
  uint8_t seq;
  uint32_t txTimeStamp;
  uint8_t remoteCount;
} __attribute__((packed)) rangePacketHeader3_t;

typedef struct {
  uint8_t id;
  uint8_t seq;
  uint32_t rxTimeStamp;
  uint16_t distance;
} __attribute__((packed)) remoteAnchorDataFull_t;

typedef struct {
  uint8_t id;
  uint8_t seq;
  uint32_t rxTimeStamp;
} __attribute__((packed)) remoteAnchorDataShort_t;

typedef struct {
  rangePacketHeader3_t header;
  uint8_t remoteAnchorData;
} __attribute__((packed)) rangePacket3_t;


// Positions for sent LPP packets
#define LPS_TDOA3_TYPE 0
#define LPS_TDOA3_SEND_LPP_PAYLOAD 1

#define TDOA3_LPP_PACKET_SEND_TIMEOUT (LOCODECK_NR_OF_ANCHORS * 5)

#define TDOA3_RECEIVE_TIMEOUT 10000

#endif // __LPS_TDOA3_TAG_H__
