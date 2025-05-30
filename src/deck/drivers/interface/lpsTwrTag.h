#ifndef __LPS_TWR_TAG_H__
#define __LPS_TWR_TAG_H__

#include "locodeck.h"
#include "libdw1000.h"

#include "mac.h"

#include "autoconf.h"

#define LPS_TWR_POLL 0x01   // Poll is initiated by the tag
#define LPS_TWR_ANSWER 0x02
#define LPS_TWR_FINAL 0x03
#define LPS_TWR_REPORT 0x04 // Report contains all measurement from the anchor

#define LPS_TWR_LPP_SHORT 0xF0

#define LPS_TWR_TYPE 0
#define LPS_TWR_SEQ 1
// LPP payload can be in the ANSWER packet
#define LPS_TWR_LPP_HEADER 2
#define LPS_TWR_LPP_TYPE 3
#define LPS_TWR_LPP_PAYLOAD 4

#define LPS_TWR_SEND_LPP_PAYLOAD 1
#define LOCODECK_NR_OF_TWR_ANCHORS CONFIG_DECK_LOCO_NR_OF_ANCHORS

extern uwbAlgorithm_t uwbTwrTagAlgorithm;

typedef struct {
  uint8_t pollRx[5];
  uint8_t answerTx[5];
  uint8_t finalRx[5];

  float pressure;
  float temperature;
  float asl;
  uint8_t pressure_ok;
} __attribute__((packed)) lpsTwrTagReportPayload_t;

typedef struct {
  const uint64_t antennaDelay;
  const int rangingFailedThreshold;

  locoAddress_t tagAddress;
  const locoAddress_t anchorAddress[LOCODECK_NR_OF_TWR_ANCHORS];

   // TWR data
  point_t anchorPosition[LOCODECK_NR_OF_TWR_ANCHORS];
  bool combinedAnchorPositionOk;

  // TWR-TDMA options
  bool useTdma;
  int tdmaSlot;
} lpsTwrAlgoOptions_t;


void uwbTwrTagSetOptions(lpsTwrAlgoOptions_t* newOptions);
float lpsTwrTagGetDistance(const uint8_t anchorId);

#define TWR_RECEIVE_TIMEOUT 1000

#endif // __LPS_TWR_TAG_H__
