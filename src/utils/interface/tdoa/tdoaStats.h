#ifndef __LPS_TDOA_STATS_H__
#define __LPS_TDOA_STATS_H__

#include <inttypes.h>
#include "statsCnt.h"

typedef struct {
  statsCntRateLogger_t packetsReceived;
  statsCntRateLogger_t packetsToEstimator;
  statsCntRateLogger_t contextHitCount;
  statsCntRateLogger_t contextMissCount;
  statsCntRateLogger_t timeIsGood;
  statsCntRateLogger_t suitableDataFound;

  // Anchor ids to use for stats
  uint8_t anchorId; // The id of the anchor to log
  uint8_t remoteAnchorId; // The id of the remote anchor to log

  // Clock correction for the anchor identified by anchorId
  float clockCorrection;
  statsCntRateLogger_t clockCorrectionCount;

  // TOF data from remoteAnchorId to anchorId, measured by anchorId
  uint16_t tof;

  // TDoA (in meters) between anchorId and remoteAnchorId
  float tdoa;

  uint32_t nextStatisticsTime;
  uint32_t previousStatisticsTime;

  uint8_t newAnchorId; // Used to change anchor to log, set as param
  uint8_t newRemoteAnchorId; // Used to change remote anchor to log, set as param
} tdoaStats_t;

void tdoaStatsInit(tdoaStats_t* tdoaStats, uint32_t now_ms);
void tdoaStatsUpdate(tdoaStats_t* tdoaStats, uint32_t now_ms);

#endif // __LPS_TDOA_STATS_H__
