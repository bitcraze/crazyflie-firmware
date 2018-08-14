#ifndef __LPS_TDOA_STATS_H__
#define __LPS_TDOA_STATS_H__

typedef struct {
  uint32_t packetsReceived;
  uint32_t packetsToEstimator;
  uint32_t contextHitCount;
  uint32_t contextMissCount;
  uint32_t timeIsGood;
  uint32_t suitableDataFound;

  // Anchor ids to use for stats
  uint8_t anchorId; // The id of the anchor to log
  uint8_t remoteAnchorId; // The id of the remote anchor to log

  // Clock correction for the anchor identified by anchorId
  float clockCorrection;
  uint32_t clockCorrectionCount;

  // TOF data from remoteAnchorId to anchorId, measured by anchorId
  uint16_t tof;

  // TDoA (in meters) between anchorId and remoteAnchorId
  float tdoa;
} lpsTdoaStats_t;

extern lpsTdoaStats_t lpsTdoaStats;

void lpsTdoaStatsInit();
void lpsTdoaStatsClear();
void lpsTdoaStatsUpdate();

#endif // __LPS_TDOA_STATS_H__
