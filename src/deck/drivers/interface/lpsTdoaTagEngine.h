#ifndef __LPS_TDOA_TAG_ENGINE_H__
#define __LPS_TDOA_TAG_ENGINE_H__

#include "stabilizer_types.h"

#define ANCHOR_STORAGE_COUNT 16
#define REMOTE_ANCHOR_DATA_COUNT 16
#define TOF_PER_ANCHOR_COUNT 16

typedef struct {
  uint8_t id; // Id of remote remote anchor
  uint8_t seqNr; // Sequence number of the packet received in the remote anchor (7 bits)
  int64_t rxTime; // Receive time of packet from anchor id in the remote anchor, in remote DWM clock
  uint32_t endOfLife;
} remoteAnchorData_t;

typedef struct {
  uint8_t id;
  int64_t tof;
  uint32_t endOfLife; // Time stamp when the tof data is outdated, local system time
} timeOfFlight_t;

typedef struct {
  bool isInitialized;
  uint32_t lastUpdateTime; // The time when this anchor was updated the last time
  uint8_t id; // Anchor id

  int64_t txTime; // Transmit time of last packet, in remote DWM clock
  int64_t rxTime; // Receive time of last packet, in local DWM clock
  uint8_t seqNr; // Sequence nr of last packet (7 bits)

  double clockCorrection; // local DWM clock frequency / remote DWM clock frequency
  int clockCorrectionBucket;

  point_t position; // The coordinates of the anchor

  timeOfFlight_t tof[TOF_PER_ANCHOR_COUNT];
  remoteAnchorData_t remoteAnchorData[REMOTE_ANCHOR_DATA_COUNT];
} anchorInfo_t;


void tdoaEngineInit();

void tdoaEngineSetRemoteRxTime(anchorInfo_t* anchorCtx, const uint8_t remoteAnchor, const int64_t remoteRxTime, const uint8_t remoteSeqNr);
void tdoaEngineSetTimeOfFlight(anchorInfo_t* anchorCtx, const uint8_t remoteAnchor, const int64_t tof);
uint8_t tdoaEngineGetId(const anchorInfo_t* anchorCtx);
void tdoaEngineSetAnchorPosition(anchorInfo_t* anchorCtx, const float x, const float y, const float z);
void tdoaEngineSetRxTxData(anchorInfo_t* anchorCtx, int64_t rxTime, int64_t txTime, uint8_t seqNr);

anchorInfo_t* getAnchorCtxForPacketProcessing(const uint8_t anchor);
void tdoaEngineProcessPacket(anchorInfo_t* anchorCtx, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T);

#endif // __LPS_TDOA_TAG_ENGINE_H__
