#ifndef __LPS_TDOA3_TAG_H__
#define __LPS_TDOA3_TAG_H__
#include "locodeck.h"

extern uwbAlgorithm_t uwbTdoa3TagAlgorithm;
// --- [Add] --- //
#define AGENT_LIST_UPDATE_INTERVAL 1000;
#define AGENT_STORAGE_COUNT 16
#define REMOTE_TX_MAX_COUNT 8
#if REMOTE_TX_MAX_COUNT > AGENT_STORAGE_COUNT
  #error "Invalid settings"
#endif
// original 20 ~ 50
// [Note] 
// if keep the inter-drone ranging at 20 ~ 50, the rate of sending tdoa to EKF is around 17 hz (slow!)
// if 20 ~ 30, the rate of sending tdoa to EKF is around 30 hz or less.
#define AGENT_MAX_TX_FREQ 30.0
// We need a lower limit of minimum tx rate. The TX timestamp in the protocol is
// only 32 bits (equal to 67 ms) and we want to avoid double wraps of the TX counter.
// To have some margin set the lowest tx frequency to 20 Hz (= 50 ms)
#define AGENT_MIN_TX_FREQ 20.0
// define a packet type for Agent info
#define LPP_SHORT_AGENT_INFO 0x07


#endif // __LPS_TDOA3_TAG_H__
