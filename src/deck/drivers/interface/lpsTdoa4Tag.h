/*
    lpsTdoa4Tag.h
    Created on : Nov.,21,2020
        Author : Wenda Zhao
        Email  : wenda.zhao@robotics.utias.utoronto.ca
    header file for TDoA 4 structure (enabling ranging between CFs)
*/

#ifndef __LPS_TDOA4_TAG_H__
#define __LPS_TDOA4_TAG_H__

#include "locodeck.h"

// [change]: define a lot of things 
#define debug(...) printf(__VA_ARGS__)

// Time length of the preamble
#define PREAMBLE_LENGTH_S ( 128 * 1017.63e-9 )
#define PREAMBLE_LENGTH (uint64_t)( PREAMBLE_LENGTH_S * 499.2e6 * 128 )

// Guard length to account for clock drift and time of flight
#define TDMA_GUARD_LENGTH_S ( 1e-6 )
#define TDMA_GUARD_LENGTH (uint64_t)( TDMA_GUARD_LENGTH_S * 499.2e6 * 128 )

#define TDMA_EXTRA_LENGTH_S ( 300e-6 )
#define TDMA_EXTRA_LENGTH (uint64_t)( TDMA_EXTRA_LENGTH_S * 499.2e6 * 128 )

#define TDMA_HIGH_RES_RAND_S ( 1e-3 )
#define TDMA_HIGH_RES_RAND (uint64_t)( TDMA_HIGH_RES_RAND_S * 499.2e6 * 128 )

#define ANCHOR_LIST_UPDATE_INTERVAL 1000;

#define ANCHOR_STORAGE_COUNT 16
#define REMOTE_TX_MAX_COUNT 8
#if REMOTE_TX_MAX_COUNT > ANCHOR_STORAGE_COUNT
  #error "Invalid settings"
#endif

#define ID_COUNT 256
#define ID_WITHOUT_CONTEXT 0xff
#define ID_INVALID 0xff
// [Note]: can be reduce for inter-range. May be handled automatically
#define SYSTEM_TX_FREQ 400.0  
#define ANCHOR_MAX_TX_FREQ 50.0
// We need a lower limit of minimum tx rate. The TX timestamp in the protocol is
// only 32 bits (equal to 67 ms) and we want to avoid double wraps of the TX counter.
// To have some margin set the lowest tx frequency to 20 Hz (= 50 ms)
#define ANCHOR_MIN_TX_FREQ 20.0


#define ANTENNA_OFFSET 154.6   // In meters
#define ANTENNA_DELAY  ((ANTENNA_OFFSET*499.2e6*128)/299792458.0) // In radio tick
#define MIN_TOF ANTENNA_DELAY

#define MAX_CLOCK_DEVIATION_SPEC 10e-6
#define CLOCK_CORRECTION_SPEC_MIN (1.0d - MAX_CLOCK_DEVIATION_SPEC * 2)
#define CLOCK_CORRECTION_SPEC_MAX (1.0d + MAX_CLOCK_DEVIATION_SPEC * 2)

#define CLOCK_CORRECTION_ACCEPTED_NOISE 0.03e-6
#define CLOCK_CORRECTION_FILTER 0.1d
#define CLOCK_CORRECTION_BUCKET_MAX 4

#define DISTANCE_VALIDITY_PERIOD M2T(2 * 1000);

// [change]: move from lpp.h
// the function handles LPP packet after receiving
// void lppHandleShortPacket(char *data, size_t length);

#define SHORT_LPP 0xF0
#define LPP_SHORT_ANCHOR_POSITION 0x01
#define LPP_SHORT_REBOOT 0x02
#define LPP_SHORT_MODE 0x03
#define LPP_SHORT_UWB 0x04
#define LPP_SHORT_UWB_MODE 0x05
// define a packet type for Agent info
#define LPP_SHORT_AGENT_INFO 0x07

#define LPP_SHORT_REBOOT_TO_BOOTLOADER 0x00
#define LPP_SHORT_REBOOT_TO_FIRMWARE 0x01

struct lppShortReboot_s {
  uint8_t bootMode;
} __attribute__((packed));

#define LPP_SHORT_MODE_TWR 0x01
#define LPP_SHORT_MODE_TDOA2 0x02
#define LPP_SHORT_MODE_TDOA3 0x03
#define LPP_SHORT_MODE_TDOA4 0x04

struct lppShortMode_s {
  uint8_t mode;
} __attribute__((packed));

struct lppShortUWB_s {
  uint8_t enableSmartPower :1;
  uint8_t forceTxPower :1;
  uint32_t txPower;
} __attribute__((packed));

struct lppShortUWBMode_s {
  uint8_t enableLowBitrate :1;
  uint8_t enableLongPreamble :1;
} __attribute__((packed));

// ----------------------------------------------------------------------//

extern uwbAlgorithm_t uwbTdoa4TagAlgorithm;

#define MAX_ANCHORS 6

// [cahnge]: move from uwb.h
typedef struct uwbConfig_s {
  uint8_t mode;
  uint8_t address[8];
  uint8_t anchorListSize;
  uint8_t anchors[MAX_ANCHORS];
  float position[3];
  bool positionEnabled;

  bool smartPower;
  bool forceTxPower;
  uint32_t txPower;

  bool lowBitrate;
  bool longPreamble;
} uwbConfig_t;

// used for short lpp packet
typedef enum {
  cfgAddress = 0,
  cfgMode,
  cfgAnchorlist,
  cfgAnchorPos,
  cfgSmartPower,
  cfgForceTxPower,
  cfgTxPower,
  cfgLowBitrate,
  cfgLongPreamble,
} ConfigField;


// [Add] Switch agent mode
int switchAgentMode();
extern int MODE;
extern int AGENT_ID;
// testing for switch mode time
// extern int xStart_s;  
#endif // __LPS_TDOA4_TAG_H__
