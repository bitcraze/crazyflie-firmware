#ifndef __MAC_H__
#define __MAC_H__

#include <stdint.h>

#include "locodeck.h"

// Packet format with compressed PAN and 64Bit addresses
// Maximum 64 bytes payload
typedef struct packet_s {
    union {
      uint16_t fcf;
      struct {
        uint16_t type:3;
        uint16_t security:1;
        uint16_t framePending:1;
        uint16_t ack:1;
        uint16_t ipan:1;
        uint16_t reserved:3;
        uint16_t destAddrMode:2;
        uint16_t version:2;
        uint16_t srcAddrMode:2;
      } fcf_s;
    };

    uint8_t seq;
    uint16_t pan;
    locoAddress_t destAddress;
    locoAddress_t sourceAddress;

    uint8_t payload[128];
} __attribute__((packed)) packet_t;

#define MAC80215_PACKET_INIT(packet, TYPE) packet.fcf_s.type = (TYPE); \
  packet.fcf_s.security = 0; \
  packet.fcf_s.framePending = 0; \
  packet.fcf_s.ack = 0; \
  packet.fcf_s.ipan = 1; \
  packet.fcf_s.destAddrMode = 3; \
  packet.fcf_s.version = 1; \
  packet.fcf_s.srcAddrMode = 3;


#define MAC802154_TYPE_BEACON 0
#define MAC802154_TYPE_DATA 1
#define MAC802154_TYPE_ACK 2
#define MAC802154_TYPE_CMD 3

#define MAC802154_HEADER_LENGTH 21

#endif
