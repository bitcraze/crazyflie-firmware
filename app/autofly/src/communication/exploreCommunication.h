#ifndef __EXPLORATION_COMMUNICATION_H__
#define __EXPLORATION_COMMUNICATION_H__

#include "stdint.h"
#include "auxiliary_tool.h"
#include "autoflyPacket.h"

#define MAPPING_REQUEST_PAYLOAD_LENGTH 1


typedef struct
{
    uavRange_t uavRange;
} explore_req_payload_t;    // 48

typedef struct
{
    coordinateF_t nextpoint;
} explore_resp_payload_t;   // 12

typedef struct
{
    uint16_t seq;
    explore_req_payload_t exploreRequestPayload;
} explore_req_packet_t; // 48+2

typedef struct
{
    uint16_t seq;
    explore_resp_payload_t exploreResponsePayload;
} explore_resp_packet_t;  // 12+2

void exploreCommunicationInit();

uint16_t getExploreRequestSeq();

bool generateExploreReqPacket(explore_req_packet_t* exploreReqPacket);
bool sendExploreRequest(uint16_t destinationId, explore_req_packet_t* exploreReqPacket);
void processExploreResp(Autofly_packet_t *autoflyPacket);
#endif