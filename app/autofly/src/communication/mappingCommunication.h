#ifndef __MAPPING_COMMUNICATION_H__
#define __MAPPING_COMMUNICATION_H__

#include "stdint.h"
#include "auxiliary_tool.h"
#include "autoflyPacket.h"

#define MAPPING_REQUEST_PAYLOAD_LENGTH 1

typedef struct
{
    uint8_t len : 4;
    uint8_t mergedNums : 4;
    coordinate_t startPoint;
    coordinate_t endPoint[6];
} mapping_req_payload_t;    // 6*7+1 = 43

typedef struct
{
    uint16_t seq;
    mapping_req_payload_t mappingRequestPayload[MAPPING_REQUEST_PAYLOAD_LENGTH];
} mapping_req_packet_t; // 43*MAPPING_REQUEST_PAYLOAD_LENGTH+2

void mappingCommunicationInit();

uint16_t getMappingRequestSeq();

bool generateMappingReqPacket(mapping_req_packet_t* mappingReqPacket);
bool sendMappingRequest(uint16_t destinationId, mapping_req_packet_t* mapping_req_packet);

#endif