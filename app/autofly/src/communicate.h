#ifndef __COMMUNICATE_H__
#define __COMMUNICATE_H__
#define DEBUG_MODULE "P2P"
#include "auxiliary_tool.h"
#define MOVE_DELAY 600
#define MAPPING_REQUEST_PAYLOAD_LENGTH 1
#define AUTOFLY_PACKET_MUT 60-3
#define UAV_COMPUTING_ID 0x00

typedef enum{
    // request
    MAPPING_REQ = 0x10, // mapping
    EXPLORE_REQ = 0x20, // explore
    PATH_REQ = 0x30,    // path
    CLUSTER_REQ = 0x40,  // cluster
    
    TERMINATE = 0xFF,   // terminate

    // response
    EXPLORE_RESP = 0x2A,
    PATH_RESP = 0x3A,
    CLUSTER_RESP = 0x4A,
}packetType_t;

typedef struct
{
    uint8_t len : 4;
    uint8_t mergedNums : 4;
    coordinate_t startPoint;
    coordinate_t endPoint[6];
} mapping_req_payload_t;    // 6*7+1 = 43

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
    mapping_req_payload_t mappingRequestPayload[MAPPING_REQUEST_PAYLOAD_LENGTH];
} mapping_req_packet_t; // 43*MAPPING_REQUEST_PAYLOAD_LENGTH+2

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

typedef struct
{
    uint8_t sourceId;
    uint8_t destinationId;
    uint8_t packetType;
    uint8_t data[AUTOFLY_PACKET_MUT];
} Autofly_packet_t;   // 60

void CommunicateInit();
void CommunicateTerminate();
bool generateMappingReqPacket(mapping_req_packet_t* mappingReqPacket);
bool generateExploreReqPacket(explore_req_packet_t* exploreReqPacket);
bool sendMappingRequest(mapping_req_packet_t* mappingReqPacket);
bool sendExploreRequest(explore_req_packet_t* exploreReqPacket);

bool sendTerminate();
#endif
