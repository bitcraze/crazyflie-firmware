#ifndef __COMMUNICATE_H__
#define __COMMUNICATE_H__
#define DEBUG_MODULE "P2P"
#include "stdint.h"
#define MAPPING_REQUEST_PAYLOAD_LENGTH 1
#define AUTOFLY_PACKET_MUT 60-4
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
    uint16_t x;
    uint16_t y;
    uint16_t z;
} coordinate_t;

typedef struct
{
    float x;
    float y;
    float z;
} coordinateF_t;

typedef struct
{
    float data[6];
    float roll;
    float pitch;
    float yaw;
} example_measure_t;

typedef struct uavRange_t
{
    example_measure_t measurement;
    coordinateF_t current_point;
}uavRange_t;

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
    uint8_t length;
    uint8_t data[AUTOFLY_PACKET_MUT];
} Autofly_packet_t;  

void CPXForwardInit();
void P2PListeningInit();
bool sendAutoflyPacket(Autofly_packet_t* AutoflyPacket);
#endif
