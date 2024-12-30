#include "stdlib.h"

#include "FreeRTOS.h"
#include "debug.h"
#include "task.h"

#include "communicate.h"
#include "mappingCommunication.h"
#include "control_tool.h"

static uint16_t mappingRequestSeq = 0;

void mappingCommunicationInit()
{
    mappingRequestSeq = 0;
}

uint16_t getMappingRequestSeq(){
    return mappingRequestSeq;
}

bool generateMappingReqPacket(mapping_req_packet_t* mappingReqPacket){
    coordinateF_t item_end;
    uint8_t len = 0;
    uavRange_t *uavRange = NULL;
    getUavRange(uavRange);
    mappingReqPacket->mappingRequestPayload[0].mergedNums = 1;

    mappingReqPacket->mappingRequestPayload[0].startPoint.x = (int)uavRange->current_point.x;
    mappingReqPacket->mappingRequestPayload[0].startPoint.y = (int)uavRange->current_point.y;
    mappingReqPacket->mappingRequestPayload[0].startPoint.z = (int)uavRange->current_point.z;

    for (rangeDirection_t dir = rangeFront; dir <= rangeDown; ++dir)
    {
        if (cal_Point(&uavRange->measurement, &uavRange->current_point, dir, &item_end))
        {
            mappingReqPacket->mappingRequestPayload[0].endPoint[len].x = item_end.x;
            mappingReqPacket->mappingRequestPayload[0].endPoint[len].y = item_end.y;
            mappingReqPacket->mappingRequestPayload[0].endPoint[len].z = item_end.z;
            ++len;
        }
    }
    // Test
    // if(len == 0)
    //     return false;
    mappingReqPacket->mappingRequestPayload[0].len = len;
    mappingReqPacket->seq = mappingRequestSeq;
    return true;
}

bool sendMappingRequest(uint16_t destinationId, mapping_req_packet_t* mapping_req_packet)
{
    return sendAutoFlyPacket(destinationId, MAPPING_REQ, (uint8_t*)mapping_req_packet, sizeof(mapping_req_packet_t));
}