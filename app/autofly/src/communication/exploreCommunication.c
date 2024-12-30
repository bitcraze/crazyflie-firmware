#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "debug.h"
#include "task.h"

#include "exploreCommunication.h"
#include "communicate.h"
#include "control_tool.h"

static uint16_t exploreRequestSeq = 0;
static bool flag_exploreResp;

void exploreCommunicationInit()
{
    exploreRequestSeq = 0;
    flag_exploreResp = false;
}

uint16_t getExploreRequestSeq()
{
    return exploreRequestSeq;
}

bool sendExploreRequest(uint16_t destinationId, explore_req_packet_t* exploreReqPacket)
{
    return sendAutoFlyPacket(destinationId, EXPLORE_REQ, (uint8_t*)exploreReqPacket, sizeof(explore_req_packet_t));
}

bool generateExploreReqPacket(explore_req_packet_t* exploreReqPacket){
    uavRange_t *uavRange = NULL;
    getUavRange(uavRange);
    if(uavRange == NULL){
        DEBUG_PRINT("[generateExploreReqPacket]uavRange is NULL\n");
        return false;
    }
    memcpy(&exploreReqPacket->exploreRequestPayload.uavRange, uavRange, sizeof(uavRange_t));
    exploreReqPacket->seq = exploreRequestSeq;
    return true;
}

void processExploreResp(Autofly_packet_t *autoflyPacket)
{
    explore_resp_packet_t* exploreRespPacket = (explore_resp_packet_t*)autoflyPacket->data;
    uavRange_t *uavRange = NULL;
    getUavRange(uavRange);
    if(exploreRespPacket->seq == exploreRequestSeq){
        flag_exploreResp = true;
        MoveToNext(&uavRange->current_point,&exploreRespPacket->exploreResponsePayload.nextpoint);
    }
    else{
        DEBUG_PRINT("[LiDAR-STM32]P2P: Received an exploration response message in the wrong order\n");
    }
}

void processExploreReq(explore_req_packet_t* exploreReqPacket)
{
    // todo: ExploreResponse 在Ai deck上实现，此处做转发操作
}