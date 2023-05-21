#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "radiolink.h"
#include "configblock.h"
#include "debug.h"
#include "cpx_internal_router.h"
#include "cpx_external_router.h"

#include "communicate.h"
#include "control_tool.h"
#include "auxiliary_tool.h"

uint8_t getSourceId();
void ListeningInit();
void P2PCallbackHandler(P2PPacket *p);
void processExploreResp(explore_resp_packet_t* exploreRespPacket);
void processPathResp();
void processClusterResp();

uint16_t mappingRequestSeq;
uint16_t exploreRequestSeq;
static uavRange_t uavRange;
static bool flag_exploreResp;

uint8_t getSourceId()
{
    uint64_t address = configblockGetRadioAddress();
    uint8_t sourceId = (uint8_t)((address) & 0x00000000ff);
    return sourceId;
}

void ListeningInit()
{
    // Register the callback function so that the CF can receive packets as well.
    p2pRegisterCB(P2PCallbackHandler);
}

void CommunicateInit(){
    // Initialize the uavRange
    inituavRange(&uavRange);
    mappingRequestSeq = 0;
    exploreRequestSeq = 0;
    flag_exploreResp = false;
    ListeningInit();
}


bool generateMappingReqPacket(mapping_req_packet_t* mappingReqPacket){
    coordinateF_t item_end;
    uint8_t len = 0;
    mappingReqPacket->mappingRequestPayload[0].mergedNums = 1;

    mappingReqPacket->mappingRequestPayload[0].startPoint.x = uavRange.current_point.x;
    mappingReqPacket->mappingRequestPayload[0].startPoint.y = uavRange.current_point.y;
    mappingReqPacket->mappingRequestPayload[0].startPoint.z = uavRange.current_point.z;
    for (rangeDirection_t dir = rangeFront; dir <= rangeDown; ++dir)
    {
        if (cal_Point(&uavRange.measurement, &uavRange.current_point, dir, &item_end))
        {
            mappingReqPacket->mappingRequestPayload[0].endPoint[len].x = item_end.x;
            mappingReqPacket->mappingRequestPayload[0].endPoint[len].y = item_end.y;
            mappingReqPacket->mappingRequestPayload[0].endPoint[len].z = item_end.z;
            ++len;
        }
    }
    if(len == 0)
        return false;
    mappingReqPacket->mappingRequestPayload[0].len = len;
    mappingReqPacket->seq = mappingRequestSeq;
    return true;
}

bool generateExploreReqPacket(explore_req_packet_t* exploreReqPacket){
    memcpy(&exploreReqPacket->exploreRequestPayload.uavRange, &uavRange, sizeof(uavRange_t));
    exploreReqPacket->seq = exploreRequestSeq;
    return true;
}

bool sendMappingRequest(mapping_req_packet_t* mappingReqPacket)
{
    // Initialize the p2p packet
    if(mappingReqPacket->mappingRequestPayload->len == 0)
        return false;

    ++mappingRequestSeq;
    static P2PPacket packet;
    packet.port = 0x00;
    uint8_t sourceId = getSourceId();
    uint8_t destinationId = UAV_COMPUTING_ID;

    Autofly_packet_t Autofly_packet;
    Autofly_packet.sourceId = sourceId;
    Autofly_packet.destinationId = destinationId;
    Autofly_packet.packetType = MAPPING_REQ;
    memcpy(&Autofly_packet.data, mappingReqPacket, sizeof(mapping_req_packet_t));
    memcpy(&packet.data, &Autofly_packet, sizeof(Autofly_packet));

    packet.size = 3+sizeof(mapping_req_packet_t);
    // Send the P2P packet
    if(radiolinkSendP2PPacketBroadcast(&packet)){
        return true;
    }
    else
        return false;
}

bool sendExploreRequest(explore_req_packet_t* exploreReqPacket)
{
    // Initialize the p2p packet
    static P2PPacket packet;
    packet.port = 0x00;

    if(flag_exploreResp)
        ++exploreRequestSeq;
    uint8_t sourceId = getSourceId();
    uint8_t destinationId = UAV_COMPUTING_ID;
    // Assemble the packet
    Autofly_packet_t Autofly_packet;
    Autofly_packet.sourceId = sourceId;
    Autofly_packet.destinationId = destinationId;
    Autofly_packet.packetType = EXPLORE_REQ;
    memcpy(&Autofly_packet.data, exploreReqPacket, sizeof(explore_req_packet_t));
    memcpy(&packet.data, &Autofly_packet, sizeof(Autofly_packet));

    packet.size = 3 + sizeof(explore_req_packet_t);
    // Send the P2P packet
    if(radiolinkSendP2PPacketBroadcast(&packet)){
        flag_exploreResp = false;
        return true;
    }
    else
        return false;
}

void P2PCallbackHandler(P2PPacket *p)
{
    // Parse the P2P packet
    uint8_t rssi = p->rssi;
    Autofly_packet_t autoflyPacket;
    memcpy(&autoflyPacket, &p->data, sizeof(autoflyPacket));

    if (autoflyPacket.destinationId != getSourceId())
    {
        return;
    }

    DEBUG_PRINT("[LiDAR-STM32]P2P: Receive response from: %d, RSSI: -%d dBm, respType: %x\n", 
        autoflyPacket.sourceId, rssi, autoflyPacket.packetType);

    switch (autoflyPacket.packetType){
        case EXPLORE_RESP:{
            explore_resp_packet_t exploreRespPacket;
            memcpy(&exploreRespPacket, autoflyPacket.data, sizeof(explore_resp_packet_t));
            processExploreResp(&exploreRespPacket);
            break;
        }
        case PATH_RESP:{
            processPathResp();
            break;
        }
        case CLUSTER_RESP:{
            processClusterResp();
            break;
        }
        default:
            break;
    }
    // TODO:使用 xQueueSend 来将下一步坐标存入队列
}

void processExploreResp(explore_resp_packet_t* exploreRespPacket)
{
    if(exploreRespPacket->seq == exploreRequestSeq){
        flag_exploreResp = true;
        MoveToNext(&uavRange.current_point,&exploreRespPacket->exploreResponsePayload.nextpoint);
    }
    else{
        DEBUG_PRINT("[LiDAR-STM32]P2P: Received an exploration response message in the wrong order\n");
    }
}

void processPathResp(){
    return;
}

void processClusterResp(){ 
    return;
}