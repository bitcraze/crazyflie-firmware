#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
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
void processReq(Autofly_packet_t* autoflyPacket);
void processResp(Autofly_packet_t* autoflyPacket);
void processExploreResp(explore_resp_packet_t* exploreRespPacket);
void processPathResp();
void processClusterResp();

uint16_t mappingRequestSeq = 0;
uint16_t exploreRequestSeq = 0;
static bool flag_exploreResp;
static uint8_t destinationId = AIDECK_ID;
static bool IsCluster = false;

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
    mappingRequestSeq = 0;
    exploreRequestSeq = 0;
    uavRange = (uavRange_t *)malloc(sizeof(uavRange_t));
    inituavRange(uavRange);
    // Initialize the uavRange
    flag_exploreResp = false;
    ListeningInit();
}

void CommunicateTerminate(){
    p2pRegisterCB(NULL);
}

bool generateMappingReqPacket(mapping_req_packet_t* mappingReqPacket){
    coordinateF_t item_end;
    uint8_t len = 0;
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

bool generateExploreReqPacket(explore_req_packet_t* exploreReqPacket){
    memcpy(&exploreReqPacket->exploreRequestPayload.uavRange, uavRange, sizeof(uavRange_t));
    exploreReqPacket->seq = exploreRequestSeq;
    return true;
}

bool generateClusterReqPacket(cluster_req_packet_t* clusterReqPacket){
    clusterReqPacket->CurrentPoint = uavRange->current_point;
    return true;
}

bool sendMappingRequest(mapping_req_packet_t* mappingReqPacket)
{
    // Test
    // if(mappingReqPacket->mappingRequestPayload->len == 0){
    //     DEBUG_PRINT("mappingRequestPayload is NULL!\n");
    //     return false;
    // }

    ++mappingRequestSeq;
    static P2PPacket packet;
    packet.port = 0x00;
    uint8_t sourceId = getSourceId();

    Autofly_packet_t Autofly_packet;
    Autofly_packet.sourceId = sourceId;
    Autofly_packet.destinationId = destinationId;
    Autofly_packet.nextdestinationId = AIDECK_ID;
    Autofly_packet.packetType = MAPPING_REQ;
    Autofly_packet.length = AUTOFLY_PACKET_HEAD_LENGTH + sizeof(mapping_req_packet_t);
    memcpy(Autofly_packet.data, mappingReqPacket, sizeof(mapping_req_packet_t));
    memcpy(packet.data, &Autofly_packet, Autofly_packet.length);

    packet.size = Autofly_packet.length;
    // Send the P2P packet
    if(radiolinkSendP2PPacketBroadcast(&packet)){
        // DEBUG_PRINT("Send mapping request successfully!,destinationId:%d,seq=%d\n",destinationId,mappingReqPacket->seq);
        mapping_req_payload_t *mappingReqPayload = (mapping_req_payload_t*)mappingReqPacket->mappingRequestPayload;
        DEBUG_PRINT("(%d,%d,%d) ", mappingReqPayload->startPoint.x, mappingReqPayload->startPoint.y, mappingReqPayload->startPoint.z);
        for(int i = 0; i < mappingReqPayload->len; i++){
            DEBUG_PRINT("(%d,%d,%d) ", mappingReqPayload->endPoint[i].x, mappingReqPayload->endPoint[i].y, mappingReqPayload->endPoint[i].z);
        }
        DEBUG_PRINT("\n");
        return true;
    }
    else{
        // DEBUG_PRINT("Send mapping request failed!,destinationId:%d,seq=%d\n",destinationId,mappingReqPacket->seq);
        return false;
    }
}

bool sendExploreRequest(explore_req_packet_t* exploreReqPacket)
{
    // Initialize the p2p packet
    static P2PPacket packet;
    packet.port = 0x00;

    if(flag_exploreResp)
        ++exploreRequestSeq;
    uint8_t sourceId = getSourceId();
    // Assemble the packet
    Autofly_packet_t Autofly_packet;
    Autofly_packet.sourceId = sourceId;
    Autofly_packet.destinationId = destinationId;
    Autofly_packet.nextdestinationId = AIDECK_ID;
    Autofly_packet.packetType = EXPLORE_REQ;
    Autofly_packet.length = AUTOFLY_PACKET_HEAD_LENGTH+sizeof(explore_req_packet_t);
    memcpy(Autofly_packet.data, exploreReqPacket, sizeof(explore_req_packet_t));
    memcpy(packet.data, &Autofly_packet, Autofly_packet.length);

    packet.size = Autofly_packet.length;
    // Send the P2P packet
    if(radiolinkSendP2PPacketBroadcast(&packet)){
        flag_exploreResp = false;
        DEBUG_PRINT("Send explore request successfully!,destinationId:%d,seq=%d\n",destinationId,exploreRequestSeq);
        return true;
    }
    else{
        DEBUG_PRINT("Send explore request failed!,destinationId:%d,seq=%d\n",destinationId,exploreRequestSeq);
        return false;
    }
}

bool sendCLusterRequest(cluster_req_packet_t* clusterReqPacket)
{
    // Initialize the p2p packet
    static P2PPacket packet;
    packet.port = 0x00;

    uint8_t sourceId = getSourceId();
    // Assemble the packet
    Autofly_packet_t Autofly_packet;
    Autofly_packet.sourceId = sourceId;
    Autofly_packet.destinationId = AIDECK_ID;
    Autofly_packet.nextdestinationId = AIDECK_ID;
    Autofly_packet.packetType = CLUSTER_REQ;
    Autofly_packet.length = AUTOFLY_PACKET_HEAD_LENGTH + sizeof(cluster_req_packet_t);
    memcpy(Autofly_packet.data, clusterReqPacket, sizeof(cluster_req_packet_t));
    memcpy(packet.data, &Autofly_packet, Autofly_packet.length);

    packet.size = Autofly_packet.length;
    // Send the P2P packet
    if(radiolinkSendP2PPacketBroadcast(&packet)){
        DEBUG_PRINT("Send cluster request successfully!,destinationId:%d\n",destinationId);
        return true;
    }
    else{
        DEBUG_PRINT("Send cluster request failed!,destinationId:%d\n",destinationId);
        return false;
    }
}

bool sendTerminate(){
    static P2PPacket packet;
    packet.port = 0x00;

    uint8_t sourceId = getSourceId();
    // Assemble the packet
    Autofly_packet_t Autofly_packet;
    Autofly_packet.sourceId = sourceId;
    Autofly_packet.destinationId = destinationId;
    Autofly_packet.nextdestinationId = AIDECK_ID;
    Autofly_packet.packetType = TERMINATE;
    Autofly_packet.length = AUTOFLY_PACKET_HEAD_LENGTH;
    memcpy(packet.data, &Autofly_packet, Autofly_packet.length);

    packet.size = Autofly_packet.length;
    // Send the P2P packet
    if(radiolinkSendP2PPacketBroadcast(&packet)){
        DEBUG_PRINT("Send terminate successfully!,destinationId:%d\n",destinationId);
        return true;
    }
    else{
        vTaskDelay(M2T(WAIT_DELAY));
        DEBUG_PRINT("Send terminate failed!,destinationId:%d\n",destinationId);
        return sendTerminate();
    }
}

void P2PCallbackHandler(P2PPacket *p)
{
    // Parse the P2P packet
    uint8_t rssi = p->rssi;
    Autofly_packet_t autoflyPacket;
    memcpy(&autoflyPacket, &p->data, sizeof(autoflyPacket));

    if (autoflyPacket.destinationId != getSourceId() || 
    (autoflyPacket.destinationId == BROADCAST_LIDAR_ID && autoflyPacket.packetType == CLUSTER_RESP))
    {
        return;
    }

    DEBUG_PRINT("[LiDAR-STM32]P2P: Receive response from: %d, RSSI: -%d dBm, respType: %x\n", 
        autoflyPacket.sourceId, rssi, autoflyPacket.packetType);

    if(autoflyPacket.packetType == MAPPING_REQ ||
        autoflyPacket.packetType == EXPLORE_REQ ||
        autoflyPacket.packetType == PATH_REQ ||
        autoflyPacket.packetType == TERMINATE){
            processReq(&autoflyPacket);
    }
    else if(autoflyPacket.packetType == EXPLORE_RESP ||
        autoflyPacket.packetType == PATH_RESP ||
        autoflyPacket.packetType == CLUSTER_RESP){
            processResp(&autoflyPacket);
    }
    // TODO:使用 xQueueSend 来将下一步坐标存入队列
}

void processReq(Autofly_packet_t* autoflyPacket){
    DEBUG_PRINT("processReq\n");
    if(IsCluster == false){
        //cluster_ID has been changed, notify its current cluster_ID
        Autofly_packet_t Autofly_packet;
        Autofly_packet.sourceId = getSourceId();
        Autofly_packet.destinationId = autoflyPacket->sourceId;
        Autofly_packet.packetType = CLUSTER_RESP;
        Autofly_packet.length = AUTOFLY_PACKET_HEAD_LENGTH+sizeof(cluster_resp_packet_t);
        cluster_resp_packet_t clusterRespPacket;
        clusterRespPacket.clusterId = destinationId;
        memcpy(Autofly_packet.data, &clusterRespPacket, sizeof(cluster_resp_packet_t));
        P2PPacket packet;
        packet.port = 0x00;
        memcpy(packet.data, &Autofly_packet, Autofly_packet.length);
        packet.size = Autofly_packet.length;
        radiolinkSendP2PPacketBroadcast(&packet);
    }
    
    // forward the request 
    if(IsCluster)
        autoflyPacket->destinationId = autoflyPacket->nextdestinationId;
    else{
        autoflyPacket->destinationId = destinationId;
    }
    P2PPacket packet;
    packet.port = 0x00;
    memcpy(packet.data, &autoflyPacket, autoflyPacket->length);
    packet.size = autoflyPacket->length;
    radiolinkSendP2PPacketBroadcast(&packet);
}

void processResp(Autofly_packet_t* autoflyPacket){
    if(IsCluster)
    {
        if(autoflyPacket->nextdestinationId == getSourceId() || autoflyPacket->nextdestinationId == BROADCAST_LIDAR_ID){
            switch (autoflyPacket->packetType)
            {
                case EXPLORE_RESP:{
                    explore_resp_packet_t exploreRespPacket;
                    memcpy(&exploreRespPacket, autoflyPacket->data, sizeof(explore_resp_packet_t));
                    processExploreResp(&exploreRespPacket);
                    break;
                }
                case PATH_RESP:{
                    processPathResp();
                    break;
                }
                case CLUSTER_RESP:{
                    cluster_req_packet_t clusterReqPacket;
                    memcpy(&clusterReqPacket, autoflyPacket->data, sizeof(cluster_req_packet_t));
                    processClusterResp(&clusterReqPacket);
                    break;
                }
                default:{
                    break;
                }
            }
        }
        else{
            // forward the response
            autoflyPacket->destinationId = autoflyPacket->nextdestinationId;
            P2PPacket packet;
            packet.port = 0x00;
            memcpy(packet.data, &autoflyPacket, autoflyPacket->length);
            packet.size = autoflyPacket->length;
            radiolinkSendP2PPacketBroadcast(&packet);
        }
    }
    else{
        switch (autoflyPacket->packetType)
        {
            case EXPLORE_RESP:{
                explore_resp_packet_t exploreRespPacket;
                memcpy(&exploreRespPacket, autoflyPacket->data, sizeof(explore_resp_packet_t));
                processExploreResp(&exploreRespPacket);
                break;
            }
            case PATH_RESP:{
                processPathResp();
                break;
            }
            case CLUSTER_RESP:{
                cluster_req_packet_t clusterReqPacket;
                memcpy(&clusterReqPacket, autoflyPacket->data, sizeof(cluster_req_packet_t));
                processClusterResp(&clusterReqPacket);
                break;
            }
            default:{
                break;
            }
        }
    }
}

void processExploreResp(explore_resp_packet_t* exploreRespPacket)
{
    if(exploreRespPacket->seq == exploreRequestSeq){
        flag_exploreResp = true;
        MoveToNext(&uavRange->current_point,&exploreRespPacket->exploreResponsePayload.nextpoint);
    }
    else{
        DEBUG_PRINT("[LiDAR-STM32]P2P: Received an exploration response message in the wrong order\n");
    }
}

void processPathResp(){
    return;
}

void processClusterResp(cluster_resp_packet_t* clusterRespPacket){ 
    if(clusterRespPacket->clusterId == getSourceId()){
        destinationId = AIDECK_ID;
        IsCluster = true;
    }
    else{
        destinationId = clusterRespPacket->clusterId;
        IsCluster = false;
    }
    return;
}