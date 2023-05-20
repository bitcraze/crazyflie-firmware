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

uint8_t getSourceId()
{
    uint64_t address = configblockGetRadioAddress();
    uint8_t sourceId = (uint8_t)((address) & 0x00000000ff);
    return sourceId;
}

void P2PCallbackHandler(P2PPacket *p)
{
    // Parse the P2P packet
    uint8_t rssi = p->rssi;
    explore_resp_packet_t exploreRespPacket;
    memcpy(&exploreRespPacket, &p->data, sizeof(explore_resp_packet_t));

    if (exploreRespPacket.destinationId != getSourceId() || exploreRespPacket.packetType != EXPLORE_RESP)
    {
        return;
    }

    DEBUG_PRINT("[LiDAR-STM32]P2P: Receive explore response from: %d, RSSI: -%d dBm, respType: %d, seq: %d\n", 
        exploreRespPacket.sourceId, rssi, exploreRespPacket.packetType, exploreRespPacket.seq);
    static explore_resp_payload_t responsePayload;
    memcpy(&responsePayload, &p->data[5], sizeof(explore_resp_payload_t));
    
    // TODO:使用 xQueueSend 来将下一步坐标存入队列

}

void ListeningInit()
{
    // Register the callback function so that the CF can receive packets as well.
    p2pRegisterCB(P2PCallbackHandler);
}

bool sendMappingRequest(mapping_req_payload_t* mappingRequestPayloadPtr, uint8_t mappingRequestPayloadLength, uint16_t mappingRequestSeq) 
{
    // Initialize the p2p packet
    static P2PPacket packet;
    packet.port = 0x00;
    uint8_t sourceId = getSourceId();
    uint8_t destinationId = UAV_COMPUTING_ID;
    // Assemble the packet
    mapping_req_packet_t mappingReqPacket;
    mappingReqPacket.sourceId = sourceId;
    mappingReqPacket.destinationId = destinationId;
    mappingReqPacket.packetType = MAPPING_REQ;
    mappingReqPacket.seq = mappingRequestSeq;
    mappingReqPacket.mappingRequestPayloadLength = mappingRequestPayloadLength;
    memcpy(&mappingReqPacket.mappingRequestPayload, mappingRequestPayloadPtr, sizeof(mapping_req_payload_t)*mappingRequestPayloadLength);

    memcpy(&packet.data, &mappingReqPacket, sizeof(mapping_req_packet_t));
    packet.size = sizeof(mapping_req_packet_t);
    // Send the P2P packet
    return radiolinkSendP2PPacketBroadcast(&packet);
}

bool sendExploreRequest(explore_req_payload_t* exploreRequestPayloadPtr, uint16_t exploreRequestSeq)
{
    // Initialize the p2p packet
    static P2PPacket packet;
    packet.port = 0x00;
    uint8_t sourceId = getSourceId();
    uint8_t destinationId = UAV_COMPUTING_ID;
    // Assemble the packet
    explore_req_packet_t exploreReqPacket;
    exploreReqPacket.sourceId = sourceId;
    exploreReqPacket.destinationId = destinationId;
    exploreReqPacket.packetType = EXPLORE_REQ;
    exploreReqPacket.seq = exploreRequestSeq;
    memcpy(&exploreReqPacket.exploreRequestPayload, exploreRequestPayloadPtr, sizeof(explore_req_payload_t));

    memcpy(&packet.data, &exploreReqPacket, sizeof(explore_req_packet_t));
    packet.size = sizeof(explore_req_packet_t);
    // Send the P2P packet
    return radiolinkSendP2PPacketBroadcast(&packet);
}
