#include <string.h>
#include "stdlib.h"

#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include "radiolink.h"
#include "configblock.h"

#include "system.h"
#include "app.h"

#include "communicate.h"

void P2PCallbackHandler(P2PPacket *p){
    uint8_t rssi = p->rssi;
    Autofly_packet_t* autoflyPacket = (Autofly_packet_t*)p->data;
    uint8_t destinationId = autoflyPacket->destinationId;
    // DEBUG_PRINT("P2P: Received packet from %d\n",autoflyPacket->sourceId);
    if (destinationId != AIDECK_ID)
    {
        return;
    }
    if(autoflyPacket->packetType == MAPPING_REQ){
        mapping_req_packet_t *mappingReqPacket = (mapping_req_packet_t*)autoflyPacket->data;
        mapping_req_payload_t *mappingReqPayload = (mapping_req_payload_t*)mappingReqPacket->mappingRequestPayload;
        DEBUG_PRINT("(%d,%d,%d) ", mappingReqPayload->startPoint.x, mappingReqPayload->startPoint.y, mappingReqPayload->startPoint.z);
        for(int i = 0; i < mappingReqPayload->len; i++){
            DEBUG_PRINT("(%d,%d,%d) ", mappingReqPayload->endPoint[i].x, mappingReqPayload->endPoint[i].y, mappingReqPayload->endPoint[i].z);
        }
        DEBUG_PRINT("\n");
    }
}

void appMain()
{
    p2pRegisterCB(P2PCallbackHandler);
    DEBUG_PRINT("init success\n");
    while (1)
    {
        vTaskDelay(M2T(1000));
    }
}
