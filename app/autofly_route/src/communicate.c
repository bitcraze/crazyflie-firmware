#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include "radiolink.h"
#include "configblock.h"

#include "cpx_internal_router.h"
#include "cpx_external_router.h"

#include "communicate.h"

#define DEBUG_PRINT_ENABLED 1


// void P2PCallbackHandler(P2PPacket *p)
// {
//     // Parse the P2P packet
//     uint8_t rssi = p->rssi;
//     Autofly_packet_t* autoflyPacket = (Autofly_packet_t*)p->data;
//     uint8_t destinationId = autoflyPacket->destinationId;
//     DEBUG_PRINT("P2P: Received packet from %d\n",autoflyPacket->sourceId);
//     if (destinationId != AIDECK_ID)
//     {
//         return;
//     }
//     else{
//         // Send msg to GAP8
//         #ifdef ENABLE_CPX
//             CPXPacket_t cpxPacket;
//             cpxInitRoute(CPX_T_STM32, CPX_T_GAP8, CPX_F_APP, &cpxPacket.route);
//             memcpy(cpxPacket.data, p->data, p->size);
//             cpxPacket.dataLength = p->size;
//             bool flag = cpxSendPacketBlockingTimeout(&cpxPacket, 1000);
//             DEBUG_PRINT("[STM32-Edge]CPX: Forward mapping request %s\n\n", flag == false ? "timeout" : "success");
//         #else
//             if(autoflyPacket->packetType == MAPPING_REQ){
//                 mapping_req_packet_t *mappingReqPacket = (mapping_req_packet_t*)autoflyPacket->data;
//                 mapping_req_payload_t *mappingReqPayload = (mapping_req_payload_t*)mappingReqPacket->mappingRequestPayload;
//                 DEBUG_PRINT("(%d,%d,%d) ", mappingReqPayload->startPoint.x, mappingReqPayload->startPoint.y, mappingReqPayload->startPoint.z);
//                 for(int i = 0; i < mappingReqPayload->len; i++){
//                     DEBUG_PRINT("(%d,%d,%d) ", mappingReqPayload->endPoint[i].x, mappingReqPayload->endPoint[i].y, mappingReqPayload->endPoint[i].z);
//                 }
//                 DEBUG_PRINT("\n");
//             }
//         #endif
//     }
// }

bool sendAutoflyPacket(Autofly_packet_t* AutoflyPacket)
{
    // Initialize the p2p packet
    static P2PPacket packet;
    packet.port = 0x00;
    // Assemble the packet
    memcpy(packet.data, AutoflyPacket, AutoflyPacket->length);
    packet.size = AutoflyPacket->length;
    // Send the P2P packet
    return radiolinkSendP2PPacketBroadcast(&packet);
}

void communicateInit() {
    // p2pRegisterCB(P2PCallbackHandler);
    #ifdef ENABLE_CPX
        DEBUG_PRINT("[Edge-STM32]CPX Forward Init...\n");
        cpxInternalRouterInit();
        cpxExternalRouterInit();
    #endif
}
