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


void P2PCallbackHandler(P2PPacket *p)
{
    // Parse the P2P packet
    uint8_t rssi = p->rssi;
    uint8_t destinationId = p->data[1];
    if (destinationId != UAV_COMPUTING_ID)
    {
        return;
    }
    else{
        // Send msg to GAP8
        CPXPacket_t cpxPacket;
        cpxInitRoute(CPX_T_STM32, CPX_T_GAP8, CPX_F_APP, &cpxPacket.route);
        memcpy(cpxPacket.data, p->data, p->size);
        cpxPacket.dataLength = p->size;
        bool flag = cpxSendPacketBlockingTimeout(&cpxPacket, 1000);
        DEBUG_PRINT("[Edge-STM32]CPX: Forward mapping request %s\n\n", flag == false ? "timeout" : "success");
    }
}

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

void CPXForwardInit() {
    p2pRegisterCB(P2PCallbackHandler);
    DEBUG_PRINT("[Edge-STM32]CPX Forward Init...\n");
    cpxInternalRouterInit();
    cpxExternalRouterInit();
}
