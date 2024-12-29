#include <string.h>
#include "stdlib.h"

#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include "system.h"
#include "app.h"

#include "cpx_internal_router.h"

#include "communicate.h"

static CPXPacket_t cpxRx;


void appMain()
{
    #ifdef ENABLE_CPX
        systemWaitStart();
    #endif
    // communicateInit();
    DEBUG_PRINT("init success\n");
    #ifdef ENABLE_CPX
        while(1) {
            cpxInternalRouterReceiveOthers(&cpxRx);
            uint8_t respType = cpxRx.data[2];
            if(respType == EXPLORE_RESP || respType == PATH_RESP || respType == CLUSTER_RESP){
                Autofly_packet_t autoflyPacket;
                memcpy(&autoflyPacket, cpxRx.data, cpxRx.dataLength);
                bool flag = sendAutoflyPacket(&autoflyPacket);
                DEBUG_PRINT("[Edge-STM32]P2P:response %s\n\n", flag == false ? "timeout" : "success");
            }
            vTaskDelay(M2T(100));
        }
    #else
        while (1)
        {
            vTaskDelay(M2T(100));
        }
    #endif

}
