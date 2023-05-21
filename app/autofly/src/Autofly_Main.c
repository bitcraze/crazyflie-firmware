// #include "pmsis.h"

#include "stdlib.h"
#include "stdbool.h"
#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include "app.h"
#include "param.h"

#include "config_autofly.h"
#include "control_tool.h"
#include "auxiliary_tool.h"
#include "octoMap.h"
#include "communicate.h"

#define MODEL 0  // 0:SGL,1:MUL,2:TEST

static bool octotree_Flying = false;
static bool octotree_Print = false;
bool flag_Terminate = false;
// static bool data_lock = false;
uint16_t seqnumber = 0;  //SGL

//MUL
explore_req_packet_t exploreReqPacket;
mapping_req_packet_t mappingReqPacket;
/*
make cload:
    CLOAD_CMDS="-w radio://0/86/2M/86E7E7E7E7" make cload
*/

void appMain()
{
    // DEBUG_PRINT("appMain start\n");
    vTaskDelay(M2T(10000));
    extern uavRange_t uavRange;
    coordinateF_t item_point;
    uavControl_t* uavControl;
    TickType_t last_time;
    octoMap_t *octoMap;
    if(MODEL){
        mappingReqPacket.seq = 0;
        mappingReqPacket.mappingRequestPayload->len = 0;
        exploreReqPacket.seq = 0;
        CommunicateInit();
        last_time = xTaskGetTickCount();
    }
    else{
        seqnumber = 0;
        octoMap = (octoMap_t *)malloc(sizeof(octoMap_t));
        octoMapInit(octoMap);
        uavControl = (uavControl_t *)malloc(sizeof(uavControl_t));
        inituavControl(uavControl);
    }
    DEBUG_PRINT("init success\n");
    while (1)
    {
        vTaskDelay(M2T(LOOP_DELAY));
        if(MODEL >= 1){
            // MUL
            if(flag_Terminate){
                // Send termination message at 2Hz
                if(xTaskGetTickCount() - last_time > M2T(TERMINATE_DIF)){
                    sendTerminate();
                    last_time = xTaskGetTickCount();
                }
                continue;
            }

            if(xTaskGetTickCount() - last_time > M2T(MAPPING_DIF)){
                item_point = uavRange.current_point;
                GetRange(&uavRange);
                if(MODEL == 1){
                    if(IsSameCell(&item_point,&uavRange.current_point)){
                        mappingReqPacket.mappingRequestPayload->mergedNums++;
                    }
                    else{
                        sendMappingRequest(&mappingReqPacket);
                        generateMappingReqPacket(&mappingReqPacket);
                    }

                    if(mappingReqPacket.mappingRequestPayload->mergedNums >= 3){
                        sendMappingRequest(&mappingReqPacket);
                        generateMappingReqPacket(&mappingReqPacket);
                    }
                }
                else if(MODEL == 2){
                    // Test packet loss rate using mapping messages
                    sendMappingRequest(&mappingReqPacket);
                    generateMappingReqPacket(&mappingReqPacket);
                }
                last_time = xTaskGetTickCount();
            }

            if(MODEL == 1 && xTaskGetTickCount() - last_time > M2T(EXPLORE_DIF)){
                generateExploreReqPacket(&exploreReqPacket);
                sendExploreRequest(&exploreReqPacket);
                last_time = xTaskGetTickCount();
            }

            if(exploreReqPacket.seq >=  EXPLORE_MAX || mappingReqPacket.seq >= MAPPING_MAX){
                sendTerminate();
                Land();
                flag_Terminate = true;
                CommunicateTerminate();
            }

        }
        else{
            // SGL
            if(octotree_Flying){
                GetRange(&uavControl->uavRange);
                ++seqnumber;
                DEBUG_PRINT("[app]SP:(%.2f,%.2f,%.2f),seq:%d\n", (double)uavControl->uavRange.current_point.x, (double)uavControl->uavRange.current_point.y, (double)uavControl->uavRange.current_point.z, seqnumber);
                UpdateMap(octoMap, &uavControl->uavRange);
                if(seqnumber % MAXRUN == 0){
                    octotree_Flying = false;
                    Land();
                }
                if(CalNextPoint(uavControl, octoMap))
                    MoveToNext(&uavControl->uavRange.current_point, &uavControl->next_point);
                else{
                    octotree_Flying = false;
                    Land();
                    DEBUG_PRINT("No next point\n");
                }
            }
            if (octotree_Print)
            {
                octotree_Flying = false;
                Land();
                DEBUG_PRINT("start to print the octotree\n");
                recursiveExportOctoMap(octoMap, octoMap->octoTree->root, octoMap->octoTree->origin, octoMap->octoTree->width);
                DEBUG_PRINT("print the octotree end\n");
                octotree_Print = false;
                // printOctomap(&octoMap);
            }
        }
        
    }
}

PARAM_GROUP_START(octotree)
PARAM_ADD(PARAM_UINT8, octotree_Flying, &octotree_Flying)
PARAM_ADD(PARAM_UINT8, octotree_Print, &octotree_Print)
PARAM_GROUP_STOP(octotree)