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
    coordinateF_t item_point;
    uavControl_t* uavControl;
    // uavRange_t* uavRange;
    TickType_t last_time_mapping, last_time_explore, last_time_terminate;
    octoMap_t *octoMap;
    if(MODEL>=1){
        mappingReqPacket.seq = 0;
        mappingReqPacket.mappingRequestPayload->len = 0;
        exploreReqPacket.seq = 0;
        CommunicateInit();
        last_time_mapping = xTaskGetTickCount();
        last_time_explore = xTaskGetTickCount();
        last_time_terminate = xTaskGetTickCount();
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
        if(MODEL == 0){
            // SGL
            if(octotree_Flying){
                GetRange(&uavControl->uavRange);
                ++seqnumber;
                DEBUG_PRINT("[app]SP:(%.2f,%.2f,%.2f),seq:%d\n", (double)uavControl->uavRange.current_point.x, (double)uavControl->uavRange.current_point.y, (double)uavControl->uavRange.current_point.z, seqnumber);
                DEBUG_PRINT("[app]M:(F:%.2f, B:%.2f, L:%.2f, R:%.2f, U:%.2f, D:%.2f, p:%.2f, r:%.2f, y:%.2f),seq:%d\n",(double)uavControl->uavRange.measurement.data[0],(double)uavControl->uavRange.measurement.data[1],
                                                                                                                (double)uavControl->uavRange.measurement.data[2],(double)uavControl->uavRange.measurement.data[3],
                                                                                                                (double)uavControl->uavRange.measurement.data[4],(double)uavControl->uavRange.measurement.data[5],
                                                                                                                (double)uavControl->uavRange.measurement.pitch,(double)uavControl->uavRange.measurement.roll,
                                                                                                                (double)uavControl->uavRange.measurement.yaw,seqnumber);
                UpdateMap(octoMap, &uavControl->uavRange);
                DEBUG_PRINT("[app]NN:%d,%d,%d,%d,%d,seq:%d\n",octoMap->octoNodeSet->length,
                                                        octoMap->octoNodeSet->numFree,octoMap->octoNodeSet->numOccupied,
                                                        octoMap->octoNodeSet->volumeFree,octoMap->octoNodeSet->volumeOccupied,
                                                        seqnumber);
                if(seqnumber % MAXRUN == 0){
                    octotree_Flying = false;
                    Land();
                    DEBUG_PRINT("Flying Finished\n");
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
        else if(MODEL == 1){
            // MUL
            if(flag_Terminate){
                // Send termination message at 2Hz
                if(xTaskGetTickCount() - last_time_terminate > M2T(TERMINATE_DIF)){
                    sendTerminate();
                    last_time_terminate = xTaskGetTickCount();
                }
                continue;
            }

            if(xTaskGetTickCount() - last_time_mapping > M2T(MAPPING_DIF)){
                item_point = uavRange->current_point;
                GetRange(uavRange);
                if(IsSameCell(&item_point,&uavRange->current_point)){
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
                last_time_mapping = xTaskGetTickCount();
            }

            if(xTaskGetTickCount() - last_time_explore > M2T(EXPLORE_DIF)){
                GetRange(uavRange);
                generateExploreReqPacket(&exploreReqPacket);
                sendExploreRequest(&exploreReqPacket);
                last_time_explore = xTaskGetTickCount();
            }

            DEBUG_PRINT("exploreRequestSeq:%d, mappingRequestSeq:%d\n",exploreRequestSeq,mappingRequestSeq);
            if(exploreRequestSeq >=  EXPLORE_MAX || mappingRequestSeq >= MAPPING_MAX){
                sendTerminate();
                last_time_terminate = xTaskGetTickCount();
                Land();
                flag_Terminate = true;
                CommunicateTerminate();
            }
        }
        else if(MODEL == 2){
            // Just test packet loss rate using mapping messages
            GetRange(uavRange);
            if(flag_Terminate){
                // Send termination message at 2Hz
                if(xTaskGetTickCount() - last_time_terminate > M2T(TERMINATE_DIF)){
                    sendTerminate();
                    last_time_terminate = xTaskGetTickCount();
                }
                continue;
            }
            if(xTaskGetTickCount() - last_time_mapping > M2T(MAPPING_DIF)){
                sendMappingRequest(&mappingReqPacket);
                generateMappingReqPacket(&mappingReqPacket);
                last_time_mapping = xTaskGetTickCount();
            }
            
            if(exploreReqPacket.seq >=  EXPLORE_MAX || mappingReqPacket.seq >= MAPPING_MAX){
                sendTerminate();
                last_time_terminate = xTaskGetTickCount();
                Land();
                flag_Terminate = true;
                CommunicateTerminate();
            }
        }
    }
}

PARAM_GROUP_START(octotree)
PARAM_ADD(PARAM_UINT8, octotree_Flying, &octotree_Flying)
PARAM_ADD(PARAM_UINT8, octotree_Print, &octotree_Print)
PARAM_GROUP_STOP(octotree)