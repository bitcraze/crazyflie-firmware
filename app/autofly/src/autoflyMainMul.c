// #include "pmsis.h"

#include <stdlib.h>
#include <stdbool.h>
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
#include "mappingCommunication.h"
#include "exploreCommunication.h"

#define MAPPING_TASK_NAME "MappingTask"
#define MAPPING_TASK_STACK_SIZE 2048
#define MAPPING_TASK_PRI 5

#define FLYING_TASK_NAME "FlyingTask"
#define FLYING_TASK_STACK_SIZE 2048
#define FLYING_TASK_PRI 5

static bool octotree_Flying = false;
static bool octotree_Print = false;
static bool flag_Terminate = false;
static bool hasLanded = false;
// static bool data_lock = false;
uint16_t seqnumber = 0;  //SGL

//MUL
explore_req_packet_t exploreReqPacket;
mapping_req_packet_t mappingReqPacket;
/*
make cload:
    CLOAD_CMDS="-w radio://0/86/2M/86E7E7E7E7" make cload
*/

static uint8_t sourceId = 0;

#define PATH_LENGTH 200
static coordinateF_t path1[PATH_LENGTH];
static coordinateF_t path2[PATH_LENGTH];
static coordinateF_t path3[PATH_LENGTH];

static void mappingTask(void *parameters){
    uavRange_t* uavRange = NULL;
    getUavRange(uavRange);
    while (true)
    {
        if(!flag_Terminate && octotree_Flying){
            coordinateF_t item_point = uavRange->current_point;
            GetRange(uavRange);
            if(IsSameCell(&item_point,&uavRange->current_point)){
                mappingReqPacket.mappingRequestPayload->mergedNums++;
            }
            else{
                sendMappingRequest(AIDECK_ID, &mappingReqPacket);
                generateMappingReqPacket(&mappingReqPacket);
            }

            if(mappingReqPacket.mappingRequestPayload->mergedNums >= 3){
                sendMappingRequest(AIDECK_ID,&mappingReqPacket);
                generateMappingReqPacket(&mappingReqPacket);
            }
        }
        vTaskDelay(M2T(MAPPING_DIF));
    }
}

static void flyingTask(void *parameters){
    int index = 0;
    uavRange_t* uavRange = NULL;
    getUavRange(uavRange);
    while (true)
    {
        if(octotree_Flying && index < PATH_LENGTH){
            if(sourceId == 1){
                DEBUG_PRINT("%d,%d,%d,%d,%d,%d,%d\n",index,(int)uavRange->current_point.x,(int)uavRange->current_point.y,(int)uavRange->current_point.z,(int)path1[index].x,(int)path1[index].y,(int)path1[index].z);
                MoveToNext(&uavRange->current_point,&path1[index]);
            }
            else if(sourceId == 2){
                DEBUG_PRINT("%d,%d,%d,%d,%d,%d,%d\n",index,(int)uavRange->current_point.x,(int)uavRange->current_point.y,(int)uavRange->current_point.z,(int)path2[index].x,(int)path2[index].y,(int)path2[index].z);
                MoveToNext(&uavRange->current_point,&path2[index]);
            }
            else if(sourceId == 3){
                DEBUG_PRINT("%d,%d,%d,%d,%d,%d,%d\n",index,(int)uavRange->current_point.x,(int)uavRange->current_point.y,(int)uavRange->current_point.z,(int)path3[index].x,(int)path3[index].y,(int)path3[index].z);
                MoveToNext(&uavRange->current_point,&path3[index]);
            }
            else{
                DEBUG_PRINT("sourceId error\n");
            }
            index++;
        }
        if(index >= PATH_LENGTH && !hasLanded){
            Land();
            flag_Terminate = true;
            hasLanded = true;
        }
        vTaskDelay(M2T(FLYING_DIF));
    }
    
}

void appMain()
{
    // DEBUG_PRINT("appMain start\n");
    vTaskDelay(M2T(5000));
    sourceId = getSourceId();
    octotree_Flying = true;
    mappingReqPacket.seq = 0;
    mappingReqPacket.mappingRequestPayload->len = 0;
    exploreReqPacket.seq = 0;
    CommunicateInit();
    autoflyControlSystemInit();
    DEBUG_PRINT("init success\n");
    xTaskCreate(mappingTask, MAPPING_TASK_NAME, MAPPING_TASK_STACK_SIZE, NULL, MAPPING_TASK_PRI, NULL);
    // xTaskCreate(flyingTask, FLYING_TASK_NAME, FLYING_TASK_STACK_SIZE, NULL, FLYING_TASK_PRI, NULL);
}

PARAM_GROUP_START(octotree)
PARAM_ADD(PARAM_UINT8, octotree_Flying, &octotree_Flying)
PARAM_ADD(PARAM_UINT8, octotree_Print, &octotree_Print)
PARAM_ADD(PARAM_UINT8, flag_Terminate, &flag_Terminate)
PARAM_GROUP_STOP(octotree)