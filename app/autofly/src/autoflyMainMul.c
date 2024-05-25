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

#define MAPPING_TASK_NAME "MappingTask"
#define MAPPING_TASK_STACK_SIZE 2048
#define MAPPING_TASK_PRI 5

#define FLYING_TASK_NAME "FlyingTask"
#define FLYING_TASK_STACK_SIZE 2048
#define FLYING_TASK_PRI 5

static bool octotree_Flying = false;
static bool octotree_Print = false;
static bool flag_Terminate = false;
// static bool data_lock = false;
uint16_t seqnumber = 0;  //SGL

//MUL
explore_req_packet_t exploreReqPacket;
mapping_req_packet_t mappingReqPacket;
/*
make cload:
    CLOAD_CMDS="-w radio://0/86/2M/86E7E7E7E7" make cload
*/

/*

*/

#define PATH_LENGTH 100
static coordinateF_t path[PATH_LENGTH] = {{25,25,25},{25,25,29},{29,25,29},{29,25,25},{29,29,25},{29,29,29},{25,29,29},{25,33,29},{25,37,29},{29,37,29},{29,33,29},{29,33,25},{29,37,25},{29,41,25},{29,45,25},{25,45,25},{25,41,25},{25,41,29},{25,45,29},{25,49,29},{25,53,29},{29,53,29},{29,49,29},{29,49,25},{29,53,25},{29,57,25},{29,61,25},{25,61,25},{25,57,25},{25,57,29},{25,61,29},{25,65,29},{25,69,29},{25,69,25},{25,65,25},{29,65,25},{29,69,25},{29,73,25},{29,77,25},{29,77,29},{29,73,29},{25,73,29},{25,77,29},{25,77,33},{25,77,37},{29,77,37},{29,77,33},{29,73,33},{29,73,37},{33,73,37},{37,73,37},{37,73,33},{33,73,33},{33,77,33},{37,77,33},{41,77,33},{45,77,33},{45,77,37},{41,77,37},{41,73,37},{45,73,37},{49,73,37},{53,73,37},{53,73,33},{49,73,33},{49,77,33},{53,77,33},{57,77,33},{61,77,33},{61,77,37},{57,77,37},{57,73,37},{61,73,37},{65,73,37},{69,73,37},{69,73,33},{65,73,33},{65,77,33},{69,77,33},{73,77,33},{73,77,37},{73,81,37},{77,81,37},{77,81,33}
,{77,85,33},{77,85,37},{73,85,37},{73,85,41},{73,85,45}};

static void mappingTask(void *parameters){
    while (true)
    {
        if(!flag_Terminate){
            coordinateF_t item_point = uavRange->current_point;
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
        }
        vTaskDelay(M2T(MAPPING_DIF));
    }
}

static void flyingTask(void *parameters){
    int index = 0;
    while (true)
    {
        if(octotree_Flying && index < PATH_LENGTH){
            MoveToNext(&uavRange->current_point,&path[index]);
            index++;
        }
        vTaskDelay(M2T(LOOP_DELAY));
    }
}

void appMain()
{
    // DEBUG_PRINT("appMain start\n");
    vTaskDelay(M2T(10000));
    mappingReqPacket.seq = 0;
    mappingReqPacket.mappingRequestPayload->len = 0;
    exploreReqPacket.seq = 0;
    CommunicateInit();
    DEBUG_PRINT("init success\n");
    xTaskCreate(mappingTask, MAPPING_TASK_NAME, MAPPING_TASK_STACK_SIZE, NULL, MAPPING_TASK_PRI, NULL);
    xTaskCreate(flyingTask, FLYING_TASK_NAME, FLYING_TASK_STACK_SIZE, NULL, FLYING_TASK_PRI, NULL);
}

PARAM_GROUP_START(octotree)
PARAM_ADD(PARAM_UINT8, octotree_Flying, &octotree_Flying)
PARAM_ADD(PARAM_UINT8, octotree_Print, &octotree_Print)
PARAM_ADD(PARAM_UINT8, flag_Terminate, &flag_Terminate)
PARAM_GROUP_STOP(octotree)