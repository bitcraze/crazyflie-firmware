// #include "pmsis.h"

#include "stdlib.h"
#include "stdbool.h"
#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include "app.h"
#include "range.h"
#include "log.h"
#include "param.h"
#include "crtp_commander_high_level.h"

#include "config_autofly.h"
#include "octoMap.h"
#include "octoTree.h"
#include "octoNodeSet.h"
#include "octoNode.h"
#include "rrtConnect.h"
#include "circularQueue.h"
#include "auxiliary_tool.h"
#include "coordinateQueue.h"

#define MODEL 0  // 0:SGL,1:MUL
#define WAIT_DELAY 300
#define LOOP_DELAY 100
#define PROBABILITY_MEM(octomap) (double)octomap->octoNodeSet->length / NODE_SET_SIZE
#define PRESENT_JUMP 2
#define SPEED 0.25
typedef struct uavRange_t
{
    example_measure_t measurement;
    coordinateF_t current_point;
}uavRange_t;

typedef struct uavControl_t
{
    // CoordinateQueue_t paths;
    coordinateF_t next_point;
    uavRange_t uavRange;

    float direction_weight[6];
    short lastdir;
    
    Queue_t queue;
    short loops[WINDOW_SIZE];
    
    bool flag_jump;
    rangeDirection_t Jump_Dir;
}uavControl_t;

static bool octotree_Flying = false;
static bool octotree_Print = false;
// static bool data_lock = false;
int seqnumber = 0;

/*
make cload:
    CLOAD_CMDS="-w radio://0/86/2M/86E7E7E7E7" make cload
*/
void inituavRange(uavRange_t* uavRange);
void inituavControl(uavControl_t* uavControl);
void GetRange(uavRange_t* uavRange);
void UpdateMap(octoMap_t *octoMap, uavRange_t* uavRange);
void CalCandidates(coordinateF_t *candidates, example_measure_t *measurement, coordinateF_t *current_F);
rangeDirection_t GetRandomDir(example_measure_t *measurement);
bool CalBestCandinates(octoMap_t *octoMap,uavControl_t* uavControl_t);
bool JumpLocalOp(uavControl_t *uavControl);
void printOctomap(octoMap_t *octoMap);
void MoveToNext(coordinateF_t* cur,coordinateF_t* next);
bool ReliabilityTest(coordinateF_t* last, coordinateF_t* cur);
bool CalNextPoint(uavControl_t* uavControl,octoMap_t* octoMap);

void inituavRange(uavRange_t* uavRange){
    uavRange->measurement.data[0] = 0;
    uavRange->measurement.data[1] = 0;
    uavRange->measurement.data[2] = 0;
    uavRange->measurement.data[3] = 0;
    uavRange->measurement.data[4] = 0;
    uavRange->measurement.data[5] = 0;
    uavRange->measurement.roll = 0;
    uavRange->measurement.pitch = 0;
    uavRange->measurement.yaw = 0;
    uavRange->current_point.x = 0;
    uavRange->current_point.y = 0;
    uavRange->current_point.z = 0;
}

void inituavControl(uavControl_t* uavControl){
    // initCoordinateQueue(&uavControl->paths);
    uavControl->next_point.x = 0;
    uavControl->next_point.y = 0;
    uavControl->next_point.z = 0;
    inituavRange(&uavControl->uavRange);
    for(int i = 0; i < 6; ++i){
        uavControl->direction_weight[i] = 1;
    }
    uavControl->lastdir = 0;
    initQueue(&uavControl->queue);
    for(int i = 0; i < WINDOW_SIZE; ++i){
        uavControl->loops[i] = 0;
    }
    uavControl->flag_jump = false;
    uavControl->Jump_Dir = rangeFront;
}

void GetRange(uavRange_t* uavRange){
    coordinateF_t item_pointF;
    while (1)
    {
        item_pointF.x = 100 * logGetFloat(logGetVarId("stateEstimate", "x")) + OFFSET_X;
        item_pointF.y = 100 * logGetFloat(logGetVarId("stateEstimate", "y")) + OFFSET_Y;
        item_pointF.z = 100 * logGetFloat(logGetVarId("stateEstimate", "z")) + OFFSET_Z;
        if(ReliabilityTest(&uavRange->current_point,&item_pointF)){
            uavRange->current_point = item_pointF;
            break;
        }
        else{
            DEBUG_PRINT("ReliabilityTest failed,Failed Point:(%.2f,%.2f,%.2f)\n",(double)item_pointF.x,(double)item_pointF.y,(double)item_pointF.z);
            vTaskDelay(M2T(WAIT_DELAY));   
        }
    }
    get_measurement(&uavRange->measurement);
    if (uavRange->current_point.z < TOP)
        uavRange->measurement.data[4] = TOP - uavRange->current_point.z;
    else
        uavRange->measurement.data[4] = 0;
    if (uavRange->current_point.z > BOTTOM)
        uavRange->measurement.data[5] = uavRange->current_point.z - BOTTOM;
    else
        uavRange->measurement.data[5] = 0;
}

void UpdateMap(octoMap_t *octoMap, uavRange_t* uavRange)
{
    coordinate_t current_I,end_point;
    coordinateF_t item_end;

    current_I.x = uavRange->current_point.x;
    current_I.y = uavRange->current_point.y;
    current_I.z = uavRange->current_point.z;
    for (rangeDirection_t dir = rangeFront; dir <= rangeDown; ++dir)
    {
        if (cal_Point(&uavRange->measurement, &uavRange->current_point, dir, &item_end))
        {
            end_point.x = item_end.x;
            end_point.y = item_end.y;
            end_point.z = item_end.z;
            octoTreeRayCasting(octoMap->octoTree, octoMap, &current_I, &end_point);
        }
    }
}

void CalCandidates(coordinateF_t *candidates, example_measure_t *measurement, coordinateF_t *current_F)
{
    float pitch = -1 * measurement->pitch;
    float roll = measurement->roll;
    float yaw = measurement->yaw;
    for (rangeDirection_t dir = rangeFront; dir <= rangeDown; ++dir)
    {
        if (measurement->data[dir] > AVOID_DISTANCE + STRIDE)
        {
            cal_PointByLength(STRIDE, pitch, roll, yaw, current_F, dir, &candidates[dir]);
        }
        else
        {
            candidates[dir].x = 30000;
            candidates[dir].y = 30000;
            candidates[dir].z = 30000;
        }
    }
}

rangeDirection_t GetRandomDir(example_measure_t *measurement)
{
    // Randomly sample twice to choose the larger
    rangeDirection_t dir = (rangeDirection_t)rand() % 6;
    rangeDirection_t maxdir = (rangeDirection_t)rand() % 6;
    int i = 0;
    // Guaranteed to get a feasible direction
    while (measurement->data[maxdir] < STRIDE + AVOID_DISTANCE && i < 20)
    {
        maxdir = (rangeDirection_t)rand() % 6;
        ++i;
    }
    // Try to get a better and feasible direction
    dir = (rangeDirection_t)rand() % 6;
    ++i;
    if (i == 20)
        return -1;
    if (measurement->data[dir] > measurement->data[maxdir])
        maxdir = dir;
    return maxdir;
}

bool CalBestCandinates(octoMap_t *octoMap,uavControl_t* uavControl){
    coordinateF_t candinates[6];
    coordinate_t item_point;
    double item_candinateCost = 0, max_candinateCost = 0;
    Cost_C_t item_sum,item_cost;
    CalCandidates(candinates, &uavControl->uavRange.measurement, &uavControl->uavRange.current_point);
    max_candinateCost = 0;
    short dir_next = -1;
    for(int i = 0;i<6;++i){
        item_candinateCost = 0;
        item_sum.cost_prune = 0;
        item_sum.income_info = 0;
        if (candinates[i].x == 30000 && candinates[i].y == 30000 && candinates[i].z == 30000)
        {
            continue;
        }
        for (rangeDirection_t dir = rangeFront; dir <= rangeDown; ++dir)
        {
            item_point.x = candinates[i].x;
            item_point.y = candinates[i].y;
            item_point.z = candinates[i].z;
            item_cost = Cost_Sum(octoMap->octoTree, octoMap, &item_point, dir);
            item_sum.cost_prune += item_cost.cost_prune;
            item_sum.income_info += item_cost.income_info;
        }
        if (item_sum.income_info == 0)
        {
            item_sum.income_info = DISCIPLINE;
        }
        item_candinateCost = (double)uavControl->direction_weight[i] * (PROBABILITY_MEM(octoMap) * item_sum.cost_prune * COST_PRUNE_TIMES +
                                                    (1.0 - PROBABILITY_MEM(octoMap)) * item_sum.income_info * INCOME_INFO_TIMES);
        if (item_candinateCost > max_candinateCost){
            dir_next = i;
            max_candinateCost = item_candinateCost;
        }
    }
    if(dir_next != -1){
        uavControl->direction_weight[dir_next] = DIRECTION_AWARD;
        uavControl->direction_weight[(uavControl->lastdir)] = 1;
        (uavControl->lastdir) = dir_next;
        uavControl->next_point = candinates[dir_next];
        return true;
    }
    else{
        DEBUG_PRINT("no next point\n");
        return false;
    }
}

bool JumpLocalOp(uavControl_t *uavControl){
    // rangeDirection_t dir = rand()%6;
    float length = fmin(uavControl->uavRange.measurement.data[uavControl->Jump_Dir],300);
    // coordinateF_t item_start_point = {current_point->x,current_point->y,current_point->z};
    coordinateF_t item_end_point;
    if(length > STRIDE + AVOID_DISTANCE){
        // cal_PointByLength(STRIDE, -1 * measurement->pitch, measurement->roll, measurement->yaw, &item_start_point, Jump_Dir, &item_end_point);
        cal_PointByLength(STRIDE, -1 * uavControl->uavRange.measurement.pitch, uavControl->uavRange.measurement.roll, uavControl->uavRange.measurement.yaw, &uavControl->uavRange.current_point, uavControl->Jump_Dir, &item_end_point);
        uavControl->next_point = item_end_point;
        if(length < STRIDE * PRESENT_JUMP + AVOID_DISTANCE){
            uavControl->flag_jump = false;
        }
        return true;
    }
    else{
        uavControl->flag_jump = false;
        return false;
    }
}

void printOctomap(octoMap_t* octoMap){
    int Free = 0;
    int Occupied = 0;
    for(int i=0;i<NODE_SET_SIZE;++i){
        for(int j=0;j<8;j++){
            if(octoMap->octoNodeSet->setData[i].data[j].isLeaf){
                if(octoMap->octoNodeSet->setData[i].data[j].logOdds == LOG_ODDS_OCCUPIED){
                    ++Occupied;
                }
                else if(octoMap->octoNodeSet->setData[i].data[j].logOdds == LOG_ODDS_FREE){
                    ++Free;
                }
            }
        }
    }
    DEBUG_PRINT("Free:%d,Occupied:%d\n",Free,Occupied);
}

void MoveToNext(coordinateF_t* cur,coordinateF_t* next)
{   
    float d = sqrt(pow(cur->x - next->x, 2) + pow(cur->y - next->y, 2) + pow(cur->z - next->z, 2));
    float time = d/100/SPEED;
    DEBUG_PRINT("time:%f\n",(double)time);
    crtpCommanderHighLevelGoTo((next->x - OFFSET_X) / 100, (next->y - OFFSET_Y) / 100, (next->z - OFFSET_Z) / 100, 0, time, 0);
    vTaskDelay(M2T(time*1000 + WAIT_DELAY));
}

bool CalNextPoint(uavControl_t* uavControl,octoMap_t* octoMap){
    short index_loop = (((int)uavControl->uavRange.current_point.x + (int)uavControl->uavRange.current_point.y + (int)uavControl->uavRange.current_point.z) / TREE_RESOLUTION) % WINDOW_SIZE;;
    ++uavControl->loops[index_loop];
    if (uavControl->loops[index_loop] < MAX_LOOP)
    {
        push(&uavControl->queue, index_loop);
        if (uavControl->queue.len >= WINDOW_SIZE)
        {
            index_loop = pop(&uavControl->queue);
            --uavControl->loops[index_loop];
        }
        if(!uavControl->flag_jump && !CalBestCandinates(octoMap, uavControl)){
            uavControl->Jump_Dir = GetRandomDir(&uavControl->uavRange.measurement);
            if(uavControl->Jump_Dir == -1){
                DEBUG_PRINT("no next dir\n");
                return false;
            }
            uavControl->flag_jump = true;
        }
    }
    else
    {
        initQueue(&uavControl->queue);
        for (int i = 0; i < WINDOW_SIZE; ++i)
        {
            uavControl->loops[i] = 0;
        }
        uavControl->Jump_Dir = GetRandomDir(&uavControl->uavRange.measurement);
        if(uavControl->Jump_Dir == -1){
            DEBUG_PRINT("no next dir\n");
            return false;
        }
        uavControl->flag_jump = true;
    }
    if(uavControl->flag_jump){
        DEBUG_PRINT("Jumping......,jump_dir:%d\n",uavControl->Jump_Dir);
        if(!JumpLocalOp(uavControl)){
            --uavControl->loops[index_loop];
            return CalNextPoint(uavControl, octoMap);
        }
    }
    return true;
}

bool ReliabilityTest(coordinateF_t* last, coordinateF_t* cur){
   static bool first = true;
    if( (abs(last->x-cur->x)+abs(last->y-cur->y)+abs(last->z-cur->z) < RELIABILITY_DISTANCE || first)
        && (abs(cur->x) <= WIDTH_X)
        && (abs(cur->y) <= WIDTH_Y)
        && (abs(cur->z) <= WIDTH_Z)){
            first = false;
            return true;
    }
    return false;
}

void appMain()
{
    // DEBUG_PRINT("appMain start\n");
    vTaskDelay(M2T(10000));
    uavRange_t* uavRange;
    uavControl_t* uavControl;
    octoMap_t *octoMap;
    if(MODEL){
        uavRange = (uavRange_t *)malloc(sizeof(uavRange_t));
        inituavRange(uavRange);
    }
    else{
        octoMap = (octoMap_t *)malloc(sizeof(octoMap_t));
        octoMapInit(octoMap);
        uavControl = (uavControl_t *)malloc(sizeof(uavControl_t));
        inituavControl(uavControl);
    }
    DEBUG_PRINT("init success\n");
    while (1)
    {
        vTaskDelay(M2T(LOOP_DELAY));
        if(MODEL){
            GetRange(uavRange);
            ++seqnumber;
            vTaskDelay(M2T(WAIT_DELAY));
        }
        else{
            if(octotree_Flying){
                GetRange(&uavControl->uavRange);
                ++seqnumber;
                DEBUG_PRINT("[app]SP:(%.2f,%.2f,%.2f),seq:%d\n", (double)uavControl->uavRange.current_point.x, (double)uavControl->uavRange.current_point.y, (double)uavControl->uavRange.current_point.z, seqnumber);
                UpdateMap(octoMap, &uavControl->uavRange);
                if(seqnumber % MAXRUN == 0){
                    vTaskDelay(M2T(WAIT_DELAY));
                    octotree_Flying = false;
                    crtpCommanderHighLevelLand(0, 0.5);
                    vTaskDelay(M2T(500 + WAIT_DELAY));
                }
                if(CalNextPoint(uavControl, octoMap))
                    MoveToNext(&uavControl->uavRange.current_point, &uavControl->next_point);
                else{
                    octotree_Flying = false;
                    vTaskDelay(M2T(WAIT_DELAY));
                    crtpCommanderHighLevelLand(0, 0.5);
                    vTaskDelay(M2T(500 + WAIT_DELAY));
                    DEBUG_PRINT("No next point\n");
                }
            }
            if (octotree_Print)
            {
                octotree_Flying = false;
                vTaskDelay(M2T(WAIT_DELAY));
                crtpCommanderHighLevelLand(0, 0.5);
                vTaskDelay(500 + M2T(WAIT_DELAY));
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