#include "stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "log.h"
#include "debug.h"
#include <stdint.h>

#include "config_autofly.h"
#include "crtp_commander_high_level.h"

#include "control_tool.h"
#include "auxiliary_tool.h"
#include "octoMap.h"
#include "octoTree.h"

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

void MoveToNext(coordinateF_t* cur,coordinateF_t* next)
{   
    float d = sqrt(pow(cur->x - next->x, 2) + pow(cur->y - next->y, 2) + pow(cur->z - next->z, 2));
    float time = d/100/SPEED;
    // DEBUG_PRINT("time:%f\n",(double)time);
    crtpCommanderHighLevelGoTo((next->x - OFFSET_X) / 100, (next->y - OFFSET_Y) / 100, (next->z - OFFSET_Z) / 100, 0, time, 0);
    // vTaskDelay(M2T(time*1000 + WAIT_DELAY));
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

void Land(){
    vTaskDelay(M2T(WAIT_DELAY));
    crtpCommanderHighLevelLand(0, 1);
    vTaskDelay(M2T(500 + WAIT_DELAY));
}