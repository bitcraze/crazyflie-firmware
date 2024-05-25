#include "stdlib.h"
#include "debug.h"
#include <stdint.h>

#include "log.h"
#include "math.h"
#include "auxiliary_tool.h"
#include "octoMap.h"
#include "octoTree.h"

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

void get_measurement(example_measure_t *measurement)
{
    // distance unit: cm
    measurement->data[0] = logGetFloat(logGetVarId("range", "front")) / 10;
    measurement->data[1] = logGetFloat(logGetVarId("range", "back")) / 10;
    // measurement->up = logGetFloat(logGetVarId("range","up")) / 10;
    measurement->data[2] = logGetFloat(logGetVarId("range", "left")) / 10;
    measurement->data[3] = logGetFloat(logGetVarId("range", "right")) / 10;

    measurement->pitch = logGetFloat(logGetVarId("stabilizer", "pitch"));
    measurement->roll = logGetFloat(logGetVarId("stabilizer", "roll"));
    measurement->yaw = logGetFloat(logGetVarId("stabilizer", "yaw"));
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

double caldistance(coordinate_t *A, coordinate_t *B)
{
    return sqrt(pow(A->x - B->x, 2) + pow(A->y - B->y, 2) + pow(A->z - B->z, 2));
}

bool ReliabilityTest(coordinateF_t* last, coordinateF_t* cur){
    static bool first = true;
    #ifndef ENABLE_RELAIBILITY_TEST
        return true;
    #endif
    if( (abs(last->x-cur->x)+abs(last->y-cur->y)+abs(last->z-cur->z) < RELIABILITY_DISTANCE || first)
        && (abs(cur->x) <= WIDTH_X)
        && (abs(cur->y) <= WIDTH_Y)
        && (abs(cur->z) <= WIDTH_Z)){
            first = false;
            return true;
    }
    return false;
}

bool IsSameCell(coordinateF_t* last, coordinateF_t* cur){
    return ((int)last->x / TREE_RESOLUTION == (int)cur->x / TREE_RESOLUTION) 
            && ((int)last->y / TREE_RESOLUTION == (int)cur->y / TREE_RESOLUTION) 
            && ((int)last->z / TREE_RESOLUTION == (int)cur->z / TREE_RESOLUTION);
}

bool cal_Point(example_measure_t *measurement, coordinateF_t *start_point, rangeDirection_t dir, coordinateF_t *res)
{
    float pitch = -1 * measurement->pitch;
    float roll = measurement->roll;
    float yaw = measurement->yaw;
    switch (dir)
    {
    case rangeFront:
        if (measurement->data[0] < SENSOR_TH && measurement->data[0] > TREE_RESOLUTION)
        {
            coordinateF_t point = {start_point->x + measurement->data[0], start_point->y, start_point->z};
            *res = rot(roll, pitch, yaw, start_point, &point);
            return TRUE;
        }
        break;
    case rangeBack:
        if (measurement->data[1] < SENSOR_TH && measurement->data[1] > TREE_RESOLUTION)
        {
            coordinateF_t point = {start_point->x - measurement->data[1], start_point->y, start_point->z};
            *res = rot(roll, pitch, yaw, start_point, &point);
            return TRUE;
        }
        break;
    case rangeLeft:
        if (measurement->data[2] < SENSOR_TH && measurement->data[2] > TREE_RESOLUTION)
        {
            coordinateF_t point = {start_point->x, start_point->y + measurement->data[2], start_point->z};
            *res = rot(roll, pitch, yaw, start_point, &point);
            return TRUE;
        }
        break;
    case rangeRight:
        if (measurement->data[3] < SENSOR_TH && measurement->data[3] > TREE_RESOLUTION)
        {
            coordinateF_t point = {start_point->x, start_point->y - measurement->data[3], start_point->z};
            *res = rot(roll, pitch, yaw, start_point, &point);
            return TRUE;
        }
        break;
    case rangeUp:
        if (measurement->data[4] < SENSOR_TH && measurement->data[4] > TREE_RESOLUTION)
        {
            coordinateF_t point = {start_point->x, start_point->y, start_point->z + measurement->data[4]};
            *res = rot(roll, pitch, yaw, start_point, &point);
            return TRUE;
        }
        break;
    case rangeDown:
        if (measurement->data[5] < SENSOR_TH && measurement->data[5] > TREE_RESOLUTION)
        {
            coordinateF_t point = {start_point->x, start_point->y, start_point->z - measurement->data[5]};
            *res = rot(roll, pitch, yaw, start_point, &point);
            return TRUE;
        }
        break;
    default:
        DEBUG_PRINT("wrong input direction\n");
        break;
    }
    return FALSE;
}

bool cal_PointByLength(float length, float pitch, float roll, float yaw, coordinateF_t *start_point, rangeDirection_t dir, coordinateF_t *res)
{
    switch (dir)
    {
    case rangeFront:
    {
        coordinateF_t point = {start_point->x + length, start_point->y, start_point->z};
        *res = rot(roll, pitch, yaw, start_point, &point);
        return TRUE;
    }
    break;
    case rangeBack:
    {
        coordinateF_t point = {start_point->x - length, start_point->y, start_point->z};
        *res = rot(roll, pitch, yaw, start_point, &point);
        return TRUE;
    }
    break;
    case rangeLeft:
    {
        coordinateF_t point = {start_point->x, start_point->y + length, start_point->z};
        *res = rot(roll, pitch, yaw, start_point, &point);
        return TRUE;
    }
    break;
    case rangeRight:
    {
        coordinateF_t point = {start_point->x, start_point->y - length, start_point->z};
        *res = rot(roll, pitch, yaw, start_point, &point);
        return TRUE;
    }
    break;
    case rangeUp:
    {
        coordinateF_t point = {start_point->x, start_point->y, start_point->z + length};
        *res = rot(roll, pitch, yaw, start_point, &point);
        return TRUE;
    }
    break;
    case rangeDown:
    {
        coordinateF_t point = {start_point->x, start_point->y, start_point->z - length};
        *res = rot(roll, pitch, yaw, start_point, &point);
        return TRUE;
    }
    break;
    default:
        DEBUG_PRINT("wrong input direction\n");
        break;
    }
    return FALSE;
}

coordinateF_t rot(float roll, float pitch, float yaw, coordinateF_t *origin, coordinateF_t *point)
{
    float cosr = cos((double)roll * M_PI / 180);
    float cosp = cos((double)pitch * M_PI / 180);
    float cosy = cos((double)yaw * M_PI / 180);

    float sinr = sin((double)roll * M_PI / 180);
    float sinp = sin((double)pitch * M_PI / 180);
    float siny = sin((double)yaw * M_PI / 180);

    float roty[3][3];
    float rotp[3][3];
    float rotr[3][3];

    roty[0][0] = cosy;
    roty[0][1] = -siny;
    roty[0][2] = 0;
    roty[1][0] = siny;
    roty[1][1] = cosy;
    roty[1][2] = 0;
    roty[2][0] = 0;
    roty[2][1] = 0;
    roty[2][2] = 1;

    rotp[0][0] = cosp;
    rotp[0][1] = 0;
    rotp[0][2] = sinp;
    rotp[1][0] = 0;
    rotp[1][1] = 1;
    rotp[1][2] = 0;
    rotp[2][0] = -sinp;
    rotp[2][1] = 0;
    rotp[2][2] = cosp;

    rotr[0][0] = 1;
    rotr[0][1] = 0;
    rotr[0][2] = 0;
    rotr[1][0] = 0;
    rotr[1][1] = cosr;
    rotr[1][2] = -sinr;
    rotr[2][0] = 0;
    rotr[2][1] = sinr;
    rotr[2][2] = cosr;
    float tmp[3][1];
    tmp[0][0] = point->x - origin->x;
    tmp[1][0] = point->y - origin->y;
    tmp[2][0] = point->z - origin->z;

    dot(roty, tmp);
    dot(rotp, tmp);
    dot(rotr, tmp);
    coordinateF_t tmp2 = {tmp[0][0] + origin->x, tmp[1][0] + origin->y, tmp[2][0] + origin->z};

    determine_threshold(&tmp2);
    return tmp2;
}

void determine_threshold(coordinateF_t *point)
{
    point->x = fmax(fmin(point->x, WIDTH_X), 0);
    point->y = fmax(fmin(point->y, WIDTH_Y), 0);
    point->z = fmax(fmin(point->z, WIDTH_Z), 0);
}

void dot(float A[][3], float B[][1])
{
    float C[3][1];
    for (int i = 0; i < 3; i++)
    {
        C[i][0] = 0;
        for (int k = 0; k < 3; k++)
        {
            C[i][0] += A[i][k] * B[k][0];
        }
    }
    for (int i = 0; i < 3; i++)
    {
        B[i][0] = C[i][0];
    }
}

/**
 * @brief find the target node's parent node and and save the maxDepth by reference
 * @param octoNode self
 * @param point the point coordinate of the observation --- (x,y,z): tuple
 * @param origin origin of this node --- (x,y,z): tuple
 * @param width width of this node --- int
 * @param maxDepth maximum depth this node can be branched --- int
 */
octoNode_t *findTargetParent(octoNode_t *octoNode, octoMap_t *octoMap, coordinate_t *point, coordinate_t* origin, uint16_t *width, uint8_t *maxDepth)
{
    if (octoNode->isLeaf == 1)
    {
        return NULL;
    }
    else
    {
        uint8_t index = octoNodeIndex(point, *origin, *width);
        // if the node is leaf node, return its parent node
        if (octoMap->octoNodeSet->setData[octoNode->children].data[index].isLeaf)
            return octoNode;
        *origin = calOrigin(index, *origin, *width);
        *width = *width / 2;
        *maxDepth = *maxDepth - 1;
        return findTargetParent(&octoMap->octoNodeSet->setData[octoNode->children].data[index], octoMap, point, origin, width, maxDepth);
    }
}

/**
 * @brief calculate the node's cost,while the node contains the point
 * @param point the calculating point coordinate --- (x,y,z): tuple
 * @param octoTree self
 * @param octoMap self
 */
costParameter_t Cost(coordinate_t *point, octoTree_t *octoTree, octoMap_t *octoMap, octoNode_t *LastoctoNode)
{
    costParameter_t costParameter;
    uint8_t Depth = octoTree->maxDepth;
    uint16_t Width = octoTree->width;
    coordinate_t origin = octoTree->origin;
    octoNode_t *octoNode = findTargetParent(octoTree->root, octoMap, point, &origin, &Width, &Depth);
    if (octoNode == NULL)
    { // if the node is root, just return the cost
        costParameter.node = octoTree->root;
        // costParameter.cost = 8 * Depth;
        costParameter.cost_prune = 0;
        costParameter.p_not_occupied = 1 - P_GLOBAL;
        return costParameter;
    }
    uint8_t index = octoNodeIndex(point, origin, Width);
    costParameter.node = &octoMap->octoNodeSet->setData[octoNode->children].data[index];
    // Duplicate node, no contribution
    if (costParameter.node == LastoctoNode)
    {
        costParameter.node = LastoctoNode;
        costParameter.cost_prune = 0;
        costParameter.p_not_occupied = 1;
        return costParameter;
    }
    // if the node is known to be not occupied, continue
    if (costParameter.node->logOdds <= MAX_NOT_OCCUPIED)
    {
        costParameter.cost_prune = 0;
        costParameter.p_not_occupied = 1;
        return costParameter;
    }
    // if the node is known to be occupied, break
    else if (costParameter.node->logOdds >= MIN_OCCUPIED)
    {
        costParameter.cost_prune = 0;
        costParameter.p_not_occupied = 0;
        return costParameter;
    }
    // calculate the cost
    // double cost = 8 * (Depth - 1);
    // calculate the cost_prune
    double cost_prune = 0;
    double p;
    int Freenum = 0;
    int Occupiednum = 0;
    int i;
    for (i = 0; i < 8; ++i)
    {
        octoNode_t *temp = &octoMap->octoNodeSet->setData[octoNode->children].data[i];
        // if the node is not leaf, return 0
        if (!temp->isLeaf)
        {
            break;
        }
        // if the node is leaf, check its occupy
        if (temp->logOdds <= MAX_NOT_OCCUPIED)
            ++Freenum;
        else if (temp->logOdds >= MIN_OCCUPIED)
            ++Occupiednum;

        if (Freenum != 0 && Occupiednum != 0)
            break;
    }
    if (i == 8 && Occupiednum != 0)
    { // occupied
        p = P_GLOBAL + (1 - P_GLOBAL) * (double)Occupiednum / 8;
        costParameter.p_not_occupied = 1 - p;
        cost_prune = 8 * pow(p, 8 - Occupiednum);
    }
    else if (i == 8 && Freenum != 0)
    { // not occupied
        p = (1 - P_GLOBAL) + P_GLOBAL * (double)Freenum / 8;
        costParameter.p_not_occupied = p;
        cost_prune = 8 * pow(p, 8 - Freenum);
    }
    else
    { // unknown
        p = 1 - P_GLOBAL;
        costParameter.p_not_occupied = p;
        cost_prune = 8 * pow(p, 8);
    } // 根据p_global值固定与否可以考虑数组存值替代幂运算
    costParameter.cost_prune = cost_prune;
    return costParameter;
    // return cost，octonode，p_not_occupied
}

Cost_C_t Cost_Sum(octoTree_t *octoTree, octoMap_t *octoMap, coordinate_t *start, rangeDirection_t dir)
{
    int16_t dx = 0;
    int16_t dy = 0;
    int16_t dz = 0;
    switch (dir)
    {
    case rangeUp:
        dz = TREE_RESOLUTION;
        break;
    case rangeDown:
        dz = -TREE_RESOLUTION;
        break;
    case rangeLeft:
        dy = TREE_RESOLUTION;
        break;
    case rangeRight:
        dy = -TREE_RESOLUTION;
        break;
    case rangeFront:
        dx = TREE_RESOLUTION;
        break;
    case rangeBack:
        dx = -TREE_RESOLUTION;
        break;
    default:
        break;
    }

    double cost_prune = 0;
    double income_info = 0;
    double p_iter = 1;
    coordinate_t point = *start;
    costParameter_t costParameter_item;
    octoNode_t *LastoctoNode = NULL;
    Cost_C_t cost_C;
    while (point.x >= 0 && point.x <= TREE_CENTER_X * 2 
        && point.y >= 0 && point.y <= TREE_CENTER_Y * 2 
        && point.z >= BOTTOM && point.z <= TREE_CENTER_Z * 2 && point.z <= TOP
        && p_iter >= 0.00001)
    {
        point.x += dx;
        point.y += dy;
        point.z += dz;
        costParameter_item = Cost(&point, octoTree, octoMap, LastoctoNode);
        if (costParameter_item.p_not_occupied == 0) // the node is occupied, break
            break;
        if (costParameter_item.node == LastoctoNode || costParameter_item.p_not_occupied == 1)
            continue;
        cost_prune += p_iter * costParameter_item.cost_prune;
        income_info += p_iter * 1;
        p_iter *= costParameter_item.p_not_occupied;
        // printf("p_iter:%f\n",p_iter);
        if (costParameter_item.node == octoTree->root)
            break;
        LastoctoNode = costParameter_item.node;
    }
    // printf("cost_sum:%f,icome_info:%f\n",cost_sum,income_info);
    cost_C.cost_prune = cost_prune;
    cost_C.income_info = income_info;
    return cost_C;
}

