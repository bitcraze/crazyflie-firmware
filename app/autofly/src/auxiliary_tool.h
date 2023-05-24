#pragma once
#include "octoMap.h"
#include "range.h"
#include "circularQueue.h"

typedef struct
{
    float x;
    float y;
    float z;
} coordinateF_t;

typedef struct
{
    float data[6];
    float roll;
    float pitch;
    float yaw;
} example_measure_t;

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

void inituavRange(uavRange_t* uavRange);
void inituavControl(uavControl_t* uavControl);

double caldistance(coordinate_t* A,coordinate_t* B);
bool ReliabilityTest(coordinateF_t* last, coordinateF_t* cur);
bool IsSameCell(coordinateF_t* last, coordinateF_t* cur);

void get_measurement(example_measure_t* measurement);
rangeDirection_t GetRandomDir(example_measure_t *measurement);

bool cal_Point(example_measure_t* measurement,coordinateF_t* start_point,rangeDirection_t dir,coordinateF_t* res); 
bool cal_PointByLength(float length,float pitch,float roll,float yaw,coordinateF_t* start_point,rangeDirection_t dir,coordinateF_t *res);

//rotate
coordinateF_t rot(float roll, float pitch, float yaw, coordinateF_t* origin, coordinateF_t* point);
void determine_threshold(coordinateF_t *point);
void dot(float A[][3], float B[][1]);

//calculate the cost_prune and income_info
octoNode_t *findTargetParent(octoNode_t *octoNode, octoMap_t *octoMap, coordinate_t *point, coordinate_t* origin, uint16_t *width, uint8_t *maxDepth);
costParameter_t Cost(coordinate_t *Point,octoTree_t *octoTree, octoMap_t *octoMap, octoNode_t *LastoctoNode);
Cost_C_t Cost_Sum(octoTree_t *octoTree, octoMap_t *octoMap, coordinate_t *start, rangeDirection_t dir);

