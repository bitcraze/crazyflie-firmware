#pragma once
#include "octoMap.h"
#include "auxiliary_tool.h"

void GetRange(uavRange_t* uavRange);
void UpdateMap(octoMap_t *octoMap, uavRange_t* uavRange);
void CalCandidates(coordinateF_t *candidates, example_measure_t *measurement, coordinateF_t *current_F);
bool CalBestCandinates(octoMap_t *octoMap,uavControl_t* uavControl_t);
bool JumpLocalOp(uavControl_t *uavControl);
void MoveToNext(coordinateF_t* cur,coordinateF_t* next);
bool CalNextPoint(uavControl_t* uavControl,octoMap_t* octoMap);

void Land();