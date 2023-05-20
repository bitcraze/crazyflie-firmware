#ifndef __OCTOTREE_H__
#define __OCTOTREE_H__
#include <stdint.h>
#include "octoMap.h"
#include "octoNode.h"

#include "config_autofly.h"

//#define BOOL int
#define TRUE 1
#define FALSE 0

octoTree_t* octoTreeInit(octoNodeSet_t* nodeSet);
void octoTreeInsertPoint(octoTree_t *octoTree, octoMap_t *octoMap, coordinate_t *point, uint8_t diffLogOdds);
void octoTreeRayCasting(octoTree_t *octoTree, octoMap_t *octoMap, coordinate_t *startPoint, coordinate_t *endPoint);
uint8_t octoTreeGetLogProbability(octoTree_t *octoTree, octoMap_t *octoMap, coordinate_t *point);
void bresenham3D(octoTree_t *octoTree, octoMap_t *octoMap, coordinate_t *startPoint, coordinate_t *endPoint);

#endif
