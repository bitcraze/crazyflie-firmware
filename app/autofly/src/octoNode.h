#ifndef __OCTONODE_H__
#define __OCTONODE_H__
#include <stdint.h>
#include <stdbool.h>
#include "octoMap.h"

void octoNodeInit(octoNode_t *node);
BOOL octoNodeHasChildren(octoNode_t *octoNode);
void octoNodeSplit(octoNode_t *octoNode, octoMap_t *octoMap);
void octoNodePrune(octoNode_t *octoNode, octoMap_t *octoMap);

uint8_t octoNodeIndex(coordinate_t *point, coordinate_t origin, uint16_t width);
coordinate_t calOrigin(uint8_t index, coordinate_t origin, uint16_t width);
void octoNodeUpdate(octoNode_t *octoNode, octoMap_t *octoMap, coordinate_t *point, uint8_t diffLogOdds, coordinate_t origin, uint16_t width, uint8_t maxDepth);
BOOL octoNodeCheckChildrenLogOdds(octoNode_t *octoNode, octoMap_t *octoMap);
void octoNodeUpdateLogOdds(octoMap_t* octoMap, octoNode_t *octoNode, uint8_t diffLogOdds);
uint8_t octoNodeLogOddsAt(octoNode_t *octoNode, octoMap_t *octoMap, coordinate_t *point, coordinate_t origin, uint16_t width);
BOOL octoNodeLogOddsIsOccupiedOrFree(octoNode_t *octoNode);

#endif
