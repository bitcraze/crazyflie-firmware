#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "octoTree.h"
#include "octoNodeSet.h"

#include "debug.h"

/**
 * @brief initialize an octoTree
 *
 * @param octoTree
 */
octoTree_t g_OctoTree;
octoNode_t g_OctoNode;
octoTree_t *octoTreeInit(octoNodeSet_t *nodeSet) {
    // init octoTree rootNode
    octoNode_t *root = &g_OctoNode;
    root->logOdds = LOG_ODDS_UNKNOWN;
    root->isLeaf = TRUE;
    // init octoTree
    g_OctoTree.center.x = TREE_CENTER_X;
    g_OctoTree.center.y = TREE_CENTER_Y;
    g_OctoTree.center.z = TREE_CENTER_Z;
    g_OctoTree.origin.x = TREE_CENTER_X - TREE_RESOLUTION * (1 << TREE_MAX_DEPTH) / 2;
    g_OctoTree.origin.y = TREE_CENTER_Y - TREE_RESOLUTION * (1 << TREE_MAX_DEPTH) / 2;
    g_OctoTree.origin.z = TREE_CENTER_Z - TREE_RESOLUTION * (1 << TREE_MAX_DEPTH) / 2;
    g_OctoTree.resolution = TREE_RESOLUTION;
    g_OctoTree.maxDepth = TREE_MAX_DEPTH;
    g_OctoTree.width = TREE_RESOLUTION * pow(2, TREE_MAX_DEPTH);
    g_OctoTree.root = root;
    return &g_OctoTree;
}

/**
 * @brief Add an observation to the octomap.
 *
 * @param octoTree self
 * @param point the coordinate of the observation lidar point --- (x,y,z): tuple
 * @param diffLogOdds the difference value of logodds, 0: free, 1: occupied
 */
void octoTreeInsertPoint(octoTree_t *octoTree, octoMap_t *octoMap, coordinate_t *point, uint8_t diffLogOdds) {
    octoNodeUpdate(octoTree->root, octoMap, point, diffLogOdds, octoTree->origin, octoTree->width, octoTree->maxDepth);
}

/**
 * @brief Add the probability of the grid occupied by the ray path to the tree
 *
 * @param octoTree self
 * @param startPoint the coordinate of the sensor --- (x,y,z): tuple
 * @param endPoint the coordinate of the observation point  --- (x,y,z): tuple
 * @param diffLogOdds the difference value of logodds, 0: free, 1: occupied
 */
void octoTreeRayCasting(octoTree_t *octoTree, octoMap_t *octoMap, coordinate_t *startPoint, coordinate_t *endPoint) {
    // call bresenham algorithm to insert free voxel
    bresenham3D(octoTree, octoMap, startPoint, endPoint);
    // Insert occupancy voxel
    octoTreeInsertPoint(octoTree, octoMap, endPoint, LOG_ODDS_OCCUPIED_FLAG);
}

/**
 * @brief Return the occupancy probability of the voxel at a given point coordinate.
 *
 * @param octoTree self
 * @param point coordinate of some voxel to get probability --- (x,y,z): tuple
 * @return uint8_t occupancy probability of the corresponding voxel
 */
uint8_t octoTreeGetLogProbability(octoTree_t *octoTree, octoMap_t *octoMap, coordinate_t *point) {
    return octoNodeLogOddsAt(octoTree->root, octoMap, point, octoTree->origin, octoTree->width);
}

/**
 * @brief bresenham algorithm for 3D ray casting
 *
 * @param octoTree self
 * @param octoMap self
 * @param start start point of the ray
 * @param end end point of the ray
 */
void bresenham3D(octoTree_t *octoTree, octoMap_t *octoMap, coordinate_t *start, coordinate_t *end)
{
    coordinate_t item_start = {start->x,start->y,start->z};
    coordinate_t item_end = {end->x,end->y,end->z};
    uint16_t steepXY = (abs(item_end.y - item_start.y) > abs(item_end.x - item_start.x));
    if (steepXY) {
        uint16_t temp = item_start.x;
        item_start.x = item_start.y;
        item_start.y = temp;
        temp = item_end.x;
        item_end.x = item_end.y;
        item_end.y = temp;
    }
    uint16_t steepXZ = (abs(item_end.z - item_start.z) > abs(item_end.x - item_start.x));
    if (steepXZ) {
        uint16_t temp = item_start.x;
        item_start.x = item_start.z;
        item_start.z = temp;
        temp = item_end.x;
        item_end.x = item_end.z;
        item_end.z = temp;
    }

    uint16_t deltaX = abs(item_end.x - item_start.x);
    uint16_t deltaY = abs(item_end.y - item_start.y);
    uint16_t deltaZ = abs(item_end.z - item_start.z);

    int errorXY = deltaX / 2;
    int errorXZ = deltaX / 2;

    uint16_t stepX = item_start.x < item_end.x ? TREE_RESOLUTION : -TREE_RESOLUTION;
    uint16_t stepY = item_start.y < item_end.y ? TREE_RESOLUTION : -TREE_RESOLUTION;
    uint16_t stepZ = item_start.z < item_end.z ? TREE_RESOLUTION : -TREE_RESOLUTION;

    uint16_t x = item_start.x;
    uint16_t y = item_start.y;
    uint16_t z = item_start.z;
    while (abs(x - item_end.x) > TREE_RESOLUTION) {
        coordinate_t pointCoordinate = {x, y, z};
        coordinate_t* point = &pointCoordinate;
        if (steepXZ) {
            int temp = point->x;
            point->x = point->z;
            point->z = temp;
        }
        if (steepXY) {
            int temp = point->x;
            point->x = point->y;
            point->y = temp;
        }

        errorXY -= deltaY;
        errorXZ -= deltaZ;
        if (errorXY < 0) {
            y += stepY;
            errorXY += deltaX;
        }
        if (errorXZ < 0) {
            z += stepZ;
            errorXZ += deltaX;
        }
        octoTreeInsertPoint(octoTree, octoMap, point, LOG_ODDS_FREE_FLAG);
        x += stepX;
    }
}
