/* octoMap.c: Do the mapping task */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "octoMap.h"
#include "octoTree.h"
#include "octoNodeSet.h"
#include "rrtConnect.h"

#include "debug.h"

#define FILE_LENGTH 1000

int count = 0;

void octoMapInit(octoMap_t *octoMap)
{
    // init node set
    DEBUG_PRINT("octoMapInit\n");
    octoNodeSet_t* nodeSet;
    nodeSet = malloc(sizeof(octoNodeSet_t));
    // print nodeSet size
    DEBUG_PRINT("sizeof(octoNodeSet_t) = %d\n", sizeof(octoNodeSet_t));
    octoNodeSetInit(nodeSet);

    // init octoMap
    octoMap->octoTree = octoTreeInit(nodeSet);
    octoMap->octoNodeSet = nodeSet;
    // avoid index 0 is used (octoNodeHasChildren will fail)
    octoMap->octoTree->root->children = octoNodeSetMalloc(octoMap->octoNodeSet);

    // print octoMap
    DEBUG_PRINT("octoMap.octoTree->center = (%d, %d, %d)\n", octoMap->octoTree->center.x, octoMap->octoTree->center.y, octoMap->octoTree->center.z);
    DEBUG_PRINT("octoMap.octoTree->origin = (%d, %d, %d)\n", octoMap->octoTree->origin.x, octoMap->octoTree->origin.y, octoMap->octoTree->origin.z);
    DEBUG_PRINT("octoMap.octoTree->resolution = %d\n", octoMap->octoTree->resolution);
    DEBUG_PRINT("octoMap.octoTree->maxDepth = %d\n", octoMap->octoTree->maxDepth);
    DEBUG_PRINT("octoMap.octoTree->width = %d\n", octoMap->octoTree->width);
    // print octoMap.octoTree->root
    DEBUG_PRINT("octoMap.octoTree->root->children = %d\n", octoMap->octoTree->root->children);
    DEBUG_PRINT("octoMap.octoTree->root->logOdds = %d\n", octoMap->octoTree->root->logOdds);
    DEBUG_PRINT("octoMap.octoTree->root->isLeaf = %d\n", octoMap->octoTree->root->isLeaf);
    // print octoMap.octoNodeSet
    DEBUG_PRINT("octoMap.octoNodeSet->freeQueueEntry = %d, octoMap.octoNodeSet->fullQueueEntry = %d\n\n", octoMap->octoNodeSet->freeQueueEntry, octoMap->octoNodeSet->fullQueueEntry);
    //print the length and numFree and numOccupied
    DEBUG_PRINT("octoMap.octoNodeSet->length = %d, octoMap.octoNodeSet->numFree = %d, octoMap.octoNodeSet->numOccupied = %d\n\n", octoMap->octoNodeSet->length, octoMap->octoNodeSet->numFree, octoMap->octoNodeSet->numOccupied);
    count = 0;
    DEBUG_PRINT("octoMap.octoNodeSet->volumeFree = %d, octoMap.octoNodeSet->volumeOccupied = %d\n\n", octoMap->octoNodeSet->volumeFree, octoMap->octoNodeSet->volumeOccupied);
}

void testFromFile(coordinate_t *(start_points[FILE_LENGTH]), coordinate_t *(end_points[FILE_LENGTH]))
{
    char *start_file = "../assets/start_points.csv";
    char *end_file = "../assets/end_points.csv";
    FILE *fp1 = fopen(start_file, "r"); // 打开起点文件
    FILE *fp2 = fopen(end_file, "r"); // 打开终点文件
    if (fp1 == NULL || fp2 == NULL) {
        DEBUG_PRINT("无法打开文件 %s 或 %s\n", start_file, end_file);
    }
    char buffer1[1024]; // 缓冲区1
    char buffer2[1024]; // 缓冲区2

    int row = 0; // 行数
    int col = 0; // 列数
    int size = 0; // 数组大小

    while (row < FILE_LENGTH && fgets(buffer1, sizeof(buffer1), fp1) && fgets(buffer2, sizeof(buffer2), fp2)) { // 同时读取两个文件的一行
        row++;

        char *token1 = strtok(buffer1, ","); // 按逗号分割字符串
        coordinate_t p_start = {0,0,0};
        col = 0;
        while (token1) {
            col++;
            if (col == 1) {
                token1 = strtok(NULL, ","); // 继续分割
                continue; // 跳过第一列
            }
            switch(col) { // 根据列数赋值给Point结构体
                case 2:
                    p_start.x = atoi(token1);
                    break;
                case 3:
                    p_start.y = atoi(token1);
                    break;
                case 4:
                    p_start.z = atoi(token1);
                    break;
                default:
                    break;
            }

            token1 = strtok(NULL, ","); // 继续分割
        }

        char *token2 = strtok(buffer2, ","); // 按逗号分割字符串
        coordinate_t p_end = {0,0,0};
        col = 0;
        while (token2) {
            col++;
            if (col == 1) {
                token2 = strtok(NULL, ","); // 继续分割
                continue; // 跳过第一列
            }
            switch(col) { // 根据列数赋值给Point结构体
                case 2:
                    p_end.x = atoi(token2);
                    break;
                case 3:
                    p_end.y = atoi(token2);
                    break;
                case 4:
                    p_end.z = atoi(token2);
                    break;
                default:
                    break;
            }

            token2 = strtok(NULL, ","); // 继续分割
        }

        size++; // 数组大小加一
        start_points[size - 1]->x = p_start.x;
        start_points[size - 1]->y = p_start.y;
        start_points[size - 1]->z = p_start.z;
        end_points[size - 1]->x = p_end.x;
        end_points[size - 1]->y = p_end.y;
        end_points[size - 1]->z = p_end.z;

        // print start_points and end_points
//        DEBUG_PRINT("row = %d\n", row);
//        DEBUG_PRINT("start_points[%d] = (%d, %d, %d)\n", size - 1, start_points[row - 1]->x, start_points[row - 1]->y, start_points[row - 1]->z);
//        DEBUG_PRINT("end_points[%d] = (%d, %d, %d)\n", size - 1, end_points[row - 1]->x, end_points[row - 1]->y, end_points[row - 1]->z);
    }
}

void recursiveExportOctoMap(octoMap_t* octoMap, octoNode_t* node, coordinate_t origin, uint16_t width) {
    if (node->isLeaf) {
        if(LOG_ODDS_FREE == node->logOdds ){
            ++count;
            DEBUG_PRINT("[app]FN:(%.2f,%.2f,%.2f),seq:%d,width:%d\n", (double)origin.x, (double)origin.y, (double)origin.z, count, width);
            vTaskDelay(100);
        }
        else if(LOG_ODDS_OCCUPIED == node->logOdds){
            ++count;
            DEBUG_PRINT("[app]ON:(%.2f,%.2f,%.2f),seq:%d,width:%d\n", (double)origin.x, (double)origin.y, (double)origin.z, count, width);
            vTaskDelay(100);
        }
        // DEBUG_PRINT("node->x = %d, node->y = %d, node->z = %d, node->width = %d, node->logOdds = %d\n", node->origin.x, node->origin.y, node->origin.z, width, node->logOdds);
        // fprintf(fp, "%d, %d, %d, %d, %d\n", node->origin.x, node->origin.y, node->origin.z, width, node->logOdds);
    } else {
        for (int i = 0; i < 8; i++) {
            if (octoNodeHasChildren(node) && width > octoMap->octoTree->resolution) {
                coordinate_t newOrigin = calOrigin(i,origin,width);
                recursiveExportOctoMap(octoMap, &octoMap->octoNodeSet->setData[node->children].data[i], newOrigin, width / 2);
            }
        }
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

// void exportOctoMap(octoMap_t* octoMap) {
//     FILE *fp = fopen("../assets/octoMap.csv", "w");
//     if (fp == NULL) {
//         DEBUG_PRINT("无法打开文件 octoMap.csv\n");
//     }
//     octoNode_t* node = octoMap->octoTree->root;
//     recursiveExportOctoMap(octoMap, node, fp, octoMap->octoTree->width);
// }

/*
int main()
{
    // malloc start_points and end_points
    coordinate_t *(start_points[FILE_LENGTH]);
    for (int i = 0; i < FILE_LENGTH; i++) {
        start_points[i] = malloc(sizeof(coordinate_t));
    }
    coordinate_t *(end_points[FILE_LENGTH]);
    for (int i = 0; i < FILE_LENGTH; i++) {
        end_points[i] = malloc(sizeof(coordinate_t));
    }
    testFromFile(start_points, end_points);

    octoMap_t* octoMap;
    octoMap = malloc(sizeof(octoMap_t));
    octoMapInit(octoMap);

    // test from file
    for (int i = 0; i < FILE_LENGTH; i++) {
        octoTreeRayCasting(octoMap->octoTree, octoMap, start_points[i], end_points[i]);
    }
    exportOctoMap(octoMap);
    //return 0;

    // test rrt
    array_t res;
    res.len = 0;
    int i = 0;
    do
    {
        ++i;
        DEBUG_PRINT("--------------------------i = %d---------------------------\n", i);
        res = planning(*start_points[0], *end_points[180], octoMap->octoTree, octoMap);
    } while (0 && res.len == 0);
    DEBUG_PRINT("res.len = %d\n", res.len);
    for (int i = 0; i < res.len; i++) {
        DEBUG_PRINT("res[%d] = (%d, %d, %d)\n", i, res.arr[i].loc.x, res.arr[i].loc.y, res.arr[i].loc.z);
    }

    // free
    for (int i = 0; i < FILE_LENGTH; i++) {
        free(start_points[i]);
        free(end_points[i]);
    }
    free(octoMap->octoNodeSet);
    free(octoMap);
    return 0;
}*/
