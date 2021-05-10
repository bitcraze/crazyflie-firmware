#include "olsrAlgo.h"
#include "olsrStruct.h"
#include "adHocOnBoardSim.h"
#include "olsrDebug.h"


/*
        _______________(8)________________
        |                               |
        --8-(1)-12-(1)-17-(1)-27-(8)-29--
*/

#define USE_NUM 5
#define MAX_CF_NUM 40


static bool adjMatrices[USE_NUM][USE_NUM] = {{0,1,0,0,1},\
                                             {1,0,1,0,0},\
                                             {0,1,0,1,0},\
                                             {0,0,1,0,1},\
                                             {1,0,0,1,0}};
static int16_t distanceMatrices[USE_NUM][USE_NUM] =  {{0,400,800,0,800},\
                                                      {400,0,400,0,0},\
                                                      {800,400,0,100,0},\
                                                      {0,0,100,0,800},\
                                                      {800,0,0,800,0}};
short hashTable[MAX_CF_NUM];
void initSimTopology()
{
    for(uint8_t i=0;i<MAX_CF_NUM;i++)
    {
        hashTable[i] = -1;
    }
    hashTable[14] = 0;
    hashTable[28] = 1;
    hashTable[29] = 2;
}

bool checkItCanReceive(olsrAddr_t from,olsrAddr_t to)
{
    if(from >=MAX_CF_NUM ||to >=MAX_CF_NUM||from<0||to<0)
    {
        DEBUG_PRINT_OLSR_SIM("error Input\n");
        return;
    }

    short fromId = hashTable[from];
    short toId = hashTable[to];

    if(fromId==-1||toId==-1)
    {
        DEBUG_PRINT_OLSR_SIM("may not init\n");
        return;
    }
    DEBUG_PRINT_OLSR_SIM("found?%d\n",adjMatrices[fromId][toId]);
    return adjMatrices[fromId][toId];
}

int16_t getDistanceFromOnboardSim(olsrAddr_t from,olsrAddr_t to)
{
    if(from >=MAX_CF_NUM ||to >=MAX_CF_NUM||from<0||to<0)
    {
        DEBUG_PRINT_OLSR_SIM("error Input\n");
        return;
    }
    short fromId = hashTable[from];
    short toId = hashTable[to];
    if(fromId==-1||toId==-1)
    {
        DEBUG_PRINT_OLSR_SIM("may not init\n");
        return;
    }
    return distanceMatrices[fromId][toId];
}