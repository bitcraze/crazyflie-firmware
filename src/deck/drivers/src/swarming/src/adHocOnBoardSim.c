#include "olsrAlgo.h"
#include "olsrStruct.h"
#include "adHocOnBoardSim.h"
#include "olsrDebug.h"


/*
        _____________________
        |                   |
        --7--13--14--18--29--
*/

#define USE_NUM 5
#define MAX_CF_NUM 40


static bool adjMatrices[USE_NUM][USE_NUM] = {{0,1,0,0,1},{1,0,1,0,0},{0,1,0,1,0},{0,0,1,0,1},{1,0,0,1,0}};
short hashTable[MAX_CF_NUM];
void initSimTopology()
{
    for(uint8_t i=0;i<MAX_CF_NUM;i++)
    {
        hashTable[i] = -1;
    }
    hashTable[7] = 0;
    hashTable[13] = 1;
    hashTable[14] = 2;
    hashTable[18] = 3;
    hashTable[29] = 4;
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