#ifndef __AD_HOC_ON_BOARD_H__
#define __AD_HOC_ON_BOARD_H__

#define MAX_CF_NUM 40


void initSimTopology();

bool checkItCanReceive(olsrAddr_t from,olsrAddr_t to);

short hashTable[MAX_CF_NUM];

#endif
