#ifndef __OCTONODESET_H__
#define __OCTONODESET_H__

#include "octoMap.h"

#define TRUE 1
#define FALSE 0

void octoNodeSetInit(octoNodeSet_t *codeSet);                     // initialize the set
setIndex_t octoNodeSetMalloc(octoNodeSet_t *nodeSet);             // malloc 8 items
BOOL octoNodeSetFree(octoNodeSet_t *nodeSet, setIndex_t delItem); // free 8 items
#endif