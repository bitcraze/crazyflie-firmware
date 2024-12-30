#include "debug.h"

#include "octoMapSerializer.h"
#include "string.h"
#include "octoMap.h"
#define MAX_QUEUE_BUFFER_SIZE 2048

#define COUNT_TIME_NUMS 10000

bool checkData(uint8_t* data1, uint8_t* data2, uint16_t dataLength1, uint16_t dataLength2);

void initOctoMapSerializerResult(octoMapSerializerResult_t *result){
    result->center.x = 0;
    result->center.y = 0;
    result->center.z = 0;
    result->resolution = 0;
    result->maxDepth = 0;
    result->width = 0;

    result->checkCode = 0;

    result->dictType = ORIGIN;
    result->dictLength = 0;

    result->dataLength = 0;

    memset(result->data,0,MAX_OCTOMAP_DATA_LENGTH);
}

// 八叉树有损序列化,每个节点仅占2bit位
bool serializeOctoMapLossy(octoMap_t *octoMap, octoMapSerializerResult_t *result){

    octoNode_t *root = octoMap->octoTree->root;
    result->center = octoMap->octoTree->center;
    result->resolution = octoMap->octoTree->resolution;
    result->maxDepth = octoMap->octoTree->maxDepth;
    result->width = octoMap->octoTree->width;

    uint8_t checkCode = CHECK_CODE_INIT_VALUE;
    // 层次遍历
    octoNode_t* queue[MAX_QUEUE_BUFFER_SIZE];
    int front = 0, rear = 0;
    octoNode_t *p = root;
    queue[rear] = p;
    rear = (rear + 1) % MAX_QUEUE_BUFFER_SIZE;
    int dataIndex = 0;
    uint16_t dataLength = 0;
    int i = 0;
    while (rear != front)
    {
        i++;
        p = queue[front];
        front = (front + 1) % MAX_QUEUE_BUFFER_SIZE;
        uint16_t index = p->children;
        octoNodeSetItem_t* brothers = &(octoMap->octoNodeSet->setData[index]);
        uint8_t item = 0;
        uint8_t curLength = 0;
        for (int i = 0; i < 8; i++)
        {
            octoNode_t *node = &(brothers->data[i]);
            if(node->isLeaf == 1){
                if(node->logOdds == LOG_ODDS_OCCUPIED){
                    item = (item << 2) + OCCUPIED;
                }
                else if(node->logOdds == LOG_ODDS_FREE){
                    item = (item << 2) + FREE;
                }
                else{
                    item = (item << 2) + UNKNOWN;
                }
            }else{
                item = (item << 2) + SPLIT;
                if( (rear+1)%MAX_QUEUE_BUFFER_SIZE != front ){
                    queue[rear] = node;
                    rear = (rear + 1) % MAX_QUEUE_BUFFER_SIZE;
                }else{
                    DEBUG_PRINT("[serializeOctoMap]queue is full\n");
                    return false;
                }
            }
            curLength += 2;
            if(curLength >= 8){
                if(dataIndex+1 >= MAX_OCTOMAP_DATA_LENGTH){
                    DEBUG_PRINT("[serializeOctoMap]data is full\n");
                    return false;
                }
                checkCode = checkCode ^ item;
                result->data[dataIndex++] = item;
                dataLength++;
                curLength = 0;
                item = 0;
            }
        }
    }
    result->checkCode = checkCode;
    result->dataLength = dataLength;

    return true;
}

// 八叉树有损反序列化 
bool deserializeOctoMapLossy(octoMap_t *octoMap, octoMapSerializerResult_t *data){

    octoMap->octoTree->center = data->center;
    octoMap->octoTree->origin.x = data->center.x - data->resolution * (1 << data->maxDepth) / 2;
    octoMap->octoTree->origin.y = data->center.y - data->resolution * (1 << data->maxDepth) / 2;
    octoMap->octoTree->origin.z = data->center.z - data->resolution * (1 << data->maxDepth) / 2;
    octoMap->octoTree->resolution = data->resolution;
    octoMap->octoTree->maxDepth = data->maxDepth;
    octoMap->octoTree->width = data->width;
    octoNodeSplit(octoMap->octoTree->root, octoMap);

    uint8_t checkCode = CHECK_CODE_INIT_VALUE;

    octoNode_t* queue[MAX_QUEUE_BUFFER_SIZE];
    int front = 0, rear = 0;
    octoNode_t *p = octoMap->octoTree->root;
    queue[rear] = p;
    rear = (rear + 1) % MAX_QUEUE_BUFFER_SIZE;
    int dataIndex = 0;
    while (rear != front)
    {
        p = queue[front];
        front = (front + 1) % MAX_QUEUE_BUFFER_SIZE;
        uint16_t index = p->children;
        octoNodeSetItem_t* brothers = &(octoMap->octoNodeSet->setData[index]);
        // 读取数据,拼成16位一起处理
        for(int i = 0;i<2;++i){
            if(dataIndex >= data->dataLength){
                DEBUG_PRINT("[deserializeOctoMap]data is not enough\n");
                return false;
            }
            uint8_t val = data->data[dataIndex++];
            checkCode = checkCode ^ val;
            for(int j = 0;j<4;++j){
                uint8_t nodeStatus = (val >> (6-2*j)) & 0x3;
                octoNode_t *node = &(brothers->data[i*4+j]);
                if(nodeStatus == OCCUPIED){
                    node->logOdds = LOG_ODDS_OCCUPIED;
                    node->isLeaf = TRUE;
                    node->children = 0;
                }
                else if(nodeStatus == FREE){
                    node->logOdds = LOG_ODDS_FREE;
                    node->isLeaf = TRUE;
                    node->children = 0;
                }
                else if(nodeStatus == UNKNOWN){
                    node->logOdds = LOG_ODDS_UNKNOWN;
                    node->isLeaf = TRUE;
                    node->children = 0;
                }
                else{
                    octoNodeSplit(node, octoMap);
                    if( (rear+1)%MAX_QUEUE_BUFFER_SIZE != front ){
                        queue[rear] = node;
                        rear = (rear + 1) % MAX_QUEUE_BUFFER_SIZE;
                    }else{
                        DEBUG_PRINT("[deserializeOctoMap]queue is full\n");
                        return false;
                    }
                }
            }
        }
    }
    if(checkCode != data->checkCode){
        DEBUG_PRINT("[deserializeOctoMap]checkCode is wrong\n");
        return false;
    }

    return true;
}

// 八叉树无损序列化,保留完整logOdds信息
bool serializeOctoMap(octoMap_t *octoMap, octoMapSerializerResult_t *result){

    octoNode_t *root = octoMap->octoTree->root;
    result->center = octoMap->octoTree->center;
    result->resolution = octoMap->octoTree->resolution;
    result->maxDepth = octoMap->octoTree->maxDepth;
    result->width = octoMap->octoTree->width;

    uint8_t checkCode = CHECK_CODE_INIT_VALUE;
    // 层次遍历
    octoNode_t* queue[MAX_QUEUE_BUFFER_SIZE];
    int front = 0, rear = 0;
    octoNode_t *p = root;
    queue[rear] = p;
    rear = (rear + 1) % MAX_QUEUE_BUFFER_SIZE;
    int dataIndex = 0;
    uint16_t dataLength = 0;
    int i = 0;
    while (rear != front)
    {
        i++;
        p = queue[front];
        front = (front + 1) % MAX_QUEUE_BUFFER_SIZE;
        uint16_t index = p->children;
        octoNodeSetItem_t* brothers = &(octoMap->octoNodeSet->setData[index]);
        uint8_t item = 0;
        uint8_t curLength = 0;

        // 由于每个节点8个孩子，每个节点占4bit，所以每个节点刚好占4个字节
        for (int i = 0; i < 8; i++)
        {
            octoNode_t *node = &(brothers->data[i]);
            if(node->isLeaf == 1){
                item = item<<4 | node->logOdds;
            }else{
                item = (item << 4) | 0xf;
                if( (rear+1)%MAX_QUEUE_BUFFER_SIZE != front ){
                    queue[rear] = node;
                    rear = (rear + 1) % MAX_QUEUE_BUFFER_SIZE;
                }else{
                    DEBUG_PRINT("[serializeOctoMap]queue is full\n");
                    return false;
                }
            }
            curLength += 4;
            if(curLength >= 8){
                if(dataIndex+1 >= MAX_OCTOMAP_DATA_LENGTH){
                    DEBUG_PRINT("[serializeOctoMap]data is full\n");
                    return false;
                }
                checkCode = checkCode ^ item;
                result->data[dataIndex++] = item;
                dataLength++;
                curLength = 0;
                item = 0;
            }
        }
    }
    result->checkCode = checkCode;
    result->dataLength = dataLength;

    return true;
}

// 八叉树反序列化
bool deserializeOctoMap(octoMap_t *octoMap, octoMapSerializerResult_t *data){

    octoMap->octoTree->center = data->center;
    octoMap->octoTree->origin.x = data->center.x - data->resolution * (1 << data->maxDepth) / 2;
    octoMap->octoTree->origin.y = data->center.y - data->resolution * (1 << data->maxDepth) / 2;
    octoMap->octoTree->origin.z = data->center.z - data->resolution * (1 << data->maxDepth) / 2;
    octoMap->octoTree->resolution = data->resolution;
    octoMap->octoTree->maxDepth = data->maxDepth;
    octoMap->octoTree->width = data->width;
    octoNodeSplit(octoMap->octoTree->root, octoMap);

    uint8_t checkCode = CHECK_CODE_INIT_VALUE;

    octoNode_t* queue[MAX_QUEUE_BUFFER_SIZE];
    int front = 0, rear = 0;
    octoNode_t *p = octoMap->octoTree->root;
    queue[rear] = p;
    rear = (rear + 1) % MAX_QUEUE_BUFFER_SIZE;
    int dataIndex = 0;
    while (rear != front)
    {
        p = queue[front];
        front = (front + 1) % MAX_QUEUE_BUFFER_SIZE;
        uint16_t index = p->children;
        octoNodeSetItem_t* brothers = &(octoMap->octoNodeSet->setData[index]);
        for (int i = 0; i < 4; i++)
        {
            uint8_t val = data->data[dataIndex++];
            checkCode = checkCode ^ val;
            // 获取前四位
            uint8_t first = (val >> 4) & 0xf;
            // 获取后四位
            uint8_t second = val & 0xf;
            octoNode_t *node = &(brothers->data[i*2]);
            if(first == 0xf){
                octoNodeSplit(node, octoMap);
                if( (rear+1)%MAX_QUEUE_BUFFER_SIZE != front ){
                    queue[rear] = node;
                    rear = (rear + 1) % MAX_QUEUE_BUFFER_SIZE;
                }else{
                    DEBUG_PRINT("[deserializeOctoMap]queue is full\n");
                    return false;
                }
            }else{
                node->isLeaf = TRUE;
                node->logOdds = first;
            }
            node = &(brothers->data[i*2+1]);
            if(second == 0xf){
                octoNodeSplit(node, octoMap);
                if( (rear+1)%MAX_QUEUE_BUFFER_SIZE != front ){
                    queue[rear] = node;
                    rear = (rear + 1) % MAX_QUEUE_BUFFER_SIZE;
                }else{
                    DEBUG_PRINT("[deserializeOctoMap]queue is full\n");
                    return false;
                }
            }else{
                node->isLeaf = TRUE;
                node->logOdds = second;
            }
        }
    }
    if(checkCode != data->checkCode){
        DEBUG_PRINT("[deserializeOctoMap]checkCode is wrong\n");
        return false;
    }

    return true;
}

bool checkOctoMapisConsist(octoMap_t *octoMap1, octoMap_t *octoMap2){
    // todo
    return true;
}

bool checkData(uint8_t* data1, uint8_t* data2, uint16_t dataLength1, uint16_t dataLength2){
    if(dataLength1 != dataLength2){
        DEBUG_PRINT("[checkData]dataLength1 = %d, dataLength2 = %d\n", dataLength1, dataLength2);
        // return false;
    }
    for (int i = 0; i < dataLength1; i++)
    {
        if(data1[i] != data2[i]){
            DEBUG_PRINT("[checkData]data1[%d] = %d, data2[%d] = %d\n", i, data1[i], i, data2[i]);
            return false;
        }
    }
    return true;
}

bool generateOctoMapSerializerResult(octoMap_t *octoMap, octoMapSerializerResult_t *result, DictType dictType){
    
}