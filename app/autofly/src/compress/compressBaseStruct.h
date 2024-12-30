#ifndef COMPRESSBASESTRUCT_H
#define COMPRESSBASESTRUCT_H

#include "stdint.h"
#include "stdbool.h"

#define value_t uint8_t
#define times_t uint16_t

#define MAX_DICT_SIZE 512

#define NULL_SEQ 0xffff
#define NULL_TIMES 0xffff
#define NULL_VALUE 0xff

#if MAX_DICT_SIZE < 256
    #error "MAX_DICT_SIZE must be greater than 256"
#endif



// 下标为key，取值为value
typedef struct 
{
    int size; //字典实际使用长度
    value_t value[MAX_DICT_SIZE]; // 字典值
    times_t times[MAX_DICT_SIZE]; // 字典值出现次数
}dict_t;

void initDict(dict_t *dict);
uint16_t addDictRecode(dict_t *dict, value_t value, times_t times);
// 删除times为0的记录
void deleteZeroTimes(dict_t *dict);
// 按照times排序，从小到大
void sortDictByTimes(dict_t *dict, int left, int right);

// 从data中生成新字典，并排序
bool fillDictFromData(uint8_t *data, uint16_t dataLength, dict_t *dict);

void printDict(dict_t *dict,uint16_t printSize);
#endif