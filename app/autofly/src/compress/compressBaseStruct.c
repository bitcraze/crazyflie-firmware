#include "debug.h"
#include "stdbool.h"

#include "compressBaseStruct.h"

void swapDict(dict_t *dict, int i, int j);
void quickSort(dict_t *dict, int left, int right);


void initDict(dict_t *dict){
    dict->size = 0;
    for(int i = 0; i < MAX_DICT_SIZE; i++){
        dict->value[i] = 0;
        dict->times[i] = NULL_TIMES;
    }
}

void swapDict(dict_t *dict, int i, int j){
    if(i == j){
        return;
    }
    value_t temp = dict->value[i];
    dict->value[i] = dict->value[j];
    dict->value[j] = temp;
    times_t temp_2 = dict->times[i];
    dict->times[i] = dict->times[j];
    dict->times[j] = temp_2;
}

//快速排序
void quickSort(dict_t *dict, int left, int right){
    if (left >= right) {
        return;
    } 
    if(right - left == 1){
        if(dict->times[left] > dict->times[right]){
            swapDict(dict, left, right);
        }
        return;
    }

    // 在仅两位时发生错误，例：19，20，基准为19，但由于i=j，找到20，然后退出循环发生19和20互换，导致错误
    int key = dict->times[left];  // 选择基准值
    int i = left + 1;
    int j = right;
    while(i<j){
        // 从右向左找第一个小于等于基准值的数
        while(i<j && dict->times[j] > key){
            j--;
        }
        // 从左向右找第一个大于基准值的数
        while(i<j && dict->times[i] <= key){
            i++;
        }
        if(i!=j){
            swapDict(dict, i, j);
        }   
    }
    swapDict(dict, left, j);

    quickSort(dict, left, j-1);
    quickSort(dict, j + 1, right);
}

uint16_t searchDict(dict_t *dict, value_t value){
    // 考虑到初始载入情况下按value值插入，所以先直接比较value值
    if(dict->value[value] == value){
        return value;
    }
    for(int i = 0; i < dict->size; i++){
        if(dict->value[i] == value){
            return i;
        }
    }
    return NULL_SEQ;
}

bool fillDictFromData(uint8_t *data, uint16_t dataLength, dict_t *dict){
    initDict(dict);
    for(int i = 0; i < dataLength; i++){
        uint8_t val = data[i];
        if(val >= MAX_DICT_SIZE){
            DEBUG_PRINT("val = %X, out of range\n", val);
            return false;
        }
        if(dict->times[val] == NULL_TIMES){
            dict->value[val] = val;
            dict->times[val] = 1;
            dict->size++;
        }
        else{
            dict->times[val]++;
        }
    }
    // DEBUG_PRINT("dict size = %d\n", dict->size);
    int top = 0;
    for(int i = 0; i < MAX_DICT_SIZE; i++){
        if(dict->times[i] != NULL_TIMES){
            dict->value[top] = dict->value[i];
            dict->times[top] = dict->times[i];
            top++;
        }
    }
    sortDictByTimes(dict,0,dict->size-1);
    // printDict(dict,dict->size);
    return true;
}

uint16_t addDictRecode(dict_t *dict, value_t value, times_t times){
    dict->value[dict->size] = value;
    dict->times[dict->size] = times;
    dict->size++;
    if(dict->size >= MAX_DICT_SIZE){
        DEBUG_PRINT("dict is full\n");
        return NULL_SEQ;
    }
    return dict->size-1;
}

void deleteZeroTimes(dict_t *dict){
    int top = 0;
    for(int i = 0;i<dict->size;++i){
        if(dict->times[i] != 0){
            if(i!=top){
                swapDict(dict, i, top);
            }
            top ++;
        }
    }
    dict->size = top;
}

void sortDictByTimes(dict_t *dict, int left, int right){
    // 按照times排序，从小到大
    quickSort(dict, left, right);
}

void printDict(dict_t *dict,uint16_t printSize){
    for(int i = 0; i < dict->size; i++){
        uint16_t length = dict->value[i] >> 8;
        DEBUG_PRINT("dict[%d] = %X, times = %d\n", i, dict->value[i], dict->times[i]);
    }
}