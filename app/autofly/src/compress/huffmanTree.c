#include "stdint.h"
#include "stdbool.h"
#include "stdlib.h"
#include "time.h"
#include "debug.h"

#include "compressBaseStruct.h"
#include "huffmanTree.h"

#ifdef HUFFMANTREE_TIMES_ENABLE
    uint32_t newDataExpectedLength = 0;      
#endif


bool push_HuffmanHeap(HeapNode *heap, uint16_t index, times_t times, int size);
HeapNode pop_HuffmanHeap(HeapNode *heap, int size);
void adjustHeapUp(HeapNode *heap, int i, int size);
void adjustHeapDown(HeapNode *heap, int i, int size);
bool processHuffmanTree(HuffmanTree *tree, uint16_t root, uint8_t length, huffman_code_t value, huffman_code_t* values);

void initHuffmanTree(HuffmanTree *tree){
    tree->size = 0;
    tree->root = NULL_SEQ;
    tree->free = 0;
    for(int i = 0; i < MAX_HUFFMAN_TREE_SIZE; i++){
        tree->nodes[i].value = NULL_VALUE;
        #ifdef HUFFMANTREE_TIMES_ENABLE
            tree->nodes[i].times = NULL_TIMES;
        #endif
        tree->nodes[i].left = NULL_SEQ;
        tree->nodes[i].right = i+1;
    }
    tree->nodes[MAX_HUFFMAN_TREE_SIZE-1].right = NULL_SEQ;
}

uint16_t mallocNode(HuffmanTree *tree){
    if(tree->free == NULL_SEQ){
        DEBUG_PRINT("HuffmanTree malloc Node failed\n");
        return NULL_SEQ;
    }
    
    uint16_t index = tree->free;
    tree->free = tree->nodes[index].right;
    // 初始化node
    tree->nodes[index].left = NULL_SEQ;
    tree->nodes[index].right = NULL_SEQ;
    tree->size++;
    return index;
}

void freeNode(HuffmanTree *tree, uint16_t index){
    if(tree->nodes[index].left != NULL_SEQ){
        freeNode(tree, tree->nodes[index].left);
    }
    if(tree->nodes[index].right != NULL_SEQ){
        freeNode(tree, tree->nodes[index].right);
    }
    tree->nodes[index].value = NULL_VALUE;
    tree->nodes[index].left = NULL_SEQ;
    tree->nodes[index].right = tree->free;
    tree->free = index;
    tree->size--;
}

// uint16_t **heap [index, times],index is the index of huffmanTree, times is the times of value
bool push_HuffmanHeap(HeapNode *heap, uint16_t index, times_t times, int size){
    heap[size].times = times;
    heap[size].index = index;
    adjustHeapUp(heap, size, size+1);
}

HeapNode pop_HuffmanHeap(HeapNode *heap, int size){
    if(size == 0){
        DEBUG_PRINT("heap is empty\n");
        return (HeapNode){NULL_SEQ, NULL_TIMES};
    }
    HeapNode res = heap[0];
    heap[0] = heap[size-1];
    adjustHeapDown(heap, 0, size-1);
    return res;
}

void adjustHeapUp(HeapNode *heap, int i, int size){
    int parent = (i-1)/2;
    while(i > 0 && heap[parent].times > heap[i].times){
        HeapNode temp = heap[parent];
        heap[parent] = heap[i];
        heap[i] = temp;
        i = parent;
        parent = (i-1)/2;
    }
}

void adjustHeapDown(HeapNode *heap, int i, int size){
    int left = 2*i+1;
    while (left < size)
    {
        if(left+1 < size && heap[left+1].times < heap[left].times){
            left++;
        }
        if(heap[i].times <= heap[left].times){
            break;
        }
        HeapNode temp = heap[i];
        heap[i] = heap[left];
        heap[left] = temp;
        i = left;
        left = 2*i+1;
    }
}

uint16_t huffmanEnCode(uint8_t *data, uint16_t dataLength, HuffmanTree *tree, uint8_t *newData, uint16_t newDataMAXLength){

    dict_t dict;
    initDict(&dict);
    if(!fillDictFromData(data, dataLength, &dict)){
        DEBUG_PRINT("fillDictFromData failed\n");
        return NULL_SEQ;
    }
    if(dict.size == 0){
        DEBUG_PRINT("dict is empty\n");
        return NULL_SEQ;
    }

    initHuffmanTree(tree);

    HeapNode heap[MAX_DICT_SIZE];
    uint16_t heapSize = 0;
    for (int i = 0; i < dict.size; i++)
    {
        uint16_t index = mallocNode(tree);
        if(index == NULL_SEQ){
            DEBUG_PRINT("mallocNode failed\n");
            return NULL_SEQ;
        }
        tree->nodes[index].value = dict.value[i];
        #ifdef HUFFMANTREE_TIMES_ENABLE
            tree->nodes[index].times = dict.times[i];
        #endif
        push_HuffmanHeap(heap, index, dict.times[i], heapSize);
        heapSize++;
    }

    // 构建Huffman树
    while (heapSize > 1)
    {
        HeapNode right = pop_HuffmanHeap(heap, heapSize);
        heapSize--;
        HeapNode left = pop_HuffmanHeap(heap, heapSize);
        heapSize--;
        uint16_t index = mallocNode(tree);
        if(index == NULL_SEQ){
            DEBUG_PRINT("mallocNode failed\n");
            return NULL_SEQ;
        }
        tree->nodes[index].left = left.index;
        tree->nodes[index].right = right.index;
        push_HuffmanHeap(heap, index, left.times+right.times, heapSize);
        heapSize++;
    }
    // tree Node实际为size-1
    tree->root = heap[0].index;
    // DEBUG_PRINT("tree.size:%d\n",tree->size);

    // 生成Huffman编码
    huffman_code_t values[256];
    #ifdef HUFFMANTREE_TIMES_ENABLE
        newDataExpectedLength = 0; 
    #endif
    
    if(!processHuffmanTree(tree,tree->root,0,0,values)){
        return NULL_SEQ;
    }
    #ifdef HUFFMANTREE_TIMES_ENABLE
        DEBUG_PRINT("newDataExpectedLength:%d\n",newDataExpectedLength/8);
        if(newDataExpectedLength > newDataMAXLength*8){
            DEBUG_PRINT("newDataExpectedLength:%d,newDataMAXLength:%d,newData will overflow\n",newDataExpectedLength,newDataMAXLength);
            return NULL_SEQ;
        }
    #endif

    // 生成newData
    uint8_t curRest = 8;
    uint16_t newDataTop = 0;

    for (int i = 0; i < dataLength; i++)
    {
        uint8_t val = data[i];
        huffman_code_t code = values[val];
        // DEBUG_PRINT("val:%d,length:%d,value:%X,code:%X\n",val,code>>24,code & ((1<<24)-1),code);
        huffman_code_t length = (code>>HUFFMAN_CODE_MAX_LENGTH) & HUFFMAN_CODE_MAX_LENGTH_VALUE;
        code = code & HUFFMAN_CODE_MAX_VALUE;
        while (length > 0) {
            // 当前剩余空间足够填充一部分
            if (curRest <= length) {
                // 填充当前字节所有剩余空间
                newData[newDataTop] |= (code >> (length - curRest));  // 获取前 curRest 位
                code &= ((1 << (length - curRest)) - 1);  // 清除已处理的部分
                length -= curRest;  // 更新剩余长度
                newDataTop++;  // 移动到下一个字节
                curRest = 8;  // 重置 curRest 为 8，表示下一个字节的剩余空间
                if (newDataTop >= newDataMAXLength) {
                    DEBUG_PRINT("newDataTop: %d, newDataMAXLength: %d, newData is full\n", newDataTop, newDataMAXLength);
                    return NULL_SEQ;
                }
                newData[newDataTop] = 0;  // 为下一个字节清零
            } else {
                // 当前剩余空间足够填充完整的字节
                newData[newDataTop] |= code << (curRest - length);  // 填充前 curRest 位
                curRest -= length;  // 更新 curRest
                length = 0;  // 所有数据都已处理
            }
        }
    }
    // 考虑huffmanTree不定长，可能残余数据，规定最后一个字节的剩余最高位为1，其余为0，若最后一位刚好用完则新开一个字节
    if(curRest > 0){
        newData[newDataTop] |= (1 << (curRest - 1));
        newDataTop++;
    }else{
        newDataTop++;
        if (newDataTop >= newDataMAXLength) {
            DEBUG_PRINT("newDataTop: %d, newDataMAXLength: %d, newData is full\n", newDataTop, newDataMAXLength);
            return NULL_SEQ;
        }
        newData[newDataTop] = 0x80;
        newDataTop++;
    }
    return newDataTop;
}

// 递归遍历Huffman树，生成Huffman编码
bool processHuffmanTree(HuffmanTree *tree, uint16_t root, uint8_t length, huffman_code_t value, huffman_code_t* values){
    // 左取0，右取1
    if(tree->nodes[root].left != NULL_SEQ){
        if(!processHuffmanTree(tree, tree->nodes[root].left, length+1, value<<1, values)){
            return false;
        }
    }
    if(tree->nodes[root].right != NULL_SEQ){
        if(!processHuffmanTree(tree, tree->nodes[root].right, length+1, (value<<1)+1, values)){
            return false;
        }
    }
    if(tree->nodes[root].left == NULL_SEQ && tree->nodes[root].right == NULL_SEQ){
        if(length > HUFFMAN_CODE_MAX_LENGTH){
            DEBUG_PRINT("HuffmanTree length is too long\n");
            return false;
        }
        
        huffman_code_t code = (length<<HUFFMAN_CODE_MAX_LENGTH) | value;
        #ifdef HUFFMANTREE_TIMES_ENABLE
            newDataExpectedLength += length * tree->nodes[root].times;
        #endif
        // 记录旧字典和Huffman字典的对应关系
        values[tree->nodes[root].value] = code;
    }
    return true;
}

void printHuffmanTree(HuffmanTree *tree, uint16_t printSize){
    DEBUG_PRINT("HuffmanTree size:%d,root:%d,free:%d\n",tree->size,tree->root,tree->free);
    for (int i = 0; i < tree->size && i < printSize; i++)
    {
        if(tree->nodes[i].value != NULL_VALUE){
            DEBUG_PRINT("index:%d,value:%X,left:%d,right:%d\n",i,tree->nodes[i].value,tree->nodes[i].left,tree->nodes[i].right);
        }
    }
}

uint16_t huffmanDecode(uint8_t *data, uint16_t dataLength, HuffmanTree *tree, uint8_t *newData, uint16_t newDataMAXLength){

    HuffmanTreeNode *root = &tree->nodes[tree->root];
    HuffmanTreeNode *p = root;
    uint16_t newDataTop = 0;

    for (int i = 0; i < dataLength; i++) {
        uint8_t val = data[i];

        for (int j = 0; j < 8; j++) {
            uint8_t bit = (val >> (7 - j)) & 1;
            if (bit == 0) {
                // 转到左子节点
                if (p->left != NULL_SEQ) {
                    p = &tree->nodes[p->left];
                } else {
                    DEBUG_PRINT("[HuffmanDecode] left is NULL_SEQ\n");
                    return NULL_SEQ;
                }
            } else {
                // 处理结束标识
                if (i == dataLength - 1 &&(val & (1 << (7 - j))-1) == 0) {
                    break;
                }
                // 转到右子节点
                if (p->right != NULL_SEQ) {
                    p = &tree->nodes[p->right];
                } else {
                    DEBUG_PRINT("[HuffmanDecode] right is NULL_SEQ\n");
                    return NULL_SEQ;
                }
            }

            // 如果到达叶子节点
            if (p->left == NULL_SEQ && p->right == NULL_SEQ) {
                if (newDataTop >= newDataMAXLength) {
                    DEBUG_PRINT("[HuffmanDecode] newDataTop: %d, newDataMAXLength: %d, newData is full\n", newDataTop, newDataMAXLength);
                    return NULL_SEQ;
                }
                newData[newDataTop++] = p->value;  // 存储叶子节点的值
                p = root;  // 返回根节点，开始解码下一个符号
            }
        }
    }

    return newDataTop;
}

