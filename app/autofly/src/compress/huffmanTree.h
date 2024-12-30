#ifndef HUFFMANTREE_H
#define HUFFMANTREE_H
#include "compressBaseStruct.h"
#include "stdbool.h"
#include "stdint.h"

#define MAX_HUFFMAN_TREE_SIZE 512
// #define HUFFMANTREE_TIMES_ENABLE

#define huffman_code_t uint16_t
#define HUFFMAN_CODE_MAX_LENGTH 12
#define HUFFMAN_CODE_MAX_LENGTH_VALUE 0xf
#define HUFFMAN_CODE_MAX_VALUE 0x0fff

typedef struct {
    uint16_t index;
    times_t times;
}HeapNode;

typedef struct HuffmanTreeNode
{
    value_t value;
    #ifdef HUFFMANTREE_TIMES_ENABLE
        times_t times;
    #endif
    uint16_t left;
    uint16_t right; //对于空闲节点指向下一个空闲节点
} HuffmanTreeNode;

typedef struct HuffmanTree
{
    uint16_t size;  //树的大小
    uint16_t root;  //根节点
    uint16_t free; //指向第一个空闲节点
    HuffmanTreeNode nodes[MAX_HUFFMAN_TREE_SIZE];
} HuffmanTree;

void initHuffmanTree(HuffmanTree *tree);
uint16_t mallocNode(HuffmanTree *tree);
void freeNode(HuffmanTree *tree, uint16_t index);

uint16_t huffmanEnCode(uint8_t *data, uint16_t dataLength, HuffmanTree *tree, uint8_t *newData, uint16_t newDataMAXLength);

uint16_t huffmanDecode(uint8_t *data, uint16_t dataLength, HuffmanTree *tree, uint8_t *newData, uint16_t newDataMAXLength);

void printHuffmanTree(HuffmanTree *tree, uint16_t printSize);
#endif