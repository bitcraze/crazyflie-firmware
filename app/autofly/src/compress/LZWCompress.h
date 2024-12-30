#ifndef LZWCOMPRESS_H
#define LZWCOMPRESS_H

#include <stdint.h>
#include "compressBaseStruct.h"

#define MAX_TRIENODE_NUM 1024
#define MAX_LZWDICT_SIZE 1024

#define MAX_TRIE_DEPTH 2

typedef struct{
    value_t value; // 字符值
    uint16_t seq; // 词典序号
    uint16_t child; // 第一个子节点
    uint16_t brother; // 下一个兄弟节点
}TrieNode;

typedef struct
{
    uint16_t size;
    uint16_t root;
    TrieNode nodes[MAX_TRIENODE_NUM];
}Trie;


typedef struct LZWDictNode{
    uint16_t pre;
    value_t value;
}LZWDictNode;

typedef struct LZWDict {
    uint16_t size;
    LZWDictNode nodes[MAX_LZWDICT_SIZE];
} LZWDict;

void initTrie(Trie* tree);
void initLZWDict(LZWDict* lzwDict);

// LZW压缩,新数据127表示位0x7F,128以上将占两个字节表示0x01ff,通过末尾的高位是否为1判断是否连续
uint16_t LZWEncode(uint8_t* data,uint16_t dataLength,LZWDict* lzwDict, uint8_t* newData, uint16_t newDataMaxLength);

uint16_t LZWDecode(uint8_t* data,uint16_t dataLength,LZWDict* lzwDict, uint8_t* newData, uint16_t newDataMaxLength);

void printLZWDict(LZWDict* lzwDict,uint16_t printSize);
#endif