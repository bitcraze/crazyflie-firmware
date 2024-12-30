#include "stdlib.h"
#include "stdbool.h"
#include "time.h"
#include "LZWCompress.h"
#include "debug.h"

uint16_t mallocTrieNode(Trie* tree);
uint16_t addLZWDictRecode(LZWDict* lzwDict,value_t value, uint16_t pre);
uint16_t searchTrieNode(Trie* tree, TrieNode* root, value_t value);
uint16_t addTrieNode(Trie* tree, TrieNode* root, value_t value, uint16_t seq);

void initLZWDict(LZWDict* lzwDict){
    lzwDict->size = 0;
    for(int i = 0;i<MAX_LZWDICT_SIZE;++i){
        lzwDict->nodes[i].pre = NULL_SEQ;
        lzwDict->nodes[i].value = NULL_VALUE;
    }
}

uint16_t addLZWDictRecode(LZWDict* lzwDict,value_t value, uint16_t pre){
    if(lzwDict->size >= MAX_LZWDICT_SIZE){
        DEBUG_PRINT("LZWDict is full\n");
        return NULL_SEQ;
    }
    lzwDict->nodes[lzwDict->size].pre = pre;
    lzwDict->nodes[lzwDict->size].value = value;
    lzwDict->size++;
    return lzwDict->size-1;
}

void initTrie(Trie* tree){
    tree->size = 0;
    tree->root = 0;
    for(int i = 0;i<MAX_TRIENODE_NUM;++i){
        tree->nodes[i].value = NULL_VALUE;
        tree->nodes[i].seq = NULL_SEQ;
        tree->nodes[i].child = NULL_SEQ;
        tree->nodes[i].brother = NULL_SEQ;
    }
}

uint16_t mallocTrieNode(Trie* tree){
    if(tree->size >= MAX_TRIENODE_NUM){
        DEBUG_PRINT("Trie is full\n");
        return NULL_SEQ;
    }
    uint16_t index = tree->size;
    tree->size++;
    return index;
}

uint16_t searchTrieNode(Trie* tree, TrieNode* root, value_t value){
    uint16_t p = root->child;
    while (p != NULL_SEQ)
    {
        if(tree->nodes[p].value == value){
            return p;
        }
        p = tree->nodes[p].brother;
    }
    return NULL_SEQ;
}

uint16_t addTrieNode(Trie* tree, TrieNode* root, value_t value, uint16_t seq){
    uint16_t p = root->child;
    if(p == NULL_SEQ){
        uint16_t index = mallocTrieNode(tree);
        if(index == NULL_SEQ){
            return NULL_SEQ;
        }
        tree->nodes[index].value = value;
        tree->nodes[index].seq = seq;
        root->child = index;
        return index;
    }
    if(tree->nodes[p].value == value){
        return p;
    }
    uint16_t q = tree->nodes[p].brother;
    while (q != NULL_SEQ)
    {
        if(tree->nodes[q].value == value){
            return q;
        }
        p = q;
        q = tree->nodes[q].brother;
    }
    uint16_t index = mallocTrieNode(tree);
    if(index == NULL_SEQ){
        return NULL_SEQ;
    }
    tree->nodes[index].value = value;
    tree->nodes[index].seq = seq;
    tree->nodes[p].brother = index;
    return index;
}

uint16_t LZWEncode(uint8_t* data,uint16_t dataLength,LZWDict* lzwDict, uint8_t* newData, uint16_t newDataMaxLength){

    Trie trie;
    initTrie(&trie);
    uint16_t rootIndex = mallocTrieNode(&trie);
    trie.root = rootIndex;
    TrieNode* root = &trie.nodes[trie.root];

    initLZWDict(lzwDict);

    uint16_t top = 0;
    uint8_t curDepth = 0;

    TrieNode* p = root;
    for(int i = 0; i < dataLength; i++){
        uint8_t val = data[i];
        uint16_t child = searchTrieNode(&trie, p, val);
        if(child != NULL_SEQ){
            p = &trie.nodes[child];
            curDepth++;
            if(curDepth == MAX_TRIE_DEPTH){
                // 达到最大深度，重新开始
                if(top >= newDataMaxLength){
                    DEBUG_PRINT("newData is full\n");
                    return 0;
                }
                if(p->seq >= 0x80){
                    newData[top++] = p->seq >> 7;
                    if(top >= newDataMaxLength){
                        DEBUG_PRINT("newData is full\n");
                        return 0;
                    }
                    newData[top] = p->seq & 0x7F;
                    newData[top] |= 0x80;
                    top++;
                }else{
                    newData[top++] = p->seq;
                }
                p = root;
                curDepth = 0;
            }
        }else{
            // 未找到,添加字典,并且添加到Trie
            uint16_t seq = addLZWDictRecode(lzwDict, val, p->seq);
            if(seq == NULL_SEQ){
                DEBUG_PRINT("addLZWDictRecode failed\n");
                return 0;
            }
            uint16_t nodeIndex = addTrieNode(&trie, p, val, seq);
            if(nodeIndex == NULL_SEQ){
                DEBUG_PRINT("addTrieNode failed\n");
                return 0;
            }
            
            if(top >= newDataMaxLength){
                DEBUG_PRINT("newData is full\n");
                return 0;
            }
            p = &trie.nodes[nodeIndex];
            if(p->seq >= 0x80){
                newData[top++] = p->seq >> 7;
                if(top >= newDataMaxLength){
                    DEBUG_PRINT("newData is full\n");
                    return 0;
                }
                newData[top] = p->seq & 0x7F;
                newData[top] |= 0x80;
                top++;
            }else{
                newData[top++] = p->seq;
            }
            p = root;
            curDepth = 0;
        }
    }
    if(p!=root){
        if(top >= newDataMaxLength){
            DEBUG_PRINT("newData is full\n");
            return 0;
        }
        if(p->seq >= 0x80){
            newData[top++] = p->seq >> 7;
            if(top >= newDataMaxLength){
                DEBUG_PRINT("newData is full\n");
                return 0;
            }
            newData[top] = p->seq & 0x7F;
            newData[top] |= 0x80;
            top++;
        }else{
            newData[top++] = p->seq;
        }
    }

    return top;
}

uint16_t LZWDecode(uint8_t* data, uint16_t dataLength, LZWDict* lzwDict, uint8_t* newData, uint16_t newDataMaxLength) {

    uint16_t newDataTop = 0;

    for (int i = 0; i < dataLength; i++) {
        uint16_t index = data[i];

        // 检查是否有第二个字节，并且第二个字节的最高位为 1
        if (i + 1 < dataLength && (data[i + 1] & 0x80)) {
            index = (index << 7) | (data[i + 1] & 0x7F);
            i++;  // 跳过第二个字节
        }

        // 检查索引是否在字典范围内
        if (index >= lzwDict->size) {
            DEBUG_PRINT("index:%d is out of range\n", index);
            return NULL_SEQ;
        }

        uint16_t p = index;
        uint16_t length = 0;

        // 计算符号的长度
        while (p != NULL_SEQ) {
            p = lzwDict->nodes[p].pre;
            length++;
        }

        p = index;
        uint16_t newDataTopTemp = newDataTop;
        newDataTop += length;
        
        // 检查 newData 是否足够存放解码后的数据
        if (newDataTop >= newDataMaxLength) {
            DEBUG_PRINT("[LZWDecode] newData is full\n");
            return NULL_SEQ;
        }

        // 将解码的数据填入 newData
        while (p != NULL_SEQ) {
            newData[newDataTopTemp + length - 1] = lzwDict->nodes[p].value;
            p = lzwDict->nodes[p].pre;
            length--;
        }
    }

    // 解码完成后，返回 newDataTop，表示解码后的数据长度

    return newDataTop;
}


void printLZWDict(LZWDict* lzwDict,uint16_t printSize){
    DEBUG_PRINT("LZWDict size:%d\n",lzwDict->size);
    for (int i = 0; i < lzwDict->size && i < printSize; i++)
    {
        DEBUG_PRINT("index:%d,pre:%d,value:%d\n",i,lzwDict->nodes[i].pre,lzwDict->nodes[i].value);
    }
}