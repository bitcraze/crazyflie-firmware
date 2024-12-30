#ifndef __OCTOMAP_DATA_COMMUNICATION_H__
#define __OCTOMAP_DATA_COMMUNICATION_H__

#include "stdint.h"
#include "autoflyPacket.h"
#include "autoflyPacket.h"

#define MAX_FRAGEMENT_DATA_LENGTH AUTOFLY_PACKET_MTU-sizeof(octoMapFragmentHeader_t)
#define MAX_SACK_DATA_LENGTH AUTOFLY_PACKET_MTU-sizeof(octoMapPacket_Sack_Header_t)

#define MAX_PACKET_LENGTH MAX_FRAGEMENT_DATA_LENGTH*255

typedef enum{
    DISCRETE = 0, // 离散
    CONTINUOUS // 连续
}sack_type_t;

typedef struct
{
    uint8_t dataId; // 数据ID
    uint8_t fragementId; // 分片ID
    uint8_t fragementCount; // 分片总数,最大255，255*53/1024 = 13.1KB
    uint16_t fragementLength; // 分片长度 不足一片也将按一片发送
}octoMapFragmentHeader_t;

typedef struct 
{
    octoMapFragmentHeader_t fragementHeader;
    uint8_t data[MAX_FRAGEMENT_DATA_LENGTH];
}octoMapFragement_t;

typedef struct 
{
    uint8_t dataId; // 数据ID
    uint8_t length; // 数据长度
    sack_type_t type; // SACK类型,离散或连续,默认离散
}octoMapPacket_Sack_Header_t;

typedef struct
{
    octoMapPacket_Sack_Header_t header;
    uint8_t missDataId[MAX_SACK_DATA_LENGTH]; // 丢失数据ID
}octoMapPacket_Sack_t;

bool sendOctoMapData(uint16_t destAddress, uint8_t *data, uint8_t length);
bool processOctoMapData(Autofly_packet_t* packet);
#endif