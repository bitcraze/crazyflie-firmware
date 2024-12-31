#ifndef __BASE_PACKET_H__
#define __BASE_PACKET_H__

#include <stdint.h>

#define AUTOFLY_PACKET_HEAD_LENGTH sizeof(Autofly_packet_Header_t)
#define AUTOFLY_PACKET_MTU 60-AUTOFLY_PACKET_HEAD_LENGTH

typedef enum{
    // request
    MAPPING_REQ = 0x10, // mapping
    EXPLORE_REQ = 0x20, // explore
    PATH_REQ = 0x30,    // path
    CLUSTER_REQ = 0x40,  // cluster
    
    TERMINATE = 0xFF,   // terminate

    // response
    EXPLORE_RESP = 0x2A,
    PATH_RESP = 0x3A,
    CLUSTER_RESP = 0x4A,

    // octoMapData
    OCTOMAP_DATA = 0x50,
    OCTOMAP_FIN = 0x51,
    OCTOMAP_DATA_FRAGEMENT = 0x52,
    OCTOMAP_DATA_FRAGEMENT_SACK = 0x53,
    OCTOMAP_ERROR = 0x54,
    OCTOMAP_ERROR_MISS_BUFFER = 0x55, // 缓冲已被清除
    OCTOMAP_ERROR_HAS_PROCESSED = 0x56, // 数据已被处理
    OCTOMAP_ERROR_TX_WAITING_TIMEOUT = 0x57, // 超时
    OCTOMAP_ERROR_RX_WAITING_TIMEOUT = 0x58, // 超时
    OCTOMAP_RECEIVE_BUSY = 0x59, // 忙碌
}packetType_t;

typedef struct{
    uint8_t sourceId;
    uint8_t destinationId;
    uint8_t packetType;
    uint8_t length;
}Autofly_packet_Header_t;  

typedef struct
{   
    Autofly_packet_Header_t header;
    uint8_t data[AUTOFLY_PACKET_MTU];
} Autofly_packet_t;   // 60

#endif