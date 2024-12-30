#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include "semphr.h"

#include "communicate.h"
#include "octoMapDataCommunication.h"
#include "octoMapSerializer.h"

#include "radiolink.h"

#define FRAGMENT_SEND_INTERVAL 20 // 分片发送间隔

// todo: 完善
static uint8_t dataTXBuffer[MAX_OCTOMAP_DATA_LENGTH];
static uint16_t dataTXLength;
static uint16_t dataTXId; // 数据ID,检测数据是否被更新

static SemaphoreHandle_t muDataRX;
static uint8_t dataRXBuffer[MAX_OCTOMAP_DATA_LENGTH];
static uint16_t dataRXLength;
static uint16_t dataRXId; // 数据ID,检测数据是否被更新
static uint8_t checkFLag[64]; // 检测某位片数据是否被接收
static uint8_t curCountNum; // 当前接收的数据数量


bool processOctoMapFragMent(Autofly_packet_t *packet);
bool processOctoMapPacket_Sack(Autofly_packet_t *packet);

void octoMapDataCommunicationInit(){
    muDataRX = xSemaphoreCreateMutex();
    dataTXLength = 0;
    dataTXId = 0;

    dataRXLength = 0;
    dataRXId = 0;
    memset(checkFLag, 0, 64);
    curCountNum = 0;
}

bool sendOctoMapData(uint16_t destAddress, uint8_t *data, uint8_t length){
    if(length > MAX_OCTOMAP_DATA_LENGTH){
        DEBUG_PRINT("[sendOctoMapData]data is too long, dataTXBuffer maxLength = %d, data length = %d\n", MAX_OCTOMAP_DATA_LENGTH, length);
        return false;
    }
    if(length >= MAX_PACKET_LENGTH){
        // 数据过长
        DEBUG_PRINT("[sendOctoMapData]data is too long, maxPacketLength = %d, data length = %d\n", MAX_PACKET_LENGTH, length);
        return false;
    }
    else{
        dataTXId++;
        memcpy(dataTXBuffer, data, length);
        // 分片发送，不足一片也按一片发送
        // 计算分片数量,向上取整
        uint8_t fragementCount = ((length + MAX_FRAGEMENT_DATA_LENGTH - 1) / MAX_FRAGEMENT_DATA_LENGTH);
        for (int i = 0; i < fragementCount; i++)
        {
            octoMapFragement_t fragement;
            fragement.fragementHeader.dataId = dataTXId;
            fragement.fragementHeader.fragementId = i;
            fragement.fragementHeader.fragementCount = fragementCount;
            if(i == fragementCount - 1){
                fragement.fragementHeader.fragementLength = length - i*MAX_FRAGEMENT_DATA_LENGTH;
            }
            else{
                fragement.fragementHeader.fragementLength = MAX_FRAGEMENT_DATA_LENGTH;
            }
            memcpy(fragement.data, dataTXBuffer+i*MAX_FRAGEMENT_DATA_LENGTH, fragement.fragementHeader.fragementLength);
            sendAutoFlyPacket(destAddress, OCTOMAP_DATA_FRAGEMENT, (uint8_t*)&fragement, fragement.fragementHeader.fragementLength + sizeof(octoMapFragmentHeader_t));
            vTaskDelay(M2T(FRAGMENT_SEND_INTERVAL));
        }
    }
}

bool processOctoMapData(Autofly_packet_t* packet){
    switch (packet->header.packetType)
    {
    case OCTOMAP_DATA_FRAGEMENT:
        return processOctoMapFragMent(packet);
    case OCTOMAP_DATA_FRAGEMENT_SACK:
        return processOctoMapPacket_Sack(packet);
    default:
        break;
    }
}

bool processOctoMapFragMent(Autofly_packet_t *packet){
    xSemaphoreTake(muDataRX, portMAX_DELAY);
    octoMapFragement_t *fragement = (octoMapFragement_t*)packet->data;
    bool res = false;
    if(fragement->fragementHeader.dataId >= dataRXId){
        if(fragement->fragementHeader.dataId > dataRXId){
            // 新数据,接收到新数据代表旧数据已被抛弃
            dataRXId = fragement->fragementHeader.dataId;
            dataRXLength = 0;
            curCountNum = 0;
            memset(checkFLag, 0, 64);
        }
        // 同一组数据
        // 判断是否重复
        uint8_t index = fragement->fragementHeader.fragementId / 8;
        uint8_t offset = fragement->fragementHeader.fragementId % 8;
        if(checkFLag[index] & (1 << offset)){
            // 重复数据
            DEBUG_PRINT("[processOctoMapData]repeat data, dataId = %d, fragementId = %d\n", fragement->fragementHeader.dataId, fragement->fragementHeader.fragementId);
            res = false;
        }
        else{
            // 未重复
            memcpy(dataRXBuffer + fragement->fragementHeader.fragementId * MAX_FRAGEMENT_DATA_LENGTH, fragement->data, fragement->fragementHeader.fragementLength);
            checkFLag[index] |= (1 << offset);
            curCountNum++;
            dataRXLength += fragement->fragementHeader.fragementLength;
            if(curCountNum == fragement->fragementHeader.fragementCount){
                // 数据接收完
                // todo: 处理数据

                // 数据处理完,清空数据
                // 发送FIN
                sendAutoFlyPacket(packet->header.sourceId, OCTOMAP_FIN, NULL, 0);
                curCountNum = 0;
                dataRXId ++;
                memset(checkFLag, 0, 64);
            }
            res = true;
        }
    }
    xSemaphoreGive(muDataRX);
    return res;
}

bool processOctoMapPacket_Sack(Autofly_packet_t *packet){
    // 处理SACK
    octoMapPacket_Sack_t *sack = (octoMapPacket_Sack_t*)packet->data;
    // 判断数据是否过期
    if(sack->header.dataId < dataTXId){
        // 数据已过期
        sendAutoFlyPacket(packet->header.sourceId, OCTOMAP_ERROR_MISS_BUFFER, NULL, 0);
        return false;
    }else if(sack->header.dataId > dataTXId){
        // 数据不存在
        sendAutoFlyPacket(packet->header.sourceId, OCTOMAP_ERROR, NULL, 0);
        return false;
    }
    else{
        // 数据丢失重传
        uint8_t fragementCount = ((dataTXLength + MAX_FRAGEMENT_DATA_LENGTH - 1) / MAX_FRAGEMENT_DATA_LENGTH);
        if(sack->header.type == DISCRETE){
            for (int i = 0; i < sack->header.length; i++)
            {
                // todo: 优化，新建单独线程负责所有发送报文内容，避免导致处理数据的线程阻塞
                // 重发丢失数据
                uint8_t fragementId = sack->missDataId[i];
                octoMapFragement_t fragement;
                fragement.fragementHeader.dataId = dataTXId;
                fragement.fragementHeader.fragementId = fragementId;
                fragement.fragementHeader.fragementCount = fragementCount;
                if(fragementId == fragementCount - 1){
                    fragement.fragementHeader.fragementLength = dataTXLength - fragementId*MAX_FRAGEMENT_DATA_LENGTH;
                }
                else{
                    fragement.fragementHeader.fragementLength = MAX_FRAGEMENT_DATA_LENGTH;
                }
                memcpy(fragement.data, dataTXBuffer+fragementId*MAX_FRAGEMENT_DATA_LENGTH, fragement.fragementHeader.fragementLength);
                sendAutoFlyPacket(packet->header.sourceId, OCTOMAP_DATA_FRAGEMENT, (uint8_t*)&fragement, fragement.fragementHeader.fragementLength + sizeof(octoMapFragmentHeader_t));
                vTaskDelay(M2T(FRAGMENT_SEND_INTERVAL));
            }
        }else{
            // 连续数据丢失
            // 读取两个值
            for (int i = 0; i < sack->header.length; i+=2)
            {
                uint8_t start = sack->missDataId[i];
                uint8_t end = sack->missDataId[i+1];
                for (int j = start; j <= end; j++)
                {
                    // 重发丢失数据
                    octoMapFragement_t fragement;
                    fragement.fragementHeader.dataId = dataTXId;
                    fragement.fragementHeader.fragementId = j;
                    fragement.fragementHeader.fragementCount = fragementCount;
                    if(j == fragementCount - 1){
                        fragement.fragementHeader.fragementLength = dataTXLength - j*MAX_FRAGEMENT_DATA_LENGTH;
                    }
                    else{
                        fragement.fragementHeader.fragementLength = MAX_FRAGEMENT_DATA_LENGTH;
                    }
                    memcpy(fragement.data, dataTXBuffer+j*MAX_FRAGEMENT_DATA_LENGTH, fragement.fragementHeader.fragementLength);
                    sendAutoFlyPacket(packet->header.sourceId, OCTOMAP_DATA_FRAGEMENT, (uint8_t*)&fragement, fragement.fragementHeader.fragementLength + sizeof(octoMapFragmentHeader_t));
                    vTaskDelay(M2T(FRAGMENT_SEND_INTERVAL));
                }
            }
        }
    }
    
}