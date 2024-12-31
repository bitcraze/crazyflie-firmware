#include <string.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include "semphr.h"

#include "octoMapDataCommunication.h"
#include "autoflyPacket.h"
#include "communicate.h"
#include "octoMapSerializer.h"

#include "radiolink.h"

#define FRAGMENT_SEND_INTERVAL 20 // 分片发送间隔

#define TX_ACK_WAIT_TIME_INIT 50 // ACK等待次数初值
#define MAX_TX_RETRY_TIME 3 // 最大重传次数

#define RX_DATA_UPDATE_WAIT_TIME_INIT 50 // 数据更新等待次数初值
#define MAX_RX_SACK_SEND_TIME 3 // 最大SACK发送次数

#define MAX_OCTOMAP_SEND_SLEEP_TIME 1000 // 发送休眠时间
#define MAX_OCTOMAP_SEND_SLEEP_CHECK_INTERVAL 10 // 发送休眠检测间隔

static TaskHandle_t octoMapDataCommunicationTxTaskHandle;
static TaskHandle_t octoMapDataCommunicationRxTaskHandle;

static OCTOMAP_DATA_COMMUNICATION_STATE TX_STATE; // 发送状态
static SemaphoreHandle_t muDataTX;
static uint8_t dataTXBuffer[MAX_OCTOMAP_DATA_LENGTH];
static uint16_t dataTXLength;
static uint16_t dataTXId; // 数据ID,检测数据是否被更新
static uint8_t destinationTxId; // 目标地址
static bool isSackReceived; // 是否收到SACK
static Autofly_packet_t lastReceiveSackPacket; // 上一个SACK包

static OCTOMAP_DATA_COMMUNICATION_STATE RX_STATE; // 接收状态
static SemaphoreHandle_t muDataRX;
static uint8_t dataRXBuffer[MAX_OCTOMAP_DATA_LENGTH];
static uint16_t dataRXLength;
static uint8_t sourceRxId; // 目标地址
static uint16_t dataRXId; // 数据ID,检测数据是否被更新
static uint8_t checkFLag[64]; // 检测某位片数据是否被接收
static uint8_t totalCountNum; // 总数据数量
static uint8_t curCountNum; // 当前接收的数据数量

static uint8_t TxAckTimeoutTimes; // 发送响应超时时间
static uint8_t maxTxAckWaitTimes; // ACK最大超时时间
static uint8_t retryTimes; // 重传次数

static uint8_t RxDataUpdateTimeoutTimes; // 接收数据更新超时次数
static uint8_t maxRxDataUpdateWaitTimes; // 数据更新最大等待次数
static uint8_t RxSackSendTimes; // SACK发送次数
static octoMapPacket_Sack_t lastSendSackPacket; // 上一个SACK包

void octoMapDataCommunicationRxTask(void * parameter);
void octoMapDataCommunicationTxTask(void * parameter);
bool processOctoMapFragMent(Autofly_packet_t *packet);
bool processOctoMapPacket_Sack(Autofly_packet_t *packet);

void octoMapDataCommunicationInit(){
    muDataTX = xSemaphoreCreateMutex();
    muDataRX = xSemaphoreCreateMutex();
    octoMapDataCommunicationRxInit();
    octoMapDataCommunicationTxInit();
    
    xTaskCreate(octoMapDataCommunicationTxTask, OCTOMAP_DATA_TX_TASK_NAME, OCTOMAP_DATA_TX_TASK_STACK_SIZE, NULL, OCTOMAP_DATA_TX_TASK_PRI, &octoMapDataCommunicationTxTaskHandle);
    xTaskCreate(octoMapDataCommunicationRxTask, OCTOMAP_DATA_RX_TASK_NAME, OCTOMAP_DATA_RX_TASK_STACK_SIZE, NULL, OCTOMAP_DATA_RX_TASK_PRI, &octoMapDataCommunicationRxTaskHandle);
}

void octoMapDataCommunicationRxInit(){
    RX_STATE = IDLE;
    dataRXLength = 0;
    dataRXId = 0;
    memset(checkFLag, 0, 64);
    curCountNum = 0;

    RxDataUpdateTimeoutTimes = 0;
    maxRxDataUpdateWaitTimes = RX_DATA_UPDATE_WAIT_TIME_INIT;
    RxSackSendTimes = 0;
}

void octoMapDataCommunicationTxInit(){
    TX_STATE = IDLE;
    dataTXLength = 0;
    dataTXId = 0;

    TxAckTimeoutTimes = 0;
    maxTxAckWaitTimes = TX_ACK_WAIT_TIME_INIT;
    retryTimes = 0;
}


bool sendOctoMapData(uint16_t destAddress, uint8_t *data, uint8_t length){
    // 进入发送状态
    xSemaphoreTake(muDataTX, portMAX_DELAY);
    TX_STATE = DATA_SENDING;
    // 初始化响应超时时间、最大超时时间和重传次数
    TxAckTimeoutTimes = 0;
    maxTxAckWaitTimes = TX_ACK_WAIT_TIME_INIT;
    retryTimes = 0;
    // 初始化SACK接收状态
    isSackReceived = false;
    // 更新数据ID和目标地址
    dataTXId++;
    destinationTxId = destAddress;
    if(length > MAX_OCTOMAP_DATA_LENGTH){
        DEBUG_PRINT("[sendOctoMapData]data is too long, dataTXBuffer maxLength = %d, data length = %d\n", MAX_OCTOMAP_DATA_LENGTH, length);
        TX_STATE = FAILED;
        return false;
    }
    if(length >= MAX_PACKET_LENGTH){
        // 数据过长
        DEBUG_PRINT("[sendOctoMapData]data is too long, maxPacketLength = %d, data length = %d\n", MAX_PACKET_LENGTH, length);
        TX_STATE = FAILED;
        return false;
    }
    else{
        memcpy(dataTXBuffer, data, length);
        dataTXLength = length;
        // 分片发送，不足一片也按一片发送
        // 计算分片数量,向上取整
        uint8_t fragementCount = ((dataTXLength + MAX_FRAGEMENT_DATA_LENGTH - 1) / MAX_FRAGEMENT_DATA_LENGTH);
        for (int i = 0; i < fragementCount; i++)
        {
            octoMapFragement_t fragement;
            fragement.fragementHeader.dataId = dataTXId;
            fragement.fragementHeader.fragementId = i;
            fragement.fragementHeader.fragementCount = fragementCount;
            if(i == fragementCount - 1){
                fragement.fragementHeader.fragementLength = dataTXLength - i*MAX_FRAGEMENT_DATA_LENGTH;
            }
            else{
                fragement.fragementHeader.fragementLength = MAX_FRAGEMENT_DATA_LENGTH;
            }
            memcpy(fragement.data, dataTXBuffer+i*MAX_FRAGEMENT_DATA_LENGTH, fragement.fragementHeader.fragementLength);
            sendAutoFlyPacket(destAddress, OCTOMAP_DATA_FRAGEMENT, (uint8_t*)&fragement, fragement.fragementHeader.fragementLength + sizeof(octoMapFragmentHeader_t));
            vTaskDelay(M2T(FRAGMENT_SEND_INTERVAL));
        }
    }
    // 数据发送完成
    xSemaphoreGive(muDataTX);
    TX_STATE = DATA_SENDED;
}

bool sendOctomapSack(){
    RX_STATE = SACK_SEND;
    // 发送SACK
    if(RxSackSendTimes > 1){
        // RxSackSendTimes > 1 说明在此期间数据未被更新，sack无需重新生成
        sendAutoFlyPacket(sourceRxId, OCTOMAP_DATA_FRAGEMENT_SACK, (uint8_t*)&lastSendSackPacket, lastSendSackPacket.header.length + sizeof(octoMapPacket_Sack_Header_t));
        RX_STATE = DATA_WAITING;
        return true;
    }
    lastSendSackPacket.header.dataId = dataRXId;
    // 统计缺失数据
    uint8_t missCountDiscrete = 0;
    uint8_t missCountContinuous = 0;
    bool iscontinuous = false;
    for (int i = 0; i < totalCountNum; i++)
    {
        uint8_t index = i / 8;
        uint8_t offset = i % 8;
        if(!(checkFLag[index] & (1 << offset))){
            missCountDiscrete++;
            if(!iscontinuous){
                iscontinuous = true;
                missCountContinuous++;
            }
        }else{
            if(iscontinuous){
                missCountContinuous++;
                iscontinuous = false;
            }
        }
    }
    if(iscontinuous){
        missCountContinuous++;
    }
    if(missCountDiscrete <= missCountContinuous){
        // 缺失数据较离散
        if(missCountDiscrete > MAX_SACK_DATA_LENGTH){
            // 缺失数据过多
            DEBUG_PRINT("[sendOctomapSack]miss data is too much, missCountDiscrete = %d\n", missCountDiscrete);
            RX_STATE = FAILED;
            return false;
        }
        lastSendSackPacket.header.type = DISCRETE;
        for (int i = 0; i < totalCountNum; i++)
        {
            uint8_t index = i / 8;
            uint8_t offset = i % 8;
            if(!(checkFLag[index] & (1 << offset))){
                lastSendSackPacket.missDataId[lastSendSackPacket.header.length++] = i;
            }
        }
    }else{
        // 缺失数据较连续
        if(missCountContinuous > MAX_SACK_DATA_LENGTH){
            // 缺失数据过多
            DEBUG_PRINT("[sendOctomapSack]miss data is too much, missCountContinuous = %d\n", missCountContinuous);
            RX_STATE = FAILED;
            return false;
        }
        lastSendSackPacket.header.type = CONTINUOUS;
        iscontinuous = false;
        for (int i = 0; i < totalCountNum; i++)
        {
            uint8_t index = i / 8;
            uint8_t offset = i % 8;
            if(!(checkFLag[index] & (1 << offset))){
                if(!iscontinuous){
                    iscontinuous = true;
                    lastSendSackPacket.missDataId[lastSendSackPacket.header.length++] = i;
                }
            }else{
                if(iscontinuous){
                    iscontinuous = false;
                    lastSendSackPacket.missDataId[lastSendSackPacket.header.length++] = i-1;
                }
            }
        }
    }
    sendAutoFlyPacket(sourceRxId, OCTOMAP_DATA_FRAGEMENT_SACK, (uint8_t*)&lastSendSackPacket, lastSendSackPacket.header.length + sizeof(octoMapPacket_Sack_Header_t));
    RX_STATE = DATA_WAITING;
    return true;
}

bool processOctoMapData(Autofly_packet_t* packet){
    octoMapPacket_Error_t *error = NULL;
    switch (packet->header.packetType)
    {
    case OCTOMAP_DATA_FRAGEMENT:
        return processOctoMapFragMent(packet);
    case OCTOMAP_DATA_FRAGEMENT_SACK:
        return processOctoMapPacket_Sack(packet);
    case OCTOMAP_FIN:
        // 数据接收完成
        TX_STATE = FIN;
        break;
    case OCTOMAP_ERROR_MISS_BUFFER:
        // 发送方已清除error->dataId数据
        error = (octoMapPacket_Error_t*)packet->data;
        if(error->dataId == dataRXId){
            RX_STATE = FAILED;    
        }
        break;
    case OCTOMAP_ERROR_HAS_PROCESSED:
        // error->dataId数据对方已处理过
        error = (octoMapPacket_Error_t*)packet->data;
        if(error->dataId == dataTXId){
            TX_STATE = FAILED;    
        }
        break;
    case OCTOMAP_ERROR_TX_WAITING_TIMEOUT:
        // 发送方对error->dataId数据等待超时
        error = (octoMapPacket_Error_t*)packet->data;
        if(error->dataId == dataRXId){
            RX_STATE = FAILED;    
        }
        break;
    case OCTOMAP_ERROR_RX_WAITING_TIMEOUT:
        // 接收方对error->dataId数据等待超时
        error = (octoMapPacket_Error_t*)packet->data;
        if(error->dataId == dataTXId){
            TX_STATE = FAILED;    
        }
        break;
    case OCTOMAP_RECEIVE_BUSY:
        // 忙碌
        TX_STATE = ACK_BUSY;
        break;
    default:
        break;
    }
}


bool processOctoMapFragMent(Autofly_packet_t *packet){
    if(RX_STATE != IDLE && packet->header.sourceId != sourceRxId){
        // 不是当前源设备发送的数据
        sendAutoFlyPacket(packet->header.sourceId, OCTOMAP_RECEIVE_BUSY, NULL, 0);
        return false;
    }
    xSemaphoreTake(muDataRX, portMAX_DELAY);
    RX_STATE = DATA_RECEIVING;
    octoMapFragement_t *fragement = (octoMapFragement_t*)packet->data;
    bool res = false;
    if(packet->header.sourceId == sourceRxId && fragement->fragementHeader.dataId < dataRXId){
        // 同一个源设备发送的数据，数据id小于当前数据id，代表已经是历史数据
        octoMapPacket_Error_t error;
        error.dataId = fragement->fragementHeader.dataId;
        sendAutoFlyPacket(packet->header.sourceId, OCTOMAP_ERROR_HAS_PROCESSED, (uint8_t*)&error, sizeof(octoMapPacket_Error_t));
    }
    else{
        if(RX_STATE = IDLE | fragement->fragementHeader.dataId > dataRXId){
            // 新数据,接收到新数据代表旧数据已被抛弃
            dataRXId = fragement->fragementHeader.dataId;
            dataRXLength = 0;
            curCountNum = 0;
            memset(checkFLag, 0, 64);
            sourceRxId = packet->header.sourceId;
            totalCountNum = fragement->fragementHeader.fragementCount;
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
            // 接收到新数据清空接收数据更新超时时间、最大超时时间和SACK发送时间
            RxDataUpdateTimeoutTimes = 0;
            if(maxRxDataUpdateWaitTimes > RX_DATA_UPDATE_WAIT_TIME_INIT){
                maxRxDataUpdateWaitTimes = maxRxDataUpdateWaitTimes/2;
            }
            RxSackSendTimes = 0;
            if(curCountNum == totalCountNum){
                // 数据接收完
                // todo: 处理数据

                // 数据处理完,清空数据
                curCountNum = 0;
                dataRXId ++;
                memset(checkFLag, 0, 64);
                RX_STATE = DATA_RECEIVED;
            }
            else{
                RX_STATE = DATA_WAITING;
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
        octoMapPacket_Error_t error;
        error.dataId = sack->header.dataId;
        sendAutoFlyPacket(packet->header.sourceId, OCTOMAP_ERROR_MISS_BUFFER, (uint8_t*)&error, sizeof(octoMapPacket_Error_t));
        return false;
    }else if(sack->header.dataId > dataTXId){
        // 数据不存在
        octoMapPacket_Error_t error;
        error.dataId = sack->header.dataId;
        sendAutoFlyPacket(packet->header.sourceId, OCTOMAP_ERROR, (uint8_t*)&error, sizeof(octoMapPacket_Error_t));
        return false;
    }
    else{
        if(TX_STATE != RETRY){
            // 收到有效新SACK，清空ACK超时次数,最大超时时间降档，重传次数清零
            TxAckTimeoutTimes = 0;
            if(maxTxAckWaitTimes > TX_ACK_WAIT_TIME_INIT){
                maxTxAckWaitTimes = maxTxAckWaitTimes/2;
            }
            retryTimes = 0;
            // 更新SACK记录
            isSackReceived = true;
            memcpy(&lastReceiveSackPacket, packet, sizeof(Autofly_packet_t));
        }
        // 收到有效SACK，转入SACK处理状态
        TX_STATE = SACK_PROCESS;
        // 缺失数据重传，转入数据发送状态
        xSemaphoreTake(muDataTX, portMAX_DELAY);
        TX_STATE = DATA_SENDING;
        uint8_t fragementCount = ((dataTXLength + MAX_FRAGEMENT_DATA_LENGTH - 1) / MAX_FRAGEMENT_DATA_LENGTH);
        if(sack->header.type == DISCRETE){
            for (int i = 0; i < sack->header.length; i++)
            {
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
        // 缺失数据重传完成
        xSemaphoreGive(muDataTX);
        TX_STATE = DATA_SENDED;
    }
    
}

void retrySendData(){
    // 重传
    if(isSackReceived){
        processOctoMapPacket_Sack(&lastReceiveSackPacket);
    }else{
        // 重传所有数据
        // 分片发送，不足一片也按一片发送
        // 计算分片数量,向上取整
        xSemaphoreTake(muDataTX, portMAX_DELAY);
        TX_STATE = DATA_SENDING;
        uint8_t fragementCount = ((dataTXLength + MAX_FRAGEMENT_DATA_LENGTH - 1) / MAX_FRAGEMENT_DATA_LENGTH);
        for (int i = 0; i < fragementCount; i++)
        {
            octoMapFragement_t fragement;
            fragement.fragementHeader.dataId = dataTXId;
            fragement.fragementHeader.fragementId = i;
            fragement.fragementHeader.fragementCount = fragementCount;
            if(i == fragementCount - 1){
                fragement.fragementHeader.fragementLength = dataTXLength - i*MAX_FRAGEMENT_DATA_LENGTH;
            }
            else{
                fragement.fragementHeader.fragementLength = MAX_FRAGEMENT_DATA_LENGTH;
            }
            memcpy(fragement.data, dataTXBuffer+i*MAX_FRAGEMENT_DATA_LENGTH, fragement.fragementHeader.fragementLength);
            sendAutoFlyPacket(destinationTxId, OCTOMAP_DATA_FRAGEMENT, (uint8_t*)&fragement, fragement.fragementHeader.fragementLength + sizeof(octoMapFragmentHeader_t));
            vTaskDelay(M2T(FRAGMENT_SEND_INTERVAL));
        }
        xSemaphoreGive(muDataTX);
        TX_STATE = DATA_SENDED;
    }
}

void octoMapDataCommunicationTxTask(void * parameter){
    while (1)
    {
        switch (TX_STATE)
        {
            case IDLE:
                break;
            case FAILED:
                octoMapDataCommunicationTxInit();
                TX_STATE = IDLE;
                break;
            case FIN:
                TX_STATE = IDLE;
                break;
            case DATA_SENDING:
                break;
            case DATA_SENDED:
                // 发送完成转入ACK等待状态
                TX_STATE = ACK_WAIT;
                break;
            case ACK_WAIT:
                // 等待ACK，累积ACK超时次数
                TxAckTimeoutTimes++;
                // 超时重传
                if(TxAckTimeoutTimes > maxTxAckWaitTimes){
                    TX_STATE = ACK_TIMEOUT;
                }
                break;
            case ACK_RECEIVED:
                break;
            case ACK_TIMEOUT:
                TX_STATE = RETRY;
                TxAckTimeoutTimes = 0;
                // 重传间隔翻倍
                maxTxAckWaitTimes *= 2;
                break;
            case RETRY:
                retryTimes++;
                if(retryTimes > MAX_TX_RETRY_TIME){
                    octoMapPacket_Error_t error;
                    error.dataId = dataTXId;
                    sendAutoFlyPacket(destinationTxId, OCTOMAP_ERROR_TX_WAITING_TIMEOUT, (uint8_t*)&error, sizeof(octoMapPacket_Error_t));
                    TX_STATE = FAILED;
                }else{
                    retrySendData();
                }
                break;
            case ACK_BUSY:
                TX_STATE = DATA_SEND_SLEEP;
                break;
            case DATA_SEND_SLEEP:
                // 在C89标准下，标签后必须紧跟一个语句，而不能直接是变量声明。
                TX_STATE = DATA_SEND_SLEEP;
                int j = 0;
                for (; j < MAX_OCTOMAP_SEND_SLEEP_TIME / MAX_OCTOMAP_SEND_SLEEP_CHECK_INTERVAL; j++)
                {
                    if(TX_STATE != DATA_SEND_SLEEP){
                        break;
                    }
                    vTaskDelay(M2T(MAX_OCTOMAP_SEND_SLEEP_CHECK_INTERVAL));
                }
                // 自然结束休眠状态，重传数据
                if(j == MAX_OCTOMAP_SEND_SLEEP_TIME / MAX_OCTOMAP_SEND_SLEEP_CHECK_INTERVAL){
                    retrySendData();
                }
                break;
            default:
                break;
        }
        vTaskDelay(M2T(STATE_CHECK_INTERVAL));
    }
    
}

void octoMapDataCommunicationRxTask(void * parameter){
    while (1)
    {
        switch (RX_STATE)
        {
            case IDLE:
                break;
            case FAILED:
                octoMapDataCommunicationRxInit();
                RX_STATE = IDLE;
                break;
            case FIN:
                RX_STATE = IDLE;
                break;
            case DATA_RECEIVING:
                break;
            case DATA_RECEIVED:
                RX_STATE = FIN_SEND;
                break;
            case DATA_WAITING:
                RxDataUpdateTimeoutTimes++;
                if(RxDataUpdateTimeoutTimes > maxRxDataUpdateWaitTimes){
                    RX_STATE = DATA_TIMEOUT;
                    RxDataUpdateTimeoutTimes = 0;
                    maxRxDataUpdateWaitTimes *= 2;
                }
                break;
            case FIN_SEND:
                sendAutoFlyPacket(dataRXId, OCTOMAP_FIN, NULL, 0);
                break;
            case DATA_TIMEOUT:
                RX_STATE = SACK_SEND;
                break;
            case SACK_SEND:
                RxSackSendTimes++;
                if(RxSackSendTimes > MAX_RX_SACK_SEND_TIME){
                    // SACK发送次数过多
                    DEBUG_PRINT("[sendOctomapSack]SACK send times is too much, RxSackSendTimes = %d\n", RxSackSendTimes);
                    octoMapPacket_Error_t error;
                    error.dataId = dataRXId;
                    sendAutoFlyPacket(sourceRxId, OCTOMAP_ERROR_RX_WAITING_TIMEOUT, (uint8_t*)&error, sizeof(octoMapPacket_Error_t));
                    RX_STATE = FAILED;
                }else{
                    sendOctomapSack();
                }
                break;
            default:
                break;
        }
        vTaskDelay(M2T(STATE_CHECK_INTERVAL));
    }
    
}