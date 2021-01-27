#ifndef __OLSR_PACKET_H__
#define __OLSR_PACKET_H__

#include "mac.h"
#include "olsrStruct.h"

/*
       0                   1                   2                   3
       0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
      +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
      |         Packet Length         |    Packet Sequence Number     |
      +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
      |  Message Type |     Vtime     |         Message Size          |//HELLO_MESSAGE
      +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
      |          Address1             |         Address2              |
      +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
      |          Address3             |         RESERVED              |
      +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
      |  Time To Live |   Hop Count   |    Message Sequence Number    |
      +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
      |          Reserved             |     Htime     |  Willingness  |//HELLO
      +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
      |   Link Code   |   Reserved    |       Link Message Size       |
      +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
      |          Address1             |         Address2              |
      +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
      |   Link Code   |   Reserved    |       Link Message Size       |
      +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
      |          Address1             |         Address2              |
      +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
      |                                                               |
      :                            MESSAGE                            :
      |                                                               |
      +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
      |  Message Type |     Vtime     |         Message Size          |//TC_MESSAGE
      +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
      |          Address1             |         Address2              |
      +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
      |          Address3             |         RESERVED              |
      +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
      |  Time To Live |   Hop Count   |    Message Sequence Number    |
      +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
      |              ANSN             |           Reserved            |//TC
      +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
      |          Address1             |         distance1             |
      +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
      |          Address2             |         distance2             |
      +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
      |                                                               |
      :                            MESSAGE                            :
      |                                                               |
      +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
      |  Message Type |    TS_SENT    |         Message Size          |//TIMESTAMP_MESSAGE
      +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
      |          srcAddress           |    Message Sequence Number    |
      +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
      |          Veloctiy(cm)         |             TS_SENT           |
      +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
      |             TS_SENT           |                               
      +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
      |            tsAddr1            |    Message Sequence Number    |     
      +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
      |                        TS_RECV1                               |
      +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
      |    TS_RECV1   |
      +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
      |            tsAddr2            |    Message Sequence Number    |     
      +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
      |                        TS_RECV2                               |
      +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
      |    TS_RECV2   |
      +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
      |  Message Type |     Vtime     |         Message Size          |//_MESSAGE
      +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
      |          Address1             |         Address2              |
      +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
      |          Address3             |         RESERVED              |
      +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
      |  Time To Live |   Hop Count   |    Message Sequence Number    |
      +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
      :                                                               :
      :                                                               :
               (etc.)
 */

#define PACKET_MAX_LENGTH 125 //got this value by testing
#define PACKET_PAYLOAD_MAX_SIZE 104 //(maxPacketLength-MAC802154_HEADER_LENGTH)
#define MESSAGE_MAX_LENGTH 100 //(payLoadMaxSize-sizeof(olsr_packet_hdr_t))
#define MESSAGE_PAYLOAD_MAX_SIZE  84 //(messageMaxSize-sizeof(olsr_message_hdr_t))
#define LINK_ADDRESS_MAX_NUM 10
#define LINK_MESSAGE_MAX_NUM ((MESSAGE_PAYLOAD_MAX_SIZE-sizeof(olsrHelloMessageHeader_t))\
                              /sizeof(olsrLinkMessage_t)) 
#define TC_PAYLOAD_MAX_NUM ((MESSAGE_PAYLOAD_MAX_SIZE-2)/sizeof(olsrTopologyMessageUint_t))
#define DATA_PAYLOAD_MAX_NUM  MESSAGE_PAYLOAD_MAX_SIZE-sizeof(olsrDataMessageHeader_t)
#define TS_PAYLOAD_MAX_NUM  9 //((MESSAGE_PAYLOAD_MAX_SIZE-sizeof(olsrTsMessageHeader_t))/sizeof(olsrTsMessageBodyUnit_t))

typedef struct{
    uint16_t m_packetLength;
    uint16_t m_packetSeq;
} __attribute__((packed)) olsrPacketHeader_t;//4

typedef struct{
    olsrPacketHeader_t m_packetHeader;
    uint8_t m_packetPayload[PACKET_PAYLOAD_MAX_SIZE]; 
    //int content_size;
} __attribute__((packed)) olsrPacket_t;

typedef enum{
    HELLO_MESSAGE = 1,
    TC_MESSAGE = 2,
    DATA_MESSAGE = 3,
    TS_MESSAGE= 4,
} olsrMessageType_t;
//1
typedef struct{
   olsrMessageType_t m_messageType;
   uint16_t m_vTime; //The validity time.
   uint16_t m_messageSize;
   olsrAddr_t m_originatorAddress;
   olsrAddr_t m_relayAddress;
   olsrAddr_t m_destinationAddress;
   uint8_t m_reserved;
   uint8_t m_timeToLive;
   uint8_t m_hopCount;
   uint16_t m_messageSeq;
} __attribute__((packed)) olsrMessageHeader_t; //16bytes

//message 
typedef struct
{
    olsrMessageHeader_t m_messageHeader; //16bytes
    uint8_t m_messagePayload[MESSAGE_MAX_LENGTH-sizeof(olsrMessageHeader_t)];//84bytes
    //int content_size;
} __attribute__((packed)) olsrMessage_t;

//hello message
typedef struct{
    uint8_t m_linkMessageNumber;
    uint16_t m_hTime;
    uint8_t m_willingness;
} __attribute__((packed)) olsrHelloMessageHeader_t;//4bytes

typedef struct{
    uint8_t m_linkCode;
    uint8_t m_reserved;
    uint16_t m_addressUsedSize;
    olsrAddr_t m_addresses; //this item may be vector if multi-interface support is needed.
} __attribute__((packed)) olsrLinkMessage_t; //6

typedef struct{
    olsrHelloMessageHeader_t  m_helloHeader;
    olsrLinkMessage_t m_linkMessage[LINK_MESSAGE_MAX_NUM];
} __attribute__((packed)) olsrHelloMessage_t;

//tc
typedef struct 
{
    olsrAddr_t m_address;
    olsrDist_t m_distance;
} __attribute__((packed)) olsrTopologyMessageUint_t;

typedef struct 
{
  uint16_t m_ansn;
  olsrTopologyMessageUint_t m_content[TC_PAYLOAD_MAX_NUM];
} __attribute__((packed)) olsrTopologyMessage_t;

//data
typedef struct{
    uint16_t m_sourcePort;
    uint16_t m_destPort;
    olsrAddr_t m_nextHop;
    uint16_t m_seq;
    uint16_t m_size;
} __attribute__((packed)) olsrDataMessageHeader_t;

typedef struct{
    olsrDataMessageHeader_t  m_dataHeader;
    uint8_t m_payload[DATA_PAYLOAD_MAX_NUM];
} __attribute__((packed)) olsrDataMessage_t;

//time stamp (ts) message
typedef struct {
  olsrMessageType_t m_messageType; // 1 byte
  uint16_t m_seq4TSsend; // 2 byte
  uint16_t m_messageSize; // 2 byte
  uint16_t m_originatorAddress; // 2 byte
  uint8_t m_dwTimeHigh8; // 1 byte
  uint32_t m_dwTimeLow32; // 4 byte
  short m_velocity;//in cm 2 byte
  uint16_t m_messageSeq; // 2 byte
} __attribute__((packed)) olsrTsMessageHeader_t; // 16 byte

typedef struct {
  uint16_t m_tsAddr;
  uint16_t m_sequence;
  uint8_t m_dwTimeHigh8;
  uint32_t m_dwTimeLow32;
} __attribute__((packed)) olsrTsMessageBodyUnit_t; // 9 byte

typedef struct {
  olsrTsMessageHeader_t m_tsHeader;
  olsrTsMessageBodyUnit_t m_neighborTime[TS_PAYLOAD_MAX_NUM];
} __attribute__((packed)) olsrTsMessage_t;
#endif //__OLSR_PACKET_H__
