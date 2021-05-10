#include "adHocApp.h"
#include "olsrAlgo.h"
#include "olsrDebug.h"
#include "uwbOlsr.h"
#include "log.h"
#define SENDER_ADDRESS 14
#define RECEIVER_ADDRESS 29
#define TOTAL_PACKET_NUM 1000
bool isSender = false;
bool isReceiver = false;
bool isRouter = false;
float receiveRate = 0.0;

void endToEndTask()
{
  adHocAppInit(ADHOC_PORT_END_TO_END_DELAY);
  int sendCount = 0;
  int recvCount = 0;
  if(myAddress==RECEIVER_ADDRESS) {
    isReceiver = true;
  }else if(myAddress==SENDER_ADDRESS) {
    isSender = true;
  }else{
    isRouter = true;
  }
  if(isReceiver)
    {
      DEBUG_PRINT_OLSR_APP("Im Receiver!\n");
    }
  if(isSender)
    {
      DEBUG_PRINT_OLSR_APP("Im Sender!\n");
    }
  if(isRouter)
    {
      DEBUG_PRINT_OLSR_APP("Im Route!\n");
    }
  int seq = 1;
  int firstSeq = 0;
  while(1)
    {
      if(isSender&&sendCount<TOTAL_PACKET_NUM)
        {
          olsrSendData(myAddress,ADHOC_PORT_END_TO_END_DELAY,RECEIVER_ADDRESS,ADHOC_PORT_END_TO_END_DELAY,seq++,NULL,0);
          sendCount++;
          vTaskDelay(500);
        }
      if(isReceiver)
        {
          olsrDataMessage_t msg;
          adHocGetfromQueue(ADHOC_PORT_END_TO_END_DELAY,&msg);
          recvCount++;
          if (!firstSeq)
            firstSeq = msg.m_dataHeader.m_seq;
          receiveRate = recvCount * 100.0 / (msg.m_dataHeader.m_seq - firstSeq + 1);
          DEBUG_PRINT_OLSR_APP("Recv: %d; Seq: %d; Rate: %.2f.\n", recvCount, msg.m_dataHeader.m_seq, receiveRate);
          vTaskDelay(500);
        }
      if(isRouter)
        {
          vTaskDelay(500);
        }
        // DEBUG_PRINT_OLSR_APP("recv pack:%d\n",recvCount);
    }

}   
LOG_GROUP_START(ReceiveRate)
  LOG_ADD(LOG_FLOAT, receiveRate, &receiveRate)
LOG_GROUP_STOP(RECEIVE_RATE)