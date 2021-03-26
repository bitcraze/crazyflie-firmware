#include "adHocApp.h"
#include "olsrAlgo.h"
#include "olsrDebug.h"
#include "uwbOlsr.h"
#define TOTAL_PACKET_NUM 200
bool isSender = false;
bool isReceiver = false;

void endToEndTask()
{
  adHocAppInit(ADHOC_PORT_END_TO_END_DELAY);
  int sendCount = 0;
  int recvCount = 0;
  if(myAddress==24) {
    isReceiver = true;
  }else if(myAddress==12) {
    isSender = true;
  }
  if(isReceiver)
    {
      DEBUG_PRINT_OLSR_APP("Im Receiver!\n");
    }
  if(isSender)
    {
      DEBUG_PRINT_OLSR_APP("Im Sender!\n");
    }
  if(!isReceiver&&!isSender)
    {
      DEBUG_PRINT_OLSR_APP("Im Route!\n");
    }
  while(1)
    {
      if(isSender&&sendCount<TOTAL_PACKET_NUM)
        {
          olsrSendData(myAddress,ADHOC_PORT_END_TO_END_DELAY,24,ADHOC_PORT_END_TO_END_DELAY,0,NULL,0);
          sendCount++;
        }
      if(isReceiver)
        {
          olsrDataMessage_t msg;
          adHocGetfromQueue(ADHOC_PORT_END_TO_END_DELAY,&msg);
          recvCount++;
        }
        DEBUG_PRINT_OLSR_APP("recv pack:%d\n",recvCount);
        vTaskDelay(2000);
    }

}   