#include "adHocApp.h"
#include "olsrAlgo.h"
#include "olsrDebug.h"
#define TOTAL_PACKET_NUM 200
bool isSender;
bool isReceiver;

void endToEndTask()
{
  adHocAppInit(ADHOC_PORT_END_TO_END_DELAY);
  isSender = true;
  isReceiver = false;
  int sendCount = 0;
  int recvCount = 0;
  while(1)
    {
      if(isSender&&sendCount<TOTAL_PACKET_NUM)
        {
          olsrSendData(0,ADHOC_PORT_END_TO_END_DELAY,0,ADHOC_PORT_END_TO_END_DELAY,0,NULL,0);
          sendCount--;
        }
      if(isReceiver)
        {
          olsrDataMessage_t msg;
          adHocGetfromQueue(ADHOC_PORT_END_TO_END_DELAY,&msg);
          recvCount++;
        }
        DEBUG_PRINT_OLSR_APP("recv pack:%d\n",recvCount);
        vTaskDelay(20);
    }

}   