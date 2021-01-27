#include "adHocApp.h"
#include "olsrAlgo.h"
#include "olsrDebug.h"
#include "uwbOlsr.h"
#define TOTAL_PACKET_NUM 200
bool isSender;
bool isReceiver;

void endToEndTask()
{
  adHocAppInit(ADHOC_PORT_END_TO_END_DELAY);
  isSender = false;
  isReceiver = true;
  int sendCount = 0;
  int recvCount = 0;
  if(isReceiver)
    {
      DEBUG_PRINT_OLSR_APP("Im Receiver!\n");
    }
  if(isSender)
    {
      DEBUG_PRINT_OLSR_APP("Im Sender!\n");
    }
  while(1)
    {
      if(isSender&&sendCount<TOTAL_PACKET_NUM)
        {
          olsrSendData(myAddress,ADHOC_PORT_END_TO_END_DELAY,7,ADHOC_PORT_END_TO_END_DELAY,0,NULL,0);
          sendCount--;
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