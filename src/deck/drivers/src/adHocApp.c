#include "adHocApp.h"
#include "task.h"
#include "semphr.h"

#include "olsrDebug.h"
#include <string.h>
#include "queue.h"
static xQueueHandle queues[ADHOC_MAX_PORT_NUM];

bool adHocPortIsUsed(AdHocPort portId)
{
    return portUsed[portId];
}

void adHocInitTaskQueue(AdHocPort portId)
{
  if(portId<ADHOC_MAX_PORT_NUM)
    {

      queues[portId] = xQueueCreate(ADHOC_RX_QUEUE_SIZE, sizeof(olsrDataMessage_t));
    }
  else
    {
      return;
    }
}

void adHocClearTaskPort()
{
  for(int i=0;i<ADHOC_MAX_PORT_NUM;i++)
    {
      portUsed[i] = false;
    }
}

bool adHocAddToQueue(AdHocPort portId,olsrDataMessage_t *dataMsg)
{
  if(adHocPortIsUsed(portId))
    {
      xQueueSend(queues[portId],dataMsg,portMAX_DELAY);
      return true;
    }
  return false;
}

void adHocGetfromQueue(AdHocPort portId,olsrDataMessage_t *dataMsg)
{
  if(adHocPortIsUsed(portId))
    {
      xQueueReceive(queues[portId],dataMsg,portMAX_DELAY);
    }
}
bool adHocAppInit(AdHocPort portId)
{
  if(!adHocPortIsUsed(portId))
    { 
      portUsed[portId] = true;
      adHocInitTaskQueue(portId);
      return true;
    }
  else
  {
    DEBUG_PRINT_OLSR_APP("port is used\n");
    return false;
  }
}