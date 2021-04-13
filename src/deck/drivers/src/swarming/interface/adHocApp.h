#ifndef __AD_HOC_APP_H__
#define __AD_HOC_APP_H__
#include "FreeRTOS.h"
#include "olsrPacket.h"

#define ADHOC_RX_QUEUE_SIZE 2
#define ADHOC_MAX_PORT_NUM 4
typedef enum {
  ADHOC_PORT_END_TO_END_DELAY         = 0x00,
} AdHocPort;

bool portUsed[ADHOC_MAX_PORT_NUM];

bool adHocPortIsUsed(AdHocPort portId);
void adHocInitTaskQueue(AdHocPort portId);
void adHocClearTaskPort();
bool adHocAppInit(AdHocPort portId);
bool adHocAddToQueue(AdHocPort portId,olsrDataMessage_t *dataMsg);
void adHocGetfromQueue(AdHocPort portId,olsrDataMessage_t *dataMsg);
#endif //__AD_HOC_APP_H__