#define DEBUG_MODULE "OFFBOARDCTRL"

#include "FreeRTOS.h"
#include "task.h"

#include "offboardctrl.h"
#include "crtp.h"
#include "motors.h"
#include "system.h"
#include "debug.h"

struct ThrustCrtpValues
{
  uint16_t thrust1;
  uint16_t thrust2;
  uint16_t thrust3;
  uint16_t thrust4;
} __attribute__((packed));

#define THRUSTS_UPDATE_FREQ 500

static bool isInit;
static bool isInactive;
static uint32_t lastUpdate;
static struct ThrustCrtpValues thrustsCmd;

static void offboardCtrlCrtpCB(CRTPPacket* pk);
void offboardCtrlTask(void* param);
static void offboardCtrlWatchdogReset(void);
static void updateThrusts(void);

void offboardCtrlInit(void)
{
  if(isInit)
    return;

  crtpInit();
  motorsInit();
  crtpRegisterPortCB(CRTP_PORT_OFFBOARDCTRL, offboardCtrlCrtpCB);

  lastUpdate = xTaskGetTickCount();
  isInactive = TRUE;
  isInit = TRUE;
}

bool offboardCtrlTest(void)
{
  bool pass = true;
  pass &= crtpTest();
  pass &= motorsTest();
  return pass;
}

static void offboardCtrlCrtpCB(CRTPPacket* pk)
{
  thrustsCmd = *((struct ThrustCrtpValues*)pk->data);
  offboardCtrlWatchdogReset();
}

void offboardCtrlWatchdog(void)
{
  uint32_t ticktimeSinceUpdate = xTaskGetTickCount() - lastUpdate;
  if (ticktimeSinceUpdate > OFFBOARDCTRL_WDT_TIMEOUT_SHUTDOWN)
  {
    thrustsCmd.thrust1 = 0;
    thrustsCmd.thrust2 = 0;
    thrustsCmd.thrust3 = 0;
    thrustsCmd.thrust4 = 0;
    isInactive = TRUE;
  }
  else
  {
    isInactive = FALSE;
  }
}

static void offboardCtrlWatchdogReset(void)
{
  lastUpdate = xTaskGetTickCount();
}

static void updateThrusts(void)
{
  offboardCtrlWatchdog();
  motorsSetRatio(MOTOR_M1,(uint32_t) thrustsCmd.thrust1);
  motorsSetRatio(MOTOR_M2,(uint32_t) thrustsCmd.thrust2);
  motorsSetRatio(MOTOR_M3,(uint32_t) thrustsCmd.thrust3);
  motorsSetRatio(MOTOR_M4,(uint32_t) thrustsCmd.thrust4);
}

void offboardCtrlTask(void* param)
{
  vTaskSetApplicationTaskTag(0, (void*)TASK_OFFBOARDCTRL_ID_NBR);
  systemWaitStart();
  uint32_t lastWakeTime = xTaskGetTickCount();
  while(1)
  {
    vTaskDelayUntil(&lastWakeTime, F2T(THRUSTS_UPDATE_FREQ));
    updateThrusts();
  }
}
