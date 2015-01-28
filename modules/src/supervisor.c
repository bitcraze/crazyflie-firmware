#define DEBUG_MODULE "SUPERVISOR"

#include "FreeRTOS.h"
#include "task.h"

#include "supervisor.h"
#include "stabilizer.h"
#include "commander.h"
#include "offboardctrl.h"
#include "system.h"
#include "crtp.h"
#include "debug.h"

struct SupervisorCrtpCommand
{
  uint16_t mode; // 0 = stabilizer, 1 = offboardctrl
} __attribute__((packed));

#define MODE_UPDATE_FREQ 50

static struct SupervisorCrtpCommand* modeCmd;
static uint16_t mode;
static bool isInit;
static xTaskHandle stabilizerTaskHandle;
static xTaskHandle offboardCtrlTaskHandle;

static void supervisorCrtpCB(CRTPPacket* pk);
static void supervisorTask(void* param);
static void startStabilizer(void);
static void stopStabilizer(void);

void supervisorInit(void)
{
  if(isInit)
    return;

  crtpInit();
  offboardCtrlInit();
  commanderInit();
  stabilizerInit();
  crtpRegisterPortCB(CRTP_PORT_SUPERVISOR, supervisorCrtpCB);
  xTaskCreate(supervisorTask, (const signed char * const)"SUPERVISOR",
              2*configMINIMAL_STACK_SIZE, NULL, /*Piority*/2, NULL);

  mode = 0;
  modeCmd = pvPortMalloc(sizeof(struct SupervisorCrtpCommand));
  modeCmd->mode = mode;
  xTaskCreate(offboardCtrlTask, (const signed char * const)"OFFBOARDCTRL",
              2*configMINIMAL_STACK_SIZE, NULL, /*Piority*/2, &offboardCtrlTaskHandle);
  vTaskSuspend(offboardCtrlTaskHandle);
  startStabilizer();

  isInit = TRUE;
}

bool supervisorTest(void)
{
  bool pass = isInit;
  pass &= crtpTest();
  pass &= offboardCtrlTest();
  pass &= commanderTest();
  pass &= stabilizerTest();
  return isInit;
}

static void supervisorCrtpCB(CRTPPacket* pk)
{
  *modeCmd = *((struct SupervisorCrtpCommand*)pk->data);
  DEBUG_PRINT("Mode received: %d\n", modeCmd->mode);
}

static void startStabilizer(void)
{
  xTaskCreate(stabilizerTask, (const signed char * const)"STABILIZER",
              2*configMINIMAL_STACK_SIZE, NULL, /*Piority*/2, &stabilizerTaskHandle);
}

static void stopStabilizer(void)
{
  vTaskDelete(stabilizerTaskHandle);
}

static void supervisorTask(void* param)
{
  vTaskSetApplicationTaskTag(0, (void*)TASK_SUPERVISOR_ID_NBR);
  systemWaitStart();
  uint32_t lastWakeTime = xTaskGetTickCount();
  while(1)
  {
    vTaskDelayUntil(&lastWakeTime, F2T(MODE_UPDATE_FREQ));
    if (mode==0&&modeCmd->mode==1)
    {
      stopStabilizer();
      vTaskResume(offboardCtrlTaskHandle);
      mode=1;
    }
    else if (mode==1&&modeCmd->mode==0)
    {
      vTaskSuspend(offboardCtrlTaskHandle);
      startStabilizer();
      mode=0;
    }
  }
}
