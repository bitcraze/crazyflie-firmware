#define DEBUG_MODULE "SPRING_GRIPPER_DEBUG"

#include "FreeRTOS.h"
#include "task.h"

#include "deck.h"
#include "system.h"
#include "debug.h"
#include "config.h"
#include "spring_gripper.h"
#include "crtp_commander.h"
#include "log.h"
#include "stabilizer.h"

#define ACTIVATION_PERIOD_MS 60*1e3
#define REACTIVATION_TIME_MS 500
#define ACTIVATION_TIME_MS 2000

static bool isInit;
const deckPin_t* engageGripperPin = &DECK_GPIO_IO2; //PB5
const deckPin_t* disengageGripperPin = &DECK_GPIO_IO1; //PB8
static int gripperState = IDLE;
static bool activateGripper = false;

void springGripperInit(DeckInfo* info)
{
  if (isInit)
    return;

  xTaskCreate(springGripperTask, SPRING_GRIPPER_TASK_NAME, SPRING_GRIPPER_TASK_STACKSIZE, NULL,
            SPRING_GRIPPER_TASK_PRI, NULL);
  
  pinMode(*engageGripperPin, OUTPUT);
  pinMode(*disengageGripperPin, OUTPUT);

  isInit = true;
}

bool springGripperTest(void)
{
  bool testStatus = true;

  if (!isInit)
    return false;

  return testStatus;
}

//TODO: add test if throttle != 0
//TODO: send zero throttle when landed?
void springGripperTask(void* arg)
{
    gripperState = IDLE;

    //turn of Mosfet
    digitalWrite(*engageGripperPin, LOW);
    digitalWrite(*disengageGripperPin, LOW);

    systemWaitStart();
    //wait for take-off
    while(getThrust() < 0.1f) {
      continue;
    }
    digitalWrite(*disengageGripperPin, HIGH);
    TickType_t activation = xTaskGetTickCount();
    while(xTaskGetTickCount() < activation + M2T(ACTIVATION_TIME_MS)) {
      continue;
    }

    gripperState = RDY2LAND;
    digitalWrite(*disengageGripperPin, LOW);
    DEBUG_PRINT("Grripper ready to land!\n");

    while (1) {
      activateGripper = getGripperStatus();
      switch(gripperState) {
        case RDY2LAND:
          if(activateGripper) {
            activation = xTaskGetTickCount();
            digitalWrite(*engageGripperPin, HIGH);
            DEBUG_PRINT("Activating gripper!\n");
            gripperState = ACTIVATING_GRIPPER;
          }
          break;
        case ACTIVATING_GRIPPER:
          if(xTaskGetTickCount() > activation + M2T(ACTIVATION_TIME_MS)) {
            digitalWrite(*engageGripperPin, LOW);
            DEBUG_PRINT("Flapper landed!\n");
            activation = xTaskGetTickCount();
            gripperState = LANDED;
            resetThrust();
          }
          break;
        case LANDED:
          if(activateGripper) {
              digitalWrite(*engageGripperPin, LOW);
              digitalWrite(*disengageGripperPin, HIGH);
              activation = xTaskGetTickCount();
              DEBUG_PRINT("Releasing gripper!\n");
              gripperState = RELEASING_GRIPPER;
          }
          else if(xTaskGetTickCount() > activation + M2T(ACTIVATION_PERIOD_MS)) {
            digitalWrite(*engageGripperPin, HIGH);
            DEBUG_PRINT("Activating gripper!\n");
            gripperState = ACTIVATING_GRIPPER_LANDED;
            activation = xTaskGetTickCount();
          }
          break;
        case RELEASING_GRIPPER:
          if(xTaskGetTickCount() > activation + M2T(ACTIVATION_TIME_MS)) {
              gripperState = RDY2LAND;
              DEBUG_PRINT("Gripper ready to land!\n");
              digitalWrite(*disengageGripperPin, LOW);
          } 
          break;
        case ACTIVATING_GRIPPER_LANDED:
        if(activateGripper && getThrust() > 0.1f) {
              digitalWrite(*engageGripperPin, LOW);
              digitalWrite(*disengageGripperPin, HIGH);
              activation = xTaskGetTickCount();
              DEBUG_PRINT("Releasing gripper!\n");
              gripperState = RELEASING_GRIPPER;
              break;
          }
          if (xTaskGetTickCount() > activation + M2T(REACTIVATION_TIME_MS)) {
            digitalWrite(*engageGripperPin, LOW);
            activation = xTaskGetTickCount();
            gripperState = LANDED;
          }
          break;
        default:
          break;
      }
    }
}

static const DeckDriver springGripper_deck = {
  .vid = 0x00,
  .pid = 0x00,
  .name = "bcSpringGripper",

  //TODO: add GPIO pins
  .usedGpio = DECK_USING_IO_2 | DECK_USING_IO_1,

  .init = springGripperInit,
  .test = springGripperTest,
};

DECK_DRIVER(springGripper_deck);


LOG_GROUP_START(springGripper)
LOG_ADD(LOG_INT16, gripperState, &gripperState)
LOG_ADD(LOG_UINT8, activateGripper, &activateGripper)
LOG_GROUP_STOP(springGripper)