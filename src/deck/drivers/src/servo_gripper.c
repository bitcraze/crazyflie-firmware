  #define DEBUG_MODULE "SERVO_GRIPPER_DEBUG"
#define SERVO_ANGLE_OPEN 60
#define SERVO_ANGLE_CLOSE 0

#include "FreeRTOS.h"
#include "task.h"

#include "deck.h"
#include "system.h"
#include "debug.h"
#include "config.h"
#include "servo_gripper.h"
#include "servo.h"
#include "crtp_commander.h"
#include "log.h"
#include "stabilizer.h"

static bool isInit;
const deckPin_t* mosfetPin = &DECK_GPIO_IO1;
const deckPin_t* activateGripperPin = &DECK_GPIO_IO2;
static int gripperState = IDLE;
static bool disengageGripper = false;

void servoGripperInit(DeckInfo* info)
{
  if (isInit)
    return;

  xTaskCreate(servoGripperTask, SERVO_GRIPPER_TASK_NAME, SERVO_GRIPPER_TASK_STACKSIZE,
        NULL, SERVO_GRIPPER_TASK_PRI, NULL);
  servoInit();
  if(!servoTest())
    return;

  pinMode(*activateGripperPin, INPUT);
  pinMode(*mosfetPin, OUTPUT);
  DEBUG_PRINT("Initialized Gripper\n");

  isInit = true;
}

bool servoGripperTest(void)
{
  bool testStatus;
  testStatus = true;

  if (!isInit)
    return false;

  return testStatus;
}

void servoGripperTask(void* arg)
{
  gripperState = IDLE;
  systemWaitStart();
  //wait for take-off
  while(getThrust() < 0.1f) {
    continue;
  }
  digitalWrite(*mosfetPin, HIGH);
  servoSetAngle(SERVO_ANGLE_OPEN);

  const int actuationTime = 150; //TODO: define delay
  TickType_t startServo =  xTaskGetTickCount();
  while(xTaskGetTickCount() < startServo + M2T(actuationTime)) {
    continue;
  }
  DEBUG_PRINT("Servo at initial position\n");
  gripperState = RDY2LAND;
  digitalWrite(*mosfetPin, LOW);
  DEBUG_PRINT("Gripper ready to land!\n");

  while (1) {
    if(digitalRead(*activateGripperPin) && gripperState == RDY2LAND) {

      startServo = xTaskGetTickCount();
      digitalWrite(*mosfetPin, HIGH);
      servoSetAngle(SERVO_ANGLE_CLOSE);
      gripperState = ACTIVATING_GRIPPER;
      DEBUG_PRINT("Activated gripper!\n");
    }
    else if((gripperState == ACTIVATING_GRIPPER || gripperState == RELEASING_GRIPPER)
                    && xTaskGetTickCount() > startServo + M2T(actuationTime)) {

      digitalWrite(*mosfetPin, LOW);
      if(gripperState == ACTIVATING_GRIPPER) {
        gripperState = LANDED;
        resetThrust(); //stop flapping
        DEBUG_PRINT("Flapper landed!\n");
      }
      else {
        gripperState = RDY2LAND;
        DEBUG_PRINT("Gripper ready to land!\n");
      }
    }
    else if(disengageGripper && gripperState == LANDED && getThrust() > 0.1f) {
      startServo = xTaskGetTickCount();
      digitalWrite(*mosfetPin, HIGH);

      servoSetAngle(SERVO_ANGLE_OPEN);
      gripperState = RELEASING_GRIPPER;
      DEBUG_PRINT("Releasing gripper!\n");
    }
    disengageGripper = getGripperStatus();
  }
}

static const DeckDriver gripper_deck = {
  .vid = 0x00,
  .pid = 0x00,
  .name = "bcServoGripper",

  //TODO: add GPIO pins
  .usedGpio = DECK_USING_IO_2 | DECK_USING_IO_1,
  .init = servoGripperInit,
  .test = servoGripperTest,
};

DECK_DRIVER(gripper_deck);


LOG_GROUP_START(servoGripper)
LOG_ADD(LOG_INT16, gripperState, &gripperState)
LOG_ADD(LOG_UINT8, disengageGripper, &disengageGripper)
LOG_GROUP_STOP(servoGripper)