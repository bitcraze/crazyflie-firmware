#define DEBUG_MODULE "ClawDeck"
#include "debug.h"

#include "deck.h"

#include "stm32fxxx.h"
#include "config.h"
#include "console.h"
#include "uart1.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "log.h"
#include "param.h"

static uint16_t clawOpen;
static uint16_t clawClose;
#define ONE_SEC 1000

static void ClawInit()
{
  DEBUG_PRINT("Initialize ClawDeck!\n");
  pinMode(DECK_GPIO_IO1, OUTPUT);     // Set pins to output
  pinMode(DECK_GPIO_IO2, OUTPUT);
  pinMode(DECK_GPIO_IO3, OUTPUT);
  pinMode(DECK_GPIO_IO4, OUTPUT);

  digitalWrite(DECK_GPIO_IO2, HIGH);
  digitalWrite(DECK_GPIO_IO4, LOW);
  digitalWrite(DECK_GPIO_IO1, HIGH);
  digitalWrite(DECK_GPIO_IO3, LOW);
  vTaskDelay(2*ONE_SEC);
}

static bool ClawTest()
{
  DEBUG_PRINT("ClawDeck passed the test!\n");
  return true;
}

static void OpenClaw(uint16_t duration)
{
  digitalWrite(DECK_GPIO_IO1, LOW);
  digitalWrite(DECK_GPIO_IO2, HIGH);
  digitalWrite(DECK_GPIO_IO3, HIGH);
  digitalWrite(DECK_GPIO_IO4, LOW);
  vTaskDelay(duration);
  digitalWrite(DECK_GPIO_IO1, HIGH);
  digitalWrite(DECK_GPIO_IO2, HIGH);
  digitalWrite(DECK_GPIO_IO3, LOW);
  digitalWrite(DECK_GPIO_IO4, LOW);
  clawOpen = 0;
  vTaskDelay(ONE_SEC);
}

static void CloseClaw(uint16_t duration)
{
  digitalWrite(DECK_GPIO_IO2, LOW);
  digitalWrite(DECK_GPIO_IO4, HIGH);
  digitalWrite(DECK_GPIO_IO1, HIGH);
  digitalWrite(DECK_GPIO_IO3, LOW);
  vTaskDelay(duration);
  digitalWrite(DECK_GPIO_IO2, HIGH);
  digitalWrite(DECK_GPIO_IO4, LOW);
  digitalWrite(DECK_GPIO_IO1, HIGH);
  digitalWrite(DECK_GPIO_IO3, LOW);
  clawClose = 0;
  vTaskDelay(ONE_SEC);
}

static void updateDeck(void)
{
  if (clawOpen > 0 && clawClose == 0)
  {
    OpenClaw(clawOpen);
  }
  else if (clawClose > 0 && clawOpen == 0)
  {
    CloseClaw(clawClose);
  }
  else
  {
    digitalWrite(DECK_GPIO_IO1, HIGH);
    digitalWrite(DECK_GPIO_IO2, HIGH);
    digitalWrite(DECK_GPIO_IO3, LOW);
    digitalWrite(DECK_GPIO_IO4, LOW);
  }
}

static const DeckDriver ClawDriver = {
  .name = "Claw",

  .usedGpio = DECK_USING_IO_1 | DECK_USING_IO_2 | DECK_USING_IO_3 | DECK_USING_IO_4,

  .init = ClawInit,
  .test = ClawTest,

};

DECK_DRIVER(ClawDriver);

PARAM_GROUP_START(Claw)
PARAM_ADD_WITH_CALLBACK(PARAM_UINT16, clawOpen, &clawOpen, &updateDeck)
PARAM_ADD_WITH_CALLBACK(PARAM_UINT16, clawClose, &clawClose, &updateDeck)
PARAM_GROUP_STOP(Claw)