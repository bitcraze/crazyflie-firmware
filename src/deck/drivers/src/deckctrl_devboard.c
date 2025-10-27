/*
 * Deck control dev board driver
 *
 * This deck driver exercises the deck control dev board
 * It is used to test the deck control dev board
 * and can be used as a reference for other deck drivers.
 * 
 * Copyright (C) 2024 Bitcraze AB
 */
#include "FreeRTOS.h"
#include "deck.h"
#include "task.h"

#define DEBUG_MODULE "DECKCTRL_DEVBOARD"
#include "debug.h"

#include "deckctrl_gpio.h"

#define N_LEDS 11

static void task(void* param) {
  DeckInfo* info = (DeckInfo*)param;

  // Put all GPIO pins as output and set them low
  for (int i = 0; i < N_LEDS; i++) {
    deckctrl_gpio_set_direction(info, (DeckCtrlGPIOPin)i, OUTPUT);
    deckctrl_gpio_write(info, (DeckCtrlGPIOPin)i, LOW);
  }

  while (1) {
    // Blink all pins in sequence
    for (int i = 0; i < N_LEDS; i++) {
      deckctrl_gpio_write(info, (DeckCtrlGPIOPin)i, HIGH);
      vTaskDelay(100 / portTICK_PERIOD_MS);
      deckctrl_gpio_write(info, (DeckCtrlGPIOPin)i, LOW);
    }

    // All ON then all OFF
    for (int i = 0; i < N_LEDS; i++) {
      deckctrl_gpio_write(info, (DeckCtrlGPIOPin)i, HIGH);
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    for (int i = 0; i < N_LEDS; i++) {
      deckctrl_gpio_write(info, (DeckCtrlGPIOPin)i, LOW);
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
  }
}

void deckctrl_devboard_init(DeckInfo* info)
{
  DEBUG_PRINT("Deck control dev deck driver init\n");

  xTaskCreate(task, "deckctrl_dev_task",
              FREERTOS_MIN_STACK_SIZE, info, tskIDLE_PRIORITY, NULL);
}

static const DeckDriver deckctrl_devboard_deck = {
  .vid = 0xBC,
  .pid = 0x20,
  .name = "deckctrl_devboard",

  .init = deckctrl_devboard_init,
};

DECK_DRIVER(deckctrl_devboard_deck);
