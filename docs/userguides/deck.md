---
title: Deck API
page_id: deck
---

The Deck API allows to easily communicates with decks installed on the
Crazyflie. The deck API is still in development, it will:

-   Enumerates installed Deck and initialize drivers (*pretty much
    done*)
-   Provides easy to use arduino-like API to access (*started*)
-   Optionally provides a task to run in (arduino-like setup and loop)
    (*not started yet*)

If you want to get started you can follow the
[howto](/docs/development/howto.md) to get your code
running.

Deck drivers
------------

Decks are enumerated automatically using a One Wire (OW) memory soldered
on the deck PCB. The Deck driver API is using a declarative syntax to
register deck drivers and initialize them when the proper deck is
installed.

### Minimal driver

This is a minimal deck driver, myled.c:

``` {.c}
#include "deck.h"

void myledInit(DeckInfo *info)
{
  pinMode(DECK_GPIO_IO1, OUTPUT);     // Set my Led pin to output
  digitalWrite(DECK_GPIO_IO1, HIGH);  // Light it up
}

bool myledTest()
{
  return true;
}

const DeckDriver myled_driver = {
  .vid = 0,
  .pid = 0,
  .name = "meMyled",


  .usedGpio = DECK_USING_IO_1,

  .init = myledInit,
  .test = myledTest,
};

DECK_DRIVER(myled_driver);
```

-   At startup the decks are enumerated and if a deck has the name
    \"meMyled\", it will be initialized.
-   Init is called on all initialized driver and then test is called.
-   The init and test functions are both optional (we have some board
    with only init and event some with only test). If absent just remove
    the *.init* or *.test* initialisation.
-   In this mode no task are created so to run some code at regular
    intervale the code needs to deal with freeRtos or with the other
    parts of the Crazyflie code.

To compile the driver, place it in deck/drivers/src/ and add it to the
Makefile:

``` {.make}
PROJ_OBJ += myled.o
```

### Forcing initialization of a driver

The deck driver will be initialized only if a deck is connected with the
right OW memory content. During development it is possible to force the
initialisation of a deck by adding a define in
\`\`\`tools/make/config.mk\`\`\`:

``` {.make}
CFLAGS += -DDECK_FORCE=meMyled
```

### Driver declaration API

*DECK\_DRIVER(const struct DeckDriver)*

To register a deck driver the DECK\_DRIVER() macro should be called with
a deck structure as argument.

#### DeckDriver structure

``` {.c}
typedef struct deck_driver {
  /* Identification of the deck (written in the board) */
  uint8_t vid;
  uint8_t pid;
  char *name;

  /* Periphreal and Gpio used _dirrectly_ by the driver */
  uint32_t usedPeriph;
  uint32_t usedGpio;

  /* Init and test functions */
  void (*init)(struct deckInfo_s *);
  bool (*test)(void);
} DeckDriver;
```
