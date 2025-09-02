---
title: Deck API
page_id: deck
---

The Deck API allows to easily communicates with decks installed on the
Crazyflie.

If you want to get started you can follow the
[howto](/docs/development/howto.md) to get your code
running.

Deck drivers
------------

Decks are enumerated automatically using a modular discovery system that supports multiple backends:

- **OneWire backend**: Reads deck information from One Wire (OW) memory soldered on the deck PCB
- **Forced backend**: Allows compile-time forcing of deck drivers via `CONFIG_DECK_FORCE`
- **Extensible architecture**: New discovery backends can be added for different communication protocols

The Deck driver API uses a declarative syntax to register deck drivers and initialize them when the proper deck is detected through any of the discovery backends.

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
    the *.init* or *.test* initialization.
-   In this mode no task are created so to run some code at regular
    intervale the code needs to deal with freeRtos or with the other
    parts of the Crazyflie code.

To compile the driver, place it in deck/drivers/src/ and add it to the
Kbuild file:

``` {.make}
obj-y += myled.o
```

### Forcing initialization of a driver

The deck driver will be initialized only if a deck is connected with the
right OW memory content or other discovery method detects it. During development 
it is possible to force the initialization of one or more decks by setting the 
`CONFIG_DECK_FORCE` config option in your `.config` either by hand or using `make menuconfig`:

- Single deck: `CONFIG_DECK_FORCE="meMyled"`
- Multiple decks: `CONFIG_DECK_FORCE="meMyled:anotherDeck"`

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

  /*
   * Peripheral and Gpio used _directly_ by the driver.
   *
   * Include the pin in usedGpio if it's used directly by the driver, do not
   * add the pin to usedGpio if it used as part of a peripheral.
   *
   * For example: If a deck driver uses SPI we add DECK_USING_SPI to
   * usedPeriph. If the deck uses the MOSI, MISO or SCK pins for other stuff
   * than SPI it would have to specify DECK_USING_[PA7|PA6|PA5].
   *
   */
  uint32_t usedPeriph;
  uint32_t usedGpio;

  /* Required system properties */
  StateEstimatorType requiredEstimator;
  bool requiredKalmanEstimatorAttitudeReversionOff;
  bool requiredLowInterferenceRadioMode;

  // Deck memory access definitions
  const struct deckMemDef_s* memoryDef;

  // Have an option to present a secondary memory area for instance for decks
  // two firmwares.
  const struct deckMemDef_s* memoryDefSecondary;

  /* Init and test functions */
  void (*init)(struct deckInfo_s *);
  bool (*test)(void);
} DeckDriver;
```

## Deck Discovery System

The deck discovery system uses a modular backend architecture to support different deck detection methods. Multiple discovery backends run during startup to detect and enumerate decks.

### Available Discovery Backends

#### OneWire Backend

The OneWire backend is the traditional deck discovery method that:
- Initializes the OneWire bus hardware
- Scans for decks with OneWire memory chips
- Reads and validates deck information from the memory
- Supports the `CONFIG_DEBUG_DECK_IGNORE_OWS` compile flag to disable OneWire deck enumeration for debugging

#### Forced Backend  

The forced backend handles compile-time deck forcing via `CONFIG_DECK_FORCE`:
- Supports colon-separated deck names (e.g., `"myDeck:anotherDeck"`)
- Bypasses hardware detection for development and testing
- Useful when deck hardware isn't available or during driver development

### Debug Configuration

For developers working on the discovery system, conditional debug prints can be enabled:

```c
// In src/deck/core/deck_discovery.c
#define DEBUG_DECK_DISCOVERY

// In src/deck/backends/deck_backend_onewire.c  
#define DEBUG_ONEWIRE_BACKEND

// In src/deck/backends/deck_backend_forced.c
#define DEBUG_FORCED_BACKEND

// In src/deck/core/deck_info.c
#define DEBUG_DECK_ENUMERATION
```

### Adding New Discovery Backends

To add a new discovery backend:

1. Create a new backend file in `src/deck/backends/`
2. Implement the `DeckDiscoveryBackend_t` interface:
   ```c
   static bool myBackendInit(void);
   static DeckInfo* myBackendGetNextDeck(void);
   
   static const DeckDiscoveryBackend_t myBackend = {
       .name = "myBackend", 
       .init = myBackendInit,
       .getNextDeck = myBackendGetNextDeck,
   };
   
   DECK_DISCOVERY_BACKEND(myBackend);
   ```
3. The backend will be automatically discovered via linker sections
4. Add the backend to the build system in `src/deck/backends/Kbuild`

The discovery system runs all backends sequentially during startup, allowing multiple discovery methods to coexist and complement each other.
