---
title: Making your first Deck driver
page_id: howto
---

This howto is going to describe step-by-step how to make and flash your
first Crazyflie 2.X deck driver. See the deck [api documentation
page](/docs/userguides/decks/) for more information about the
code.

Development environment
-----------------------

You should have the
[crazyflie-firmware](https://github.com/bitcraze/crazyflie-firmware) and
[crazyflie-clients-python](https://github.com/bitcraze/crazyflie-clients-python)
cloned in the same folder. If you are using the [Bitcraze
VM](https://github.com/bitcraze/bitcraze-vm) this is already the case.

The Crazyflie firmware should be on Master branch.

For the rest of the howto you will work in the crazyflie-firmware
project.

Writing the driver
------------------

Deck drivers are in the deck/driver/src folder. Create this file named
hello.c in the deck/driver/src folder:

``` {.c}
#define DEBUG_MODULE "HelloDeck"
#include "debug.h"

#include "deck.h"


static void helloInit()
{
  DEBUG_PRINT("Hello Crazyflie 2.0 deck world!\n");
}

static bool helloTest()
{
  DEBUG_PRINT("Hello test passed!\n");
  return true;
}

static const DeckDriver helloDriver = {
  .name = "myHello",
  .init = helloInit,
  .test = helloTest,
};

DECK_DRIVER(helloDriver);
```

Adding the driver to the build
------------------------------

Add this to the Makefile, after the line \'\# Decks\':

``` {.make}
PROJ_OBJ += hello.o
```

Enabling the driver
-------------------

Decks can have a memory that contains its name. In our case the hello
driver would be initialised only when a deck identified as \"myHello\"
is installed on the Crazyflie. For development purpose it is possible to
force enabling a deck driver with a compile flag. To do so create the
file tools/make/config.mk with the content:

``` {.make}
CFLAGS += -DDECK_FORCE=myHello

DEBUG=1
```

DEBUG=1 allows to get more information from the Crazyflie console when
it starts. Debug should not be enabled if you intend to fly the
Crazyflie out of the lab (it disables the watchdog).

**Note** Each time you modify config.mk you
should recompile the full firmware by cleaning up the build folder with
\'make clean\'

Compile, flash and run!
-----------------------

Now the last step is to compile and flash your new firmware. Launch the
following commands in a shell:

``` {.bash}
crazyflie-firmware$ make
crazyflie-firmware$ make cload
```

The output will be similar to the following:

``` {.bash}
crazyflie-firmware$ make
(...)
  CC    hello.o
(...)
Build for the CF2 platform!
Build 22:f8243162f727 (2020.04 +22) MODIFIED
Version extracted from git
Crazyloader build!
Flash |  218132/1032192 (21%),  814060 free | text: 213024, data: 5108, ccmdata: 0
RAM   |   71564/131072  (55%),   59508 free | bss: 66456, data: 5108
CCM   |   43528/65536   (66%),   22008 free | ccmbss: 43528, ccmdata: 0
crazyflie-firmware$ make cload
../crazyflie-clients-python/bin/cfloader flash cf2.bin stm32-fw
Restart the Crazyflie you want to bootload in the next
 10 seconds ...
 done!
Connected to bootloader on Crazyflie 2.0 (version=0x10)
Target info: nrf51 (0xFE)
Flash pages: 232 | Page size: 1024 | Buffer pages: 1 | Start page: 88
144 KBytes of flash available for firmware image.
Target info: stm32 (0xFF)
Flash pages: 1024 | Page size: 1024 | Buffer pages: 10 | Start page: 16
1008 KBytes of flash available for firmware image.

Flashing 1 of 1 to stm32 (fw): 161867 bytes (159 pages) ..........10..........10..........10..........10..........10..........10..........10..........10..........10..........10..........10..........10..........10..........10..........10.........9
Reset in firmware mode ...
$
```

Now you can connect your Crazyflie with the client and see your driver
in the console!

![deck hello console](/docs/images/deckhelloconsole.png){:align-center}
