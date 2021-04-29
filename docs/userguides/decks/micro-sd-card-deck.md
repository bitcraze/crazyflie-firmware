---
title: Micro-SD card deck
page_id: micro-sd-card-deck
---

The microSD card deck allows to attach a micro SD card to the Crazyflie for reading and writing files. The default firmware additionally supports to use the uSD card for high-speed logging of [logging variables](/docs/userguides/logparam.md) at rates up to 1 kHz.

## Hardware

The specification of the deck can be found in the [wiki](https://wiki.bitcraze.io/projects:crazyflie2:expansionboards:microsd).

## File system

The SD-card used in the deck must be formatted as a FAT32 file system (exFAT does not work).

We use [FatFS](http://elm-chan.org/fsw/ff/00index_e.html) as the file system handler. Have a look at the API and examples on the FatFS site fore usage information.

The FatFS module will be initialized by the micro-SD deck driver when the deck is detected. After that the API calls can be used such as `f_mount`, `f_open`, `f_read`, `f_close` etc.

## Data Logging

Without any changes to `usddeck.c` there's a data logging functionality implemented. It can be enabled and configured by placing a config file on the µSD-Card. The file is a simple text file and an example is available [config.txt](https://github.com/bitcraze/crazyflie-firmware/blob/master/tools/usdlog/config.txt). The file format is as follows:

```
1     # version
512   # buffer size in bytes
log   # file name
0     # enable on startup (0/1)
on:fixedFrequency
100     # frequency in Hz
1     # mode (0: disabled, 1: synchronous stabilizer, 2: asynchronous)
log entry 1
log entry 2
log entry 3
...
on:myEvent1
log entry 4
log entry 5
...
on:myEvent2
...
```

The config file supports logging of [log variables](/docs/userguides/logparam.md) as well as [event triggers](/docs/userguides/eventtrigger.md).
For the fixed frequency logging, the frequency is an integer value in Hertz, for example 250 means that a data block is written every 4ms. The buffer size is used to decouple the writing to the card and the data logging. Higher frequencies require a larger buffer, otherwise some data might be lost. The Crazyflie console will show how many events had to be discarded due to insufficient buffer size:
```
uSD: Wrote 161378 B to: log00 (2237 of 2237 events)
```
In general, logging 10 variables with 1kHz works well with a buffer of 512 Bytes. But you may have to try around a bit to get feeling on what is possible.

The file name should be 10 characters or less and a running number is appended automatically (e.g., if the file name is log, there will be files log00, log01, log02 etc.). The log entries are the names of logging variables in the firmware.

The `config.txt` file will be read only once on startup, therefore make sure that the µSD-Card is inserted before power up. If everything seems to be fine a µSD-task will be created and buffer space will be allocated. If malloc fails the Crazyflie will be stuck with LED M1 and M4 glowing. Data logging starts automatically after sensor calibration if `enable on startup` in `config.txt` was set to 1. Otherwise, logging can be started by setting the `usd.logging` parameter to 1. The logfiles will be enumerated in ascending order from 00-99 to allow multiple logs without the need of creating new config files. Just reset the Crazyflie to start a new file. Logging needs to be explicitly stopped by setting the `usd.logging` parameter to 0, which protects the logfile data with a CRC32.

## Data Analysis

For performance reasons the logfile is a binary file, using the following format (version 2):

```
uint8_t 0xBC header
uint16_t version
uint16_t num_event_types
for each event type:
   uint16_t event_id
   eventName<null>
   uint16_t num_variables
   varname1(vartype1)<null>varname2(vartype2)<null>...<null>varnameN(vartypeN)<null>
for each event:
   uint16_t event_id
   uint64_t timestamp (in microseconds)
   data (length defined by TOC event type)
```

Here, the vartype is a singe character and we support a subset of the ones defined [here](https://docs.python.org/3/library/struct.html#format-characters).

We provide a [helper script](https://github.com/bitcraze/crazyflie-firmware/blob/master/tools/usdlog/cfusdlog.py) to decode the data:

```
import cfusdlog
logData = cfusdlog.decode(fileName)
```

where fileName is a file from the µSD-Card. For convenience there is also an [example.py](https://github.com/bitcraze/crazyflie-firmware/blob/master/tools/usdlog/example.py) which shows how to access and plot the data.

## Alternate Pins

The SD-Card is using the SPI bus on the deck port to communicate. In some cases, it might be beneficial to use another SPI bus. One option is to use a "hidden" SPI on the deck port that is multiplexed with TX1, RX1 and IO_4. This SPI port is called SPI3 in the STM32F405 and there is a possibility to switch to this SPI bus.

  - First thing the SD-card deck needs to be patched, CS->RX2(PA3), SCLK->TX1(PC10), MISO->RX1(PC11), MOSI->IO_4(PC12) and this is a quite hard to cut the small lines and solder small patch wires. Easier is to use a sd-card breakout deck such as [this](https://www.sparkfun.com/products/544) and solder wires to e.g. the battery holder board.
  - Compile the firmware with `CFLAGS += -DUSDDECK_USE_ALT_PINS_AND_SPI` in your config.mk file.
  - If you want to use the Loco-deck at the same time, the alternative pins for this must be used. This means cutting GS1 and GS2 underneath the loco-deck and soldering the bridged GS3 and GS4. Then add `CFLAGS += -DLOCODECK_USE_ALT_PINS` to you config.mk as well.
