## Copy this file to config.mk and modify to get you personal build configuration

## Set CRTP link to E-SKY receiver
# CFLAGS += -DUSE_ESKYLINK

## Redirect the console output to the UART
# CFLAGS += -DDEBUG_PRINT_ON_UART

## Load a deck driver that has no OW memory
# CFLAGS += -DDECK_FORCE=bcBuzzer

## Enable biq quad deck features
# CFLAGS += -DENABLE_BQ_DECK
# CFLAGS += -DBQ_DECK_ENABLE_PM
# CFLAGS += -DBQ_DECK_ENABLE_OSD

## Use morse when flashing the LED to indicate that the Crazyflie is calibrated
# CFLAGS += -DCALIBRATED_LED_MORSE

## Turn on monitoring of queue usages
# CFLAGS += -DDEBUG_QUEUE_MONITOR

## Automatically reboot to bootloader before flashing
# CLOAD_CMDS = -w radio://0/100/2M/E7E7E7E7E7

## Set number of anchor in LocoPositioningSystem
# CFLAGS += -DLOCODECK_NR_OF_ANCHORS=8

## Set alternative pins for LOCO deck (IRQ=IO_2, RESET=IO_3, default are RX1 and TX1)
# CFLAGS += -DLOCODECK_USE_ALT_PINS

## Compile positioning system for TDoA mode
# LPS_TDOA_ENABLE=1

## Use J-Link as Debugger/flasher
# OPENOCD_INTERFACE ?= interface/jlink.cfg
# OPENOCD_TARGET    ?= target/stm32f4x.cfg
# OPENOCD_CMDS      ?= -c "transport select swd"
