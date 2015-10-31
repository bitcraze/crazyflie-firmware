# CrazyFlie's Makefile
# Copyright (c) 2011,2012 Bitcraze AB
# This Makefile compiles all the objet file to ./bin/ and the resulting firmware
# image in ./cfX.elf and ./cfX.bin

# Put your personal build config in tools/make/config.mk and DO NOT COMMIT IT!
# Make a copy of tools/make/config.mk.example to get you started
-include tools/make/config.mk

CFLAGS += $(EXTRA_CFLAGS)

######### JTAG and environment configuration ##########
OPENOCD           ?= openocd
OPENOCD_INTERFACE ?= interface/stlink-v2.cfg
CROSS_COMPILE     ?= arm-none-eabi-
PYTHON2           ?= python2
DFU_UTIL          ?= dfu-util
CLOAD             ?= 1
DEBUG             ?= 0
CLOAD_SCRIPT      ?= ../crazyflie-clients-python/bin/cfloader
PLATFORM					?= CF2

ifeq ($(PLATFORM), CF1)
OPENOCD_TARGET    ?= target/stm32f1x_stlink.cfg
USE_FPU            = 0
endif
ifeq ($(PLATFORM), CF2)
OPENOCD_TARGET    ?= target/stm32f4x_stlink.cfg
USE_FPU           ?= 1
endif


ifeq ($(PLATFORM), CF1)
REV               ?= F
endif
ifeq ($(PLATFORM), CF2)
# Now needed for SYSLINK
CFLAGS += -DUSE_RADIOLINK_CRTP     # Set CRTP link to radio
CFLAGS += -DENABLE_UART          # To enable the uart
REV               ?= D
endif

#OpenOCD conf
RTOS_DEBUG        ?= 0

############### Location configuration ################
FREERTOS = lib/FreeRTOS
ifeq ($(USE_FPU), 1)
PORT = $(FREERTOS)/portable/GCC/ARM_CM4F
else
PORT = $(FREERTOS)/portable/GCC/ARM_CM3
endif

ifeq ($(PLATFORM), CF1)
LINKER_DIR = tools/make/F103/linker
ST_OBJ_DIR  = tools/make/F103
endif
ifeq ($(PLATFORM), CF2)
LINKER_DIR = tools/make/F405/linker
ST_OBJ_DIR  = tools/make/F405
endif

STLIB = lib

################ Build configuration ##################
# St Lib
VPATH_CF1 += $(STLIB)/CMSIS/Core/CM3
VPATH_CF1 += $(STLIB)/CMSIS/Core/CM3/startup/gcc
VPATH_CF1 += $(STLIB)/STM32_CPAL_Driver/src
VPATH_CF1 += $(STLIB)/STM32_CPAL_Driver/devices/stm32f10x
CRT0_CF1 = startup_stm32f10x_md.o system_stm32f10x.o

VPATH_CF2 += $(STLIB)/CMSIS/STM32F4xx/Source/
VPATH_CF2 += $(STLIB)/STM32_CPAL_Driver/src
VPATH_CF2 += $(STLIB)/STM32_USB_Device_Library/Core/src
VPATH_CF2 += $(STLIB)/STM32_USB_OTG_Driver/src
VPATH_CF2 += $(STLIB)/STM32_CPAL_Driver/devices/stm32f4xx
VPATH_CF2 += deck/api deck/core deck/drivers/src
CRT0_CF2 = startup_stm32f40xx.o system_stm32f4xx.o

# Should maybe be in separate file?
-include $(ST_OBJ_DIR)/st_obj.mk

ST_OBJ_CF1 =  #cpal_hal.o cpal_i2c.o cpal_usercallback_template.o cpal_i2c_hal_stm32f10x.o

ST_OBJ_CF2 = cpal_hal.o cpal_i2c.o cpal_usercallback_template.o cpal_i2c_hal_stm32f4xx.o
# USB obj
ST_OBJ_CF2 += usb_core.o usb_dcd_int.o usb_dcd.o
# USB Device obj
ST_OBJ_CF2 += usbd_ioreq.o usbd_req.o usbd_core.o


# FreeRTOS
VPATH += $(PORT)
PORT_OBJ = port.o
VPATH +=  $(FREERTOS)/portable/MemMang
MEMMANG_OBJ = heap_4.o

VPATH += $(FREERTOS)
FREERTOS_OBJ = list.o tasks.o queue.o timers.o $(MEMMANG_OBJ)

# Crazyflie sources
VPATH += init hal/src modules/src utils/src drivers/src
VPATH_CF1 += platform/cf1
VPATH_CF2 += platform/cf2

ifeq ($(PLATFORM), CF1)
VPATH +=$(VPATH_CF1)
endif
ifeq ($(PLATFORM), CF2)
VPATH +=$(VPATH_CF2)
endif


############### Source files configuration ################

# Init
PROJ_OBJ += main.o
PROJ_OBJ_CF1 += platform_cf1.o
PROJ_OBJ_CF2 += platform_cf2.o

# Drivers
PROJ_OBJ += exti.o nvic.o motors.o
PROJ_OBJ_CF1 += led_f103.o i2cdev_f103.o i2croutines.o adc_f103.o mpu6050.o
PROJ_OBJ_CF1 += hmc5883l.o ms5611.o nrf24l01.o eeprom.o watchdog.o
PROJ_OBJ_CF2 += led_f405.o mpu6500.o i2cdev_f405.o ws2812_cf2.o lps25h.o
PROJ_OBJ_CF2 += ak8963.o eeprom.o maxsonar.o piezo.o
PROJ_OBJ_CF2 += uart_syslink.o swd.o uart1.o uart2.o watchdog.o
# USB Files
PROJ_OBJ_CF2 += usb_bsp.o usblink.o usbd_desc.o usb.o

# Hal
PROJ_OBJ += crtp.o ledseq.o freeRTOSdebug.o
PROJ_OBJ_CF1 += imu_cf1.o pm_f103.o nrf24link.o ow_none.o uart.o
PROJ_OBJ_CF2 += imu_cf2.o pm_f405.o syslink.o radiolink.o ow_syslink.o proximity.o

# Modules
PROJ_OBJ += system.o comm.o console.o pid.o crtpservice.o param.o mem.o
PROJ_OBJ += commander.o controller.o sensfusion6.o stabilizer.o
PROJ_OBJ += log.o worker.o trigger.o sitaw.o queuemonitor.o
PROJ_OBJ_CF2 += platformservice.o

# Deck Core
PROJ_OBJ_CF2 += deck.o deck_info.o deck_drivers.o

# Deck API
PROJ_OBJ_CF2 += deck_constants.o
PROJ_OBJ_CF2 += deck_digital.o
PROJ_OBJ_CF2 += deck_analog.o
PROJ_OBJ_CF2 += buzzer.o

# Decks
PROJ_OBJ_CF2 += bigquad.o
PROJ_OBJ_CF2 += exptest.o
PROJ_OBJ_CF2 += ledring12.o

# Utilities
PROJ_OBJ += filter.o cpuid.o cfassert.o  eprintf.o crc.o fp16.o debug.o
PROJ_OBJ += version.o
PROJ_OBJ_CF1 += configblockflash.o
PROJ_OBJ_CF2 += configblockeeprom.o

OBJ = $(FREERTOS_OBJ) $(PORT_OBJ) $(ST_OBJ) $(PROJ_OBJ)
ifeq ($(PLATFORM), CF1)
OBJ += $(CRT0_CF1) $(ST_OBJ_CF1) $(PROJ_OBJ_CF1)
endif
ifeq ($(PLATFORM), CF2)
OBJ += $(CRT0_CF2) $(ST_OBJ_CF2) $(PROJ_OBJ_CF2)
endif

ifdef P
  C_PROFILE = -D P_$(P)
endif

############### Compilation configuration ################
AS = $(CROSS_COMPILE)as
CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)gcc
SIZE = $(CROSS_COMPILE)size
OBJCOPY = $(CROSS_COMPILE)objcopy

INCLUDES  = -I$(FREERTOS)/include -I$(PORT) -I.
INCLUDES += -Iconfig -Ihal/interface -Imodules/interface
INCLUDES += -Iutils/interface -Idrivers/interface -Iplatform
INCLUDES += -I$(STLIB)/CMSIS/Include

INCLUDES_CF1 += -I$(STLIB)/STM32F10x_StdPeriph_Driver/inc
INCLUDES_CF1 += -I$(STLIB)/CMSIS/Core/CM3
INCLUDES_CF1 += -I$(STLIB)/STM32_CPAL_Driver/inc
INCLUDES_CF1 += -I$(STLIB)/STM32_CPAL_Driver/devices/stm32f10x

INCLUDES_CF2 += -I$(STLIB)/STM32F4xx_StdPeriph_Driver/inc
INCLUDES_CF2 += -I$(STLIB)/CMSIS/STM32F4xx/Include
INCLUDES_CF2 += -I$(STLIB)/STM32_CPAL_Driver/inc
INCLUDES_CF2 += -I$(STLIB)/STM32_CPAL_Driver/devices/stm32f4xx
INCLUDES_CF2 += -I$(STLIB)/STM32_USB_Device_Library/Core/inc
INCLUDES_CF2 += -I$(STLIB)/STM32_USB_OTG_Driver/inc
INCLUDES_CF2 += -Ideck/interface -I deck/drivers/interface

ifeq ($(USE_FPU), 1)
	PROCESSOR = -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
	CFLAGS += -fno-math-errno
else
	ifeq ($(PLATFORM), CF1)
		PROCESSOR = -mcpu=cortex-m3 -mthumb
	endif
	ifeq ($(PLATFORM), CF2)
		PROCESSOR = -mcpu=cortex-m4 -mthumb
	endif
endif

#Flags required by the ST library
STFLAGS_CF1 = -DSTM32F10X_MD -DHSE_VALUE=16000000 -include stm32f10x_conf.h -DPLATFORM_CF1
STFLAGS_CF2 = -DSTM32F4XX -DSTM32F40_41xxx -DHSE_VALUE=8000000 -DUSE_STDPERIPH_DRIVER -DPLATFORM_CF2

ifeq ($(DEBUG), 1)
  CFLAGS += -O0 -g3 -DDEBUG
else
  CFLAGS += -Os -g3
endif

ifeq ($(LTO), 1)
  CFLAGS += -flto
endif

ifeq ($(USE_ESKYLINK), 1)
  CFLAGS += -DUSE_ESKYLINK
endif

CFLAGS += -DBOARD_REV_$(REV)

CFLAGS += $(PROCESSOR) $(INCLUDES) $(STFLAGS)
ifeq ($(PLATFORM), CF1)
CFLAGS += $(INCLUDES_CF1) $(STFLAGS_CF1)
endif
ifeq ($(PLATFORM), CF2)
CFLAGS += $(INCLUDES_CF2) $(STFLAGS_CF2)
endif

CFLAGS += -Wall -fno-strict-aliasing $(C_PROFILE)
# Compiler flags to generate dependency files:
CFLAGS += -MD -MP -MF $(BIN)/dep/$(@).d -MQ $(@)
#Permits to remove un-used functions and global variables from output file
CFLAGS += -ffunction-sections -fdata-sections

ASFLAGS = $(PROCESSOR) $(INCLUDES)
LDFLAGS = --specs=nano.specs $(PROCESSOR) -Wl,-Map=$(PROG).map,--cref,--gc-sections

#Flags required by the ST library
ifeq ($(CLOAD), 1)
  LDFLAGS += -T $(LINKER_DIR)/FLASH_CLOAD.ld
  LOAD_ADDRESS = 0x8004000
else
  LDFLAGS += -T $(LINKER_DIR)/FLASH.ld
  LOAD_ADDRESS = 0x8000000
endif

ifeq ($(LTO), 1)
  LDFLAGS += -Os -flto -fuse-linker-plugin
endif

#Program name
ifeq ($(PLATFORM), CF1)
PROG = cf1
else
PROG = cf2
endif
#Where to compile the .o
BIN = bin
VPATH += $(BIN)

#Dependency files to include
DEPS := $(foreach o,$(OBJ),$(BIN)/dep/$(o).d)

##################### Misc. ################################
ifeq ($(SHELL),/bin/sh)
  COL_RED=\033[1;31m
  COL_GREEN=\033[1;32m
  COL_RESET=\033[m
endif

#################### Targets ###############################


all: build
build: clean_version compile print_version size
compile: clean_version $(PROG).hex $(PROG).bin $(PROG).dfu

clean_version:
ifeq ($(SHELL),/bin/sh)
	@echo "  CLEAN_VERSION"
	@rm -f version.c
endif

print_version: compile
ifeq ($(PLATFORM), CF1)
	@echo "Crazyflie Nano (1.0) build!"
endif
ifeq ($(PLATFORM), CF2)
	@echo "Crazyflie 2.0 build!"
endif
	@$(PYTHON2) tools/make/versionTemplate.py --print-version
ifeq ($(CLOAD), 1)
	@echo "Crazyloader build!"
endif


size: compile
	@$(SIZE) -B $(PROG).elf

#Radio bootloader
cload:
ifeq ($(CLOAD), 1)
	$(CLOAD_SCRIPT) flash $(PROG).bin stm32-fw
else
	@echo "Only cload build can be bootloaded. Launch build and cload with CLOAD=1"
endif

#Flash the stm.
flash:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) -f $(OPENOCD_TARGET) -c init -c targets -c "reset halt" \
                 -c "flash write_image erase $(PROG).elf" -c "verify_image $(PROG).elf" -c "reset run" -c shutdown

flash_dfu:
	$(DFU_UTIL) -a 0 -D $(PROG).dfu

#STM utility targets
halt:
	$(OPENOCD) -d0 -f $(OPENOCD_INTERFACE) -f $(OPENOCD_TARGET) -c init -c targets -c "halt" -c shutdown

reset:
	$(OPENOCD) -d0 -f $(OPENOCD_INTERFACE) -f $(OPENOCD_TARGET) -c init -c targets -c "reset" -c shutdown

openocd:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) -f $(OPENOCD_TARGET) -c init -c targets

trace:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) -f $(OPENOCD_TARGET) -c init -c targets -f tools/trace/enable_trace.cfg

#Print preprocessor #defines
prep:
	@$(CC) -dD

include tools/make/targets.mk

#include dependencies
-include $(DEPS)
