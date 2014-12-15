# CrazyFlie's Makefile
# Copyright (c) 2011,2012 Bitcraze AB
# This Makefile compiles all the objet file to ./bin/ and the resulting firmware
# image in ./cflie.elf and ./cflie.bin

#Put your personal build config in config.mk and DO NOT COMMIT IT!
-include config.mk

######### JTAG and environment configuration ##########
OPENOCD           ?= openocd
OPENOCD_INTERFACE ?= interface/stlink-v2.cfg
OPENOCD_TARGET    ?= target/stm32f4x_stlink.cfg
CROSS_COMPILE     ?= arm-none-eabi-
PYTHON2           ?= python2
DFU_UTIL          ?= dfu-util
CLOAD             ?= 1
F405              ?= 1
USE_FPU           ?= 0
DEBUG             ?= 0
CLOAD_SCRIPT      ?= ../crazyflie-clients-python/bin/cfloader

# Now needed for SYSLINK
CFLAGS += -DUSE_RADIOLINK_CRTP     # Set CRTP link to radio
CFLAGS += -DENABLE_UART          # To enable the uart

## Flag that can be added to config.mk
# CFLAGS += -DUSE_ESKYLINK         # Set CRTP link to E-SKY receiver
# CFLAGS += -DDEBUG_PRINT_ON_UART  # Redirect the console output to the UART

REV               ?= E

#OpenOCD conf
RTOS_DEBUG        ?= 0

############### Location configuration ################
FREERTOS = lib/FreeRTOS
ifeq ($(USE_FPU), 1)
PORT = $(FREERTOS)/portable/GCC/ARM_CM4F
else
PORT = $(FREERTOS)/portable/GCC/ARM_CM3
endif

ifeq ($(F405), 1)
LINKER_DIR = scripts/F405/linker
ST_OBJ_DIR  = scripts/F405
else
LINKER_DIR = scripts/F103/linker
ST_OBJ_DIR  = scripts/F103
endif

STLIB = lib

################ Build configuration ##################
# St Lib
ifeq ($(F405), 1)
	VPATH += $(STLIB)/CMSIS/STM32F4xx/Source/
	VPATH += $(STLIB)/STM32_CPAL_Driver/src
	VPATH += $(STLIB)/STM32_USB_Device_Library/Core/src
	VPATH += $(STLIB)/STM32_USB_OTG_Driver/src
	VPATH += $(STLIB)/STM32_CPAL_Driver/devices/stm32f4xx
	CRT0 = startup_stm32f40xx.o system_stm32f4xx.o
endif

# Should maybe be in separate file?
-include $(ST_OBJ_DIR)/st_obj.mk
ifeq ($(F405), 1)
	ST_OBJ += cpal_hal.o cpal_i2c.o cpal_usercallback_template.o cpal_i2c_hal_stm32f4xx.o
	# USB obj
	ST_OBJ += usb_core.o usb_dcd_int.o usb_dcd.o
	# USB Device obj
	ST_OBJ += usbd_ioreq.o usbd_req.o usbd_core.o
endif


# FreeRTOS
VPATH += $(PORT)
PORT_OBJ=port.o
VPATH +=  $(FREERTOS)/portable/MemMang
MEMMANG_OBJ = heap_4.o

VPATH += $(FREERTOS)
FREERTOS_OBJ = list.o tasks.o queue.o timers.o $(MEMMANG_OBJ)

# Crazyflie
ifeq ($(F405), 1)
	VPATH += init hal/src modules/src utils/src drivers/src platform/cf2
else
	VPATH += init hal/src modules/src utils/src drivers/src
endif


############### Source files configuration ################

# Init
ifeq ($(F405), 1)
	PROJ_OBJ = main.o platform_cf2.o
else
	PROJ_OBJ = main.o
endif

# Drivers
PROJ_OBJ += led.o exti.o nvic.o  

ifeq ($(F405), 1)
  PROJ_OBJ += mpu6500.o motors_f405.o i2cdev_f405.o ws2812.o lps25h.o ak8963.o eeprom.o
  PROJ_OBJ += uart_syslink.o swd.o
  # USB Files
  PROJ_OBJ += usbd_usr.o usb_bsp.o usblink.o usbd_desc.o usb.o
else
  PROJ_OBJ += mpu6050.o motors.o hmc5883l.o ms5611.o
endif

# Hal
PROJ_OBJ += crtp.o ledseq.o freeRTOSdebug.o syslink.o
ifeq ($(F405), 1)
PROJ_OBJ += imu_cf2.o pm_f405.o radiolink.o ow.o
else
PROJ_OBJ += imu.o pm.o
endif

# Modules
PROJ_OBJ += system.o comm.o console.o pid.o crtpservice.o param.o mem.o platformservice.o
PROJ_OBJ += commander.o controller.o sensfusion6.o stabilizer.o
PROJ_OBJ += log.o worker.o neopixelring.o expbrd.o


# Expansion boards
PROJ_OBJ += exptest.o

# Utilities
PROJ_OBJ += filter.o cpuid.o cfassert.o configblockeeprom.o eprintf.o crc.o fp16.o debug.o
PROJ_OBJ += version.o


OBJ = $(CRT0) $(FREERTOS_OBJ) $(PORT_OBJ) $(ST_OBJ) $(PROJ_OBJ)

ifdef P
  C_PROFILE = -D P_$(P)
endif

############### Compilation configuration ################
AS = $(CROSS_COMPILE)as
CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)gcc
SIZE = $(CROSS_COMPILE)size
OBJCOPY = $(CROSS_COMPILE)objcopy


INCLUDES = -I$(FREERTOS)/include -I$(PORT) -I.
INCLUDES+= -Iconfig -Ihal/interface -Imodules/interface
INCLUDES+= -Iutils/interface -Idrivers/interface -Iplatform
INCLUDES+= -I$(STLIB)/CMSIS/Include

ifeq ($(F405), 1)
INCLUDES+= -I$(STLIB)/STM32F4xx_StdPeriph_Driver/inc
INCLUDES+= -I$(STLIB)/STM32_CPAL_Driver/inc
INCLUDES+= -I$(STLIB)/STM32_USB_Device_Library/Core/inc
INCLUDES+= -I$(STLIB)/STM32_USB_OTG_Driver/inc
INCLUDES+= -I$(STLIB)/STM32_CPAL_Driver/devices/stm32f4xx
INCLUDES+= -I$(STLIB)/CMSIS/STM32F4xx/Include 
endif



ifeq ($(USE_FPU), 1)
PROCESSOR = -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
else
PROCESSOR = -mcpu=cortex-m4 -mthumb
endif

#Flags required by the ST library
ifeq ($(F405), 1)
STFLAGS = -DSTM32F4XX -DSTM32F40_41xxx -DHSE_VALUE=8000000 -DUSE_STDPERIPH_DRIVER
else
STFLAGS = -DSTM32F10X_MD -DHSE_VALUE=16000000 -include stm32f10x_conf.h
endif


ifeq ($(DEBUG), 1)
  CFLAGS += -O0 -g3
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

CFLAGS += $(PROCESSOR) $(INCLUDES) $(STFLAGS) -Wall -fno-strict-aliasing $(C_PROFILE)
# Compiler flags to generate dependency files:
CFLAGS += -MD -MP -MF $(BIN)/dep/$(@).d -MQ $(@)
#Permits to remove un-used functions and global variables from output file
CFLAGS += -ffunction-sections -fdata-sections

ASFLAGS = $(PROCESSOR) $(INCLUDES)
LDFLAGS = $(PROCESSOR) -Wl,-Map=$(PROG).map,--cref,--gc-sections

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
PROG = cflie
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
ifeq ($(SHELL),/bin/sh)
	@./scripts/print_revision.sh
endif
ifeq ($(CLOAD), 1)
	@echo "CrazyLoader build!"
endif

size: compile
	@$(SIZE) -B $(PROG).elf

#Radio bootloader
cload:
ifeq ($(CLOAD), 1)
	$(CLOAD_SCRIPT) flash cflie.bin stm32-fw
else
	@echo "Only cload build can be bootloaded. Launch build and cload with CLOAD=1"
endif

#Flash the stm.
flash:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) -f $(OPENOCD_TARGET) -c init -c targets -c "reset halt" \
                 -c "flash write_image erase cflie.elf" -c "verify_image cflie.elf" -c "reset run" -c shutdown

flash_dfu:
	$(DFU_UTIL) -a 0 -D cflie.dfu

#STM utility targets
halt:
	$(OPENOCD) -d0 -f $(OPENOCD_INTERFACE) -f $(OPENOCD_TARGET) -c init -c targets -c "halt" -c shutdown

reset:
	$(OPENOCD) -d0 -f $(OPENOCD_INTERFACE) -f $(OPENOCD_TARGET) -c init -c targets -c "reset" -c shutdown

openocd:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) -f $(OPENOCD_TARGET) -c init -c targets

#Print preprocessor #defines
prep:
	@$(CC) -dD

include scripts/targets.mk

#include dependencies
-include $(DEPS)
