# CrazyFlie's Makefile
# Copyright (c) 2011,2012 Bitcraze AB
# This Makefile compiles all the objet file to ./bin/ and the resulting firmware
# image in ./cflie.elf and ./cflie.bin

#Put your personal build config in config.mk and DO NOT COMMIT IT!
-include config.mk

######### JTAG and environment configuration ##########
OPENOCD_INTERFACE ?= interface/jtagkey.cfg
OPENOCD_TARGET    ?= target/stm32f1x.cfg
CROSS_COMPILE     ?= arm-none-eabi-
PYTHON2           ?= python
CLOAD             ?= 1
DEBUG             ?= 0
CLOAD_SCRIPT      ?= ../crazyflie-clients-python/bin/cfloader

## Flag that can be added to config.mk
# CFLAGS += -DUSE_UART_CRTP        # Set CRTP link to UART
# CFLAGS += -DUSE_ESKYLINK         # Set CRTP link to E-SKY receiver
# CFLAGS += -DENABLE_UART          # To enable the uart
# CFLAGS += -DDEBUG_PRINT_ON_UART  # Redirect the console output to the UART
# CFLAGS += -DENABLE_FAST_CHARGE   # Will enable ~800mA USB current for wall adapters. Should only be used with batteries 
	  															 # that can handle ~740mA charge current or it will degrade the battery.

REV               ?= E

#OpenOCD conf
RTOS_DEBUG        ?= 0

############### Location configuration ################
FREERTOS = lib/FreeRTOS
PORT = $(FREERTOS)/portable/GCC/ARM_CM3
STLIB = lib/

################ Build configuration ##################
# St Lib
VPATH += $(STLIB)/CMSIS/Core/CM3/startup/gcc
CRT0=startup_stm32f10x_md.o

include scripts/st_obj.mk

# FreeRTOS
VPATH += $(PORT)
PORT_OBJ=port.o
VPATH +=  $(FREERTOS)/portable/MemMang
MEMMANG_OBJ = heap_4.o

VPATH += $(FREERTOS)
FREERTOS_OBJ = list.o tasks.o queue.o timers.o $(MEMMANG_OBJ)

# Crazyflie
VPATH += init hal/src modules/src utils/src drivers/src

############### Source files configuration ################

# Init
PROJ_OBJ = main.o

# Drivers
PROJ_OBJ += led.o uart.o adc.o nrf24l01.o  exti.o  nvic.o motors.o
PROJ_OBJ += mpu6050.o i2cdev.o i2croutines.o hmc5883l.o
PROJ_OBJ += ms5611.o

# Hal
PROJ_OBJ += crtp.o ledseq.o freeRTOSdebug.o imu.o pm.o radiolink.o eskylink.o
PROJ_OBJ += usec_time.o

# Modules
PROJ_OBJ += system.o comm.o console.o pid.o crtpservice.o param.o
PROJ_OBJ += commander.o controller.o sensfusion6.o stabilizer.o
PROJ_OBJ += log.o worker.o

# Utilities
PROJ_OBJ += filter.o cpuid.o cfassert.o configblock.o eprintf.o crc.o fp16.o debug.o abort.o
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
INCLUDES+= -I$(STLIB)/STM32F10x_StdPeriph_Driver/inc
INCLUDES+= -I$(STLIB)/CMSIS/Core/CM3
INCLUDES+= -Iconfig -Ihal/interface -Imodules/interface
INCLUDES+= -Iutils/interface -Idrivers/interface

PROCESSOR = -mcpu=cortex-m3 -mthumb

#Flags required by the ST library
STFLAGS = -DSTM32F10X_MD -include stm32f10x_conf.h

ifeq ($(DEBUG), 1)
  CFLAGS += -O0 -g3
else
  CFLAGS += -Os -g3
endif

ifeq ($(LTO), 1)
  CFLAGS += -flto -fuse-linker-plugin
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
LDFLAGS = $(CFLAGS) -Wl,-Map=$(PROG).map,--cref,--gc-sections -nostdlib

ifeq ($(CLOAD), 1)
  LDFLAGS += -T scripts/STM32F103_32K_20K_FLASH_CLOAD.ld
else
  LDFLAGS += -T scripts/STM32F103_32K_20K_FLASH.ld
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
compile: clean_version $(PROG).hex $(PROG).bin

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
	$(CLOAD_SCRIPT) flash cflie.bin
else
	@echo "Only cload build can be bootloaded. Launch build and cload with CLOAD=1"
endif

#Flash the stm.
flash:
	openocd -d0 -f $(OPENOCD_INTERFACE) -f $(OPENOCD_TARGET) -c init -c targets -c "reset halt" \
                 -c "flash write_image erase cflie.elf" -c "verify_image cflie.elf" -c "reset run" -c shutdown

#STM utility targets
halt:
	openocd -d0 -f $(OPENOCD_INTERFACE) -f $(OPENOCD_TARGET) -c init -c targets -c "halt" -c shutdown

reset:
	openocd -d0 -f $(OPENOCD_INTERFACE) -f $(OPENOCD_TARGET) -c init -c targets -c "reset" -c shutdown

openocd:
	openocd -d0 -f $(OPENOCD_INTERFACE) -f $(OPENOCD_TARGET) -c init -c targets

#Print preprocessor #defines
prep:
	@$(CC) -dD

include scripts/targets.mk

#include dependencies
-include $(DEPS)
