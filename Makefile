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
OPENOCD_CMDS      ?=
CROSS_COMPILE     ?= arm-none-eabi-
PYTHON2           ?= python2
DFU_UTIL          ?= dfu-util
CLOAD             ?= 1
DEBUG             ?= 0
CLOAD_SCRIPT      ?= python3 -m cfloader
CLOAD_CMDS        ?=
CLOAD_ARGS        ?=
PLATFORM					?= CF2
LPS_TDMA_ENABLE   ?= 0
LPS_TDOA_ENABLE   ?= 0

######### Stabilizer configuration ##########
##### Sets the name of the stabilizer module to use.
ESTIMATOR          ?= any
CONTROLLER         ?= Any # one of Any, PID, Mellinger
POWER_DISTRIBUTION ?= stock
SENSORS 					 ?= cf2

######### Test activation ##########
FATFS_DISKIO_TESTS  ?= 0	# Set to 1 to enable FatFS diskio function tests. Erases card.

ifeq ($(PLATFORM), CF2)
OPENOCD_TARGET    ?= target/stm32f4x_stlink.cfg
USE_FPU           ?= 1
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
FREERTOS = src/lib/FreeRTOS
ifeq ($(USE_FPU), 1)
PORT = $(FREERTOS)/portable/GCC/ARM_CM4F
else
PORT = $(FREERTOS)/portable/GCC/ARM_CM3
endif

ifeq ($(PLATFORM), CF2)
LINKER_DIR = tools/make/F405/linker
ST_OBJ_DIR  = tools/make/F405
endif

LIB = src/lib

################ Build configuration ##################
# St Lib
VPATH_CF2 += $(LIB)/CMSIS/STM32F4xx/Source/
VPATH_CF2 += $(LIB)/STM32_USB_Device_Library/Core/src
VPATH_CF2 += $(LIB)/STM32_USB_OTG_Driver/src
VPATH_CF2 += src/deck/api src/deck/core src/deck/drivers/src src/deck/drivers/src/test
CRT0_CF2 = startup_stm32f40xx.o system_stm32f4xx.o

# Should maybe be in separate file?
-include $(ST_OBJ_DIR)/st_obj.mk

# USB obj
ST_OBJ_CF2 += usb_core.o usb_dcd_int.o usb_dcd.o
# USB Device obj
ST_OBJ_CF2 += usbd_ioreq.o usbd_req.o usbd_core.o

# libdw dw1000 driver
VPATH_CF2 += vendor/libdw1000/src

# vl53l1 driver
VPATH_CF2 += $(LIB)/vl53l1/core/src

# FreeRTOS
VPATH += $(PORT)
PORT_OBJ = port.o
VPATH +=  $(FREERTOS)/portable/MemMang
MEMMANG_OBJ = heap_4.o

VPATH += $(FREERTOS)
FREERTOS_OBJ = list.o tasks.o queue.o timers.o $(MEMMANG_OBJ)

#FatFS
VPATH_CF2 += $(LIB)/FatFS
FATFS_OBJ  = diskio.o ff.o syscall.o unicode.o fatfs_sd.o
ifeq ($(FATFS_DISKIO_TESTS), 1)
FATFS_OBJ += diskio_function_tests.o
CFLAGS += -DUSD_RUN_DISKIO_FUNCTION_TESTS
endif

# Crazyflie sources
VPATH += src/init src/hal/src src/modules/src src/utils/src src/drivers/bosch/src src/drivers/src
VPATH_CF2 += src/platform/cf2

ifeq ($(PLATFORM), CF2)
VPATH +=$(VPATH_CF2)
endif


############### Source files configuration ################

# Init
PROJ_OBJ += main.o
PROJ_OBJ_CF2 += platform_cf2.o

# Drivers
PROJ_OBJ += exti.o nvic.o motors.o
PROJ_OBJ_CF2 += led_f405.o mpu6500.o i2cdev_f405.o ws2812_cf2.o lps25h.o i2c_drv.o
PROJ_OBJ_CF2 += ak8963.o eeprom.o maxsonar.o piezo.o
PROJ_OBJ_CF2 += uart_syslink.o swd.o uart1.o uart2.o watchdog.o
PROJ_OBJ_CF2 += cppm.o
PROJ_OBJ_CF2 += bmi055_accel.o bmi055_gyro.o bmi160.o bmp280.o bstdr_comm_support.o bmm150.o
PROJ_OBJ_CF2 += pca9685.o vl53l0x.o pca95x4.o vl53l1x.o pmw3901.o
# USB Files
PROJ_OBJ_CF2 += usb_bsp.o usblink.o usbd_desc.o usb.o

# Hal
PROJ_OBJ += crtp.o ledseq.o freeRTOSdebug.o buzzer.o
PROJ_OBJ_CF2 +=  pm_f405.o syslink.o radiolink.o ow_syslink.o proximity.o usec_time.o

PROJ_OBJ_CF2 +=  sensors_$(SENSORS).o
# libdw
PROJ_OBJ_CF2 += libdw1000.o libdw1000Spi.o

# vl53l1 lib
PROJ_OBJ_CF2 += vl53l1_api_core.o vl53l1_api.o vl53l1_core.o vl53l1_silicon_core.o vl53l1_api_strings.o
PROJ_OBJ_CF2 += vl53l1_api_calibration.o vl53l1_api_debug.o vl53l1_api_preset_modes.o vl53l1_error_strings.o
PROJ_OBJ_CF2 += vl53l1_register_funcs.o vl53l1_wait.o vl53l1_core_support.o

# Modules
PROJ_OBJ += system.o comm.o console.o pid.o crtpservice.o param.o
PROJ_OBJ += log.o worker.o trigger.o sitaw.o queuemonitor.o msp.o
PROJ_OBJ_CF2 += platformservice.o sound_cf2.o extrx.o sysload.o mem_cf2.o

# Stabilizer modules
PROJ_OBJ += commander.o crtp_commander.o crtp_commander_rpyt.o
PROJ_OBJ += crtp_commander_generic.o crtp_localization_service.o
PROJ_OBJ += attitude_pid_controller.o sensfusion6.o stabilizer.o
PROJ_OBJ += position_estimator_altitude.o position_controller_pid.o
PROJ_OBJ += estimator.o estimator_complementary.o
PROJ_OBJ += controller.o controller_pid.o controller_mellinger.o
PROJ_OBJ += power_distribution_$(POWER_DISTRIBUTION).o
PROJ_OBJ_CF2 += estimator_kalman.o

# High-Level Commander
PROJ_OBJ += crtp_commander_high_level.o planner.o pptraj.o

# Deck Core
PROJ_OBJ_CF2 += deck.o deck_info.o deck_drivers.o deck_test.o

# Deck API
PROJ_OBJ_CF2 += deck_constants.o
PROJ_OBJ_CF2 += deck_digital.o
PROJ_OBJ_CF2 += deck_analog.o
PROJ_OBJ_CF2 += deck_spi.o

# Decks
PROJ_OBJ_CF2 += bigquad.o
PROJ_OBJ_CF2 += rzr.o
PROJ_OBJ_CF2 += ledring12.o
PROJ_OBJ_CF2 += buzzdeck.o
PROJ_OBJ_CF2 += gtgps.o
PROJ_OBJ_CF2 += cppmdeck.o
PROJ_OBJ_CF2 += usddeck.o
PROJ_OBJ_CF2 += zranger.o zranger2.o
PROJ_OBJ_CF2 += locodeck.o
PROJ_OBJ_CF2 += lpsTwrTag.o
PROJ_OBJ_CF2 += lpsTdoa2Tag.o
PROJ_OBJ_CF2 += lpsTdoa3Tag.o lpsTdoaTagEngine.o lpsTdoaTagStats.o
PROJ_OBJ_CF2 += outlierFilter.o
PROJ_OBJ_CF2 += flowdeck_v1v2.o
PROJ_OBJ_CF2 += oa.o
PROJ_OBJ_CF2 += multiranger.o

ifeq ($(LPS_TDOA_ENABLE), 1)
CFLAGS += -DLPS_TDOA_ENABLE
endif

ifeq ($(LPS_TDMA_ENABLE), 1)
CFLAGS += -DLPS_TDMA_ENABLE
endif

#Deck tests
PROJ_OBJ_CF2 += exptest.o
#PROJ_OBJ_CF2 += bigquadtest.o


# Utilities
PROJ_OBJ += filter.o cpuid.o cfassert.o  eprintf.o crc.o num.o debug.o
PROJ_OBJ += version.o FreeRTOS-openocd.o
PROJ_OBJ_CF2 += configblockeeprom.o crc_bosch.o
PROJ_OBJ_CF2 += sleepus.o

# Libs
PROJ_OBJ_CF2 += libarm_math.a

OBJ = $(FREERTOS_OBJ) $(PORT_OBJ) $(ST_OBJ) $(PROJ_OBJ)
ifeq ($(PLATFORM), CF2)
OBJ += $(CRT0_CF2) $(ST_OBJ_CF2) $(FATFS_OBJ) $(PROJ_OBJ_CF2)
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
GDB = $(CROSS_COMPILE)gdb

INCLUDES  = -I$(FREERTOS)/include -I$(PORT) -Isrc
INCLUDES += -Isrc/config -Isrc/hal/interface -Isrc/modules/interface
INCLUDES += -Isrc/utils/interface -Isrc/drivers/interface -Isrc/platform
INCLUDES += -Ivendor/CMSIS/CMSIS/Include -Isrc/drivers/bosch/interface

INCLUDES_CF2 += -I$(LIB)/STM32F4xx_StdPeriph_Driver/inc
INCLUDES_CF2 += -I$(LIB)/CMSIS/STM32F4xx/Include
INCLUDES_CF2 += -I$(LIB)/STM32_USB_Device_Library/Core/inc
INCLUDES_CF2 += -I$(LIB)/STM32_USB_OTG_Driver/inc
INCLUDES_CF2 += -Isrc/deck/interface -Isrc/deck/drivers/interface
INCLUDES_CF2 += -Ivendor/libdw1000/inc
INCLUDES_CF2 += -I$(LIB)/FatFS
INCLUDES_CF2 += -I$(LIB)/vl53l1
INCLUDES_CF2 += -I$(LIB)/vl53l1/core/inc

ifeq ($(USE_FPU), 1)
	PROCESSOR = -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
	CFLAGS += -fno-math-errno -DARM_MATH_CM4 -D__FPU_PRESENT=1 -D__TARGET_FPU_VFP
else
	ifeq ($(PLATFORM), CF2)
		PROCESSOR = -mcpu=cortex-m4 -mthumb
	endif
endif

#Flags required by the ST library
STFLAGS_CF2 = -DSTM32F4XX -DSTM32F40_41xxx -DHSE_VALUE=8000000 -DUSE_STDPERIPH_DRIVER

ifeq ($(DEBUG), 1)
  CFLAGS += -O0 -g3 -DDEBUG
else
	# Fail on warnings
  CFLAGS += -Os -g3 -Werror
endif

ifeq ($(LTO), 1)
  CFLAGS += -flto
endif

ifeq ($(USE_ESKYLINK), 1)
  CFLAGS += -DUSE_ESKYLINK
endif

CFLAGS += -DBOARD_REV_$(REV) -DESTIMATOR_NAME=$(ESTIMATOR)Estimator -DCONTROLLER_NAME=ControllerType$(CONTROLLER) -DPOWER_DISTRIBUTION_TYPE_$(POWER_DISTRIBUTION)

CFLAGS += $(PROCESSOR) $(INCLUDES) $(STFLAGS)
ifeq ($(PLATFORM), CF2)
CFLAGS += $(INCLUDES_CF2) $(STFLAGS_CF2)
endif

CFLAGS += -Wall -Wmissing-braces -fno-strict-aliasing $(C_PROFILE) -std=gnu11
# Compiler flags to generate dependency files:
CFLAGS += -MD -MP -MF $(BIN)/dep/$(@).d -MQ $(@)
#Permits to remove un-used functions and global variables from output file
CFLAGS += -ffunction-sections -fdata-sections
# Prevent promoting floats to doubles
CFLAGS += -Wdouble-promotion


ASFLAGS = $(PROCESSOR) $(INCLUDES)
LDFLAGS = --specs=nano.specs $(PROCESSOR) -Wl,-Map=$(PROG).map,--cref,--gc-sections,--undefined=uxTopUsedPriority

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
PROG = cf2
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


all: check_submodules build
build:
# Each target is in a different line, so they are executed one after the other even when the processor has multiple cores (when the -j option for the make command is > 1). See: https://www.gnu.org/software/make/manual/html_node/Parallel.html
	@$(MAKE) --no-print-directory clean_version
	@$(MAKE) --no-print-directory compile
	@$(MAKE) --no-print-directory print_version
	@$(MAKE) --no-print-directory size
compile: $(PROG).hex $(PROG).bin $(PROG).dfu

libarm_math.a:
	+$(MAKE) -C tools/make/cmsis_dsp/ V=$(V)

clean_version:
ifeq ($(SHELL),/bin/sh)
	@echo "  CLEAN_VERSION"
	@rm -f version.c
endif

print_version:
ifeq ($(PLATFORM), CF2)
	@echo "Crazyflie 2.0 build!"
endif
	@$(PYTHON2) tools/make/versionTemplate.py --print-version
ifeq ($(CLOAD), 1)
	@echo "Crazyloader build!"
endif
ifeq ($(FATFS_DISKIO_TESTS), 1)
	@echo "WARNING: FatFS diskio tests enabled. Erases SD-card!"
endif

size:
	@$(SIZE) -B $(PROG).elf

#Radio bootloader
cload:
ifeq ($(CLOAD), 1)
	$(CLOAD_SCRIPT) $(CLOAD_CMDS) flash $(CLOAD_ARGS) $(PROG).bin stm32-fw
else
	@echo "Only cload build can be bootloaded. Launch build and cload with CLOAD=1"
endif

#Flash the stm.
flash:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) $(OPENOCD_CMDS) -f $(OPENOCD_TARGET) -c init -c targets -c "reset halt" \
                 -c "flash write_image erase $(PROG).bin $(LOAD_ADDRESS) bin" \
                 -c "verify_image $(PROG).bin $(LOAD_ADDRESS) bin" -c "reset run" -c shutdown

#verify only
flash_verify:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) $(OPENOCD_CMDS) -f $(OPENOCD_TARGET) -c init -c targets -c "reset halt" \
                 -c "verify_image $(PROG).bin $(LOAD_ADDRESS) bin" -c "reset run" -c shutdown

flash_dfu:
	$(DFU_UTIL) -a 0 -D $(PROG).dfu

#STM utility targets
halt:
	$(OPENOCD) -d0 -f $(OPENOCD_INTERFACE) $(OPENOCD_CMDS) -f $(OPENOCD_TARGET) -c init -c targets -c "halt" -c shutdown

reset:
	$(OPENOCD) -d0 -f $(OPENOCD_INTERFACE) $(OPENOCD_CMDS) -f $(OPENOCD_TARGET) -c init -c targets -c "reset" -c shutdown

openocd:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) $(OPENOCD_CMDS) -f $(OPENOCD_TARGET) -c init -c targets -c "\$$_TARGETNAME configure -rtos auto"

trace:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) $(OPENOCD_CMDS) -f $(OPENOCD_TARGET) -c init -c targets -f tools/trace/enable_trace.cfg

gdb: $(PROG).elf
	$(GDB) -ex "target remote localhost:3333" -ex "monitor reset halt" $^

erase:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) -f $(OPENOCD_TARGET) -c init -c targets -c "halt" -c "stm32f4x mass_erase 0" -c shutdown

#Print preprocessor #defines
prep:
	@$(CC) $(CFLAGS) -dM -E - < /dev/null

check_submodules:
	@$(PYTHON2) tools/make/check-for-submodules.py

include tools/make/targets.mk

#include dependencies
-include $(DEPS)

unit:
	rake unit "DEFINES=$(CFLAGS)" "FILES=$(FILES)"
