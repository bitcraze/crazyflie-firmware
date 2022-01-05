# CrazyFlie's Makefile
# Copyright (c) 2011-2021 Bitcraze AB
# This Makefile compiles all the object file to ./bin/ and the resulting firmware
# image in ./cfX.elf and ./cfX.bin

CRAZYFLIE_BASE ?= ./

# Put your personal build config in tools/make/config.mk and DO NOT COMMIT IT!
# Make a copy of tools/make/config.mk.example to get you started
-include tools/make/config.mk

CFLAGS += $(EXTRA_CFLAGS)

######### JTAG and environment configuration ##########
OPENOCD           ?= openocd
OPENOCD_INTERFACE ?= interface/stlink.cfg
OPENOCD_CMDS      ?=
CROSS_COMPILE     ?= arm-none-eabi-
PYTHON            ?= python3
DFU_UTIL          ?= dfu-util
CLOAD             ?= 1
DEBUG             ?= 0
CLOAD_SCRIPT      ?= python3 -m cfloader
CLOAD_CMDS        ?=
CLOAD_ARGS        ?=
PLATFORM          ?= cf2
LPS_TDMA_ENABLE   ?= 0
LPS_TDOA_ENABLE   ?= 0
LPS_TDOA3_ENABLE  ?= 0


# Platform configuration handling
-include current_platform.mk
include $(CRAZYFLIE_BASE)/tools/make/platform.mk

CFLAGS += -DCRAZYFLIE_FW

#OpenOCD conf
RTOS_DEBUG        ?= 0

LIB = $(CRAZYFLIE_BASE)/src/lib
FREERTOS = $(CRAZYFLIE_BASE)/vendor/FreeRTOS
CFLAGS += -DBLOBS_LOC='"$(CRAZYFLIE_BASE)/blobs/"'

# Communication Link
UART2_LINK        ?= 0


############### CPU-specific build configuration ################

ifeq ($(CPU), stm32f4)
PORT = $(FREERTOS)/portable/GCC/ARM_CM4F
LINKER_DIR = $(CRAZYFLIE_BASE)/tools/make/F405/linker
ST_OBJ_DIR  = $(CRAZYFLIE_BASE)/tools/make/F405

OPENOCD_TARGET    ?= target/stm32f4x.cfg


# St Lib
VPATH += $(LIB)/CMSIS/STM32F4xx/Source/
VPATH += $(LIB)/STM32_USB_Device_Library/Core/src
VPATH += $(LIB)/STM32_USB_OTG_Driver/src
VPATH += $(CRAZYFLIE_BASE)/src/deck/api $(CRAZYFLIE_BASE)/src/deck/core $(CRAZYFLIE_BASE)/src/deck/drivers/src $(CRAZYFLIE_BASE)/src/deck/drivers/src/test
VPATH += $(CRAZYFLIE_BASE)/src/utils/src/tdoa $(CRAZYFLIE_BASE)/src/utils/src/lighthouse
CRT0 = startup_stm32f40xx.o system_stm32f4xx.o

# Add ST lib object files
-include $(ST_OBJ_DIR)/st_obj.mk

# USB obj
ST_OBJ += usb_core.o usb_dcd_int.o usb_dcd.o
# USB Device obj
ST_OBJ += usbd_ioreq.o usbd_req.o usbd_core.o

PROCESSOR = -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -fno-math-errno -DARM_MATH_CM4 -D__FPU_PRESENT=1 -mfp16-format=ieee

#Flags required by the ST library
CFLAGS += -DSTM32F4XX -DSTM32F40_41xxx -DHSE_VALUE=8000000 -DUSE_STDPERIPH_DRIVER

LOAD_ADDRESS_stm32f4 = 0x8000000
LOAD_ADDRESS_CLOAD_stm32f4 = 0x8004000
MEM_SIZE_FLASH_K = 1008
MEM_SIZE_RAM_K = 128
MEM_SIZE_CCM_K = 64
endif

################ Build configuration ##################

# libdw dw1000 driver
VPATH += $(CRAZYFLIE_BASE)/vendor/libdw1000/src

# vl53l1 driver
VPATH += $(LIB)/vl53l1/core/src

# FreeRTOS
VPATH += $(PORT)
PORT_OBJ = port.o
VPATH +=  $(FREERTOS)/portable/MemMang
MEMMANG_OBJ ?= heap_4.o

VPATH += $(FREERTOS)
FREERTOS_OBJ = list.o tasks.o queue.o timers.o event_groups.o $(MEMMANG_OBJ)

#FatFS
VPATH += $(LIB)/FatFS
PROJ_OBJ += ff.o ffunicode.o fatfs_sd.o
ifeq ($(FATFS_DISKIO_TESTS), 1)
PROJ_OBJ += diskio_function_tests.o
CFLAGS += -DUSD_RUN_DISKIO_FUNCTION_TESTS
endif

ifeq ($(APP), 1)
CFLAGS += -DAPP_ENABLED=1
endif
ifdef APP_STACKSIZE
CFLAGS += -DAPP_STACKSIZE=$(APP_STACKSIZE)
endif
ifdef APP_PRIORITY
CFLAGS += -DAPP_PRIORITY=$(APP_PRIORITY)
endif

# Crazyflie sources
VPATH += $(CRAZYFLIE_BASE)/src/init $(CRAZYFLIE_BASE)/src/hal/src $(CRAZYFLIE_BASE)/src/modules/src $(CRAZYFLIE_BASE)/src/modules/src/lighthouse $(CRAZYFLIE_BASE)/src/modules/src/kalman_core $(CRAZYFLIE_BASE)/src/utils/src $(CRAZYFLIE_BASE)/src/drivers/bosch/src $(CRAZYFLIE_BASE)/src/drivers/src $(CRAZYFLIE_BASE)/src/platform $(CRAZYFLIE_BASE)/src/drivers/esp32/src
VPATH += $(CRAZYFLIE_BASE)/src/utils/src/kve

############### Source files configuration ################

# Init
PROJ_OBJ += main.o
PROJ_OBJ += platform.o platform_utils.o platform_$(PLATFORM).o platform_$(CPU).o

# Drivers
PROJ_OBJ += exti.o nvic.o motors.o
PROJ_OBJ += led.o mpu6500.o i2cdev.o ws2812_cf2.o lps25h.o i2c_drv.o
PROJ_OBJ += ak8963.o eeprom.o maxsonar.o piezo.o
PROJ_OBJ += uart_syslink.o swd.o uart1.o uart2.o watchdog.o
PROJ_OBJ += cppm.o
PROJ_OBJ += bmi055_accel.o bmi055_gyro.o bmi160.o bmp280.o bstdr_comm_support.o bmm150.o
PROJ_OBJ += bmi088_accel.o bmi088_gyro.o bmi088_fifo.o bmp3.o
PROJ_OBJ += pca9685.o vl53l0x.o pca95x4.o pca9555.o vl53l1x.o pmw3901.o
PROJ_OBJ += amg8833.o lh_bootloader.o
PROJ_OBJ += esp_rom_bootloader.o esp_slip.o

# USB Files
PROJ_OBJ += usb_bsp.o usblink.o usbd_desc.o usb.o

# Hal
PROJ_OBJ += crtp.o ledseq.o freeRTOSdebug.o buzzer.o
PROJ_OBJ += pm_$(CPU).o syslink.o radiolink.o ow_syslink.o ow_common.o proximity.o usec_time.o
PROJ_OBJ += sensors.o
PROJ_OBJ += storage.o

# libdw
PROJ_OBJ += libdw1000.o libdw1000Spi.o

# vl53l1 lib
PROJ_OBJ += vl53l1_api_core.o vl53l1_api.o vl53l1_core.o vl53l1_silicon_core.o vl53l1_api_strings.o
PROJ_OBJ += vl53l1_api_calibration.o vl53l1_api_debug.o vl53l1_api_preset_modes.o vl53l1_error_strings.o
PROJ_OBJ += vl53l1_register_funcs.o vl53l1_wait.o vl53l1_core_support.o

# Modules
PROJ_OBJ += system.o comm.o console.o pid.o crtpservice.o param_task.o param_logic.o
PROJ_OBJ += log.o worker.o queuemonitor.o msp.o
PROJ_OBJ += platformservice.o sound_cf2.o extrx.o sysload.o mem.o
PROJ_OBJ += range.o app_handler.o static_mem.o app_channel.o
PROJ_OBJ += eventtrigger.o supervisor.o

# Stabilizer modules
PROJ_OBJ += commander.o crtp_commander.o crtp_commander_rpyt.o
PROJ_OBJ += crtp_commander_generic.o crtp_localization_service.o peer_localization.o
PROJ_OBJ += attitude_pid_controller.o sensfusion6.o stabilizer.o
PROJ_OBJ += position_estimator_altitude.o position_controller_pid.o position_controller_indi.o
PROJ_OBJ += estimator.o estimator_complementary.o
PROJ_OBJ += controller.o controller_pid.o controller_mellinger.o controller_indi.o
PROJ_OBJ += power_distribution_$(POWER_DISTRIBUTION).o
PROJ_OBJ += collision_avoidance.o health.o

# Kalman estimator
PROJ_OBJ += estimator_kalman.o kalman_core.o kalman_supervisor.o
PROJ_OBJ += mm_distance.o mm_absolute_height.o mm_position.o mm_pose.o mm_tdoa.o mm_flow.o mm_tof.o mm_yaw_error.o mm_sweep_angles.o
PROJ_OBJ += mm_tdoa_robust.o mm_distance_robust.o

# High-Level Commander
PROJ_OBJ += crtp_commander_high_level.o planner.o pptraj.o pptraj_compressed.o

# Deck Core
PROJ_OBJ += deck.o deck_info.o deck_drivers.o deck_test.o deck_memory.o

# Deck API
PROJ_OBJ += deck_constants.o
PROJ_OBJ += deck_digital.o
PROJ_OBJ += deck_analog.o
PROJ_OBJ += deck_spi.o
PROJ_OBJ += deck_spi3.o

# Decks
PROJ_OBJ += bigquad.o
PROJ_OBJ += ledring12.o
PROJ_OBJ += buzzdeck.o
PROJ_OBJ += gtgps.o
PROJ_OBJ += cppmdeck.o
PROJ_OBJ += usddeck.o
PROJ_OBJ += zranger.o zranger2.o
PROJ_OBJ += locodeck.o
PROJ_OBJ += clockCorrectionEngine.o
PROJ_OBJ += lpsTwrTag.o
PROJ_OBJ += lpsTdoa2Tag.o
PROJ_OBJ += lpsTdoa3Tag.o tdoaEngineInstance.o tdoaEngine.o tdoaStats.o tdoaStorage.o
PROJ_OBJ += outlierFilter.o
PROJ_OBJ += flowdeck_v1v2.o
PROJ_OBJ += oa.o
PROJ_OBJ += multiranger.o
PROJ_OBJ += lighthouse.o
PROJ_OBJ += activeMarkerDeck.o

# Uart2 Link for CRTP communication is not compatible with decks using uart2
ifeq ($(UART2_LINK), 1)
CFLAGS += -DUART2_LINK_COMM
else
PROJ_OBJ += aideck.o
endif

ifeq ($(LPS_TDOA_ENABLE), 1)
CFLAGS += -DLPS_TDOA_ENABLE
endif

ifeq ($(LPS_TDOA3_ENABLE), 1)
CFLAGS += -DLPS_TDOA3_ENABLE
endif

ifeq ($(LPS_TDMA_ENABLE), 1)
CFLAGS += -DLPS_TDMA_ENABLE
endif

ifdef SENSORS
SENSORS_UPPER = $(shell echo $(SENSORS) | tr a-z A-Z)
CFLAGS += -DSENSORS_FORCE=SensorImplementation_$(SENSORS)

# Add sensor file to the build if needed
ifeq (,$(findstring DSENSOR_INCLUDED_$(SENSORS_UPPER),$(CFLAGS)))
CFLAGS += -DSENSOR_INCLUDED_$(SENSORS_UPPER)
PROJ_OBJ += sensors_$(SENSORS).o
endif
endif

#Deck tests
PROJ_OBJ += exptest.o
PROJ_OBJ += exptestRR.o
PROJ_OBJ += exptestBolt.o
#PROJ_OBJ += bigquadtest.o
#PROJ_OBJ += uarttest.o
#PROJ_OBJ += aidecktest.o


# Utilities
PROJ_OBJ += filter.o cpuid.o cfassert.o  eprintf.o crc32.o num.o debug.o
PROJ_OBJ += version.o FreeRTOS-openocd.o
PROJ_OBJ += configblockeeprom.o
PROJ_OBJ += sleepus.o statsCnt.o rateSupervisor.o
PROJ_OBJ += lighthouse_core.o pulse_processor.o pulse_processor_v1.o pulse_processor_v2.o lighthouse_geometry.o ootx_decoder.o lighthouse_calibration.o lighthouse_deck_flasher.o lighthouse_position_est.o lighthouse_storage.o lighthouse_transmit.o
PROJ_OBJ += kve_storage.o kve.o
PROJ_OBJ += esp_deck_flasher.o

ifeq ($(DEBUG_PRINT_ON_SEGGER_RTT), 1)
VPATH += $(LIB)/Segger_RTT/RTT
INCLUDES += -I$(LIB)/Segger_RTT/RTT
PROJ_OBJ += SEGGER_RTT.o SEGGER_RTT_printf.o
CFLAGS += -DDEBUG_PRINT_ON_SEGGER_RTT
endif

# Libs
PROJ_OBJ += libarm_math.a

OBJ = $(FREERTOS_OBJ) $(PORT_OBJ) $(ST_OBJ) $(PROJ_OBJ) $(APP_OBJ) $(CRT0)

############### Compilation configuration ################
AS = $(CROSS_COMPILE)as
CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)gcc
SIZE = $(CROSS_COMPILE)size
OBJCOPY = $(CROSS_COMPILE)objcopy
GDB = $(CROSS_COMPILE)gdb

INCLUDES += -I$(CRAZYFLIE_BASE)/vendor/CMSIS/CMSIS/Core/Include -I$(CRAZYFLIE_BASE)/vendor/CMSIS/CMSIS/DSP/Include
INCLUDES += -I$(CRAZYFLIE_BASE)/vendor/libdw1000/inc
INCLUDES += -I$(FREERTOS)/include -I$(PORT)

INCLUDES += -I$(CRAZYFLIE_BASE)/src/config
INCLUDES += -I$(CRAZYFLIE_BASE)/src/platform

INCLUDES += -I$(CRAZYFLIE_BASE)/src/deck/interface -I$(CRAZYFLIE_BASE)/src/deck/drivers/interface
INCLUDES += -I$(CRAZYFLIE_BASE)/src/drivers/interface -I$(CRAZYFLIE_BASE)/src/drivers/bosch/interface -I$(CRAZYFLIE_BASE)/src/drivers/esp32/interface
INCLUDES += -I$(CRAZYFLIE_BASE)/src/hal/interface
INCLUDES += -I$(CRAZYFLIE_BASE)/src/modules/interface -I$(CRAZYFLIE_BASE)/src/modules/interface/kalman_core -I$(CRAZYFLIE_BASE)/src/modules/interface/lighthouse
INCLUDES += -I$(CRAZYFLIE_BASE)/src/utils/interface -I$(CRAZYFLIE_BASE)/src/utils/interface/kve -I$(CRAZYFLIE_BASE)/src/utils/interface/lighthouse -I$(CRAZYFLIE_BASE)/src/utils/interface/tdoa

INCLUDES += -I$(LIB)/FatFS
INCLUDES += -I$(LIB)/CMSIS/STM32F4xx/Include
INCLUDES += -I$(LIB)/STM32_USB_Device_Library/Core/inc
INCLUDES += -I$(LIB)/STM32_USB_OTG_Driver/inc
INCLUDES += -I$(LIB)/STM32F4xx_StdPeriph_Driver/inc
INCLUDES += -I$(LIB)/vl53l1 -I$(LIB)/vl53l1/core/inc

CFLAGS += -g3
ifeq ($(DEBUG), 1)
  CFLAGS += -O0 -DDEBUG

  # Prevent silent errors when converting between types (requires explicit casting)
  CFLAGS += -Wconversion
else
  CFLAGS += -Os

  # Fail on warnings
  CFLAGS += -Werror
endif

# Disable warnings for unaligned addresses in packed structs (added in GCC 9)
CFLAGS += -Wno-address-of-packed-member

# Disable warnings for incorrectly detected region size (added in GCC 11)
# The compiler is not detecting properly GPIO structure size
CFLAGS += -Wno-array-bounds
CFLAGS += -Wno-stringop-overread
CFLAGS += -Wno-stringop-overflow

ifeq ($(LTO), 1)
  CFLAGS += -flto
endif

CFLAGS += -DBOARD_REV_$(REV) -DESTIMATOR_NAME=$(ESTIMATOR)Estimator -DCONTROLLER_NAME=ControllerType$(CONTROLLER) -DPOWER_DISTRIBUTION_TYPE_$(POWER_DISTRIBUTION)

CFLAGS += $(PROCESSOR) $(INCLUDES)


CFLAGS += -Wall -Wmissing-braces -fno-strict-aliasing $(C_PROFILE) -std=gnu11
# CFLAGS += -O0 -Wmissing-braces -fno-strict-aliasing $(C_PROFILE) -std=gnu11 #Use this compiler during debugger, as it has a different optimizer so you can better track variables
# Compiler flags to generate dependency files:
CFLAGS += -MD -MP -MF $(BIN)/dep/$(@).d -MQ $(@)
#Permits to remove un-used functions and global variables from output file
CFLAGS += -ffunction-sections -fdata-sections
# Prevent promoting floats to doubles
CFLAGS += -Wdouble-promotion


ASFLAGS = $(PROCESSOR) $(INCLUDES)
LDFLAGS += --specs=nosys.specs --specs=nano.specs $(PROCESSOR) -Wl,-Map=$(PROG).map,--cref,--gc-sections,--undefined=uxTopUsedPriority
LDFLAGS += -L$(CRAZYFLIE_BASE)/tools/make/F405/linker

#Flags required by the ST library
ifeq ($(CLOAD), 1)
  LDFLAGS += -T $(LINKER_DIR)/FLASH_CLOAD.ld
  LOAD_ADDRESS = $(LOAD_ADDRESS_CLOAD_$(CPU))
else
  LDFLAGS += -T $(LINKER_DIR)/FLASH.ld
  LOAD_ADDRESS = $(LOAD_ADDRESS_$(CPU))
endif

ifeq ($(LTO), 1)
  LDFLAGS += -Os -flto -fuse-linker-plugin
endif

#Program name
PROG ?= $(PLATFORM)
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

# This define n-thing is a standard hack to get newlines in GNU Make.
define n


endef

# Make sure that the submodules are up to date.
# Check if there are any files in the vendor directories, if not warn the user.
ifeq ($(wildcard $(CRAZYFLIE_BASE)/vendor/*/*),)
  $(error $n                                                                   \
    The submodules does not seem to be present, consider fetching them by:$n   \
      $$ git submodule init$n                                                  \
      $$ git submodule update$n                                                \
  )
endif

#################### Targets ###############################

all: bin/ bin/dep bin/vendor check_submodules build
build:
# Each target is in a different line, so they are executed one after the other even when the processor has multiple cores (when the -j option for the make command is > 1). See: https://www.gnu.org/software/make/manual/html_node/Parallel.html
	@$(MAKE) --no-print-directory clean_version CRAZYFLIE_BASE=$(CRAZYFLIE_BASE)
	@$(MAKE) --no-print-directory compile CRAZYFLIE_BASE=$(CRAZYFLIE_BASE)
	@$(MAKE) --no-print-directory print_version CRAZYFLIE_BASE=$(CRAZYFLIE_BASE)
	@$(MAKE) --no-print-directory size CRAZYFLIE_BASE=$(CRAZYFLIE_BASE)
compile: $(PROG).hex $(PROG).bin $(PROG).dfu

bin/:
	mkdir -p bin

bin/dep:
	mkdir -p bin/dep

bin/vendor:
	mkdir -p bin/vendor

libarm_math.a:
	+$(MAKE) -C $(CRAZYFLIE_BASE)/tools/make/cmsis_dsp/ CRAZYFLIE_BASE=$(abspath $(CRAZYFLIE_BASE)) PROJ_ROOT=$(CURDIR) V=$(V) CROSS_COMPILE=$(CROSS_COMPILE)

clean_version:
ifeq ($(SHELL),/bin/sh)
	@echo "  CLEAN_VERSION"
	@rm -f version.c
endif

print_version:
	@echo "Build for the $(PLATFORM_NAME_$(PLATFORM))!"
	@$(PYTHON) $(CRAZYFLIE_BASE)/tools/make/versionTemplate.py --crazyflie-base $(CRAZYFLIE_BASE) --print-version
ifeq ($(CLOAD), 1)
	@echo "Crazyloader build!"
endif
ifeq ($(FATFS_DISKIO_TESTS), 1)
	@echo "WARNING: FatFS diskio tests enabled. Erases SD-card!"
endif

size:
	@$(PYTHON) $(CRAZYFLIE_BASE)/tools/make/size.py $(SIZE) $(PROG).elf $(MEM_SIZE_FLASH_K) $(MEM_SIZE_RAM_K) $(MEM_SIZE_CCM_K)

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

rtt:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) $(OPENOCD_CMDS) -f $(OPENOCD_TARGET) -c init -c targets \
	           -c "rtt setup 0x20000000 262144 \"SEGGER RTT\"" -c "rtt start" -c "rtt server start 2000 0"

gdb: $(PROG).elf
	$(GDB) -ex "target remote localhost:3333" -ex "monitor reset halt" $^
  
erase:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) -f $(OPENOCD_TARGET) -c init -c targets -c "halt" -c "stm32f4x mass_erase 0" -c shutdown

#Print preprocessor #defines
prep:
	@$(CC) $(CFLAGS) -dM -E - < /dev/null

check_submodules:
	@cd $(CRAZYFLIE_BASE); $(PYTHON) tools/make/check-for-submodules.py

include $(CRAZYFLIE_BASE)/tools/make/targets.mk

#include dependencies
-include $(DEPS)

unit:
# The flag "-DUNITY_INCLUDE_DOUBLE" allows comparison of double values in Unity. See: https://stackoverflow.com/a/37790196
	rake unit "DEFINES=$(CFLAGS) -DUNITY_INCLUDE_DOUBLE" "FILES=$(FILES)" "UNIT_TEST_STYLE=$(UNIT_TEST_STYLE)"

# Python bindings
MOD_INC = $(CRAZYFLIE_BASE)/src/modules/interface
MOD_SRC = $(CRAZYFLIE_BASE)/src/modules/src

bindings_python: bindings/setup.py bin/cffirmware_wrap.c $(MOD_SRC)/*.c
	$(PYTHON) bindings/setup.py build_ext --inplace

bin/cffirmware_wrap.c cffirmware.py: bindings/cffirmware.i $(MOD_INC)/*.h
	swig -python -I$(MOD_INC) -o bin/cffirmware_wrap.c bindings/cffirmware.i
	mv bin/cffirmware.py cffirmware.py

test_python: bindings_python
	$(PYTHON) -m pytest test_python

.PHONY: all clean build compile unit prep erase flash check_submodules trace openocd gdb halt reset flash_dfu flash_verify cload size print_version clean_version bindings_python
