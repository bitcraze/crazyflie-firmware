
OPENOCD           ?= openocd
OPENOCD_INTERFACE ?= interface/stlink-v2.cfg
OPENOCD_TARGET    ?= target/stm32f4x.cfg
OPENOCD_CMDS      ?=

CPU                        = stm32f4
LOAD_ADDRESS_stm32f4       = 0x8000000
LOAD_ADDRESS_CLOAD_stm32f4 = 0x8004000

PYTHON            ?= python3

# Cload is handled in a special way on windows in WSL to use the Windows python interpreter
ifdef WSL_DISTRO_NAME
CLOAD_SCRIPT      ?= python.exe -m cfloader
else
CLOAD_SCRIPT      ?= $(PYTHON) -m cfloader
endif

DFU_UTIL          ?= dfu-util

CLOAD_CMDS        ?=
CLOAD_ARGS        ?=

ARCH := stm32f4
SRCARCH := stm32f4

# NOTE: CONFIG_PLATFORM_SIM is not known yet at this point in the file --
# CONFIG_* variables only become available after the auto.conf include
# below -- so the ARCH_CFLAGS/FREERTOS/PORT/LDFLAGS/INCLUDES platform branch
# lives further down, right after that include (matching where the existing
# CONFIG_PLATFORM_* -> PLATFORM name conditionals already do the same).

# Here we tell Kbuild where to look for Kbuild files which will tell the
# buildsystem which sources to build
objs-y += src
objs-y += vendor

objs-y += app_api
objs-y += $(OOT)

MEM_SIZE_FLASH_K = 1008
MEM_SIZE_RAM_K = 128
MEM_SIZE_CCM_K = 64


#
# Make sure Kbuild use our config that hinders some configs from being enabled
# on allyesconfig or randconfig.
#
export KCONFIG_ALLCONFIG ?= configs/all.config

KBUILD_OUTPUT ?= build

-include $(KBUILD_OUTPUT)/include/config/auto.conf

ifeq ($(CONFIG_PLATFORM_SIM),y)

# Simmyflie: native Linux build on the vendored FreeRTOS POSIX port. No
# cross-compiler and no embedded link flags (see CONFIG_CROSS_COMPILE in
# configs/sim_defconfig, which blanks the arm-none-eabi- default).
# -Wno-unused-variable: the vendored FreeRTOS POSIX port (ThirdParty, not
# ours to fix) has a couple of genuinely-unused locals.
ARCH_CFLAGS += -g3 -pthread -D_GNU_SOURCE -Wno-unused-variable

FREERTOS = $(srctree)/vendor/FreeRTOS
PORT = $(FREERTOS)/portable/ThirdParty/GCC/Posix
LIB = $(srctree)/src/lib

LDFLAGS =
image_LDFLAGS += -pthread
image_LDFLAGS += -Wl,-Map=$(PROG).map,--cref,--gc-sections

INCLUDES += -I$(srctree)/src/config/sim
INCLUDES += -I$(PORT)/utils
# instance_sim.h (Phase 2) lives alongside main_sim.c in src/init/, not a
# dedicated interface/ dir like the other INCLUDES below -- Communication
# (Phase 3, src/hal/src/udplink_sim.c) needs instanceGetSocketFd() from it.
INCLUDES += -I$(srctree)/src/init

# src/config/sim must come before the common -I$(srctree)/src/config below
# so our FreeRTOSConfig.h wins over the mainline one.

else

ARCH_CFLAGS += -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g3
ARCH_CFLAGS += -fno-math-errno -DARM_MATH_CM4 -D__FPU_PRESENT=1 -mfp16-format=ieee
ARCH_CFLAGS += -Wno-array-bounds -Wno-stringop-overread
ARCH_CFLAGS += -Wno-stringop-overflow
ARCH_CFLAGS += -DSTM32F4XX -DSTM32F40_41xxx -DHSE_VALUE=8000000 -DUSE_STDPERIPH_DRIVER

FREERTOS = $(srctree)/vendor/FreeRTOS
PORT = $(FREERTOS)/portable/GCC/ARM_CM4F
LIB = $(srctree)/src/lib
PROCESSOR = -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
LINKER_DIR = $(srctree)/tools/make/F405/linker

LDFLAGS = $(PROCESSOR)
image_LDFLAGS += --specs=nosys.specs --specs=nano.specs -nostdlib
image_LDFLAGS += -z noexecstack
image_LDFLAGS += -Wl,-Map=$(PROG).map,--cref,--gc-sections,--undefined=uxTopUsedPriority
image_LDFLAGS += -L$(srctree)/tools/make/F405/linker
image_LDFLAGS += -T $(LINKER_DIR)/FLASH_CLOAD.ld

endif

INCLUDES += -I$(srctree)/vendor/CMSIS/CMSIS/Core/Include -I$(srctree)/vendor/CMSIS/CMSIS/DSP/Include
INCLUDES += -I$(srctree)/vendor/libdw1000/inc
INCLUDES += -I$(FREERTOS)/include -I$(PORT)
INCLUDES += -I$(srctree)/src/config
INCLUDES += -I$(srctree)/src/platform/interface
INCLUDES += -I$(srctree)/src/deck/interface -I$(srctree)/src/deck/drivers/interface
INCLUDES += -I$(srctree)/src/drivers/interface -I$(srctree)/src/drivers/bosch/interface
INCLUDES += -I$(srctree)/src/drivers/esp32/interface
INCLUDES += -I$(srctree)/src/hal/interface
INCLUDES += -I$(srctree)/src/modules/interface -I$(srctree)/src/modules/interface/kalman_core -I$(srctree)/src/modules/interface/lighthouse  -I$(srctree)/src/modules/interface/outlierfilter
INCLUDES += -I$(srctree)/src/modules/interface/cpx -I$(srctree)/src/modules/interface/p2pDTR -I$(srctree)/src/modules/interface/controller  -I$(srctree)/src/modules/interface/estimator
INCLUDES += -I$(srctree)/src/utils/interface -I$(srctree)/src/utils/interface/kve -I$(srctree)/src/utils/interface/lighthouse -I$(srctree)/src/utils/interface/tdoa
INCLUDES += -I$(LIB)/FatFS
INCLUDES += -I$(LIB)/CMSIS/STM32F4xx/Include
INCLUDES += -I$(LIB)/STM32_USB_Device_Library/Core/inc
INCLUDES += -I$(LIB)/STM32_USB_OTG_Driver/inc
INCLUDES += -I$(LIB)/STM32F4xx_StdPeriph_Driver/inc
INCLUDES += -I$(LIB)/vl53l1 -I$(LIB)/vl53l1/core/inc
INCLUDES += -I$(KBUILD_OUTPUT)/include/generated

#
# Special hack to handle float define. Kconfig has no float values
# so we use string. To avoid having to do atof in code we instead
# catch it here and convert to an, unquoted, float define.
#
ifneq ($(CONFIG_DECK_LOCO_2D_POSITION_HEIGHT),)
unquoted = $(patsubst "%",%,$(CONFIG_DECK_LOCO_2D_POSITION_HEIGHT))
ARCH_CFLAGS += -DDECK_LOCO_2D_POSITION_HEIGHT=$(unquoted)
endif

ifeq ($(CONFIG_PLATFORM_CF21BL), y)
PLATFORM = cf21bl
endif

ifeq ($(CONFIG_PLATFORM_TAG),y)
PLATFORM = tag
endif

ifeq ($(CONFIG_PLATFORM_BOLT), y)
PLATFORM = bolt
endif

ifeq ($(CONFIG_PLATFORM_FLAPPER),y)
PLATFORM = flapper
endif

ifeq ($(CONFIG_PLATFORM_SIM),y)
PLATFORM = sim
endif


PLATFORM  ?= cf2
PROG ?= $(PLATFORM)

ifeq ($(CONFIG_DEBUG),y)
ARCH_CFLAGS	+= -O0 -Wconversion
else
ARCH_CFLAGS += -Os -Werror
endif

_all:

ifeq ($(CONFIG_PLATFORM_SIM),y)
all: $(PROG).elf
	@echo "Build for the sim platform!"
else
all: $(PROG).hex $(PROG).bin
	@echo "Build for the $(PLATFORM) platform!"
	@$(PYTHON) $(srctree)/tools/make/versionTemplate.py --crazyflie-base $(srctree) --print-version
	@$(PYTHON) $(srctree)/tools/make/size.py $(SIZE) $(PROG).elf $(MEM_SIZE_FLASH_K) $(MEM_SIZE_RAM_K) $(MEM_SIZE_CCM_K)
endif

include tools/make/targets.mk

# Simmyflie: build the sim platform into build/sim/ using the same
# configuration as the normal build (build/.config, e.g. after
# 'make cf2_defconfig' and optionally 'make menuconfig'), with just the
# sim-specific overrides from configs/sim_defconfig layered on top. Anything
# that only makes sense for real hardware (decks, sensors, ... -- gated by
# 'depends on PLATFORM_CF2' and friends in Kconfig) necessarily falls away
# once the platform choice flips to PLATFORM_SIM; everything else (e.g.
# controller/estimator selection) carries through unchanged.
#
# Kbuild's own generic goal-redirect (see tools/kbuild/Makefile.kbuild)
# means this recipe actually runs with cwd == $(KBUILD_OUTPUT) (e.g. build/),
# not the source tree root -- hence $(CURDIR) (this dir) for sim's own
# output and $(srctree) (source root) for configs/sim_defconfig. The nested
# builds below run in a scrubbed environment (env -i, PATH/HOME only), not
# just with KBUILD_SRC/MAKEFLAGS cleared: this recipe is itself already
# running inside one redirect for build/'s own (hardware) config, which by
# this point has 'export'ed CC/CROSS_COMPILE/AS/LD/... (see
# tools/kbuild/Makefile.kbuild) into THIS process's environment. Left in
# place, those beat build/sim/'s own CONFIG_CROSS_COMPILE="" (Kbuild's
# CROSS_COMPILE ?= ... only takes the Kconfig value when nothing already
# set it) and silently link sim against the ARM cross compiler. The
# explicit silentoldconfig between alldefconfig and the real build matters
# for the same reason: on a brand new build/sim/, skipping straight to the
# build would have it generate include/config/auto.conf itself mid-parse,
# restart, and re-export CC from the pre-restart (still unconfigured) pass.
.PHONY: sim
sim:
	@test -f .config || { echo "No configuration in $(CURDIR) -- run 'make cf2_defconfig' (or another *_defconfig) first."; exit 1; }
	@mkdir -p sim
	@cat .config $(srctree)/configs/sim_defconfig > sim/.config.seed
	env -i PATH=$$PATH HOME=$$HOME KBUILD_OUTPUT=$(CURDIR)/sim KCONFIG_ALLCONFIG=$(CURDIR)/sim/.config.seed $(MAKE) -C $(srctree) alldefconfig
	env -i PATH=$$PATH HOME=$$HOME KBUILD_OUTPUT=$(CURDIR)/sim $(MAKE) -C $(srctree) silentoldconfig
	env -i PATH=$$PATH HOME=$$HOME KBUILD_OUTPUT=$(CURDIR)/sim $(MAKE) -C $(srctree)
	cp sim/sim.elf sim.elf
	@echo "Simmyflie built: $(CURDIR)/sim.elf"

size:
	@$(PYTHON) $(srctree)/tools/make/size.py $(SIZE) $(PROG).elf $(MEM_SIZE_FLASH_K) $(MEM_SIZE_RAM_K) $(MEM_SIZE_CCM_K)

# Radio bootloader
CLOAD ?= 1
cload:
ifeq ($(CLOAD), 1)
	$(CLOAD_SCRIPT) $(CLOAD_CMDS) flash $(CLOAD_ARGS) $(PROG).bin stm32-fw
else
	@echo "Only cload build can be bootloaded. Launch build and cload with CLOAD=1"
endif

# Flags required by the ST library
ifeq ($(CLOAD), 1)
  LOAD_ADDRESS = $(LOAD_ADDRESS_CLOAD_$(CPU))
else
  LOAD_ADDRESS = $(LOAD_ADDRESS_$(CPU))
endif

unit:
# The flag "-DUNITY_INCLUDE_DOUBLE" allows comparison of double values in Unity. See: https://stackoverflow.com/a/37790196
	rake unit "DEFINES=$(ARCH_CFLAGS) -DUNITY_INCLUDE_DOUBLE" "FILES=$(FILES)" "UNIT_TEST_STYLE=$(UNIT_TEST_STYLE)"

#Flash the stm.
flash:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) $(OPENOCD_CMDS) -f $(OPENOCD_TARGET) -c init -c targets -c "reset halt" \
                 -c "flash write_image erase $(PROG).bin $(LOAD_ADDRESS) bin" \
                 -c "verify_image $(PROG).bin $(LOAD_ADDRESS) bin" -c "reset run" -c shutdown

#verify only
flash_verify:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) $(OPENOCD_CMDS) -f $(OPENOCD_TARGET) -c init -c targets -c "reset halt" \
                 -c "verify_image $(PROG).bin $(LOAD_ADDRESS) bin" -c "reset run" -c shutdown

#sends a usb message to the CF to place it in DFU mode, then updates firmware over usb
flash_dfu:
	$(PYTHON) $(srctree)/tools/make/usb-bootloader.py
	$(DFU_UTIL) -d 0483:df11 -a 0 -s 0x08004000:leave -D $(PROG).bin

#uses the dfu utility to flash the firmware at 0x08004000, just after the bootloader
#call this target directly if CF cannont be flashed automatically through flash_dfu
flash_dfu_manual:
	$(DFU_UTIL) -d 0483:df11 -a 0 -s 0x08004000:leave -D $(PROG).bin

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
	@cd $(srctree); $(PYTHON) tools/make/check-for-submodules.py

# Give control over to Kbuild
-include tools/kbuild/Makefile.kbuild


ifeq ($(KBUILD_SRC),)
# Python bindings
MOD_INC = src/modules/interface
MOD_SRC = src/modules/src

bindings_python build/cffirmware.py: bindings/setup.py $(MOD_SRC)/*.c
	swig -python -I$(MOD_INC) -Isrc/hal/interface -Isrc/utils/interface -I$(MOD_INC)/controller -Isrc/platform/interface -I$(MOD_INC)/outlierfilter -I$(MOD_INC)/kalman_core -o build/cffirmware_wrap.c bindings/cffirmware.i
	$(PYTHON) bindings/setup.py build_ext --inplace
	cp cffirmware_setup.py build/setup.py

test_python: build/cffirmware.py
	PYTHONPATH=build $(PYTHON) -m pytest test_python

python_wheel: build/cffirmware.py
	$(PYTHON) bindings/setup.py bdist_wheel
endif

.PHONY: all sim clean build compile unit prep erase flash check_submodules trace openocd gdb halt reset flash_dfu flash_dfu_manual flash_verify cload size print_version clean_version bindings_python test_python python_wheel
