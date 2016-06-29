include obj.mk

PROJ_ROOT=../../..
BIN=$(PROJ_ROOT)/bin/vendor
PROJ_BIN=$(PROJ_ROOT)/bin
DSP_SRC=$(PROJ_ROOT)/vendor/CMSIS/CMSIS/DSP_Lib/Source
DSP_INC=$(PROJ_ROOT)/vendor/CMSIS/CMSIS/Include

VPATH += $(BIN)
VPATH += $(DSP_SRC)/BasicMathFunctions/
VPATH += $(DSP_SRC)/CommonTables/
VPATH += $(DSP_SRC)/ComplexMathFunctions/
VPATH += $(DSP_SRC)/ControllerFunctions/
VPATH += $(DSP_SRC)/FastMathFunctions/
VPATH += $(DSP_SRC)/FilteringFunctions/
VPATH += $(DSP_SRC)/MatrixFunctions/
VPATH += $(DSP_SRC)/StatisticsFunctions/
VPATH += $(DSP_SRC)/SupportFunctions/
VPATH += $(DSP_SRC)/TransformFunctions/


LIB=$(PROJ_BIN)/libarm_math.a

CSFLAGS = -O2 -I$(DSP_INC) -std=c11 -DARM_MATH_CM4 -D__FPU_PRESENT=1 -D__TARGET_FPU_VFP  -mfloat-abi=hard -mfpu=fpv4-sp-d16
CSFLAGS += -mcpu=cortex-m4 -mthumb -ffunction-sections -fdata-sections
CFLAGS=$(CSFLAGS) -include stdint.h

CROSS_COMPILE     ?= arm-none-eabi-
AS = $(CROSS_COMPILE)as
CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)gcc
AR = $(CROSS_COMPILE)ar
SIZE = $(CROSS_COMPILE)size
OBJCOPY = $(CROSS_COMPILE)objcopy
GDB = $(CROSS_COMPILE)gdb

all: $(LIB)

$(LIB): $(DSP_OBJ) #$(foreach o,$(DSP_OBJ),$(BIN)/$(o))
	@$(AR) rcs $@ $(foreach o,$(DSP_OBJ),$(BIN)/$(o))
	@echo "  AR    libarm_math.a"

include ../targets.mk
