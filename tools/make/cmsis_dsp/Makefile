include obj.mk

BIN=$(PROJ_ROOT)/bin/vendor
PROJ_BIN=$(PROJ_ROOT)/bin
CMSIS_INC=$(CRAZYFLIE_BASE)/vendor/CMSIS/CMSIS/Core/Include
DSP_INC=$(CRAZYFLIE_BASE)/vendor/CMSIS/CMSIS/DSP/Include
DSP_SRC=$(CRAZYFLIE_BASE)/vendor/CMSIS/CMSIS/DSP/Source

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

CFLAGS = -O2 -I$(CMSIS_INC) -I$(DSP_INC) -std=c11
CFLAGS += -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -DARM_MATH_CM4 -D__FPU_PRESENT=1 -ffunction-sections -fdata-sections

CROSS_COMPILE     ?= arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
AR = $(CROSS_COMPILE)ar

all: $(LIB)

$(LIB): $(DSP_OBJ) #$(foreach o,$(DSP_OBJ),$(BIN)/$(o))
	@$(AR) rcs $@ $(foreach o,$(DSP_OBJ),$(BIN)/$(o))
	@echo "  AR    libarm_math.a"

include ../targets.mk
