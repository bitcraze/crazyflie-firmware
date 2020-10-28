
# enable app support
APP=1
APP_STACKSIZE=300

## Weight of the Crazyflie + Qi + Lighthouse deck
CFLAGS += -DCF_MASS=0.036f

## To build with lighthouse support
CFLAGS += -DDISABLE_LIGHTHOUSE_DRIVER=0

## Force lighthouse 2
CFLAGS += -DLIGHTHOUSE_FORCE_TYPE=2

# CLOAD_CMDS = -w radio://0/30/2M
# DEBUG=1


VPATH += src/
PROJ_OBJ += app.o

CRAZYFLIE_BASE=../../..
include $(CRAZYFLIE_BASE)/Makefile
