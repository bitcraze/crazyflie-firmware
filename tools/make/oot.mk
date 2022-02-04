#
# We tell the firmware that this is an "Out-of-Tree" build so look for
# kbuild files in this directory.
#
OOT ?= $(PWD)

#
# We need to tell the firmware where to find extra config options for this
# build.
#
OOT_CONFIG ?= $(OOT)/oot-config

OOT_ARGS ?= -C $(CRAZYFLIE_BASE) OOT=$(OOT) OOT_CONFIG=$(OOT_CONFIG)

.PHONY: all clean

all:
	$(MAKE) $(OOT_ARGS) oot-config
	$(MAKE) $(OOT_ARGS) -j 12

clean:
	$(MAKE) $(OOT_ARGS) clean

cload:
	$(MAKE) $(OOT_ARGS) cload
