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

OOT_ARGS ?= -C $(CRAZYFLIE_BASE) OOT=$(OOT) EXTRA_CFLAGS="$(EXTRA_CFLAGS)"

CONFIG ?= alldefconfig

ifneq ($(OOT_USES_CXX),)
OOT_ARGS += OOT_USES_CXX=1
endif

.PHONY: all clean

all:
	$(MAKE) KBUILD_OUTPUT=$(OOT)/build $(OOT_ARGS) KCONFIG_ALLCONFIG=$(OOT_CONFIG) $(CONFIG)
	$(MAKE) KBUILD_OUTPUT=$(OOT)/build $(OOT_ARGS) -j 12

clean:
	$(MAKE) KBUILD_OUTPUT=$(OOT)/build $(OOT_ARGS) clean

cload:
	$(MAKE) KBUILD_OUTPUT=$(OOT)/build $(OOT_ARGS) cload
