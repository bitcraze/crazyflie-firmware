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
ANY_CONFIG := $(OOT)/build/oot-config

OOT_ARGS ?= -C $(CRAZYFLIE_BASE) OOT=$(OOT) EXTRA_CFLAGS="$(EXTRA_CFLAGS)"

CONFIG ?= alldefconfig

ifneq ($(OOT_USES_CXX),)
OOT_ARGS += OOT_USES_CXX=1
endif

.PHONY: all clean cload any

#
# the alldefconfig does not exist as a file in the code as for example cf2_defconfig.
# Selecting it with a path given with KCONFIG_ALLCONFIG var has the firmware be compiled
# with the config file pointed by this path.
#
# default path to alldefconfig file is: $CRAZYFLIE_BASE/configs/all.config
#
all:
	$(MAKE) KBUILD_OUTPUT=$(OOT)/build $(OOT_ARGS) KCONFIG_ALLCONFIG=$(OOT_CONFIG) alldefconfig
	$(MAKE) KBUILD_OUTPUT=$(OOT)/build $(OOT_ARGS) -j 12

clean:
	$(MAKE) KBUILD_OUTPUT=$(OOT)/build $(OOT_ARGS) clean

cload:
	$(MAKE) KBUILD_OUTPUT=$(OOT)/build $(OOT_ARGS) cload

#
# eg: sh$ CONFIG=bolt_defconfig make any
# will provide as config file, the concatenation of cf2_defconfig+$(OOT_CONFIG)
#
# NB. we use 'grep -h' and not 'cat' here to ensure newline character is present between files
#
any:
	mkdir -p $(OOT)/build
	[ "${CONFIG}" = alldefconfig ] && cp $(OOT_CONFIG) $(ANY_CONFIG) \
		|| grep -h "" $(OOT)/$(CRAZYFLIE_BASE)/configs/$(CONFIG) $(OOT_CONFIG) > $(ANY_CONFIG)
	OOT='$(OOT)' OOT_CONFIG='$(ANY_CONFIG)' OOT_ARGS='$(OOT_ARGS)' \
		$(MAKE) all
