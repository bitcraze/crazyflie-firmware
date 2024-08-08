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

DEFAULT_CONFIG ?= alldefconfig

ifneq ($(OOT_USES_CXX),)
OOT_ARGS += OOT_USES_CXX=1
endif

.PHONY: all clean menuconfig nconfig xconfig gconfig cload

%_defconfig:
	@if [ -f $(CRAZYFLIE_BASE)/configs/$@ ]; then \
		echo "Loading defconfig $@ from configs folder"; \
		$(MAKE) KBUILD_OUTPUT=$(OOT)/build $(OOT_ARGS) KCONFIG_ALLCONFIG=configs/$@ $(DEFAULT_CONFIG); \
	else \
		echo "Defconfig $@ not found in configs folder"; \
		exit 1; \
	fi

all:
	@if [ ! -f $(OOT)/build/.config ]; then \
		$(MAKE) KBUILD_OUTPUT=$(OOT)/build $(OOT_ARGS) $(DEFAULT_CONFIG); \
	fi
	@if [ -f $(OOT_CONFIG) ]; then \
        echo "Merging $(OOT_CONFIG) into .config"; \
        $(CRAZYFLIE_BASE)/scripts/kconfig/merge_config.sh -O $(OOT)/build -m $(OOT)/build/.config $(OOT_CONFIG); \
    else \
        echo "$(OOT_CONFIG) does not exist"; \
    fi
	$(MAKE) KBUILD_OUTPUT=$(OOT)/build $(OOT_ARGS) oldconfig
	$(MAKE) KBUILD_OUTPUT=$(OOT)/build $(OOT_ARGS)

clean:
	$(MAKE) KBUILD_OUTPUT=$(OOT)/build $(OOT_ARGS) clean

menuconfig:
	$(MAKE) KBUILD_OUTPUT=$(OOT)/build $(OOT_ARGS) menuconfig

nconfig:
	$(MAKE) KBUILD_OUTPUT=$(OOT)/build $(OOT_ARGS) nconfig

xconfig:
	$(MAKE) KBUILD_OUTPUT=$(OOT)/build $(OOT_ARGS) xconfig

gconfig:
	$(MAKE) KBUILD_OUTPUT=$(OOT)/build $(OOT_ARGS) gconfig

cload:
	$(MAKE) KBUILD_OUTPUT=$(OOT)/build $(OOT_ARGS) cload
