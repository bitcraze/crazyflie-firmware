# Build the firmware with this app linked in.
#
#   cd examples/app_tdoa_flight
#   make cf21bl_defconfig      # or cf2_defconfig
#   make -j$(nproc)
#
# The first command configures: platform defconfig, then the TDoA capture
# fragment and this app's app-config merged on top, then the resolution pass
# that merge_config.sh -m deliberately skips. The second builds.
#
# Everything lands in ./build, so the repo root build/ is left alone and the two
# can coexist.
#
# This deliberately does not include tools/make/oot.mk: its 'all' target runs
# `oldconfig`, which prompts for symbols the capture fragment unlocks (turning
# on DECK_LOCODECK_USE_ALT_PINS reveals LOCODECK_ALT_PIN_RESET, and so on). The
# targets below use olddefconfig instead, so a build is never interactive.

CRAZYFLIE_BASE := ../..
OOT := $(CURDIR)
OOT_BUILD := $(CURDIR)/build

# Merged onto the platform defconfig, in order. Later fragments win.
OOT_FRAGMENTS := \
	$(CRAZYFLIE_BASE)/tools/usdlog/tdoa3_lh_groundtruth.conf \
	$(CURDIR)/app-config

FW_MAKE := $(MAKE) -C $(CRAZYFLIE_BASE) KBUILD_OUTPUT=$(OOT_BUILD) OOT=$(OOT)

.PHONY: all clean menuconfig cload flash

all:
	@test -f $(OOT_BUILD)/.config || { \
		echo "No $(OOT_BUILD)/.config yet. Run 'make cf21bl_defconfig' first"; \
		echo "(or cf2_defconfig -- see configs/ for the platform list)."; \
		exit 1; }
	$(FW_MAKE)

# e.g. `make cf21bl_defconfig`. Configures only; run `make` afterwards.
%_defconfig:
	$(FW_MAKE) $@
	$(CRAZYFLIE_BASE)/scripts/kconfig/merge_config.sh -m -O $(OOT_BUILD) \
		$(OOT_BUILD)/.config $(OOT_FRAGMENTS)
	$(FW_MAKE) olddefconfig
	@echo
	@echo "Configured. Now: make -j\$$(nproc)"

clean:
	$(FW_MAKE) clean

menuconfig:
	$(FW_MAKE) menuconfig

cload:
	$(FW_MAKE) cload
