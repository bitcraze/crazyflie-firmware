# Part of CrazyFlie  Makefile
# Copyright (c) 2021 Bitcraze AB

RUST_DECK_DRIVERS_PATH = $(CRAZYFLIE_BASE)/src/deck/drivers/rust

ifeq ($(RUST_DECK_DRIVERS_ENABLED), 1)
# This function generates a target for a deck driver written in
# rust. It must be located at $(RUST_DECK_DRIVERS_PATH) and the name
# of the directory must be the name of the Cargo project.
define rust-driver-target
cargo_args = --target thumbv7em-none-eabihf --manifest-path=$(1)/Cargo.toml
rustc_args = -C opt-level=3 --emit=obj

$(BIN)/$(notdir $(1)).o: $(1)/src/lib.rs
	cargo rustc $$(cargo_args) -- $$(rustc_args)
	@##
	@## Make sure we copy the latest version (rustc adds a hash to the name for
	@## each new object file).
	@##
	@cp `ls -t $(1)/target/*/*/deps/$(notdir $(1))*.o | head -n1` $$@

PROJ_OBJ += $(notdir $(1)).o
endef

# For each of the drivers found in $(RUST_DECK_DRIVERS_PATH) we will
# generate a make target for them with the function defined above.
drivers = $(wildcard $(RUST_DECK_DRIVERS_PATH)/*)
$(foreach d,$(drivers),$(eval $(call rust-driver-target,$(d))))
endif
