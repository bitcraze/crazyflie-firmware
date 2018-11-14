# Platform handling

# List all available platform
platforms_files = $(wildcard tools/make/platforms/*.mk)
platforms_available = $(patsubst tools/make/platforms/%.mk,%, $(platforms_files))

# Find out if the requested platform exists
ifeq ($(filter $(PLATFORM),$(platforms_available)),)
$(info Platform '$(PLATFORM)' not found!)
$(info -------------------)
override PLATFORM=
undefine PLATFORM
endif

# If there is no platform requested or the platform does not exist, print some help
ifndef PLATFORM
PLATFORM_DOC=1
include tools/make/platforms/*.mk
$(info You must provide a platform with 'make PLATFORM=<platform>')
$(info Available platforms:)
$(foreach plat,$(platforms_available),$(info - $(plat) :	$(PLATFORM_HELP_$(plat))))
$(info -------------------)
$(info The platform will be written in 'current_platform.mk' and will be used by)
$(info default for subsequent calls.)
$(info if you want to reset and see this message again type 'make mrproper')
$(info -------------------)
$(error Platform not defined)
endif

# Include the platform makefile configuration
include tools/make/platforms/$(PLATFORM).mk

# Write current platform in a file to make it stick for future call to make
$(shell echo "PLATFORM=$(PLATFORM)" > current_platform.mk)

