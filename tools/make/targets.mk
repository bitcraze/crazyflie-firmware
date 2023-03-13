# Part of CrazyFlie's Makefile
# Copyright (c) 2009, Arnaud Taffanel
# Common targets with verbose support


ifeq ($(V),)
  VERBOSE=_SILENT
endif

ifeq ($(V),0)
  QUIET=1
endif

target = @$(if $(QUIET), ,echo $($1_COMMAND$(VERBOSE)) ); @$($1_COMMAND)

VTMPL_COMMAND=$(PYTHON) $(CRAZYFLIE_BASE)/tools/make/versionTemplate.py --crazyflie-base $(CRAZYFLIE_BASE) $< $@
#$(BIN)/$(lastword $(subst /, ,$@))
VTMPL_COMMAND_SILENT="  VTMPL $@"
%.c: %.vtpl
	@$(if $(QUIET), ,echo $(VTMPL_COMMAND$(VERBOSE)) )
	@$(VTMPL_COMMAND)

CC_COMMAND=$(CC) $(CFLAGS) -c $< -o $(BIN)/$@
CC_COMMAND_SILENT="  CC    $@"
.c.o:
	@$(if $(QUIET), ,echo $(CC_COMMAND$(VERBOSE)) )
	@$(CC_COMMAND)

CCS_COMMAND=$(CC) $(CSFLAGS) -c $< -o $(BIN)/$@
CCS_COMMAND_SILENT="  CCS   $@"
.S.o:
	@$(if $(QUIET), ,echo $(CCS_COMMAND$(VERBOSE)) )
	@$(CCS_COMMAND)

HEX_COMMAND=$(OBJCOPY) $< -O ihex $@
HEX_COMMAND_SILENT="  COPY  $@"
$(PROG).hex: $(PROG).elf
	@$(if $(QUIET), ,echo $(HEX_COMMAND$(VERBOSE)) )
	@$(HEX_COMMAND)

BIN_COMMAND=$(OBJCOPY) $< -O binary --pad-to 0 --remove-section=.bss --remove-section=.nzds  --remove-section=._usrstack $@
BIN_COMMAND_SILENT="  COPY  $@"
$(PROG).bin: $(PROG).elf
	@$(if $(QUIET), ,echo $(BIN_COMMAND$(VERBOSE)) )
	@$(BIN_COMMAND)

DFU_COMMAND=$(PYTHON) $(CRAZYFLIE_BASE)/tools/make/dfu-convert.py -b $(LOAD_ADDRESS):$< $@
DFU_COMMAND_SILENT="  DFUse $@"
$(PROG).dfu: $(PROG).bin
	@$(if $(QUIET), ,echo $(DFU_COMMAND$(VERBOSE)) )
	@$(DFU_COMMAND)

AS_COMMAND=$(AS) $(ASFLAGS) $< -o $(BIN)/$@
AS_COMMAND_SILENT="  AS    $@"
.s.o:
	@$(if $(QUIET), ,echo $(AS_COMMAND$(VERBOSE)) )
	@$(AS_COMMAND)

CLEAN_O_COMMAND=rm -f $(foreach o,$(OBJ),$(BIN)/$(o))
CLEAN_O_COMMAND_SILENT="  CLEAN_O"
clean_o: clean_version
	@$(if $(QUIET), ,echo $(CLEAN_O_COMMAND$(VERBOSE)) )
	@$(CLEAN_O_COMMAND)

clean_cf:
	@rm -f $(srctree)/$(PROG).*
	@rm -f $(srctree)/cffirmware.py
	@rm -f $(srctree)/_cffirmware*.so
	@rm -f $(srctree)/build/cffirmware_wrap.c
