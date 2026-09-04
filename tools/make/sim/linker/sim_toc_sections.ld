/*
 * sim_toc_sections.ld - linker symbol shims for CONFIG_PLATFORM_SIM
 * (Simmyflie), Phase 4.4. Phase 4.5 (Logging) adds the same treatment for
 * log.c's _log_start/_log_stop -- identical reasoning, just a second
 * TOC-shaped array (LOG_GROUP_START(...)'s ".log.NAME" input sections
 * instead of PARAM_GROUP_START(...)'s ".param.NAME" ones).
 *
 * Real hardware links against tools/make/F405/linker/*.ld, whose explicit
 * SECTIONS block gathers each PARAM_GROUP_START(...)'s per-group
 * ".param.NAME" input section into one contiguous ".param" output section
 * and exposes its bounds as _param_start/_param_stop -- param_logic.c walks
 * that array directly (paramLogicInit(): "params = &_param_start;
 * paramsLen = &_param_stop - &_param_start;"). Simmyflie's host build has
 * no custom linker script at all (plain gcc/ld, see the Makefile's
 * CONFIG_PLATFORM_SIM branch) -- left alone, every differently-named
 * ".param.NAME" input section becomes its own separate, unbounded output
 * section, and _param_start/_param_stop are simply undefined references.
 * This "INSERT AFTER" appends the same collection to the host's default
 * script without replacing it (same trick GNU ld provides for exactly this
 * "add a bit to the default script" case) -- unrelated to and much smaller
 * than the ARM side's full custom FLASH/RAM layout.
 *
 * param_logic.c's paramGetDefault() also takes the address of
 * _sdata/_edata/_sidata/_stext/_etext, to recover a parameter's *compiled-in*
 * default value from flash even after RAM has been overwritten -- a
 * consequence of ARM's flash-holds-the-true-initializer/RAM-is-just-a-copy
 * split. A host ELF executable has no such split (.data's initial bytes
 * come from the same address space the running program mutates; there is
 * no separate flash image to fall back to), so that whole code path has no
 * sim equivalent to point at. Rather than gate it out in mainline
 * param_logic.c (unlike the NRF-companion-chip or I2C-bus gaps 4.0-4.2
 * gated, this one isn't "hardware doesn't exist in sim", it's "the address
 * space model itself doesn't apply"), _sdata/_stext are pinned here to a
 * point *after* every real static/global's address (right after .bss, the
 * last of the three) so paramGetDefault()'s "is this address in the
 * data/text range" checks can never match any real parameter -- every
 * parameter falls through to the dummyZero64 case, a default of 0. _edata
 * (end of .data) and _etext (end of .text) are already provided by the
 * default host script; only _sdata/_stext need defining, made deliberately
 * "after" so [_sdata, _edata] and [_stext, _etext] are always empty ranges
 * regardless of real memory layout. _sidata has no host meaning either way
 * (nothing ever reads it once its range check is unreachable) -- pointed at
 * the same place for consistency.
 *
 * MVP has no persistent parameters (see storage_sim.c), and no verification
 * step exercises MISC_GET_DEFAULT_VALUE(_V2) -- if a future phase needs a
 * real default-value answer in sim, that's the point to revisit this.
 */

SECTIONS
{
  .param : ALIGN(4)
  {
    _param_start = .;
    KEEP(*(.param))
    KEEP(*(.param.*))
    _param_stop = .;
  }

  .log : ALIGN(4)
  {
    _log_start = .;
    KEEP(*(.log))
    KEEP(*(.log.*))
    _log_stop = .;
  }

  .sim_toc_after_bss : ALIGN(4)
  {
    _sdata = .;
    _sidata = .;
    _stext = .;
  }
}
INSERT AFTER .bss;
