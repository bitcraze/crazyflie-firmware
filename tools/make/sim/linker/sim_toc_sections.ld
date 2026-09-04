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
  /* SUBALIGN(4), not the ld default of "each input section's own alignment":
   * x86-64 gcc emits every LOG_GROUP_START(...)/PARAM_GROUP_START(...)
   * per-group array (".log.NAME"/".param.NAME") 32-byte-aligned even though
   * struct log_s/param_s only need 8-byte (pointer) alignment -- a vector-
   * friendly default this target's gcc applies to const arrays that
   * tools/make/F405/linker/sections_FLASH.ld's arm-none-eabi-gcc build
   * never triggers (there, KEEP(*(.log.*)) is exactly this same one-liner,
   * unadorned, and it's fine). Left at ld's default, any group whose byte
   * size isn't itself a multiple of 32 leaves a zero-filled gap before the
   * next 32-aligned group -- invisible to a hex dump, but logsLen/paramsLen
   * are computed via typed pointer subtraction (_log_stop - _log_start),
   * i.e. "gap bytes / sizeof(struct log_s)", so any gap that isn't itself a
   * whole multiple of sizeof(struct log_s) shifts every following entry's
   * apparent index -- and the specific gap size 32-byte over-alignment
   * produces (32 - (size % 32) bytes, zero-filled) generally isn't. First
   * hit in Phase 4.6: adding pm_sim.c's LOG_GROUP_START(pm) as a third .log
   * group (after crtp's and mem.c's) landed a 24-byte gap -- exactly one
   * phantom all-zero struct log_s -- between memTst's group and pm's,
   * which logInit() then read as a real entry: type=0 (fails the "is this
   * a LOG_GROUP marker" check), name=NULL -> strlen(NULL) segfault. Two
   * groups never triggered it (4.3-4.5): the only inter-group boundary
   * present (crtp -> memTst) happened to already sit on a 32-byte boundary.
   * SUBALIGN(4) forces every input section placed here to be treated as
   * 4-byte aligned regardless of its own recorded alignment, matching what
   * the real ARM linker script gets "for free" from its compiler -- closes
   * this off for any future group, not just pm's. */
  .param : ALIGN(4) SUBALIGN(4)
  {
    _param_start = .;
    KEEP(*(.param))
    KEEP(*(.param.*))
    _param_stop = .;
  }

  .log : ALIGN(4) SUBALIGN(4)
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
