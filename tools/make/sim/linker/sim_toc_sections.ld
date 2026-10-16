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
   * this off for any future group, not just pm's.
   *
   * Phase 4.8 update: SUBALIGN(4) alone was not enough once enough groups
   * (13 new ones, mostly stabilizer.c's) were linked at once -- struct
   * log_s is 24 bytes on a 64-bit host (8-byte pointer alignment for its
   * name/address members), and 4-byte sub-alignment can still leave a
   * group boundary short of the 8 bytes a pointer-containing struct
   * actually needs, the same "boundary happens not to land on a multiple
   * of sizeof(struct)" class of gap as the original bug, just a smaller
   * one -- confirmed empirically: readelf reported .log at 0x21e8 (8680)
   * bytes, and 8680 is not a multiple of 24 (361 entries plus 16 leftover
   * bytes -- a real gap, not a rounding artifact of this comment's math).
   * SUBALIGN(8), not 16: the Makefile's PLATFORM_SIM branch also now
   * passes -malign-data=abi, which caps x86-64 gcc's per-array alignment
   * request at 16 instead of 32 -- but that request only governs what
   * ld's *default* placement (no SUBALIGN override) would honor, and
   * SUBALIGN exists precisely to override it, so it doesn't need to match.
   * What this bug actually depends on is SUBALIGN's value evenly dividing
   * every struct size placed in the section, so the running offset after
   * any whole number of groups is already aligned and no padding is ever
   * inserted. struct log_s is 24 bytes and struct param_s is 40 bytes on
   * a 64-bit host (8-byte pointer members) -- both multiples of 8, neither
   * a multiple of 16. SUBALIGN(16) would therefore reintroduce exactly
   * this class of gap for .param: a lone-entry or odd-sized group can land
   * short of the next 16-byte boundary by an amount that isn't a multiple
   * of 40, corrupting paramsLen the same way the original bug corrupted
   * logsLen. SUBALIGN(8) is the largest value both struct sizes are
   * guaranteed multiples of -- not whatever alignment the compiler
   * happened to request upstream. See the Makefile's own comment for why
   * -malign-data=abi was still added separately (a second, earlier
   * merge-stage instance of this same bug, in Kbuild's intermediate
   * `ld -r` partial links, that this file's SUBALIGN cannot reach). Still
   * a safe superset of what ARM's 4-byte-aligned struct param_s/log_s
   * need, so no behavior change there. */
  .param : ALIGN(8) SUBALIGN(8)
  {
    _param_start = .;
    KEEP(*(.param))
    KEEP(*(.param.*))
    _param_stop = .;
  }

  .log : ALIGN(8) SUBALIGN(8)
  {
    _log_start = .;
    KEEP(*(.log))
    KEEP(*(.log.*))
    _log_stop = .;
  }

  /* Phase 4.9: eventtrigger.c's _eventtrigger_start/_eventtrigger_stop --
   * same TOC-array shape as .param/.log above (EVENTTRIGGER(...)'s per-name
   * ".eventtrigger.NAME" input sections, walked via eventtrigger*
   * pointer-subtraction in eventtriggerGetId()/eventtriggerGetByName()),
   * same SUBALIGN(8) reasoning: struct eventtrigger_s is pointer-heavy
   * (name/payloadDesc/payload), so 8-byte-aligned and a multiple of 8 in
   * size on a 64-bit host -- SUBALIGN(8) is the largest value guaranteed to
   * divide it evenly, closing off the same x86-64-gcc-over-alignment gap
   * class 4.6/4.8 found for .log/.param before it can recur here. Unlike
   * .log/.param, every EVENTTRIGGER(...) invocation in this build lives in
   * one file (estimator.c) with a distinct NAME per section, so there's no
   * cross-file same-name collision needing --unique at an intermediate
   * Kbuild merge stage (see the Makefile's EXTRA_LDFLAGS comment) -- only
   * this final-link SUBALIGN is needed. */
  .eventtrigger : ALIGN(8) SUBALIGN(8)
  {
    _eventtrigger_start = .;
    KEEP(*(.eventtrigger))
    KEEP(*(.eventtrigger.*))
    _eventtrigger_stop = .;
  }

  .sim_toc_after_bss : ALIGN(4)
  {
    _sdata = .;
    _sidata = .;
    _stext = .;
  }
}
INSERT AFTER .bss;
