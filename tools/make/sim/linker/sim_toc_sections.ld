/*
 * sim_toc_sections.ld - linker script additions for the Simmyflie.
 *
 * The ARM linker scripts (tools/make/F405/linker/) gather the per-group
 * ".param.NAME", ".log.NAME" and ".eventtrigger.NAME" input sections into
 * one output section each and define its bounds, _param_start/_param_stop
 * and so on. param_logic.c, log.c and eventtrigger.c walk these as arrays.
 * The host build uses the default host linker script; this file adds the
 * same sections to it with INSERT AFTER instead of replacing it.
 *
 * param_logic.c's paramGetDefault() also uses _sdata/_edata/_sidata and
 * _stext/_etext to find a parameter's default value in flash. A host
 * executable has no flash copy of .data. _sdata, _sidata and _stext are
 * placed after .bss, so the address range checks in paramGetDefault() never
 * match and every parameter reports a default value of 0. _edata and _etext
 * come from the default host script.
 */

SECTIONS
{
  /* The arrays are counted as (stop - start) / element size, so there must
   * be no padding between the groups from different files. x86-64 gcc may
   * align the arrays more than their element type needs, and ld would pad
   * to that alignment. SUBALIGN(8) overrides it: on a 64-bit host
   * struct param_s (40 bytes), struct log_s (24 bytes) and
   * struct eventtrigger_s are all multiples of 8 bytes, so with 8-byte
   * sub-alignment no padding is ever inserted. 16 would not divide 40 or
   * 24. Partial links before this one need -Wl,--unique (see src/Kbuild). */
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

  /* All EVENTTRIGGER()s are in one file (estimator.c), so these sections
   * do not need --unique in the partial links. */
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
