---
title: Simmyflie development
page_id: simmyflie_development
---

The Simmyflie is the Crazyflie firmware built to run as a program on a
computer (see the [Simmyflie user guide](/docs/userguides/simmyflie.md)). It
compiles as much of the normal firmware as possible and replaces only what
needs hardware. This page describes where its files live and the rules that
decide it, for anyone changing the firmware or the Simmyflie.

## Building and testing

```sh
make cf21bl_defconfig   # or another hardware configuration
make sim                # builds build/sim.elf
make unit_sim           # unit tests of the Simmyflie files
make unit_sim FILES=test/sim/drivers/test_motors.c
```

`make sim` configures `build/sim/` from `build/.config` with
`configs/sim_defconfig` on top, so `CONFIG_PLATFORM_SIM` is set there.
`make unit_sim` uses the same configuration. `make unit` does not run the
Simmyflie tests.

## Kinds of files

- **Replacement**: the Simmyflie implementation of a mainline interface,
  built instead of the mainline implementation (`pm`, `motors`, `sensors`,
  `platform`, `main`, ...).
- **Stub**: a Replacement with empty functions, for hardware the Simmyflie
  does not have (`syslink`, `usblink`, `cpxlink`, `vcp_esc_passthrough`,
  `adc`). Placed and named as a Replacement.
- **Sim-only component**: has no mainline counterpart: the physics engine,
  the physics engine manager, the beacon sender, the UDP link and instance
  management.
- **Shadow header**: a header with the same name as a mainline header, found
  first on the Simmyflie's include path, so mainline code that includes it
  compiles on the host unchanged.
- **Sim-only header**: the interface of a Sim-only component, or
  declarations shared between Simmyflie files.
- **Build and configuration**: Kconfig, defconfig, linker script.
- **Test**: unit tests of Simmyflie files.
- **Tool**: host programs (Simfield, Simviewer).
- **Documentation**: the user guide and this page.
- **Mainline edit**: a change to a mainline file. Not a file of its own, but
  governed by a rule below.

## Tree

```
src/sim/
  Kconfig
  shadow/        Shadow headers, first on -I
    config.h FreeRTOSConfig.h motors.h pm.h platform.h deck.h
    i2cdev.h watchdog.h stm32fxxx.h stm32f4xx.h stm32f10x.h
  interface/     Sim-only headers, last on -I
    physics_engine.h physics_engine_manager.h true_state.h
    beacon_sender.h udplink.h instance.h
  src/
    hal/ drivers/ platform/ init/ utils/ modules/ deck/
                 Replacements and Stubs, by mainline subsystem
    physics/     physics_engine_manager.c physics_engine_builtin.c
    host/        instance.c udplink.c beacon_sender.c
test/sim/        same subfolders as src/sim/src/, test_<name>.c
tools/test/gcc_sim.yml
tools/sim/       simfield.py simviewer.py
tools/make/sim/linker/sim_toc_sections.ld
configs/sim_defconfig
docs/userguides/simmyflie.md
docs/development/simmyflie/index.md
```

`src/Kbuild` builds `src/sim/src/` when `CONFIG_PLATFORM_SIM` is set.

## Rules

### 0. One sim tree

Every Simmyflie source and header lives under `src/sim/`, not next to the
mainline file it stands in for.

*Why:* it is one directory for a mainline reviewer to skip and for the
Simmyflie to own, and a single directory first on `-I` is what makes header
shadowing simple. The Simmyflie build is all or nothing, so keeping a file
next to its mainline counterpart would gain little. The subsystem folders
under `src/sim/src/` say where the mainline counterpart lives. The cost:
nothing next to `pm_stm32f4.c` says that a Simmyflie version exists.

### 1. Replacements and Stubs

They live in `src/sim/src/<subsystem>/`, where `<subsystem>` is the mainline
subsystem of the interface. The file is named after the interface header it
implements, not after the mainline `.c` it replaces: `pm.h` ->
`src/sim/src/hal/pm.c` (mainline: `pm_stm32f4.c`).

*Why:* the Simmyflie swaps the implementation of an interface, and the
header is that interface. Naming after the `.c` fails where there is none
(`adc.h`), where it is generated (`version.h`), or where there are several
(`sensors.c` plus `sensors_bosch.c`, `platform.c` plus `platform_cf2.c`).
Stubs are placed as Replacements, rather than removed by guarding their
callers, because guarding would need `#ifdef`s in mainline code, and a stub
may later get behaviour (an `adc` that reports the battery voltage) without
moving.

### 2. Sim-only components

They live in `src/sim/src/physics/` (what owns the true state) or
`src/sim/src/host/` (what connects the process to the host computer).

*Why:* the names say what is inside. Not `sim/` (`src/sim/src/sim/` would
use "sim" for both the whole tree and a part of it), not `simmyflie/` (the
name of the whole build).

### 3. Shadow headers

Shadow headers live in `src/sim/shadow/`:

- only where the mainline header cannot be compiled on the host;
- one file per shadowed header, holding the declarations itself, never a
  redirect to another file;
- it keeps the mainline header's contract: every mainline file that the
  Simmyflie compiles must compile unchanged against it and mean the same
  thing. Within that, it differs from the mainline header in three ways
  only:
  1. *Omit* declarations that no compiled file uses (`config.h` without the
     radio, startup-sound and battery-sag settings).
  2. *Make hardware-dependent definitions host-safe*: a value, macro or type
     that mainline defines in terms of the hardware gets a host definition
     with the same meaning (`MOTORS_TIM_BEEP_CLK_FREQ` as a literal,
     `MCU_ID_ADDRESS` pointing at a variable, an `SCB` whose `ICSR` reads 0:
     no exception is active). A hardware type in a function signature
     becomes opaque when no compiled file looks inside it
     (`motorsInit(const void **)` for `motorsInit(const MotorPerifDef **)`).
     This is link-compatible, since C links by symbol name only. The header
     may also declare what such a definition needs, and nothing else
     (`extern const uint32_t simMcuId[3]` for `MCU_ID_ADDRESS`).
  3. *Configure the host where mainline configures the target*: values that
     tune the platform, such as the FreeRTOS port, heap and tick
     (`FreeRTOSConfig.h`).

  Anything that changes behaviour belongs in the Replacement `.c`, and
  Simmyflie-only additions are excluded by rule 4.

*Why shadowing:* mainline files such as `stabilizer.c` and `health.c`
include `motors.h`, `pm.h` and `platform.h` unconditionally, and these pull
in the STM32 headers. Guarding the includes in place would touch many files,
some in control flow; a Shadow header leaves them unmodified. *Why only
where needed:* every shadowed declaration is a copy that can drift from
mainline, and a mainline signature change is not noticed by the Simmyflie
build until a file that uses it is compiled. *Why the contract:* the
Simmyflie implementation is then compiled against the same contract as the
mainline one, and the three kinds of difference are exactly what the host
forces. A new case is judged against the rule, not added to a list.

### 4. Sim-only declarations

They never go in a Shadow header. Used by one file: private in that `.c`.
Shared: in a Sim-only header in `src/sim/interface/`, without a `_sim`
suffix.

*Why:* a Shadow header that holds only mainline declarations can be checked
against mainline. `interface/` is last on `-I`, so if mainline later adds a
header with the same name, the mainline header wins and the Simmyflie build
fails loudly, instead of mainline code silently getting the Simmyflie
header. That is why it is not the same directory as `shadow/`.

### 5. Kbuild

Mainline Kbuild files list what the Simmyflie leaves out, inside
`ifneq ($(CONFIG_PLATFORM_SIM),y)`. Every Simmyflie source is listed under
`src/sim/src/`. A new mainline file is built into the Simmyflie by default.
`vendor/Kbuild` is the exception: its `CONFIG_PLATFORM_SIM` branch selects
the FreeRTOS Posix port and the few CMSIS DSP files the Simmyflie uses.

A directory that would build nothing for the Simmyflie is left out where
its parent recurses into it, since an empty directory fails its partial
link.

*Why:* the Simmyflie compiles as much of the normal firmware as possible,
and an exclude-list keeps that true as mainline grows. With an include-list,
a new self-registering module (a CRTP service, a param group) would be
silently missing from the Simmyflie. The mainline diff shows what the
Simmyflie leaves out, which is what a reviewer wants to see. The cost: a
mainline change can break the Simmyflie build, so CI should build it.

When adding a mainline file that cannot build on the host, add it to the
`ifneq` block of its Kbuild file, keeping the order of the `obj-y` lines:
the ARM build must stay identical, including the order of objects in each
partial link.

### 6. Mainline edits

- Portability fixes (correct on ARM and on a 64-bit host, no `#ifdef`) are
  allowed. Typical cases: `uintptr_t` instead of `uint32_t` for an address,
  a cast to match a `printf` format.
- A `CONFIG_PLATFORM_SIM` guard around an `#include` is not allowed; use a
  Shadow header or a portability fix instead. The one exception is the
  include of the Simmyflie's link in `comm.c` (below), whose header only
  exists in the Simmyflie build.
- `#ifdef CONFIG_PLATFORM_SIM` in code is a last resort, for a seam that
  mainline has no plug-in point for, and kept minimal. The current ones:
  - `comm.c`: the CRTP link is the UDP link instead of the radio, and the
    radio and localization service are not started.
  - `console.c`: console output is also written to the terminal.
  - `system.c`: no `wfi` instruction in the idle hook.
  - `platform_defaults.h`: the Simmyflie uses the Crazyflie 2.1 Brushless
    defaults.

*Why:* portability fixes are bugs that only went unnoticed because ARM has
32-bit pointers and `long`; they can be reviewed without knowing the
Simmyflie. Guarded includes always have an alternative.

### 7. Tests

Tests of Simmyflie files live in `test/sim/`, mirroring `src/sim/src/`, and
run with `make unit_sim`. It uses `tools/test/gcc_sim.yml`, with the
Simmyflie's include order and its own build directory
(`generated/test_sim/`), and the Simmyflie build's configuration in
`build/sim/`. `tools/test/gcc.yml` excludes `test/sim/`.

*Why:* the test harness compiles the `.c` that matches each included header
by searching one include list shared by all tests, and builds every test
into one flat directory. With Simmyflie files named like mainline files, a
Simmyflie test would otherwise compile the mainline file, and a
`test_platform.c` would clash with a mainline one. In `gcc_sim.yml` the
Simmyflie source folders come before the mainline ones, so a test that
includes `motors.h` builds `src/sim/src/drivers/motors.c`.

### 8. Tools

Host programs live in `tools/sim/`, following the `tools/<topic>/`
convention. Simfield launches Simmyflie processes. Simviewer listens to the
beacons, whose format is defined in `src/sim/interface/beacon_sender.h`, so
a format change and the viewer change go in the same commit.

### 9. Documentation

The user guide is `docs/userguides/simmyflie.md`. Developer pages are in
`docs/development/simmyflie/`.

### 10. Comments

Comments describe the code as it is and why. Reasoning about the file
layout belongs on this page rather than in every file header.
