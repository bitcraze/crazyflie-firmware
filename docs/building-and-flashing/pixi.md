---
title: Building using pixi
page_id: pixi
---

The [pixi package manager](https://pixi.sh) can be used to build and flash the firmware without installing the ARM toolchain, Python or the Crazyflie client on your system.
All that is needed is to have pixi installed, following the [pixi installation instructions](https://pixi.sh/latest/#installation).

> On Windows, pixi isn't supported for this project (the ARM toolchain and `ncurses` dependencies aren't built for `win-64`). Use WSL instead: [enable WSL](https://docs.microsoft.com/en-us/windows/wsl/install), then follow the instructions below from inside your WSL Ubuntu shell.

## Development shell

To get into a development shell where required tools for building and flashing the project are present run:

``` bash
pixi shell
```

## Building with pixi

The following command will produce a clean build for the Crazyflie 2.1(+). The command can be changed to build for other targets:

``` bash
pixi run bash -c 'make cf2_defconfig && make -j$(nproc)'
```

Alternatively, once inside `pixi shell`, the normal `make` commands described in the [building instructions](./build.md#compiling) can be used directly.

### Advanced configuration (menuconfig)

The pixi environment also provides `ncurses`, so `make menuconfig` (see the [kbuild instructions](/docs/development/kbuild.md)) works out of the box, either from inside `pixi shell` or via `pixi run make menuconfig`.

## Flashing with pixi

The pixi environment also provides `cfclient`/`cfloader`, so the [flashing instructions](./build.md#flashing) work unchanged, either from inside `pixi shell` or by prefixing commands with `pixi run`, for example:

``` bash
pixi run make cload
```
