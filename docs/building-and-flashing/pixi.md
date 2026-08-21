---
title: Building using pixi
page_id: pixi
---

The [pixi package manager](https://pixi.sh) can be used to build and flash the firmware without installing the ARM toolchain, Python or the Crazyflie client system-wide on your machine.
All that is needed is to have pixi installed, following the [pixi installation instructions](https://pixi.sh/latest/#installation).

> On Windows, pixi isn't supported for this project (the ARM toolchain and `ncurses` dependencies aren't built for `win-64`). Use WSL instead: [enable WSL](https://docs.microsoft.com/en-us/windows/wsl/install), then follow the instructions below from inside your WSL Ubuntu shell.

All commands below should be run from the root of the crazyflie-firmware project.

## Development shell

``` bash
pixi shell
```

installs the required dependencies into an isolated, project-local environment and opens a shell with them added to `PATH`. This is the recommended way to use pixi: once inside the shell, the `make` commands described in the [building and flashing](./build.md) instructions work unchanged, for example:

``` bash
$ make cf2_defconfig
$ make menuconfig
$ make -j$(nproc)
$ make cload
```

Run `exit` (or close the terminal) to leave the pixi shell.

Alternatively, individual commands can be run through pixi, without entering a shell, by prefixing them with `pixi run`, for example:

``` bash
pixi run make cf2_defconfig
```
