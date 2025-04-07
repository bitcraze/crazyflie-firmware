---
title: Building using nix
page_id: nix
---

The [Nix package manager](https://nixos.org) can be used to build the firmware in a reliable way without having to install all the dependencies on your system.
All that is needed is to have the nix package manager installed with [Flake support enabled](https://nixos.wiki/wiki/flakes).
The easiest if nix is not installed yet is to install it from [zero to Nix](https://zero-to-nix.com/start/install/).
Nix support in this project is currently experimental.

## Development shell

To get into a development shell where required tools for building the project are present:

``` bash
nix develop
```

## Building with Nix

The following command will produce a clean build for the Crazyflie 2. The command can be changed to build for other target:

``` bash
nix develop -i --command bash -c 'make mrproper && make cf2_defconfig && make -j`nproc`'
```

## Updating the flake

One great advantage of working with Nix is that the versions are locked in the `nix.lock` file. This means that build are reproducable. Running `nix flake update` can be used to update dependencies. The file `nix.lock` then need to be commited and pushed.