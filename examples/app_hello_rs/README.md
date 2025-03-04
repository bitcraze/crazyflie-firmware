# Rust Hello world App for Crazyflie 2.x

Rust version of the Hello-world app.

This folder shows a proof of concept of how to compile some Rust in the Crazyflie to call firmware functions.

This is a very minimal examples that can be used as a starting point to build better Rust integration for the Crazyflie fiwmware.

## Build dependencies

The prerequisite to build the rust app is to have a recent rust compiler (tested with 1.82.0) and the target for the Crazyflie CPU.
To install the target with [rustup](https://rustup.rs):
```
rustup target add thumbv7em-none-eabihf
```

## Architecture

The Crazyflie build system is based on KBuild and it had to be creatively bent to allow to link a static library.
This is done my renaming the Rust crate output static lib into a .o and make sure GCC copies the full lib content into the resulting firmware (See [Kbuild](Kbuild)).

The Rust project is setup to generate a C static library.
The Makefile calls `cargo build` to build the lib and copies the resulting lib into ```app.o```.

The Crazyflie firmware will call `appMain()` at startup.
This function is declared in rust as `pub extern "C"` to be callable from C.

The FreeRTOS delay function and Crazyflie console putchar function are manually declared `extern "C"` which allows Rust to call them.