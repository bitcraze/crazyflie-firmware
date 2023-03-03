---
title: App layer
page_id: app_layer
---

## Introduction

The app-layer is a set of functionality and APIs to allow user to add custom code to the Crazyflie.
This is still an experimental functionalities but the intention is to eventually provide documentation and APIs to easily extend the Crazyflie and implement autonomous capabilities.

## App entry-point

When compiling the Crazyflie with `CONFIG_APP_ENABLE=y` the firmware will call a function `void appMain()` from a task after the startup sequence has completed. This function should not return.

If you want more control, you can define a function `void appInit()`. `appInit()` will be called by the firmware during initialization, no task will be created and so `appMain()` will not be automatically called.
This function must return to allow the Crazyflie initialization sequence to continue.

The following `Makefile` and out-of-tree config can be used to build an app.
For more information see the documentation for [out-of-tree build](/docs/development/oot.md).

```Makefile
# The firmware uses the Kbuild build system. There are 'Kbuild' files in this
# example that outlays what needs to be built. (check src/Kbuild).
#
# The firmware is configured using options in Kconfig files, the
# values of these end up in the .config file in the firmware directory.
#
# By setting the OOT_CONFIG (it is '$(PWD)/oot-config' by default) environment
# variable you can provide a custom configuration. It is important that you
# enable the app-layer. See app-config in this directory for example.

#
# We want to execute the main Makefile for the firmware project,
# it will handle the build for us.
#
CRAZYFLIE_BASE := ../..

#
# We override the default OOT_CONFIG here, we could also name our config
# to oot-config and that would be the default.
#
OOT_CONFIG := $(PWD)/app-config

include $(CRAZYFLIE_BASE)/tools/make/oot.mk
```

Where `app-config` is:

```Kconfig
CONFIG_APP_ENABLE=y
CONFIG_APP_PRIORITY=0
CONFIG_APP_STACKSIZE=300
```

Your source file should be defined in a `Kbuild` file, like:

```Makefile
obj-y += your-app.o
```

You can look at the applications in the `examples/` folder of the firmware repository.

## Building the app layer

In order to build the app layer, go to the root folder of the app example and run:

```
make clean
make 
```

or with [the toolbelt](https://www.bitcraze.io/documentation/repository/toolbelt/master/), from the crazyflie-firmware root:

```
tb make_app examples/app_hello_world/ clean
tb make_app examples/app_hello_world/ -j8
```

Then flash the resulting bin on your crazyflie according to [the flashing instructions](/docs/building-and-flashing/build.md). Make sure to point to the right build binary.


## Internal log and param system

For the app-layer, it would be good to have access to log and/or parameter values and to set parameter values. This way, your app will be able to read out sensor data or to switch controller/estimator on air. To check out these functions, look at `src/modules/interface/log.h` or `.../param.h` for the internal access functions. There is also an example to be found in `/examples/app_internal_param_log/`.

Check which Logs and Params you can use by checking out the [log group and variable](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/api/logs/) and the [parameter group and variable documentation](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/api/params/).

## LED sequences

It is possible to run LED sequences from the app layer to control the four LEDs on the Crazyflie and provide runtime information to the user. See the `src/hal/interface/ledseq.h` file for more information.

## App channel: packet based communication between the Crazyflie and the Python lib

The Appchannel API allows to communicate using radio packets with an app.
The packets can contain anything of a size up to 31 bytes, the protocol is defined by the app.

For more information about the API see the header file `src/modules/interface/app_channel.h`.
An example of how to use the app channel is in `examples/app_appchannel_test/`

## Examples

In the [example folder](https://github.com/bitcraze/crazyflie-firmware/tree/master/examples) of the crazyflie-firmware repository, there are several examples showing how to use the app layer, including a simple hello world example.
