---
title: Out of tree build
page_id: oot
---

It is possible to have an out-of-tree build of parts of the crazyflie firmware. This enables developers to work on elements without worrrying about merging it with the full code base. 

# General out-of-tree build process
In a seperate folder make a Makefile which contain the following content:

```Makefile
CRAZYFLIE_BASE := [LOCATION OF THE CRAZYFLIE FIRMWARE]

#
# We override the default OOT_CONFIG here, we could also name our config
# to oot-config and that would be the default.
#
OOT_CONFIG := $(PWD)/config

include $(CRAZYFLIE_BASE)/tools/make/oot.mk
```

This will make the crazyflie-firmware build system look for a `Kbuild` file in your folder.

The following variables are understood by `oot.mk`:

| Variable      | Function                                            | Default                                               |
| --------      | --------------------------------------------------- | ----------------------------------------------------- |
| `OOT`         | Specify where your code (`Kbuild file`) is located. | `$(PWD)` (Your current directory)                     |
| `OOT_CONFIG`  | Location of your OOT specific `Kconfig` file, will be merged with the default config. | `$(OOT)/oot-config` |
| `EXTRA_CFLAGS`| Extra CFLAGS needed by your app                     | Empty.

And `oot.mk` also expects `$(CRAZYFLIE_BASE)` to be set to the path to the `crazyflie-firmware` repository.

The `Kbuild` file in the `$(OOT)` folder should point out your source files:

```Makefile
obj-y += your_estimator_out_of_tree.o
```

It can also add point out another folder where the code resides:

```Makefile
obj-y += src/
```

And since you are providing a config file by way of `$(OOT_CONFIG)` you do not need to run any make command to create a config like `make menuconfig` or `make defconfig`. Just a simple `make` will suffice.
# OOT estimators
The `config` file needs to enable ESTIMATOR_OOT, and can also set other config options:

```
CONFIG_ESTIMATOR_OOT=y
```

in [your_estimator_out_of_tree].c in the src folder you will just need to make sure that the following functions are filled:

* ```init = estimatorOutOfTreeInit```
* ```test = estimatorOutOfTreeTest```
* ```update = estimatorOutOfTree```
* ```name = "OOT```

# OOT Controllers

Not yet implemented. Please request this feature in the [crazyflie-firmware issue list](https://github.com/bitcraze/crazyflie-firmware/issues)

# App layer
Technically the app layer is an example of an out of tree build. Follow the [app layer instructions](/docs/userguides/app_layer.md) for this.
