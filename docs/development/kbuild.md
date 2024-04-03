---
title: Customize firmware with kbuild
page_id: kbuild
---

The Crazyflie firmware uses a version of the [*KBuild*](https://www.kernel.org/doc/html/latest/kbuild/index.html) build system. Similar to what the [Linux Kernel](https://www.kernel.org/) or the [Busybox project](https://busybox.net/) uses.

What to build is determined by a configuration file, which is named `.config` and resides in the root of the firmware repository. If you only type `make` in a fresh repository the build system will generate the default config. You can inspect it by going, for instance:

```bash
$ less .config
#
# Automatically generated file; DO NOT EDIT.
# Crazyflie Platform Configuration
#

#
# Build and compiler options
#
CONFIG_CROSS_COMPILE="arm-none-eabi-"
# CONFIG_DEBUG is not set

#
# Platform configuration
#
CONFIG_PLATFORM_CF2=y
# CONFIG_PLATFORM_TAG is not set

#
# Sensor configuration
#
CONFIG_SENSORS_MPU9250_LPS25H=y
CONFIG_SENSORS_BMI088_BMP3XX=y
[...]
```

If you want to customize your build you can use the menuconfig by typing:

```bash
$ make menuconfig
```

This will drop you into a terminal based user interface where you can configure and customize what will be included in the firmware.
To use this you need to have some extra packages installed on your system, the equivelent of the following Ubuntu packages:

```bash
$ sudo apt install build-essential libncurses5-dev
```

To get an idea of how `menuconfig` will look, please see the images below.

![Main menu of menuconfig](/docs/images/kbuild1.png)

![Configuring deck drivers](/docs/images/kbuild2.png)

### Platform specific options

Read more about platforms in the [platform section.](/docs/userguides/platform.md)

