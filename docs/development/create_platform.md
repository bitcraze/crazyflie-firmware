---
title: Creating Your Own Platform
page_id: create_platform
---

Creating your own platform can be a good way of saving work by ...

 * Tailoring a config that is specific to your product or project!
 * To make sure you have sensible default values for parameters
 * Or to make sure that the initialization is done correctly

If you contribute it to the main Bitcraze repository it is also a way to distribute your work!
So how do you do it? Follow these steps!

## Add your platform to Kconfig
To make it possible to select your platform for the build you need to add it to
a `Kconfig` file. You can find the platform selection in the root `Kconfig` file.

Right now the selction looks like:

```Kconfig
menu "Platform configuration"

choice
    prompt "Platform to build"
    default CONFIG_PLATFORM_CF2

config PLATFORM_CF2
    bool "Build for CF2"
    select SENSORS_BMI088_BMP3XX
    select SENSORS_MPU9250_LPS25H

config PLATFORM_BOLT
    bool "Build for Bolt"
    select SENSORS_BMI088_BMP3XX
    select SENSORS_BMI088_SPI

config PLATFORM_TAG
    bool "Build for the roadrunner"
    select SENSORS_BMI088_BMP3XX

endchoice
```

Let's add our own platform, `RINCEWIND`, which has the same sensors as the Bolt.

```Kconfig
menu "Platform configuration"

choice
    prompt "Platform to build"
    default CONFIG_PLATFORM_CF2

config PLATFORM_CF2
    bool "Build for CF2"
    select SENSORS_BMI088_BMP3XX
    select SENSORS_MPU9250_LPS25H

config PLATFORM_BOLT
    bool "Build for Bolt"
    select SENSORS_BMI088_BMP3XX
    select SENSORS_BMI088_SPI

config PLATFORM_TAG
    bool "Build for the roadrunner"
    select SENSORS_BMI088_BMP3XX

config PLATFORM_RINCEWIND
    bool "Build for the Rincewind platform"
    select SENSORS_BMI088_BMP3XX
    select SENSORS_BMI088_SPI

endchoice
```

This creates the `CONFIG_PLATFORM_RINCEWIND` variable, usable both from C code as a define (when you include `autoconf.h`) as well as an environment variable usable from the Makefiles and the Kbuild files.

## Add a init source file to Kbuild
We need to add a entry point for your platform. The way the build system determines which platform init file to include is found in the `src/platform/src/Kbuild` file:

```Makefile
obj-$(CONFIG_PLATFORM_BOLT) += platform_bolt.o
obj-$(CONFIG_PLATFORM_CF2) += platform_cf2.o
obj-$(CONFIG_PLATFORM_TAG) += platform_tag.o
obj-y += platform.o
obj-y += platform_stm32f4.o
obj-y += platform_utils.o
```

Let's add `RINCEWIND`:

```Makefile
obj-$(CONFIG_PLATFORM_BOLT) += platform_bolt.o
obj-$(CONFIG_PLATFORM_CF2) += platform_cf2.o
obj-$(CONFIG_PLATFORM_TAG) += platform_tag.o
obj-$(CONFIG_PLATFORM_RINCEWIND) += platform_rincewind.o
obj-y += platform.o
obj-y += platform_stm32f4.o
obj-y += platform_utils.o
```

We will base the `src/platform/src/platform_rincewind.c` file on the bolt:

```c
#define DEBUG_MODULE "PLATFORM"

#include <string.h>

#include "platform.h"
#include "exti.h"
#include "nvic.h"
#include "debug.h"

static platformConfig_t configs[] = {
#ifdef CONFIG_SENSORS_BMI088_SPI
  {
    .deviceType = "CB10",
    .deviceTypeName = "Rincewind",
    .sensorImplementation = SensorImplementation_bmi088_spi_bmp3xx,
    .physicalLayoutAntennasAreClose = false,
    .motorMap = motorMapBoltBrushless,
  }
#endif
};

const platformConfig_t* platformGetListOfConfigurations(int* nrOfConfigs) {
  *nrOfConfigs = sizeof(configs) / sizeof(platformConfig_t);
  return configs;
}

void platformInitHardware() {
  // Low level init: Clock and Interrupt controller
  nvicInit();

  // EXTI interrupts
  extiInit();
}

const char* platformConfigGetPlatformName() {
  return "Rincewind";
}

```

The `platformInitHardware()` and `platformGetListOfConfigurations()` functions is called by `platform.c` as part of general init of the device.


## Add platform default parameter values
Your platform need to define suitable default values to (persistent) parameters. To do this we will need to create a `src/platform/interface/platform_rincewind.h` and make sure that this gets included when we build for the `RINCEWIND` platform. This is done by adding to the `src/platform/interface/platform_defaults.h`:

```c
#pragma once

#define __INCLUDED_FROM_PLATFORM_DEFAULTS__

#ifdef CONFIG_PLATFORM_CF2
    #include "platform_defaults_cf2.h"
#endif
#ifdef CONFIG_PLATFORM_BOLT
    #include "platform_defaults_bolt.h"
#endif
#ifdef CONFIG_PLATFORM_TAG
    #include "platform_defaults_tag.h"
#endif
```

Becomes:


```c
#pragma once

#define __INCLUDED_FROM_PLATFORM_DEFAULTS__

#ifdef CONFIG_PLATFORM_CF2
    #include "platform_defaults_cf2.h"
#endif
#ifdef CONFIG_PLATFORM_BOLT
    #include "platform_defaults_bolt.h"
#endif
#ifdef CONFIG_PLATFORM_TAG
    #include "platform_defaults_tag.h"
#endif
#ifdef CONFIG_RINCEWIND
    #include "platform_defaults_bolt.h"
    #include "platform_defaults_rincewind.h"
#endif
```

We base it on the default parameters for the Bolt. To determine how to add content to `platform_defaults_rincewind.h` please check what is added to the existing platforms.

## Add a platform default config
To make it easier for people to build for `RINCEWIND` we can add a `defconfig` for it. This is done by adding a file in the `configs/` folder. Let us look at `bolt_defconfig`:

```Makefile
CONFIG_PLATFORM_BOLT=y

CONFIG_ESTIMATOR_AUTO_SELECT=y
CONFIG_CONTROLLER_AUTO_SELECT=y
```

Based on this a start of `rincewind_defconfig` could be:

```Makefile
CONFIG_PLATFORM_RINCEWIND=y

CONFIG_ESTIMATOR_AUTO_SELECT=y
CONFIG_CONTROLLER_AUTO_SELECT=y
```

Then `RINCEWIND` platform could be built by:

```Bash
$ make rincewind_defconfig
make[1]: Entering directory '/home/jonasdn/sandbox/kbuild-firmware/build'
  GEN     ./Makefile
#
# configuration written to .config
#
make[1]: Leaving directory '/home/jonasdn/sandbox/kbuild-firmware/build'
$ make -j 12
[...]
```

## Need a different power distribution?

Suppose our new platform is a car with 4 wheels, then we will need a new power distribution function, that is
the translation from roll, pitch and yaw to motor power. The default implementation can be found in
`src/modules/src/power_distribution_quadrotor.c` but now we need to write a new function that works for cars.

The first step is to add a car power distrbution setting to the configuration.
In the standard configuration, the quadrotor power distribution is the only option and it is used by all platforms.
Edit `src/modules/src/Kconfig` and
find the location where the power distribution is configured

```Makefile
choice
    prompt "Type of power distribution"
    default POWER_DISTRIBUTION_QUADROTOR

config POWER_DISTRIBUTION_QUADROTOR
    bool "Quadrotor power distribution"
    depends on PLATFORM_CF2 || PLATFORM_BOLT || PLATFORM_TAG
    help
        Power distribution function for quadrotors

endchoice
```
Now add a new car distribution setting and make it available for your platform.

```Makefile
choice
    prompt "Type of power distribution"
    default POWER_DISTRIBUTION_QUADROTOR

config POWER_DISTRIBUTION_QUADROTOR
    bool "Quadrotor power distribution"
    depends on PLATFORM_CF2 || PLATFORM_BOLT || PLATFORM_TAG
    help
        Power distribution function for quadrotors

config POWER_DISTRIBUTION_CAR
    bool "Car power distribution"
    depends on PLATFORM_RINCEWIND
    help
        Power distribution function for cars

endchoice
```

The next step is to add an implementation of the power distribution function. Copy
`power_distribution_quadrotor.c` into a new file, `power_distribution_car.c` and modify the
`powerDistribution()` function to fit your needs. Also modify the `powerDistributionCap()`, this function is responsible
for limiting the thrust to the valid range [0 - UINT16_MAX].

The final step is to add the c file to the build. Open `src/modules/src/Kbuild` and add your new file.
```Makefile
obj-$(CONFIG_POWER_DISTRIBUTION_CAR) += power_distribution_car.o
```

And you are done! You have created your own platform, good job!
