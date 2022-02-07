# Make configuration for the Crazyflie 2 platform

PLATFORM_HELP_bolt = The Crazyflie Bolt platform, includes the Crazyflie Bolt
PLATFORM_NAME_bolt = Bolt platform

CPU=stm32f4

######### Sensors configuration ##########
CFLAGS += -DSENSOR_INCLUDED_MPU9250_LPS25H
PROJ_OBJ += sensors_mpu9250_lps25h.o

CFLAGS += -DSENSOR_INCLUDED_BMI088_BMP388
CFLAGS += -DSENSOR_INCLUDED_BMI088_SPI_BMP388
PROJ_OBJ += sensors_bmi088_bmp388.o sensors_bmi088_i2c.o sensors_bmi088_spi.o

######### Stabilizer configuration ##########
ESTIMATOR          ?= any
CONTROLLER         ?= Any # one of Any, PID, Mellinger, INDI
POWER_DISTRIBUTION ?= stock
