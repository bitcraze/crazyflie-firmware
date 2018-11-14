# Make configuration for Crazyflie 2 family of platform

PLATFORM_HELP_cf2 = Crazyflie2 family, includes Crazyflie 2.0 and Crazyflie 2.1
PLATFORM_NAME_cf2 = CF2 family

CPU=stm32f4

######### Sensors configuration ##########
CFLAGS += -DSENSOR_INCLUDED_BMI088_BMP388
PROJ_OBJ += sensors_bmi088_bmp388.o

CFLAGS += -DSENSOR_INCLUDED_MPU9250_LPS25H
PROJ_OBJ += sensors_mpu9250_lps25h.o

######### Stabilizer configuration ##########
ESTIMATOR          ?= any
CONTROLLER         ?= Any # one of Any, PID, Mellinger
POWER_DISTRIBUTION ?= stock
