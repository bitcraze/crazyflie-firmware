# Make configuration for Crazyflie 2 family of platform

PLATFORM_HELP_cf2 = Crazyflie2 family, includes Crazyflie 2.0 and Crazyflie 2.1
PLATFORM_NAME_cf2 = CF2 family

CPU=stm32f4

######### Stabilizer configuration ##########
ESTIMATOR          ?= any
CONTROLLER         ?= Any # one of Any, PID, Mellinger
POWER_DISTRIBUTION ?= stock
SENSORS 		   ?= cf2
