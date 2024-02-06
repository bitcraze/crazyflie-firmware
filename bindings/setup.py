"""Compiles the cffirmware C extension."""

import distutils.command.build
from distutils.core import setup, Extension
import os

include = [
    "src/modules/interface",
    "src/modules/interface/controller",
    "src/modules/interface/kalman_core",
    "src/modules/interface/outlierfilter",
    "src/hal/interface",
    "src/utils/interface/lighthouse",
    "src/utils/interface",
    "build/include/generated",
    "src/config",
    "src/drivers/interface",
    "src/platform/interface",
    "vendor/CMSIS/CMSIS/DSP/Include",
    "vendor/CMSIS/CMSIS/Core/Include",
]

fw_sources = [
    'vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_add_f32.c',
    'vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_dot_prod_f32.c',
    'vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_scale_f32.c',
    'vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_sub_f32.c',
    'vendor/CMSIS/CMSIS/DSP/Source/CommonTables/arm_common_tables.c',
    'vendor/CMSIS/CMSIS/DSP/Source/FastMathFunctions/arm_cos_f32.c',
    'vendor/CMSIS/CMSIS/DSP/Source/FastMathFunctions/arm_sin_f32.c',
    'vendor/CMSIS/CMSIS/DSP/Source/StatisticsFunctions/arm_power_f32.c',
    'vendor/CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_f32.c',
    'vendor/CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_scale_f32.c',
    'vendor/CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_f32.c',
    "src/modules/src/pptraj.c",
    "src/modules/src/pptraj_compressed.c",
    "src/modules/src/planner.c",
    "src/modules/src/collision_avoidance.c",
    "src/modules/src/controller/controller_pid.c",
    "src/modules/src/controller/position_controller_pid.c",
    "src/modules/src/controller/attitude_pid_controller.c",
    "src/modules/src/controller/controller_mellinger.c",
    "src/modules/src/controller/controller_brescianini.c",
    "src/utils/src/pid.c",
    "src/utils/src/filter.c",
    "src/utils/src/num.c",
    "src/modules/src/power_distribution_quadrotor.c",
    # "src/modules/src/power_distribution_flapper.c",
    "src/modules/src/axis3fSubSampler.c",
    "src/modules/src/kalman_core/kalman_core.c",
    "src/modules/src/kalman_core/mm_tdoa.c",
    "src/modules/src/outlierfilter/outlierFilterTdoa.c",
]

cffirmware = Extension(
    "_cffirmware",
    include_dirs=include,
    sources=fw_sources + ["build/cffirmware_wrap.c"],
    extra_compile_args=[
        "-O3",
        # The following flags are also used for compiling the actual firmware
        "-fno-strict-aliasing",
        "-Wno-address-of-packed-member",
        "-DUNIT_TEST_MODE",
    ],
)

# Override build command to specify custom "build" directory
class BuildCommand(distutils.command.build.build):
    def initialize_options(self):
        distutils.command.build.build.initialize_options(self)
        self.build_base = "build"

setup(
    name="cffirmware",
    version="1.0",
    cmdclass={"build": BuildCommand},
    ext_modules=[cffirmware],
    py_modules=["cffirmware"],
    package_dir={"": "build"},
)
