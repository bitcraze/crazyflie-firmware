"""Compiles the cffirmware C extension."""

import distutils.command.build
from distutils.core import setup, Extension
import os

include = [
    "src/modules/interface",
    "src/hal/interface",
    "src/utils/interface/lighthouse",
    "src/utils/interface",
    "build/include/generated",
    "src/config",
    "src/drivers/interface",
    "src/platform/interface",
]

fw_sources = [
    "src/modules/src/pptraj.c",
    "src/modules/src/pptraj_compressed.c",
    "src/modules/src/planner.c",
    "src/modules/src/collision_avoidance.c",
    "src/modules/src/controller_pid.c",
    "src/modules/src/position_controller_pid.c",
    "src/modules/src/attitude_pid_controller.c",
    "src/modules/src/pid.c",
    "src/utils/src/filter.c",
    "src/utils/src/num.c",
    "src/modules/src/controller_mellinger.c",
    "src/modules/src/power_distribution_quadrotor.c",
    "src/modules/src/power_distribution_flapper.c",
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
    ext_modules=[cffirmware]
)
