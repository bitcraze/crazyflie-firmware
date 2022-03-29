"""Compiles the cffirmware C extension."""

import distutils.command.build
from distutils.core import setup, Extension
import os

fw_dir = "."
include = [
    os.path.join(fw_dir, "src/modules/interface"),
    os.path.join(fw_dir, "src/hal/interface"),
    os.path.join(fw_dir, "src/utils/interface/lighthouse"),
]

modules = [
    "pptraj.c",
    "pptraj_compressed.c",
    "planner.c",
    "collision_avoidance.c"
]
fw_sources = [os.path.join(fw_dir, "src/modules/src", mod) for mod in modules]

cffirmware = Extension(
    "_cffirmware",
    include_dirs=include,
    sources=fw_sources + ["bin/cffirmware_wrap.c"],
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
        self.build_base = "bin"

setup(
    name="cffirmware",
    version="1.0",
    cmdclass={"build": BuildCommand},
    ext_modules=[cffirmware]
)
