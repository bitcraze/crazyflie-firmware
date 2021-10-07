"""Compiles the cffirmware C extension."""

from distutils.core import setup, Extension
import os

import numpy as np


fw_dir = "."
include = [
    os.path.join(fw_dir, "src/modules/interface"),
    os.path.join(fw_dir, "src/hal/interface"),
    os.path.join(fw_dir, "src/utils/interface/lighthouse"),
    np.get_include(),
]

modules = [
    "collision_avoidance.c",
    "planner.c",
    "pptraj.c",
    "pptraj_compressed.c",
]
fw_sources = [os.path.join(fw_dir, "src/modules/src", mod) for mod in modules]

cffirmware = Extension(
    "_cffirmware",
    include_dirs=include,
    sources=fw_sources + ["bin/cffirmware_wrap.c"],
    extra_compile_args=[
        "-O3",
        # One Lighthouse header uses ARM's half-precision float, but we don't
        # need it (for now) in bindings, so we simply pretend it's a uint16_t.
        "-D__fp16=uint16_t",
    ],
)

setup(name="cffirmware", version="1.0", ext_modules=[cffirmware])
