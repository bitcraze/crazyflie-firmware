"""Compiles the cffirmware C extension."""

from distutils.core import setup, Extension
import os

fw_dir = "."
include = [
    os.path.join(fw_dir, "src/modules/interface"),
]

modules = [
    # list firmware c-files here
]
fw_sources = [os.path.join(fw_dir, "src/modules/src", mod) for mod in modules]

cffirmware = Extension(
    "_cffirmware",
    include_dirs=include,
    sources=fw_sources + ["bin/cffirmware_wrap.c"],
    extra_compile_args=[
        "-O3",
    ],
)

setup(name="cffirmware", version="1.0", ext_modules=[cffirmware])
