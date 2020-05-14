#!/usr/bin/env python

import argparse
import subprocess

# Calls the size progeam and prints pretty memory usage

def check_output(*args):
    """A wrapper for subprocess.check_output() to handle differences in python 2 and 3.
    Returns a string.
    """
    result = subprocess.check_output(*args)

    if isinstance(result, bytes):
        return result.decode('utf-8')
    else:
        return result


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("size_app", help="path to the size program")
    parser.add_argument("source", help=".elf file to use")
    parser.add_argument("flash_size_k", help="size of the flash, in k bytes", type=int)
    parser.add_argument("ram_size_k", help="size of the RAM, in k bytes", type=int)
    parser.add_argument("ccm_size_k", help="size of the CCM, in k bytes", type=int)
    args = parser.parse_args()

    output = check_output([args.size_app, '-A', args.source])
    sizes = {}
    for line in output.splitlines():
        parts = line.split()
        if len(parts) == 3 and parts[0] != 'section':
            sizes[parts[0]] = int(parts[1])

    flash_available = args.flash_size_k * 1024
    flash_used = sizes['.text'] + sizes['.data'] + sizes['.ccmdata']
    flash_free = flash_available - flash_used
    flash_fill = 100 * flash_used / flash_available

    ram_available = args.ram_size_k * 1024
    ram_used = sizes['.bss'] + sizes['.data']
    ram_free = ram_available - ram_used
    ram_fill = 100 * ram_used / ram_available

    ccm_available = args.ccm_size_k * 1024
    ccm_used = sizes['.ccmbss'] + sizes['.ccmdata']
    ccm_free = ccm_available - ccm_used
    ccm_fill = 100 * ccm_used / ccm_available

    print("Flash | {:7d}/{:<7d} ({:2.0f}%), {:7d} free | text: {}, data: {}, ccmdata: {}".format(flash_used, flash_available, flash_fill, flash_free, sizes['.text'], sizes['.data'], sizes['.ccmdata']))
    print("RAM   | {:7d}/{:<7d} ({:2.0f}%), {:7d} free | bss: {}, data: {}".format(ram_used, ram_available, ram_fill, ram_free, sizes['.bss'], sizes['.data']))
    print("CCM   | {:7d}/{:<7d} ({:2.0f}%), {:7d} free | ccmbss: {}, ccmdata: {}".format(ccm_used, ccm_available, ccm_fill, ccm_free, sizes['.ccmbss'], sizes['.ccmdata']))
