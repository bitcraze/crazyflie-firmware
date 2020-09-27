#!/usr/bin/env python3

from subprocess import check_output
import sys

# Get the status of the submodules from git. If any of the submodules start
# with "-" then the module is not initalized. Issue a warning.

result = str(check_output(["git", "submodule", "status"]))
lines = result.split("\n")

initialized = True
for line in lines:
    if len(line) > 0 and line[0] == '-':
        initialized = False

if not initialized:
    print('ERROR: One or more of the git submodules are not initialized. Try')
    print('    git submodule init')
    print('    git submodule update')
    sys.exit(-1)
