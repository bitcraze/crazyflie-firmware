#!/usr/bin/env python3

#  ,---------,       ____  _ __
#  |  ,-^-,  |      / __ )(_) /_______________ _____  ___
#  | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#     +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Crazyflie control firmware
#
#  Copyright (C) 2020 Bitcraze AB
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, in version 3.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program. If not, see <http://www.gnu.org/licenses/>.
#
#
#  Get calibration data from a lighthouse 2 base station.
#
#  This script connects to a lighthouse base station and
#  extracts the calibration parameters. The base station is a serial device
#  when connected via USB.
#
#  Usage:
#  1. Connect a USB cable to a lighthouse 2 base station
#  2. Run the script, use the serial port (usually something like
#     /dev/tty.usb123456) together with the --dev switch.
#     >>> get_lh2_clib_data.py --dev /dev/tty.usb123456
#  3. Copy/paste the output into the appropriate app or other file.
#

import argparse
import io
import serial
import sys

parser = argparse.ArgumentParser()
parser.add_argument('--dev', help='serial device to use when connecting to the base station.')
args = parser.parse_args()
if args.dev:
    dev = args.dev
else:
    print("Error: --dev required")
    sys.exit(1)

print ('Connecting to ' + dev)

ser = serial.Serial(dev, timeout=0.4)
sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser))

# Request fcal parameters
sio.write("\r\nparam list fcal\r\n")
sio.flush()
lines = sio.readlines()
ser.close()

# Parse data
data = {'0': {}, '1': {}}
for line in lines:
    if line.startswith('fcal.'):
        parts = line.split()
        subs = parts[0].split('.')

        sweep = subs[1]
        param = subs[2]
        value = parts[1]

        data[sweep][param] = value

print('C-format')
print('    .sweep = {')
for sweep, params in data.items():
    print('      {', end='')
    for param, value in params.items():
        print('.' + param + ' = ' + value + ', ', end='')
    print('},')
print('    },')
print('    .valid = true,')
print()
print('python-format')
print('calib = LighthouseBsCalibration()')
for sweep, params in data.items():
    for param, value in params.items():
        print('calib.sweeps[{}].{} = {}'.format(sweep, param, value))
print('calib.valid = True')
