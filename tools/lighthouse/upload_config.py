#!/usr/bin/env python3

#  ,---------,       ____  _ __
#  |  ,-^-,  |      / __ )(_) /_______________ _____  ___
#  | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#     +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Crazyflie control firmware
#
#  Copyright (C) 2021 Bitcraze AB
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
#  Upload geometry and calibration data to the Crazyflie storage
#
#  This script reads geometry and calibration data from a system configuration
#  file, usually created from the client. The data is uploaded to a crazyflie and
#  written to the data to persistent memory to make it available after
#  re-boot.
#
#  This functionality can also be executed from the client, but this
#  script is handy when uploading configurations to a swarm for instance.

import argparse
import logging
from threading import Event

import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.localization import LighthouseConfigWriter


class WriteMem:
    def __init__(self, uri, file_name):
        self._event = Event()

        with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
            writer = LighthouseConfigWriter(scf.cf)
            writer.write_and_store_config_from_file(self._data_written, file_name)
            self._event.wait()

    def _data_written(self, sucess):
        print('Data written')
        self._event.set()


uri = 'radio://0/80'

parser = argparse.ArgumentParser(description='Uploads a lighthouse system conficuration to a Crazyflie')
parser.add_argument('config_file', help='the file name of the configuration file to load')
parser.add_argument('--uri', help='uri to use when connecting to the Crazyflie. Default: ' + uri)
args = parser.parse_args()
if args.uri:
    uri = args.uri

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)
cflib.crtp.init_drivers(enable_debug_driver=False)

WriteMem(uri, args.config_file)
