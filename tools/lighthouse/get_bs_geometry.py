#!/usr/bin/env python3

#  ,---------,       ____  _ __
#  |  ,-^-,  |      / __ )(_) /_______________ _____  ___
#  | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#     +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Crazyflie control firmware
#
#  Copyright (C) 2020 - 2021 Bitcraze AB
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
#  Calculate Lighthouse base station geometry based on data from the
#  Crayzyflie. Requires a lighthouse deck.
#
#  This script connects to the Crazyflie and reads the sweep angles
#  for the base station(s) and calculates their position and orientation in
#  a coordinate system with origin at the position of the Crazyflie.
#
#  Usage:
#  1. Place the Crazyflie in the origin of your coordinate system, facing
#     positive X.
#  2. Make sure the Lighthouse deck is mounted completely parallell with the
#     ground (Crazyflie PCB) since this is what is going to define the
#     coordiate system.
#  3. Run the script
#  4. Copy/paste the output into lighthouse.c, recompile and flash the Crazyflie.
#

import argparse
import logging
import numpy as np
from threading import Event

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.mem import LighthouseBsGeometry
from cflib.crazyflie.mem import LighthouseMemHelper
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.localization import LighthouseBsGeoEstimator
from cflib.localization import LighthouseSweepAngleAverageReader


class Estimator:
    def __init__(self):
        self.sensor_vectors_all = None
        self.collection_event = Event()
        self.write_event = Event()

    def angles_collected_cb(self, angles):
        self.sensor_vectors_all = angles
        self.collection_event.set()

    def write_done_cb(self, success):
        if not success:
            print("Write to CF failed!")
        self.write_event.set()

    def estimate(self, uri, do_write):
        cf = Crazyflie(rw_cache='./cache')
        with SyncCrazyflie(uri, cf=cf) as scf:
            print("Reading sensor data...")
            sweep_angle_reader = LighthouseSweepAngleAverageReader(scf.cf, self.angles_collected_cb)
            sweep_angle_reader.start_angle_collection()
            self.collection_event.wait()

            print("Estimating position of base stations...")
            geometries = {}
            estimator = LighthouseBsGeoEstimator()
            for id in sorted(self.sensor_vectors_all.keys()):
                average_data = self.sensor_vectors_all[id]
                sensor_data = average_data[1]
                rotation_bs_matrix, position_bs_vector = estimator.estimate_geometry(sensor_data)
                is_valid = estimator.sanity_check_result(position_bs_vector)
                if is_valid:
                    geo = LighthouseBsGeometry()
                    geo.rotation_matrix = rotation_bs_matrix
                    geo.origin = position_bs_vector
                    geo.valid = True

                    geometries[id] = geo

                    self.print_geo(rotation_bs_matrix, position_bs_vector, is_valid)
                else:
                    print("Warning: could not find valid solution for " + id + 1)

                print()

            if do_write:
                print("Uploading geo data to CF")
                helper = LighthouseMemHelper(scf.cf)
                helper.write_geos(geometries, self.write_done_cb)
                self.write_event.wait()

    def print_geo(self, rotation_cf, position_cf, is_valid):
        print('C-format')
        if is_valid:
            valid_c = 'true'
        else:
            valid_c = 'false'

        print('{.valid = ' + valid_c + ', .origin = {', end='')
        for i in position_cf:
            print("{:0.6f}, ".format(i), end='')

        print("}, .mat = {", end='')

        for i in rotation_cf:
            print("{", end='')
            for j in i:
                print("{:0.6f}, ".format(j), end='')
            print("}, ", end='')

        print("}},")

        print()
        print('python-format')
        print('geo = LighthouseBsGeometry()')
        print('geo.origin =', np.array2string(position_cf, separator=','))
        print('geo.rotation_matrix = [', end='')
        for row in rotation_cf:
            print(np.array2string(row, separator=','), end='')
            print(', ', end='')
        print(']')
        print('geo.valid =', is_valid)


parser = argparse.ArgumentParser()
uri = "radio://0/80/2M"
parser.add_argument("--uri", help="uri to use when connecting to the Crazyflie. Default: " + uri)
parser.add_argument("--write", help="upload the calculated geo data to the Crazflie", action="store_true")
args = parser.parse_args()
if args.uri:
    uri = args.uri

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)
cflib.crtp.init_drivers(enable_debug_driver=False)

estimator = Estimator()
estimator.estimate(uri, args.write)
