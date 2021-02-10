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
import math
import time
import numpy as np
import cv2 as cv
from threading import Semaphore
import sys

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.mem import LighthouseBsGeometry
from cflib.crazyflie.mem import MemoryElement
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.localization.lighthouse_bs_geo import LighthouseBsGeoEstimator
from cflib.localization.lighthouse_bs_vector import LighthouseBsVector


# Number of sample to accumulate for the geometry calculation
N_SAMPLE = 50


def read_sensors(scf: SyncCrazyflie):
    # Mutext released by the callback function when enough measurements have
    # been accumulated
    reading_finished = Semaphore(0)

    measurements = {}

    # Callback accumulating measurements
    def _loc_callback(pk):
        if pk.type != scf.cf.loc.LH_ANGLE_STREAM:
            return

        print(pk.data["basestation"], end='')
        sys.stdout.flush()

        if not pk.data["basestation"] in measurements:
            measurements[pk.data["basestation"]] = {
                "count": 1,
                "sensors": [
                    pk.data["x"],
                    pk.data["y"]
                ]
            }
        else:
            for axis, axis_name in enumerate(['x', 'y']):
                for sensor in range(4):
                    measurements[pk.data["basestation"]]["sensors"][axis][sensor] += \
                        pk.data[axis_name][sensor]

            measurements[pk.data["basestation"]]["count"] += 1

            if measurements[pk.data["basestation"]]["count"] > N_SAMPLE:
                print()
                reading_finished.release()

    # Setup callback function and start streaming
    scf.cf.loc.receivedLocationPacket.add_callback(_loc_callback)
    scf.cf.param.set_value("locSrv.enLhAngleStream", 1)

    # Wait for enough sample to be acquired
    reading_finished.acquire()

    # Stop streaming and disable callback function
    scf.cf.param.set_value("locSrv.enLhAngleStream", 0)
    scf.cf.loc.receivedLocationPacket.remove_callback(_loc_callback)

    # Average the sum of measurements and generate t
    for basestation in measurements:
        for axis, axis_name in enumerate(['x', 'y']):
            for sensor in range(4):
                measurements[basestation]["sensors"][axis][sensor] /= \
                    measurements[basestation]["count"]

    # Reorganize data to be used by the geometry functions
    sensor_vectors = {}
    for bs in measurements:
        sensor_vectors[bs] = ([None, None, None, None])
        for sensor in range(4):
            horiz_angle = measurements[bs]["sensors"][0][sensor]
            vertical_angle = measurements[bs]["sensors"][1][sensor]
            bs_vector = LighthouseBsVector(horiz_angle, vertical_angle)
            sensor_vectors[bs][sensor] = bs_vector

    return sensor_vectors

def print_geo(rotation_cf, position_cf, is_valid):
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


class WriteMem:
    def __init__(self, scf, bs, geo):
        mems = scf.cf.mem.get_mems(MemoryElement.TYPE_LH)

        count = len(mems)
        if count != 1:
            raise Exception('Unexpected nr of memories found:', count)

        self.data_written = False
        mems[0].write_geo_data(bs, geo, self._data_written,
                                write_failed_cb=self._write_failed)

        while not self.data_written:
            time.sleep(0.1)

    def _data_written(self, mem, addr):
        self.data_written = True
        print('Data written')

    def _write_failed(self, mem, addr):
        raise Exception("Write failed")


def upload_geo_data(scf, geometries):
    for bs, data in geometries.items():
        geo = LighthouseBsGeometry()
        geo.rotation_matrix = data[0]
        geo.origin = data[1]
        geo.valid = data[2]

        WriteMem(scf, bs, geo)


##################################################


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

cf = Crazyflie(rw_cache='./cache')
with SyncCrazyflie(uri, cf=cf) as scf:
    print("Reading sensor data...")
    sensor_vectors_all = read_sensors(scf)
    print("Estimating position of base stations...")

    geometries = {}
    estimator = LighthouseBsGeoEstimator()
    for bs in sorted(sensor_vectors_all.keys()):
        sensor_vectors = sensor_vectors_all[bs]
        print("Base station ", bs)

        rotation_cf, position_cf = estimator.estimate_geometry(sensor_vectors)

        is_valid = estimator.sanity_check_result(position_cf)
        if not is_valid:
            position_cf = np.array([0, 0, 0])
            rotation_cf = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
            print("Warning: could not find valid solution")

        print_geo(rotation_cf, position_cf, is_valid)
        geometries[bs] = [rotation_cf, position_cf, is_valid]
        print()

    if args.write:
        print("Uploading geo data")
        upload_geo_data(scf, geometries)
