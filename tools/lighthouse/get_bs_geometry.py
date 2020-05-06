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

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.mem import LighthouseBsGeometry
from cflib.crazyflie.mem import MemoryElement
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger

def read_sensors(scf):
    logs = {
        'lighthouse.angle0x':   [0, 0, 0],
        'lighthouse.angle0y':   [0, 0, 1],
        'lighthouse.angle0x_1': [0, 1, 0],
        'lighthouse.angle0y_1': [0, 1, 1],
        'lighthouse.angle0x_2': [0, 2, 0],
        'lighthouse.angle0y_2': [0, 2, 1],
        'lighthouse.angle0x_3': [0, 3, 0],
        'lighthouse.angle0y_3': [0, 3, 1],
        'lighthouse.angle1x':   [1, 0, 0],
        'lighthouse.angle1y':   [1, 0, 1],
        'lighthouse.angle1x_1': [1, 1, 0],
        'lighthouse.angle1y_1': [1, 1, 1],
        'lighthouse.angle1x_2': [1, 2, 0],
        'lighthouse.angle1y_2': [1, 2, 1],
        'lighthouse.angle1x_3': [1, 3, 0],
        'lighthouse.angle1y_3': [1, 3, 1],
    }

    storage = {}

    confs = [
        LogConfig(name='lh1', period_in_ms=10),
        LogConfig(name='lh2', period_in_ms=10),
        LogConfig(name='lh3', period_in_ms=10),
    ]

    # Set up log blocks
    count = 0
    floats_per_block = 6
    for name in logs.keys():
        confs[int(count / floats_per_block)].add_variable(name, 'float')
        count += 1

    # Read log data
    with SyncLogger(scf, confs) as logger:
        end_time = time.time() + 1.0
        for log_entry in logger:
            data = log_entry[1]
            for log_name in data:
                if not log_name in storage:
                    storage[log_name] = []
                storage[log_name].append(data[log_name])
            if time.time() > end_time:
                break

    # Average sensor data
    sensor_sweeps = np.zeros([2, 4, 2])
    for log_name, data in storage.items():
        sensor_info = logs[log_name]
        sensor_sweeps[sensor_info[0]][sensor_info[1]][sensor_info[2]] = np.average(data)

    return sensor_sweeps


# Create a hash from a vector with sensor ids
def hash_sensor_order(order):
    hash = 0
    for i in range(4):
        hash += order[i] * 4 ** i
    return hash

def estimate_yaw_to_base_station(sensor_sweeps):
    direction = {
        hash_sensor_order([2, 0, 1, 3]): math.radians(0),
        hash_sensor_order([2, 0, 3, 1]): math.radians(25),
        hash_sensor_order([2, 3, 0, 1]): math.radians(65),
        hash_sensor_order([3, 2, 0, 1]): math.radians(90),
        hash_sensor_order([3, 2, 1, 0]): math.radians(115),
        hash_sensor_order([3, 1, 2, 0]): math.radians(155),
        hash_sensor_order([1, 3, 2, 0]): math.radians(180),
        hash_sensor_order([1, 3, 0, 2]): math.radians(205),
        hash_sensor_order([1, 0, 3, 2]): math.radians(245),
        hash_sensor_order([0, 1, 3, 2]): math.radians(270),
        hash_sensor_order([0, 1, 2, 3]): math.radians(295),
        hash_sensor_order([0, 2, 1, 3]): math.radians(335),
    }

    # Assume bs is faicing slighly downwards and fairly horizontal
    # Sort sensors in the order they are hit by the horizontal sweep
    # and use the order to figure out roughly the direction to the
    # base station
    sweeps_x = {0: sensor_sweeps[0][0], 1: sensor_sweeps[1][0], 2: sensor_sweeps[2][0], 3: sensor_sweeps[3][0]}
    ordered_map = {k: v for k, v in sorted(sweeps_x.items(), key=lambda item: item[1])}
    sensor_order = list(ordered_map.keys())

    # The base station is roughly in this direction, in CF (world) coordinates
    return direction[hash_sensor_order(sensor_order)]


def generate_initial_estimate(bs_direction):
    # Base station height
    bs_h = 2
    # Distance to base station along the floor
    bs_fd = 3
    # Distance to base station
    bs_dist = math.sqrt(bs_h ** 2 + bs_fd ** 2)
    elevation = math.atan2(bs_h, bs_fd)

    # Initial position of the CF in camera coordinate system, open cv style
    tvec_start = np.array([0, 0, bs_dist])

    # Calculate rotation matrix
    d_c = math.cos(-bs_direction + math.pi)
    d_s = math.sin(-bs_direction + math.pi)
    R_rot_y = np.array([
        [d_c, 0.0, d_s],
        [0.0, 1.0, 0.0],
        [-d_s, 0.0, d_c],
    ])

    e_c = math.cos(elevation)
    e_s = math.sin(elevation)
    R_rot_x = np.array([
        [1.0, 0.0, 0.0],
        [0.0, e_c, -e_s],
        [0.0, e_s, e_c],
    ])

    R = np.dot(R_rot_x, R_rot_y)
    rvec_start, _ = cv.Rodrigues(R)

    return rvec_start, tvec_start


def calc_initial_estimate(sensor_sweeps):
    yaw = estimate_yaw_to_base_station(sensor_sweeps)
    return generate_initial_estimate(yaw)


def cam_to_world(rvec_c, tvec_c):
    R_c, _ = cv.Rodrigues(rvec_c)
    R_w = np.linalg.inv(R_c)
    tvec_w = -np.matmul(R_w, tvec_c)
    return R_w, tvec_w


def opencv_to_cf(R_cv, t_cv):
    R_opencv_to_cf = np.array([
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ])

    R_cf_to_opencv = np.array([
        [0.0, -1.0, 0.0],
        [0.0, 0.0, -1.0],
        [1.0, 0.0, 0.0],
    ])

    t_cf = np.dot(R_opencv_to_cf, t_cv)
    R_cf = np.dot(R_opencv_to_cf, np.dot(R_cv, R_cf_to_opencv))

    return R_cf, t_cf


def estimate_geometry(sensor_sweeps, rvec_start, tvec_start):
    sensor_distance_width = 0.015
    sensor_distance_length = 0.03

    # Sensor positions in world coordinates, open cv style
    lighthouse_3d = np.float32(
        [
            [-sensor_distance_width / 2, 0, -sensor_distance_length / 2],
            [sensor_distance_width / 2, 0, -sensor_distance_length / 2],
            [-sensor_distance_width / 2, 0, sensor_distance_length / 2],
            [sensor_distance_width / 2, 0, sensor_distance_length / 2]
        ])

    # Sensors as seen by the "camera"
    lighthouse_image_projection = np.float32(
        [
            [-math.tan(sensor_sweeps[0][0]), -math.tan(sensor_sweeps[0][1])],
            [-math.tan(sensor_sweeps[1][0]), -math.tan(sensor_sweeps[1][1])],
            [-math.tan(sensor_sweeps[2][0]), -math.tan(sensor_sweeps[2][1])],
            [-math.tan(sensor_sweeps[3][0]), -math.tan(sensor_sweeps[3][1])]
        ])

    # Camera matrix
    K = np.float64(
        [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0]
        ])

    dist_coef = np.zeros(4)

    _ret, rvec_est, tvec_est = cv.solvePnP(
        lighthouse_3d,
        lighthouse_image_projection,
        K,
        dist_coef,
        flags=cv.SOLVEPNP_ITERATIVE,
        rvec=rvec_start,
        tvec=tvec_start,
        useExtrinsicGuess=True)

    if not _ret:
        raise Exception("No solution found")

    Rw_ocv, Tw_ocv = cam_to_world(rvec_est, tvec_est)
    return Rw_ocv, Tw_ocv

def print_geo(rotation_cf, position_cf):
    print("{.origin = {", end='')
    for i in position_cf:
        print("{:0.6f}, ".format(i), end='')

    print("}, .mat = {", end='')

    for i in rotation_cf:
        print("{", end='')
        for j in i:
            print("{:0.6f}, ".format(j), end='')
        print("}, ", end='')

    print("}},")

class WriteMem:
    def __init__(self, scf, bs1, bs2):
        self.data_written = False

        mems = scf.cf.mem.get_mems(MemoryElement.TYPE_LH)

        count = len(mems)
        if count != 1:
            raise Exception('Unexpected nr of memories found:', count)

        mems[0].geometry_data = [bs1, bs2]
        mems[0].write_data(self._data_written)

        while not self.data_written:
            time.sleep(1)

    def _data_written(self, mem, addr):
        self.data_written = True
        print('Data written')


def upload_geo_data(scf, geometries):
    bs1 = LighthouseBsGeometry()
    bs1.rotation_matrix = geometries[0][0]
    bs1.origin = geometries[0][1]

    bs2 = LighthouseBsGeometry()
    bs2.rotation_matrix = geometries[1][0]
    bs2.origin = geometries[1][1]

    WriteMem(scf, bs1, bs2)


def sanity_check(position_cf):
    max_pos = 10.0
    for coord in position_cf:
        if (abs(coord) > max_pos):
            return False
    return True

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
    sensor_sweeps_all = read_sensors(scf)
    print("Estimating position of base stations...")

    geometries = []
    for bs in range(2):
        sensor_sweeps = sensor_sweeps_all[bs]
        rvec_start, tvec_start = calc_initial_estimate(sensor_sweeps)
        geometry = estimate_geometry(sensor_sweeps, rvec_start, tvec_start)
        rotation_cf, position_cf = opencv_to_cf(geometry[0], geometry[1])

        if not sanity_check(position_cf):
            position_cf = [0, 0, 0]
            rotation_cf = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
            print("Could not find valid solution for base station ", bs)

        print_geo(rotation_cf, position_cf)
        geometries.append([rotation_cf, position_cf])

    if args.write:
        print("Uploading geo data")
        upload_geo_data(scf, geometries)
