#!/usr/bin/env python3

#  ,---------,       ____  _ __
#  |  ,-^-,  |      / __ )(_) /_______________ _____  ___
#  | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#     +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Crazyflie control firmware
#
#  Copyright (C) 2023 Bitcraze AB
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
#  This script makes a rough estimate of the coverage in a lighthouse system
#
#  Geometry data is read from a system configuration file, usually created from the client.
#  An estimated coverage is computed and a 3D visualization of uncovered areas is displayed.

import argparse
from cflib.localization import LighthouseConfigFileManager
import numpy as np
from vispy import scene
from vispy.scene import LinePlot, TurntableCamera


MAX_BS_DIST = 6.0
HORIZ_ANG = np.radians(160 / 2)
VERT_ANG = np.radians(120 / 2)


class VoxelSpace:
    def __init__(self, room_x, room_y, room_z, resolution=5.0) -> None:
        # Resolution, voxels/m
        self._res = resolution

        # Room size in meters
        self._room_x = room_x
        self._room_y = room_y
        self._room_z = room_z

        # Voxels, x and z swapped
        self._voxels = np.zeros((self.m_to_vs(self._room_z) + 1, self.m_to_vs(self._room_y) + 1, self.m_to_vs(self._room_x) + 1))

    def m_to_vs(self, m):
        return int(self.m_to_vs_f(m))

    def m_to_vs_f(self, m):
        return m * self._res

    def vs_to_m(self, vs):
        return np.array((vs[2], vs[1], vs[0])) / self._res

    def pos_m_to_scale(self, pos):
        return self.m_to_vs_f(pos[0]), self.m_to_vs_f(pos[1]), self.m_to_vs_f(pos[2])

    def pos_m_to_vs(self, pos):
        return self.m_to_vs(pos[2]), self.m_to_vs(pos[1]), self.m_to_vs(pos[0])

    def add_vs(self, vs, val):
        if self._is_in_volume(vs):
            self._voxels[vs[0], vs[1], vs[2]] += val

    def get_value(self, vs):
        if self._is_in_volume(vs):
            return self._voxels[vs[0], vs[1], vs[2]]
        return None

    def set_value(self, vs, value):
        if self._is_in_volume(vs):
            self._voxels[vs[0], vs[1], vs[2]] = value

    def _is_in_volume(self, vs):
        z = vs[0]
        y = vs[1]
        x = vs[2]

        shape = self._voxels.shape

        return z >= 0 and z < shape[0] and y >= 0 and y < shape[1] and x >= 0 and x < shape[2]

    def add_pos_m(self, pos, val):
        vs = self.pos_m_to_vs(pos)
        self.add_vs(vs, val)

    def volume_data(self):
        return self._voxels

    def min_corner_m(self):
        return np.array((0.0, 0.0, 0.0))

    def max_corner_m(self):
        return np.array((self._room_x, self._room_y, self._room_z))

    def all_voxel_positions(self):
        result = []
        shape = self._voxels.shape
        for z in range(shape[0]):
            for y in range(shape[1]):
                for x in range(shape[2]):
                    vs = np.array((z, y, x))
                    result.append((self.vs_to_m(vs), vs))
        return result

    def get_ratio(self, min_req):
        count = 0.0
        shape = self._voxels.shape
        for z in range(shape[0]):
            for y in range(shape[1]):
                for x in range(shape[2]):
                    if self._voxels[z, y, x] >= min_req:
                        count += 1.0

        return count / (shape[0] * shape[1] * shape[2])


def line(context, p1, p2, color='black'):
    voxel_space = context[1]
    LinePlot([voxel_space.pos_m_to_scale(p1), voxel_space.pos_m_to_scale(p2)], color=color, marker_size=0.0, parent=context[0])


def line_rt(context, p1, p2, R, t, color='black'):
    line(context, np.dot(R, p1) + t, np.dot(R, p2) + t, color)


def box(context, p1, p2, color='black'):
    line(context, (p1[0], p1[1], p1[2]), (p1[0], p2[1], p1[2]), color)
    line(context, (p1[0], p2[1], p1[2]), (p2[0], p2[1], p1[2]), color)
    line(context, (p2[0], p2[1], p1[2]), (p2[0], p1[1], p1[2]), color)
    line(context, (p2[0], p1[1], p1[2]), (p1[0], p1[1], p1[2]), color)

    line(context, (p1[0], p1[1], p2[2]), (p1[0], p2[1], p2[2]), color)
    line(context, (p1[0], p2[1], p2[2]), (p2[0], p2[1], p2[2]), color)
    line(context, (p2[0], p2[1], p2[2]), (p2[0], p1[1], p2[2]), color)
    line(context, (p2[0], p1[1], p2[2]), (p1[0], p1[1], p2[2]), color)

    line(context, (p1[0], p1[1], p1[2]), (p1[0], p1[1], p2[2]), color)
    line(context, (p1[0], p2[1], p1[2]), (p1[0], p2[1], p2[2]), color)
    line(context, (p2[0], p2[1], p1[2]), (p2[0], p2[1], p2[2]), color)
    line(context, (p2[0], p1[1], p1[2]), (p2[0], p1[1], p2[2]), color)


def base_station_r(context, pos, R, show_bs):
    dist = MAX_BS_DIST
    horiz_ang_max = HORIZ_ANG
    vert_ang_max = VERT_ANG

    pos = np.array(pos)

    if show_bs:
        # Coordinate system
        line_rt(context, np.array((0.0, 0, 0)), np.array((1.0, 0, 0)), R, pos, color='red')
        line_rt(context, np.array((0.0, 0, 0)), np.array((0, 1.0, 0)), R, pos, color='green')
        line_rt(context, np.array((0.0, 0, 0)), np.array((0, 0, 1.0)), R, pos, color='blue')

    R_inv = np.transpose(R)

    voxel_space = context[1]
    for vox_pos, vs in voxel_space.all_voxel_positions():
        vox_pos_bs = np.dot(R_inv, vox_pos - pos)
        if vox_pos_bs[0] >= 0:
            ang_horiz = np.arctan2(vox_pos_bs[2], vox_pos_bs[0])
            ang_vert = np.arctan2(vox_pos_bs[1], vox_pos_bs[0])
            if np.abs(ang_horiz) < vert_ang_max and np.abs(ang_vert) < horiz_ang_max and np.linalg.norm(vox_pos_bs) <= dist:
                voxel_space.add_vs(vs, 1)


def get_space(geos):
    # Approximate the space required by using the base station positions and a point in front of each base station
    in_front = np.array((MAX_BS_DIST, 0.0, 0.0))

    points = []
    for _id, geo in geos.items():
        pos = geo.origin
        rot = geo.rotation_matrix
        points.append(pos)
        points.append(np.dot(rot, in_front) + pos)

    points_n = np.array(points)
    pos_min = np.min(points_n, axis=0)
    pos_max = np.max(points_n, axis=0)

    # Limit the room downwards at the assumed floor
    if pos_min[2] < 0.0:
        pos_min[2] = 0.0

    size = np.ceil(np.clip(pos_max - pos_min, 1.0, None))

    return size, pos_min


def populate_base_stations(context, geos, offset, show_bs):
    for _id, geo in geos.items():
        pos = geo.origin
        rot = geo.rotation_matrix
        base_station_r(context, pos - offset, rot, show_bs)


parser = argparse.ArgumentParser(description='Visualize coverage of a lighthouse system')
parser.add_argument('config_file', help='the file name of the configuration file to load')
parser.add_argument('-n', '--novisualize', action='store_true', help='Do not show the 3d visualization', default=False)
parser.add_argument('-b', '--basestation', action='store_true', help='Show base station coordinate systems', default=False)
args = parser.parse_args()

geos, _calibs, _sys_type = LighthouseConfigFileManager.read(args.config_file)
size, offset = get_space(geos)

canvas = scene.SceneCanvas(keys='interactive', size=(800, 500), show=True)
view = canvas.central_widget.add_view()
view.bgcolor = '#eeeeee'
camera = TurntableCamera(fov=10.0, distance=30.0, up='+z', center=(0.0, 3.0, 0.0))
view.camera = camera
parent = view.scene

print(f'Using room size: {size}, offset: {offset}')

voxel_space = VoxelSpace(size[0], size[1], size[2], resolution=1.0)
context = (parent, voxel_space)

populate_base_stations(context, geos, offset, args.basestation)

# Outline
box(context, voxel_space.min_corner_m(), voxel_space.max_corner_m())

print(f'Coverage 1 or more base stations: {100 * voxel_space.get_ratio(1)}%')
print(f'Coverage 2 or more base stations: {100 * voxel_space.get_ratio(2)}%')

# Render uncovered space (as opposed to covered space)
if not args.novisualize:
    for vox_pos, vs in voxel_space.all_voxel_positions():
        if voxel_space.get_value(vs) == 0:
            voxel_space.set_value(vs, 1)
        else:
            voxel_space.set_value(vs, 0)

    volume = scene.visuals.Volume(voxel_space.volume_data(), parent=parent, clim=(0, 3), threshold=0.225)
    volume.cmap = 'reds'
    canvas.app.run()
