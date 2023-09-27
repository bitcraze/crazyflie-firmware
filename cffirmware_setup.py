#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
#  Distutils setup file for Crazyflie Python firmware bindings
#
#  Automatically copied to build/ directory when you run
#
#      make bindings_python
#
#  Then you can cd to build/ and do
#
#      sudo python3 setup.py install
#
#  Copyright (C) 2023 Bitcraze AB and Simon D. Levy
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.

from distutils.core import setup

setup(
    name='cffirmware',
    version='0.1',
    packages=[''],
    package_dir={'': '.'},
    package_data={'': ['_cffirmware.cpython-38-x86_64-linux-gnu.so']},
)
