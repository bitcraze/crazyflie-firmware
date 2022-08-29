#!/usr/bin/env bash
#
# ,---------,       ____  _ __
# |  ,-^-,  |      / __ )(_) /_______________ _____  ___
# | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
# | / ,--'  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
# Copyright (C) 2022 Bitcraze AB
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, in version 3.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.

# This script is used to flash a firmware to all Crazyflies listed in the
# uris array.Keep in mind that the file must be at the same location as the
# firmware/app you want to flash.

COLOR_RESET='\033[0m'       # Text Reset
YELLOW='\033[0;33m'         # Yellow

# define array of uris
uris=(
    "radio://0/10/2M/E7E7E7E701"
    "radio://0/10/2M/E7E7E7E702"
    "radio://0/10/2M/E7E7E7E703"
    "radio://0/10/2M/E7E7E7E704"
    "radio://0/10/2M/E7E7E7E705"
    "radio://0/10/2M/E7E7E7E706"
    "radio://0/10/2M/E7E7E7E707"
    "radio://0/10/2M/E7E7E7E708"
    "radio://0/10/2M/E7E7E7E709"
)

for uri in "${uris[@]}" ; do
    printf "${YELLOW}Flashing CF $uri ${COLOR_RESET}\n"
    CLOAD_CMDS="-w ${uri}" make cload
done
