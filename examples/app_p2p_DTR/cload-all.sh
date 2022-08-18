#!/usr/bin/env bash
# Reset
COLOR_RESET='\033[0m'       # Text Reset
YELLOW='\033[0;33m'         # Yellow

make -j 12

printf "${YELLOW}Flashing CF 00${COLOR_RESET}\n"
CLOAD_CMDS="-w radio://0/20/2M/E7E7E7E700" make cload
printf "${YELLOW}Flashing CF 01${COLOR_RESET}\n"
CLOAD_CMDS="-w radio://0/20/2M/E7E7E7E701" make cload
printf "${YELLOW}Flashing CF 02${COLOR_RESET}\n"
CLOAD_CMDS="-w radio://0/20/2M/E7E7E7E702" make cload
printf "${YELLOW}Flashing CF 03${COLOR_RESET}\n"
CLOAD_CMDS="-w radio://0/20/2M/E7E7E7E703" make cload
# printf "${YELLOW}Flashing CF 04${COLOR_RESET}\n"
# CLOAD_CMDS="-w radio://0/20/2M/E7E7E7E704" make cload
# printf "${YELLOW}Flashing CF 05${COLOR_RESET}\n"
# CLOAD_CMDS="-w radio://0/20/2M/E7E7E7E705" make cload
# printf "${YELLOW}Flashing CF 06${COLOR_RESET}\n"
# CLOAD_CMDS="-w radio://0/20/2M/E7E7E7E706" make cload
# printf "${YELLOW}Flashing CF 07${COLOR_RESET}\n"
# CLOAD_CMDS="-w radio://0/20/2M/E7E7E7E707" make cload
# printf "${YELLOW}Flashing CF 08${COLOR_RESET}\n"
# CLOAD_CMDS="-w radio://0/20/2M/E7E7E7E708" make cload
# printf "${YELLOW}Flashing CF 09${COLOR_RESET}\n"
# CLOAD_CMDS="-w radio://0/20/2M/E7E7E7E709" make cload
