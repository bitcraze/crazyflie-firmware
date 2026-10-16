/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2026 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * instance_sim.h - Simmyflie instance/process management (CONFIG_PLATFORM_SIM):
 * command-line arguments, bind-before-scheduler-start UDP socket setup, and
 * the start of the Physics engine manager.
 */
#ifndef INSTANCE_SIM_H_
#define INSTANCE_SIM_H_

#include <stdint.h>

/**
 * Parses argv (see --help: --port/-p, --start-x, --start-y, --start-yaw,
 * --ground-level, each as "--name value" or "--name=value"), creates and
 * binds a UDP socket to the port (default 50000), then fills the physics
 * engine configuration from the arguments and the Kconfig defaults and
 * starts the Physics engine manager with it. Must be called before
 * vTaskStartScheduler().
 *
 * --help prints the arguments and exits with status 0. On any failure (bad
 * argument, port already in use, engine that cannot start) prints an error
 * to stderr and exits with status 1 -- this function does not return in
 * those cases.
 */
void instanceInit(int argc, char **argv);

/* The UDP port instanceInit() bound. */
uint16_t instanceGetPort(void);

/* The UDP socket instanceInit() bound, for Communication (Phase 3) to use. */
int instanceGetSocketFd(void);

#endif // INSTANCE_SIM_H_
