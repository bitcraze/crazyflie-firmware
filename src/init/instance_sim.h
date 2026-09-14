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
 * CLI port argument and bind-before-scheduler-start UDP socket setup.
 */
#ifndef INSTANCE_SIM_H_
#define INSTANCE_SIM_H_

#include <stdint.h>

/**
 * Parses the UDP port number from argv (default 50000 if omitted, or via
 * "--port <n>" / "-p <n>" / "--port=<n>"), then creates and binds a UDP
 * socket to it. Must be called before vTaskStartScheduler().
 *
 * On any failure (bad argument, port already in use, ...) prints an error
 * to stderr and terminates the process with a non-zero exit status --
 * this function does not return in that case.
 */
void instanceInit(int argc, char **argv);

/* The UDP port instanceInit() bound. */
uint16_t instanceGetPort(void);

/* The UDP socket instanceInit() bound, for Communication (Phase 3) to use. */
int instanceGetSocketFd(void);

#endif // INSTANCE_SIM_H_
