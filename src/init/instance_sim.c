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
 * instance_sim.c - see instance_sim.h.
 *
 * The socket is bound here, before vTaskStartScheduler(), and kept open
 * (not close()d after a probe) so the port stays reserved for this
 * instance with no rebind race -- Communication (Phase 3) picks up the
 * same fd via instanceGetSocketFd() instead of binding its own.
 */

#include "instance_sim.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#define INSTANCE_DEFAULT_PORT 50000

static uint16_t instancePort = INSTANCE_DEFAULT_PORT;
static int instanceSocketFd = -1;

static uint16_t parsePort(const char *s)
{
  char *end;
  errno = 0;
  long value = strtol(s, &end, 10);
  if (errno != 0 || end == s || *end != '\0' || value <= 0 || value > 65535) {
    fprintf(stderr, "Simmyflie: invalid port '%s'\n", s);
    exit(1);
  }
  return (uint16_t)value;
}

static uint16_t parseArgs(int argc, char **argv)
{
  for (int i = 1; i < argc; i++) {
    if (strncmp(argv[i], "--port=", 7) == 0) {
      return parsePort(argv[i] + 7);
    }
    if ((strcmp(argv[i], "--port") == 0 || strcmp(argv[i], "-p") == 0)) {
      if (i + 1 >= argc) {
        fprintf(stderr, "Simmyflie: %s requires an argument\n", argv[i]);
        exit(1);
      }
      return parsePort(argv[i + 1]);
    }
    fprintf(stderr, "Simmyflie: unrecognized argument '%s'\n", argv[i]);
    exit(1);
  }
  return INSTANCE_DEFAULT_PORT;
}

void instanceInit(int argc, char **argv)
{
  instancePort = parseArgs(argc, argv);

  int fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) {
    fprintf(stderr, "Simmyflie: cannot create UDP socket: %s\n", strerror(errno));
    exit(1);
  }

  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(instancePort);

  if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    fprintf(stderr, "Simmyflie: failed to bind UDP port %u: %s\n",
            instancePort, strerror(errno));
    close(fd);
    exit(1);
  }

  instanceSocketFd = fd;
  printf("Simmyflie: bound UDP port %u\n", instancePort);
  fflush(stdout);
}

uint16_t instanceGetPort(void)
{
  return instancePort;
}

int instanceGetSocketFd(void)
{
  return instanceSocketFd;
}
