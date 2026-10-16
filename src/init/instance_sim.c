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
 *
 * The command line also gives the per-instance part of the physics engine
 * configuration (start pose, ground level). The rest comes from Kconfig, and
 * the Physics engine manager is started with it here, before the scheduler.
 *
 * The beacon socket is created and configured here too, so a bad --beacon or
 * a socket error fails at startup and not in a task. It sends to a host-local
 * multicast group: out of loopback, TTL 0 so beacons never leave the host,
 * looped back so receivers on this host get them, and non-blocking.
 */

#include "instance_sim.h"

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "autoconf.h"
#include "beacon_sender.h"
#include "physics_engine_manager.h"

#define INSTANCE_DEFAULT_PORT 50000
#define DEFAULT_GROUND_LEVEL 0.0f  // m
#define DEFAULT_BEACON_GROUP "239.255.50.0"
#define DEFAULT_BEACON_PORT 49999

#define STRINGIFY_(x) #x
#define STRINGIFY(x) STRINGIFY_(x)

#define RAD_PER_DEG (3.14159265358979f / 180.0f)

static uint16_t instancePort = INSTANCE_DEFAULT_PORT;
static int instanceSocketFd = -1;

// The per-instance part of the physics engine configuration, as given on the
// command line
typedef struct {
  bool hasStartX, hasStartY, hasStartYaw;
  float startX;       // m
  float startY;       // m
  float startYaw;     // degrees
  float groundLevel;  // m
  struct sockaddr_in beaconAddress;
} instanceArgs_t;

static void printHelp(const char *program)
{
  printf("Usage: %s [options]\n"
         "\n"
         "  --port <n>, -p <n>    UDP port (default %d)\n"
         "  --start-x <m>         Start position x, m (default %.3f)\n"
         "  --start-y <m>         Start position y, m (default %.3f)\n"
         "  --start-yaw <deg>     Start yaw, degrees (default %d)\n"
         "  --ground-level <m>    Ground level, m (default %.3f)\n"
         "  --beacon <group:port> Beacon address, an IPv4 multicast group and\n"
         "                        a UDP port (default %s:%d)\n"
         "  --help                Print this help and exit\n"
         "\n"
         "Every option with a value is also accepted as --name=value.\n",
         program, INSTANCE_DEFAULT_PORT,
         CONFIG_SIM_START_X_MM / 1000.0, CONFIG_SIM_START_Y_MM / 1000.0,
         CONFIG_SIM_START_YAW_DEG, (double)DEFAULT_GROUND_LEVEL,
         DEFAULT_BEACON_GROUP, DEFAULT_BEACON_PORT);
}

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

static float parseFloat(const char *name, const char *s)
{
  char *end;
  errno = 0;
  float value = strtof(s, &end);
  if (errno != 0 || end == s || *end != '\0' || !isfinite(value)) {
    fprintf(stderr, "Simmyflie: invalid value '%s' for %s\n", s, name);
    exit(1);
  }
  return value;
}

// "<group>:<port>", where group is an IPv4 multicast address (224.0.0.0/4)
static void parseBeaconAddress(const char *s, struct sockaddr_in *addr)
{
  char group[INET_ADDRSTRLEN];
  const char *colon = strrchr(s, ':');
  const size_t groupLength = colon != NULL ? (size_t)(colon - s) : 0;

  bool valid = colon != NULL && groupLength > 0 && groupLength < sizeof(group);
  struct in_addr groupAddr;
  long port = 0;
  if (valid) {
    memcpy(group, s, groupLength);
    group[groupLength] = '\0';
    valid = inet_pton(AF_INET, group, &groupAddr) == 1 && IN_MULTICAST(ntohl(groupAddr.s_addr));
  }
  if (valid) {
    char *end;
    errno = 0;
    port = strtol(colon + 1, &end, 10);
    valid = errno == 0 && end != colon + 1 && *end == '\0' && port > 0 && port <= 65535;
  }
  if (!valid) {
    fprintf(stderr, "Simmyflie: invalid beacon address '%s', expected <multicast group>:<port>,"
                    " e.g. %s:%d\n", s, DEFAULT_BEACON_GROUP, DEFAULT_BEACON_PORT);
    exit(1);
  }

  memset(addr, 0, sizeof(*addr));
  addr->sin_family = AF_INET;
  addr->sin_addr = groupAddr;
  addr->sin_port = htons((uint16_t)port);
}

// Reads every argument. An option's value is either the part after '=' or
// the next argument, so a negative value can follow the option.
static void parseArgs(int argc, char **argv, instanceArgs_t *args)
{
  for (int i = 1; i < argc; i++) {
    char name[32];
    const char *value = NULL;

    const char *equals = strchr(argv[i], '=');
    if (strncmp(argv[i], "--", 2) == 0 && equals != NULL) {
      size_t length = (size_t)(equals - argv[i]);
      if (length >= sizeof(name)) {
        length = sizeof(name) - 1;
      }
      memcpy(name, argv[i], length);
      name[length] = '\0';
      value = equals + 1;
    } else {
      snprintf(name, sizeof(name), "%s", argv[i]);
    }

    if (strcmp(name, "--help") == 0 && value == NULL) {
      printHelp(argv[0]);
      exit(0);
    }

    const bool known = strcmp(name, "--port") == 0 || strcmp(name, "-p") == 0 ||
                       strcmp(name, "--start-x") == 0 || strcmp(name, "--start-y") == 0 ||
                       strcmp(name, "--start-yaw") == 0 || strcmp(name, "--ground-level") == 0 ||
                       strcmp(name, "--beacon") == 0;
    if (!known) {
      fprintf(stderr, "Simmyflie: unrecognized argument '%s'\n", argv[i]);
      exit(1);
    }

    if (value == NULL) {
      if (i + 1 >= argc) {
        fprintf(stderr, "Simmyflie: %s requires an argument\n", name);
        exit(1);
      }
      value = argv[++i];
    }

    if (strcmp(name, "--port") == 0 || strcmp(name, "-p") == 0) {
      instancePort = parsePort(value);
    } else if (strcmp(name, "--start-x") == 0) {
      args->startX = parseFloat(name, value);
      args->hasStartX = true;
    } else if (strcmp(name, "--start-y") == 0) {
      args->startY = parseFloat(name, value);
      args->hasStartY = true;
    } else if (strcmp(name, "--start-yaw") == 0) {
      args->startYaw = parseFloat(name, value);
      args->hasStartYaw = true;
    } else if (strcmp(name, "--beacon") == 0) {
      parseBeaconAddress(value, &args->beaconAddress);
    } else {
      args->groundLevel = parseFloat(name, value);
    }
  }
}

// Command-line value where given, otherwise the Kconfig default; in SI units
static void fillEngineConfig(const instanceArgs_t *args, physicsEngineConfig_t *config)
{
  config->startPose.x = args->hasStartX ? args->startX : CONFIG_SIM_START_X_MM / 1000.0f;
  config->startPose.y = args->hasStartY ? args->startY : CONFIG_SIM_START_Y_MM / 1000.0f;
  config->startPose.yaw = (args->hasStartYaw ? args->startYaw : (float)CONFIG_SIM_START_YAW_DEG) * RAD_PER_DEG;
  config->groundLevel = args->groundLevel;
  config->crashSpeedLimit = CONFIG_SIM_CRASH_SPEED_LIMIT_MM_S / 1000.0f;
  config->crashAngleLimit = CONFIG_SIM_CRASH_ANGLE_LIMIT_DEG * RAD_PER_DEG;
}

// A socket for beacons: host-only multicast out of loopback, non-blocking
static int createBeaconSocket(void)
{
  int fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) {
    fprintf(stderr, "Simmyflie: cannot create beacon socket: %s\n", strerror(errno));
    exit(1);
  }

  struct in_addr loopback = {.s_addr = htonl(INADDR_LOOPBACK)};
  unsigned char ttl = 0;
  unsigned char loop = 1;
  const int flags = fcntl(fd, F_GETFL, 0);
  if (setsockopt(fd, IPPROTO_IP, IP_MULTICAST_IF, &loopback, sizeof(loopback)) < 0 ||
      setsockopt(fd, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(ttl)) < 0 ||
      setsockopt(fd, IPPROTO_IP, IP_MULTICAST_LOOP, &loop, sizeof(loop)) < 0 ||
      flags < 0 || fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
    fprintf(stderr, "Simmyflie: cannot configure beacon socket: %s\n", strerror(errno));
    close(fd);
    exit(1);
  }
  return fd;
}

void instanceInit(int argc, char **argv)
{
  instanceArgs_t args = {.groundLevel = DEFAULT_GROUND_LEVEL};
  parseBeaconAddress(DEFAULT_BEACON_GROUP ":" STRINGIFY(DEFAULT_BEACON_PORT), &args.beaconAddress);
  parseArgs(argc, argv, &args);

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

  const int beaconFd = createBeaconSocket();
  beaconSenderInit(beaconFd, &args.beaconAddress, instancePort);
  char beaconGroup[INET_ADDRSTRLEN];
  inet_ntop(AF_INET, &args.beaconAddress.sin_addr, beaconGroup, sizeof(beaconGroup));
  printf("Simmyflie: sending beacons to %s:%u\n", beaconGroup, ntohs(args.beaconAddress.sin_port));
  fflush(stdout);

  physicsEngineConfig_t engineConfig;
  fillEngineConfig(&args, &engineConfig);
  if (!physicsEngineManagerInit(&engineConfig)) {
    fprintf(stderr, "Simmyflie: the physics engine could not start\n");
    close(fd);
    exit(1);
  }
}

uint16_t instanceGetPort(void)
{
  return instancePort;
}

int instanceGetSocketFd(void)
{
  return instanceSocketFd;
}
