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
 * test_beacon_sender.c - unit tests of the Beacon sender of the Simmyflie.
 *
 * Beacons are sent to a unicast UDP socket on 127.0.0.1 that the tests own,
 * so what is on the wire is checked byte by byte.
 */

// File under test
#include "beacon_sender.h"

#include <errno.h>
#include <string.h>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include "unity.h"

#define UDP_PORT 50123

static int senderFd;
static int receiverFd;
static struct sockaddr_in receiverAddr;

// What every tick passes
static trueState_t tickState;
static physicsEngineStatus_t tickStatus;

// A state that differs from zero, and from field to field, in every field
static trueState_t someState(void)
{
  return (trueState_t){
    .position = {.x = 1.5f, .y = -2.25f, .z = 0.75f},
    .velocity = {.x = 9.0f, .y = 9.0f, .z = 9.0f},
    .acceleration = {.x = 9.0f, .y = 9.0f, .z = 9.0f},
    .attitude = {.w = 0.5f, .x = -0.5f, .y = 0.25f, .z = 0.625f},
    .angularRate = {.x = 9.0f, .y = 9.0f, .z = 9.0f},
    .rotorSpeeds = {1000.0f, 1100.0f, 1200.0f, 1300.0f},
  };
}

// The number of bytes received, 0 if nothing is waiting
static int receive(uint8_t *buffer, const size_t size)
{
  const ssize_t n = recv(receiverFd, buffer, size, MSG_DONTWAIT);
  if (n < 0) {
    TEST_ASSERT_TRUE_MESSAGE(errno == EAGAIN || errno == EWOULDBLOCK, strerror(errno));
    return 0;
  }
  return (int)n;
}

static int countReceived(void)
{
  uint8_t buffer[128];
  int count = 0;
  while (receive(buffer, sizeof(buffer)) > 0) {
    count++;
  }
  return count;
}

static void tick(const int nbrOfTicks, uint32_t *simTime)
{
  for (int i = 0; i < nbrOfTicks; i++) {
    (*simTime)++;
    beaconSenderTick(&tickState, tickStatus, *simTime);
  }
}

static uint16_t u16At(const uint8_t *b, const int offset)
{
  return (uint16_t)(b[offset] | (b[offset + 1] << 8));
}

static uint32_t u32At(const uint8_t *b, const int offset)
{
  return (uint32_t)b[offset] | ((uint32_t)b[offset + 1] << 8) | ((uint32_t)b[offset + 2] << 16) |
         ((uint32_t)b[offset + 3] << 24);
}

static float f32At(const uint8_t *b, const int offset)
{
  const uint32_t bits = u32At(b, offset);
  float value;
  memcpy(&value, &bits, sizeof(value));
  return value;
}

// Ticks until one beacon is sent, and receives it
static int sendOne(uint8_t *buffer, const size_t size, const uint32_t simTimeAtSend)
{
  for (int i = 1; i < BEACON_INTERVAL_TICKS; i++) {
    beaconSenderTick(&tickState, tickStatus, simTimeAtSend - BEACON_INTERVAL_TICKS + i);
  }
  beaconSenderTick(&tickState, tickStatus, simTimeAtSend);
  return receive(buffer, size);
}

void setUp(void)
{
  tickState = someState();
  tickStatus = physicsEngineStatusAtRest;

  receiverFd = socket(AF_INET, SOCK_DGRAM, 0);
  TEST_ASSERT_TRUE(receiverFd >= 0);
  memset(&receiverAddr, 0, sizeof(receiverAddr));
  receiverAddr.sin_family = AF_INET;
  receiverAddr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  receiverAddr.sin_port = 0;
  TEST_ASSERT_EQUAL_INT(0, bind(receiverFd, (struct sockaddr *)&receiverAddr, sizeof(receiverAddr)));
  socklen_t length = sizeof(receiverAddr);
  TEST_ASSERT_EQUAL_INT(0, getsockname(receiverFd, (struct sockaddr *)&receiverAddr, &length));

  senderFd = socket(AF_INET, SOCK_DGRAM, 0);
  TEST_ASSERT_TRUE(senderFd >= 0);
  beaconSenderInit(senderFd, &receiverAddr, UDP_PORT);
}

void tearDown(void)
{
  close(senderFd);
  close(receiverFd);
}

// Rate

void testThatNoBeaconIsSentBeforeTheTwentiethTick(void)
{
  // Fixture
  uint32_t simTime = 0;

  // Test
  tick(BEACON_INTERVAL_TICKS - 1, &simTime);

  // Assert
  TEST_ASSERT_EQUAL_INT(0, countReceived());
}

void testThatABeaconIsSentOnTheTwentiethTick(void)
{
  // Fixture
  uint32_t simTime = 0;

  // Test
  tick(BEACON_INTERVAL_TICKS, &simTime);

  // Assert
  TEST_ASSERT_EQUAL_INT(1, countReceived());
}

void testThatABeaconIsSentOnEveryTwentiethTick(void)
{
  // Fixture
  uint32_t simTime = 0;

  // Test
  tick(10 * BEACON_INTERVAL_TICKS + BEACON_INTERVAL_TICKS - 1, &simTime);

  // Assert
  TEST_ASSERT_EQUAL_INT(10, countReceived());
}

void testThatInitRestartsTheTickCount(void)
{
  // Fixture
  uint32_t simTime = 0;
  tick(BEACON_INTERVAL_TICKS - 1, &simTime);
  beaconSenderInit(senderFd, &receiverAddr, UDP_PORT);

  // Test
  tick(BEACON_INTERVAL_TICKS - 1, &simTime);

  // Assert
  TEST_ASSERT_EQUAL_INT(0, countReceived());
}

void testThatAFailedSendIsDroppedAndTheNextBeaconIsSent(void)
{
  // Fixture: a socket that cannot send
  uint32_t simTime = 0;
  beaconSenderInit(-1, &receiverAddr, UDP_PORT);
  tick(BEACON_INTERVAL_TICKS, &simTime);
  beaconSenderInit(senderFd, &receiverAddr, UDP_PORT);

  // Test
  tick(BEACON_INTERVAL_TICKS, &simTime);

  // Assert
  TEST_ASSERT_EQUAL_INT(1, countReceived());
}

// Wire format

void testThatABeaconIsFiftyTwoBytesOfVersionOne(void)
{
  // Fixture
  uint8_t buffer[128];

  // Test
  const int size = sendOne(buffer, sizeof(buffer), 1000);

  // Assert
  TEST_ASSERT_EQUAL_INT(52, size);
  TEST_ASSERT_EQUAL_UINT8(1, buffer[0]);
}

void testThatABeaconHoldsTheUdpPortAndSimTime(void)
{
  // Fixture
  uint8_t buffer[128];

  // Test
  sendOne(buffer, sizeof(buffer), 0x12345678);

  // Assert
  TEST_ASSERT_EQUAL_UINT16(UDP_PORT, u16At(buffer, 2));
  TEST_ASSERT_EQUAL_HEX32(0x12345678, u32At(buffer, 4));
}

void testThatABeaconHoldsThePositionAttitudeAndRotorSpeedsOfTheState(void)
{
  // Fixture
  uint8_t buffer[128];
  const trueState_t s = someState();

  // Test
  sendOne(buffer, sizeof(buffer), 1000);

  // Assert
  TEST_ASSERT_EQUAL_FLOAT(s.position.x, f32At(buffer, 8));
  TEST_ASSERT_EQUAL_FLOAT(s.position.y, f32At(buffer, 12));
  TEST_ASSERT_EQUAL_FLOAT(s.position.z, f32At(buffer, 16));
  TEST_ASSERT_EQUAL_FLOAT(s.attitude.w, f32At(buffer, 20));
  TEST_ASSERT_EQUAL_FLOAT(s.attitude.x, f32At(buffer, 24));
  TEST_ASSERT_EQUAL_FLOAT(s.attitude.y, f32At(buffer, 28));
  TEST_ASSERT_EQUAL_FLOAT(s.attitude.z, f32At(buffer, 32));
  TEST_ASSERT_EQUAL_FLOAT(s.rotorSpeeds[0], f32At(buffer, 36));
  TEST_ASSERT_EQUAL_FLOAT(s.rotorSpeeds[1], f32At(buffer, 40));
  TEST_ASSERT_EQUAL_FLOAT(s.rotorSpeeds[2], f32At(buffer, 44));
  TEST_ASSERT_EQUAL_FLOAT(s.rotorSpeeds[3], f32At(buffer, 48));
}

void testThatABeaconHoldsTheStateGivenInTheTickItIsSent(void)
{
  // Fixture
  uint8_t buffer[128];
  uint32_t simTime = 0;
  tick(BEACON_INTERVAL_TICKS - 1, &simTime);
  tickState.position.x = 42.0f;

  // Test
  tick(1, &simTime);
  receive(buffer, sizeof(buffer));

  // Assert
  TEST_ASSERT_EQUAL_FLOAT(42.0f, f32At(buffer, 8));
}

void testThatTheStatusAtRestIsZero(void)
{
  // Fixture
  uint8_t buffer[128];
  tickStatus = physicsEngineStatusAtRest;

  // Test
  sendOne(buffer, sizeof(buffer), 1000);

  // Assert
  TEST_ASSERT_EQUAL_UINT8(0, buffer[1]);
}

void testThatTheStatusFlyingIsOne(void)
{
  // Fixture
  uint8_t buffer[128];
  tickStatus = physicsEngineStatusFlying;

  // Test
  sendOne(buffer, sizeof(buffer), 1000);

  // Assert
  TEST_ASSERT_EQUAL_UINT8(1, buffer[1]);
}

void testThatTheStatusCrashedIsTwo(void)
{
  // Fixture
  uint8_t buffer[128];
  tickStatus = physicsEngineStatusCrashed;

  // Test
  sendOne(buffer, sizeof(buffer), 1000);

  // Assert
  TEST_ASSERT_EQUAL_UINT8(2, buffer[1]);
}
