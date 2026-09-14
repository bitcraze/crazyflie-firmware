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
 * udplink_sim.c - see udplink_sim.h.
 */

#include "udplink_sim.h"

#include <errno.h>
#include <stdbool.h>
#include <string.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "crtp.h"
#include "instance_sim.h"

#define UDPLINK_RX_QUEUE_LENGTH 16
#define UDPLINK_RX_TASK_STACKSIZE (2 * configMINIMAL_STACK_SIZE)
/* tskIDLE_PRIORITY, not CRTP_RX_TASK_PRI: recvfrom() is a raw POSIX blocking
 * syscall, not a FreeRTOS-aware wait -- this port's scheduler has no way to
 * know this task is "blocked" while parked in it, so it looks perpetually
 * ready and would starve every strictly-lower-priority task forever (found
 * by hand: heartbeatTask silently stopped ticking once this task existed at
 * priority 2). Phase 0's drift spike hit the same class of task and sidesteps
 * it the same way -- see its main.c comment on taskBusy/taskBlockingIO
 * sharing the lowest priority so time-slicing still gives everyone a turn. */
#define UDPLINK_RX_TASK_PRI 0

static int fd = -1;
static struct sockaddr_in peerAddr;
static volatile bool peerKnown = false;

static xQueueHandle rxQueue;

static void udplinkRxTask(void *param);

static int udplinkSetEnable(bool enable)
{
  (void)enable;
  return 0;
}

static int udplinkReset(void)
{
  peerKnown = false;
  return 0;
}

static bool udplinkIsConnected(void)
{
  return peerKnown;
}

static int udplinkReceivePacket(CRTPPacket *p)
{
  return (xQueueReceive(rxQueue, p, portMAX_DELAY) == pdTRUE) ? 0 : -1;
}

static int udplinkSendPacket(CRTPPacket *p)
{
  if (!peerKnown) {
    return 0;
  }

  /* p->raw holds the header byte followed by p->size data bytes. */
  int dataSize = p->size + 1;
  int sent = sendto(fd, p->raw, dataSize, 0, (struct sockaddr *)&peerAddr, sizeof(peerAddr));
  return sent > 0;
}

static struct crtpLinkOperations udplinkOp = {
  .setEnable         = udplinkSetEnable,
  .sendPacket        = udplinkSendPacket,
  .receivePacket     = udplinkReceivePacket,
  .isConnected       = udplinkIsConnected,
  .reset             = udplinkReset,
};

static void udplinkRxTask(void *param)
{
  (void)param;
  CRTPPacket p;

  for (;;) {
    struct sockaddr_in from;
    socklen_t fromLen = sizeof(from);

    int n = recvfrom(fd, p.raw, sizeof(p.raw), 0, (struct sockaddr *)&from, &fromLen);
    if (n < 0) {
      /* EINTR from the POSIX port's SIGALRM tick -- retry, per Phase 0's
       * drift-spike findings. Any other error: nothing useful to do but
       * keep listening. */
      continue;
    }
    if (n < 1) {
      continue;
    }

    peerAddr = from;
    peerKnown = true;

    p.size = n - 1;
    xQueueSend(rxQueue, &p, 0);
  }
}

void udplinkInit(void)
{
  fd = instanceGetSocketFd();

  rxQueue = xQueueCreate(UDPLINK_RX_QUEUE_LENGTH, sizeof(CRTPPacket));

  xTaskCreate(udplinkRxTask, "UDPLINK_RX", UDPLINK_RX_TASK_STACKSIZE, NULL, UDPLINK_RX_TASK_PRI, NULL);
}

struct crtpLinkOperations *udplinkGetLink(void)
{
  return &udplinkOp;
}
