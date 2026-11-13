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
 * udplink.c - see udplink.h.
 */

#include "udplink.h"

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
#include "instance.h"

#define UDPLINK_RX_QUEUE_LENGTH 16
#define UDPLINK_RX_TASK_STACKSIZE (2 * configMINIMAL_STACK_SIZE)
/* The lowest priority, not CRTP_RX_TASK_PRI: recvfrom() blocks in the host,
 * not in FreeRTOS, so the Posix port sees this task as always ready. At a
 * higher priority it would starve every task below it; at the lowest,
 * time-slicing still gives the other tasks at that priority their turn. */
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
      /* EINTR from the Posix port's SIGALRM tick: retry. Any other error:
       * nothing useful to do but keep listening. */
      continue;
    }
    if (n < 1) {
      continue;
    }

    /* Null packet (header 0xFF, no payload) is cflib's UdpDriver scan probe.
     * Answer it straight away, like the radio's auto-ack, so a scan finds us
     * without waiting for unrelated outgoing traffic. Handled here rather than
     * passed to CRTP so a scan neither takes over the peer of a connected
     * client nor reaches the firmware. */
    if (n == 1 && p.raw[0] == 0xFF) {
      sendto(fd, p.raw, 1, 0, (struct sockaddr *)&from, fromLen);
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
