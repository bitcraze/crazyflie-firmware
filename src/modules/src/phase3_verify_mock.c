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
 * phase3_verify_mock.c - TEMPORARY Phase 3 verification scaffolding.
 *
 * NOT the real Firmware core (Phase 4) -- just enough for the real,
 * unmodified Python `cflib` (cfclient's library; also what `cfcli`, the
 * Rust CLI, was first tried against -- but crazyflie-link-rs turned out to
 * have no udp:// transport at all, only usb:///radio://, so cflib is the
 * actual UDP-capable "standard tool" here) to complete its Crazyflie.open_link()
 * connection sequence against Simmyflie over the Phase 3 UDP link, as an
 * alternative to a throwaway raw-byte script.
 *
 * cflib's cflib/crazyflie/__init__.py _start_connection_setup() chains,
 * unconditionally, before the `connected` callback fires:
 *   1. Link (15) ch.1 (linkSource): a "who are you" probe -- reply must
 *      start with "Bitcraze Crazyflie" or cflib gives up on protocol
 *      versioning (platformservice.py::_crt_service_callback). This is
 *      crtpservice.c's existing linkSource case, copied unchanged below.
 *   2. Platform (13) ch.1: GET_PROTOCOL_VERSION -> reply required
 *      (platformservice.py::_request_protocol_version). Retired in Phase
 *      4.2 -- the real platformservice.c now owns this port entirely (see
 *      platformserviceInit() in main_sim.c's systemLaunch()).
 *   3. Log (5) ch.1: CMD_RESET_LOGGING -> reply required first
 *      (log.py::refresh_toc()/_send_reset_packet()) -- cflib will not send
 *      the TOC_INFO request at all until this ack comes back. Then
 *      Log (5) ch.0: TOC_INFO -> reply required (toc.py::TocFetcher). An
 *      empty TOC (0 items) is fine -- no further per-item exchange needed.
 *      Protocol version 12 >= 4 makes cflib use the V2 wire format (u16
 *      count + u32 crc32), which this also matches.
 *   4. Param (2) ch.0: same TOC_INFO/V2 exchange as Log.
 *   5. Memory (4) ch.0 (CHAN_INFO): CMD_INFO_NBR -> reply nbr_of_mems=0
 *      (cflib/crazyflie/mem/__init__.py). Also directly satisfies
 *      requirements.md's "Port 4 Memory shall report zero memories/decks".
 *
 * Meant to be removed (or replaced wholesale) once Phase 4 adds the real
 * crtpservice.c/log.c/param.c/mem.c wiring. platformservice.c's real
 * wiring landed in Phase 4.2 -- see platformCommandProcess()/
 * versionCommandProcess() in platformservice.c instead of this file.
 */

#include "phase3_verify_mock.h"

#include <string.h>

#include "crtp.h"

#define TOC_CHANNEL       0
#define TOC_INFO          0x03

#define LOG_SETTINGS_CHANNEL 1
#define LOG_CMD_RESET_LOGGING 0x05

#define LINK_SOURCE_CHANNEL 1

#define MEM_CHAN_INFO     0
#define MEM_CMD_INFO_NBR  0x01

static void linkSourceCB(CRTPPacket *p)
{
  if (p->channel != LINK_SOURCE_CHANNEL) {
    return;
  }

  p->size = CRTP_MAX_DATA_SIZE;
  memset(p->data, 0, CRTP_MAX_DATA_SIZE);
  strcpy((char *)p->data, "Bitcraze Crazyflie");
  crtpSendPacketBlock(p);
}

static void emptyTocCB(CRTPPacket *p)
{
  if (p->channel != TOC_CHANNEL || p->data[0] != TOC_INFO) {
    return;
  }

  /* toc_len = 0 (data[1..2]), crc32 = 0 (data[3..6]) */
  memset(&p->data[1], 0, 6);
  p->size = 7;
  crtpSendPacketBlock(p);
}

static void logCB(CRTPPacket *p)
{
  if (p->channel == LOG_SETTINGS_CHANNEL && p->data[0] == LOG_CMD_RESET_LOGGING) {
    /* cflib's log.py refresh_toc() blocks on this ack (id, error_status=0)
     * before it will send the TOC_INFO request on channel 0 -- see
     * _new_packet_cb()'s CMD_RESET_LOGGING branch. */
    p->data[1] = 0; /* id, unused by cflib's check */
    p->data[2] = 0; /* error_status = 0 (success) */
    p->size = 3;
    crtpSendPacketBlock(p);
    return;
  }

  emptyTocCB(p);
}

static void memInfoCB(CRTPPacket *p)
{
  if (p->channel != MEM_CHAN_INFO || p->data[0] != MEM_CMD_INFO_NBR) {
    return;
  }

  p->data[1] = 0; /* nbr_of_mems = 0 */
  p->size = 2;
  crtpSendPacketBlock(p);
}

void phase3VerifyMockInit(void)
{
  crtpRegisterPortCB(CRTP_PORT_LINK, linkSourceCB);
  crtpRegisterPortCB(CRTP_PORT_LOG, logCB);
  crtpRegisterPortCB(CRTP_PORT_PARAM, emptyTocCB);
  crtpRegisterPortCB(CRTP_PORT_MEM, memInfoCB);
}
