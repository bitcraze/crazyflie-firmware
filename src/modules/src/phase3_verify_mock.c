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
 *      count + u32 crc32), which this also matches. Retired in Phase 4.5 --
 *      the real log.c now owns this port (see logInit() in main_sim.c's
 *      systemLaunch()), unmodified, so the TOC now reports crtp.c's/mem.c's
 *      real log variables (and any others already linked in), not an empty
 *      one -- and the CMD_RESET_LOGGING ack comes from log.c's own
 *      logControlProcess() CONTROL_RESET case, not this mock.
 *   4. Param (2) ch.0: same TOC_INFO/V2 exchange as Log.
 *   5. Memory (4) ch.0 (CHAN_INFO): CMD_INFO_NBR -> reply nbr_of_mems=0
 *      (cflib/crazyflie/mem/__init__.py). Retired in Phase 4.3 -- the real
 *      mem.c/crtp_mem.c now own this port (see memInit()/crtpMemInit() in
 *      main_sim.c's systemLaunch()), unmodified, so nbr_of_mems is now 1
 *      (the always-registered memTester), matching real hardware with no
 *      decks attached exactly -- not the 0 this mock replied with.
 *   6. Param (2) ch.0: same TOC_INFO/V2 exchange as Log, empty TOC.
 *      Retired in Phase 4.4 -- the real param_logic.c/param_task.c now own
 *      this port (see paramInit() in main_sim.c's systemLaunch()),
 *      unmodified, so the TOC now reports mem.c's real memTst.resetW entry
 *      (and any others already linked in), not an empty one.
 *
 * Meant to be removed (or replaced wholesale) once Phase 4 adds the real
 * crtpservice.c wiring for its one remaining mocked piece (Link source).
 * platformservice.c's, mem.c's, param_logic.c's/param_task.c's, and log.c's
 * real wiring landed in Phase 4.2/4.3/4.4/4.5 -- see
 * platformCommandProcess()/versionCommandProcess() in platformservice.c,
 * memSettingsProcess() in crtp_mem.c, paramTOCProcess()/paramWriteProcess()
 * in param_logic.c, and logTOCProcess()/logControlProcess() in log.c
 * instead of this file.
 */

#include "phase3_verify_mock.h"

#include <string.h>

#include "crtp.h"

#define LINK_SOURCE_CHANNEL 1

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

void phase3VerifyMockInit(void)
{
  crtpRegisterPortCB(CRTP_PORT_LINK, linkSourceCB);
}
