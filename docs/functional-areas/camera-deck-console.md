---
title: Camera Deck console
page_id: camera_deck_console
---

The Camera Deck sends startup and runtime console output to the Crazyflie over
UART. Clients can discover and enable it as the `deck:bcCam` source of the
[CRTP console](crtp/crtp_console.md). During normal startup, the source becomes
available after the Crazyflie passes its self-tests and starts the deck
service.

The `bcCam` driver enumerates the Camera Deck's Common Link service catalog
and binds a compatible `bitcraze.console` service. It uses the advertised
service handle rather than assuming a fixed handle or catalog position.

When a client enables `deck:bcCam`, the Crazyflie grants one Common Link
receive slot for Console output. It forwards received bytes into the CRTP
transmit queue, splitting a Common Link frame across CRTP packets when needed.
It grants another slot only after the previous frame has been queued. The
Console slot is independent of the Control service slot, so a stalled Control
probe does not prevent Console output.

If the CRTP transmit queue is full, the Crazyflie keeps the accepted frame and
stops granting Console receive slots. Later output remains in the Camera Deck's
spool, subject to the deck's storage limit. A radio activity timeout does not
clear Console packets already queued on the Crazyflie. Queue acceptance does
not confirm delivery to a client.

Accepted frames remain pending while the source is disabled and can be
forwarded after it is enabled again. They also survive a Camera Deck reboot
or a link fault, so output sent just before a reboot is not lost on the
Crazyflie.

The Camera Deck startup-recovery watchdog works at the link level only. The
Crazyflie resets the deck immediately on a link fault, and when the UART link
is not active within 3 seconds. Once the link is active, the watchdog does not
reset the deck, whatever the state of the services above the link, and
enabling the console has no effect on it. If startup still has not completed
3 seconds after the link came up, for example because the Control probe
stalled, the Crazyflie reports it once on its own console. The deck's console
output then remains available to explain the stall.
