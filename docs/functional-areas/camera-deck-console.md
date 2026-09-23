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

The Camera Deck startup-recovery watchdog is suspended while the source is
enabled and its Console service is bound. This leaves the deck running during
diagnosis, even if the Control probe has stalled. Disabling the source starts a
fresh recovery timeout if Control is still stalled. Accepted frames remain
pending while the source is disabled and can be forwarded after it is enabled
again.
