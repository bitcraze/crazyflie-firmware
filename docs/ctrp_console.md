---
title: Concole
page_id: ctrp_concole
---

This port is used as a one-way text console for printing text from the
Crazyflie to a host using the consoleprintf function.

Communication protocol
======================

    Answer (Crazyflie to host):
            +---------//-----------+
            | PRINTED CONSOLE TEXT |
            +---------//-----------+
    Length          0-31

The contents of the buffer on the copter side is sent if any of the
following is fulfilled:

-   The output buffer (of 31 bytes) is full
-   A \"newline\" character has to be send (\\n and/or \\r)
-   A flush command as been issued


