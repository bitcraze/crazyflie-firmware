---
title: Console
page_id: crtp_console
---

This port is used as a one-way text console for printing text from the
Crazyflie to a host using the consoleprintf function.

## Communication protocol

    Port: 0
    Channel: 0

    Payload (Crazyflie to host):
            +---------//-----------+
            | PRINTED CONSOLE TEXT |
            +---------//-----------+
    Length          0-31

The content is the text from the console. The encoding is ascii/UTF8 however
this cannot be guaranteed so the packet receiver should be ready for binary
buffer.

The best is to not make any assumption about the data received on the console
port: it could be a line, multiple line, part of a line, ...


The current algorithm in the Crazyflie is to send a console packet is any of
these condition are fulfilled:

 - The output buffer (of 31 bytes) is full
 - A \"newline\" character has to be send (\\n and/or \\r)
 - The flush function has been called
