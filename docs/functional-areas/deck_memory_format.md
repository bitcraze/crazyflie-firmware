---
title: Deck memory format
page_id: deck_memory_format
---

The OneWire (OW) memory contained in all Crazyflie 2.X decks is used to
detect the boards connected in order to:

-   Decide if it is safe to start the system.
-   When the system is started, initialize the required drivers for the
    deck installed.

The memory format has CRC to guarantee data integrity. It is separated
in two part:

-   The header is 7 bytes long and contains the identity of the board
    encoded in VID/PID (Vendor ID and Product ID) as well as a bitfield
    describing the pins the deck is driving (to avoid conflict).
-   The key/value area is extendable and contains more information like
    the deck name, board revision and driver-specific information (ie.
    for configuration or calibration for example).

## Content

      1. Header     1 Byte 0xEB
      2. UsedPins       4 Bytes
      3. VID        1 Byte
      4. PID        1 Byte
      5. crc        1 Byte CRC32[0] (LSB byte) from 1 to 4.
      6. Version        1Byte, 0 for now
      7. DataLength     1 Byte (255 max data length…)
      8. key/value data <DataLength> bytes
      9. crc        1 Byte CRC32[0] (LSB byte) from 6 to 8

-   The header area is from 1 to 5.
-   The key/value area is from 6 do 9

## Header

The header allows to detect the deck installed and to verify that the
deck stack is compatible.

The *UsedPins* value indicates which pins the **deck is driving** on the
expansion port. For example a UART TX pin is driving both high and low
when an i2c port is only driving low. For every GPIO on the expantion
port one bit indicate if the GPIO can be driven high and one bit if it
can be driven low:

|  Drive     | PC11  | PC10 |  PB7 |  PB6  | PB8  | PB5  | PB4  | PC12  | PA2  | PA3  | PA5  | PA6  | PA7 |  P0.11  | P0.12  | P0.08 |
|  ---------- |------| ------| -----| -----| -----| -----| -----| ------| -----| -----| -----| ----- |-----| -------| -------| -------|
|  **Low**   | 0      |1     | 2    | 3    | 4    | 5    | 6    | 7     | 8    | 9    | 10   | 11   | 12  |  13     | 14     | 15|
|  **High**  | 16     |17    | 18   | 19   | 20   | 21   | 22   | 23    | 24   | 25   | 26   | 27   | 28  |  29     | 31    |  31|

For example is a deck has a GPS that has its UART TX on PC11 (push-pull,
drives high and low), the *UsedPins* value will be (1\<\<0) \| (1\<\<16)
= 0x00010001. As another example is a deck has an I2C sensor on PB6,PB7
(Open collector, only drives low), the *UsedPins* value is then:
(1\<\<2) \| (1\<\<3) = 0x0000000C

The *VID* is the vendor ID (or Maker ID :). Currently only 0xBC is
reserved for Bitcraze and 0x00 is reserved for test/development. Contact
Bitcraze if you plan to distribute decks and want a *VID*.

A deck with *VID/PID* = 0/0 must have its name in the key/value area
(the driver will then be chosen by name instead of pid/vid).

## Key/value area

The *version* is 0. The key/value data contains a succession of
elements. Elements id and types are predefined. Unknown elements are
ignored.

Elements have this format:


 | **Id**      | 1Byte|
 | **length**  | 1Byte|
 | **data**     |\<length\> Bytes|


The following elements are defined:

|  Element     | Element\_id  | Data type  | Note|
|  ------------| -------------| -----------| -------------------------------------------|
|  boardName   | 1            | string    |  Short. Starts by "bc" for Bitcraze boards|
|  revision    | 2            | string    |  |
|  customData  | 3            | binary    |  Data understood by the deck driver|

## Example

This is an example of OW memory content for the led ring deck.

    0000000: eb 00 00 00 00 bc 01 44 00 0e 01 09 62 63 4c 65  .......D....bcLe
    0000010: 64 52 69 6e 67 02 01 62 55                       dRing..bU

Content:

    {
        "header": {
            "usedPin": 0,
            "vid": 188,
            "pid": 1
        },
        "data": {
            "boardName": "bcLedRing",
            "revision": "b"
        }
    }

Element IDs and types (DTD):

    {
        "boardName": {
            "type": "string",
            "id": 1
        },
        "revision": {
            "type": "string",
            "id": 2
        },
        "customData": {
            "type": "binary",
            "id": 3
        }
    }
