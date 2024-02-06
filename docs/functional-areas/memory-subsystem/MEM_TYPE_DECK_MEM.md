---
title: Deck memory - MEM_TYPE_DECK_MEM
page_id: mem_type_deck_mem
---

This memory is used to access memory on decks when available. Decks that have firmware may also
support firmware updates through write operations to the memory.

The information section contains information that allows a client to enumerate installed decks,
identify decks with firmware that needs to be updated and the address to write new firmware to.

The information in a `Deck memory info` record is only valid if the `Is valid` bit is set to 1. The `Is Valid` bit
is always valid, also when set to 0.

Some decks may take a while to boot, the `Is started` bit indicates if the deck has started and is ready.
Data in the `Deck memory info` record should only be used if both the `Is Valid` and the `Is started`
bits are set.

A deck driver may implement read and/or write operations to data on a deck, a camera deck could for
instance provide image data through a memory read operation. The deck driver can freely choose the addresses
where data is mapped. From a client point of view the address will be relative to the base address of the deck,
the base address is acquired by a client from the information section.

If a deck has firmware upgrade capabilities, the write address for firmware upgrades must always be at 0 (relative
to the base address). A deck is ready to receive FW if the `Bootloader active` flag is set.

The firmware that is required by a deck is uniquely identified through the tuple `(required hash, required length, name)`.

Some decks have two hardware devices that requires two firmwares and memory mappings, like the AI deck with the ESP and
the GAP8 modules. For this purpose the system supports two mappings per deck.

A deck may support commands, currently two flavors are available: reset to firmware and to bootloader. The deck signals
the availability of the commands in the info bit fields.

Some decks can not be reset to bootloader mode by a client and will automatically be set in bootloader mode when the
Crazyflie detects that the deck firmware is incompatible with the Crazyflie firmware.


## Memory layout

| Address                            | Type            | Description                                              |
|------------------------------------|-----------------|----------------------------------------------------------|
| 0x0000                             | Info section    | Information on installed decks and the mapping to memory |
| 0x1000                             | Command section | Commands sent to the decks for control                   |
| deck 1, main mem base address      | raw memory      | Mapped to address 0 in the main memory on deck 1         |
| deck 1, secondary mem base address | raw memory      | Mapped to address 0 in the secondary memory on deck 1    |
| deck 2, main mem base address      | raw memory      | Mapped to address 0 in the main memory on deck 2         |
| deck 2, secondary mem base address | raw memory      | Mapped to address 0 in the secondary memory on deck 2    |
| deck 3, main mem base address      | raw memory      | Mapped to address 0 in the main memory on deck 3         |
| deck 3, secondary mem base address | raw memory      | Mapped to address 0 in the secondary memory on deck 3    |
| deck 4, main mem base address      | raw memory      | Mapped to address 0 in the main memory on deck 4         |
| deck 4, secondary mem base address | raw memory      | Mapped to address 0 in the secondary memory on deck 4    |


### Info section memory layout

| Address | Type             | Description                              |
|---------|------------------|------------------------------------------|
| 0x0000  | uint8            | Version, currently 3                     |
| 0x0001  | Deck memory info | Information for deck 1, main memory      |
| 0x0021  | Deck memory info | Information for deck 1, secondary memory |
| 0x0041  | Deck memory info | Information for deck 2, main memory      |
| 0x0061  | Deck memory info | Information for deck 2, secondary memory |
| 0x0081  | Deck memory info | Information for deck 3, main memory      |
| 0x00A1  | Deck memory info | Information for deck 3, secondary memory |
| 0x00C1  | Deck memory info | Information for deck 4, main memory      |
| 0x00E1  | Deck memory info | Information for deck 4, secondary memory |


### Deck memory info memory layout

Addresses relative to the deck memory info base address

| Address | Type        | Description                                                         |
|---------|-------------|---------------------------------------------------------------------|
| 0x0000  | uint8       | Bit field 1 describing the properties of the deck memory, see below |
| 0x0001  | uint8       | Bit field 2 describing the properties of the deck memory, see below |
| 0x0002  | uint32      | required hash - the hash for the reuired firmware                   |
| 0x0006  | uint32      | required length - the length of the required firmware               |
| 0x000A  | uint32      | base address - the start address of the deck memory                 |
| 0x000E  | uint8\[19\] | name - zero terminated string, max 18 bytes in total.               |


### Deck memory info bit fields

Bit field 1:

| Bit | Property                | 0                                                          | 1                                                   |
|-----|-------------------------|------------------------------------------------------------|-----------------------------------------------------|
| 0   | Is valid                | no deck is installed or does not support memory operations | a deck is installed and the data is valid           |
| 1   | Is started              | the deck is in start up mode, data is not reliable yet     | deck has started, data is reliable                  |
| 2   | Supports read           | read not supported                                         | memory is readable                                  |
| 3   | Supports write          | write not supported                                        | memory is writeable                                 |
| 4   | Supports upgrade        | no upgradable firmware                                     | firmware upgrades possible                          |
| 5   | Upgrade required        | the firmware is up to date                                 | the firmware must be be upgraded                    |
| 6   | Bootloader active       | the deck is running FW                                     | the deck is in bootloader mode, ready to receive FW |
| 7   | Reserved                |                                                            |                                                     |

Bit field 2:

| Bit | Property                | 0                                                          | 1                                                   |
|-----|-------------------------|------------------------------------------------------------|-----------------------------------------------------|
| 0   | Can reset deck          | the deck does not support the reset command                | the deck supports the reset command                 |
| 1   | Can reset to bootloader | the deck does not support reset to bootloader mode         | the deck supports reset to bootloader mode          |
| 2-7 | Reserved                |                                                            |                                                     |

### Command section memory layout

| Address | Type                     | Description                               |
|---------|--------------------------|-------------------------------------------|
| 0x1000  | Deck memory command data | Command data for deck 1, main memory      |
| 0x1020  | Deck memory command data | Command data for deck 1, secondary memory |
| 0x1040  | Deck memory command data | Command data for deck 2, main memory      |
| 0x1060  | Deck memory command data | Command data for deck 2, secondary memory |
| 0x1080  | Deck memory command data | Command data for deck 3, main memory      |
| 0x10A0  | Deck memory command data | Command data for deck 3, secondary memory |
| 0x10C0  | Deck memory command data | Command data for deck 4, main memory      |
| 0x10E0  | Deck memory command data | Command data for deck 4, secondary memory |

### Deck memory command data memory layout

Addresses relative to the deck memory command data base address

| Address | Type        | Description                                                        |
|---------|-------------|--------------------------------------------------------------------|
| 0x0000  | uint32      | Size of binary that will be written to flash                       |
| 0x0004  | uint8       | A bit field representing commands                                  |

### Deck memory command bit field

| Bit | Command                    | 0         | 1                                   |
|-----|----------------------------|-----------|-------------------------------------|
| 0   | Reset device to firmware   | no action | reset the device                    |
| 1   | Reset device to bootloader | no action | reset the device to bootloader mode |

## Using deck memory in a deck driver

To use the deck memory functionality, define a `DeckMemDef_t` struct in your deck driver.

```
static const DeckMemDef_t myDeckMemoryDef = {
  .write = write_mem,
  .read = read_mem,
  .properties = propertiesQuery,
  .supportsUpgrade = true,

  .requiredSize = ...,
  .requiredHash = ...,
};
```

and add it to the deck driver struct

```
static const DeckDriver my_dek = {
  // ...
  .memoryDef = &myDeckMemoryDef,
  // ...
};
```

Implement the `propertiesQuery()` function to let the system know what properties and capabilities the memory mapping
for your deck supports.

Also implement the `write_mem` and `read_mem` functions as needed, depending on the capabilities of the deck.

### Secondary memory mapping

Some decks have the need to support two hardware devices with two firmwares and memory mappings, like the AI deck with
the ESP and the GAP8 modules. To support this, it is possible to add a second `DeckMemDef_t` definition:

```
static const DeckMemDef_t memoryDefEsp = {
  .write = write_esp,
  .read = read_esp,
  .properties = properties_esp,
  .supportsUpgrade = true,

  .requiredSize = ...,
  .requiredHash = ...,

  .id = "esp"
};

static const DeckMemDef_t memoryDefGap8 = {
  .write = write_gap8,
  .read = read_gap8,
  .properties = properties_gap8,
  .supportsUpgrade = true,

  .requiredSize = ...,
  .requiredHash = ...,

  .id = "gap8"
};
```
And then you can add them like:

```
static const DeckDriver aideck_deck = {
    .vid = 0xBC,
    .pid = 0x12,
    .name = "bcAI",

    .usedGpio = DECK_USING_IO_4,
    .usedPeriph = DECK_USING_UART1,

    .init = aideckInit,
    .test = aideckTest,

    .memoryDef = memoryDefEsp,
    .memoryDefSecondary = memoryDefGap8,
};
```

And the names visible to the lib would be:
    "bcAI:esp" and
    "bcAI:gap8"
