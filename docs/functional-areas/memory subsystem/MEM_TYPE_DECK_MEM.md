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

The firmware that is required by a deck is uniquely identified through the tuple `(required hash, required length, name)`.

A deck driver may implement read and/or write operations to data on a deck, a camera deck could for
instance proved image data through a memory read operation. The deck driver can freely choose the addresses
where data is mapped. From a client point of view the address will be relative to the base address of the deck,
the base address is aquired by a client from the information section.

If a deck has firmware upgrade capabilities, the write address for firmware upgrades must allways be at 0 (relative
to the base address). A deck is ready to receive FW if the `Bootloader active` flag is set.

## Memory layout

| Address           | Type         | Description                                              |
|-------------------|--------------|----------------------------------------------------------|
| 0x0000            | Info section | Information on installed decks and the mapping to memory |
| deck 1 base addr  | raw memory   | Mapped to the memory on deck 1                           |
| deck 2 base addr  | raw memory   | Mapped to the memory on deck 2                           |
| deck 3 base addr  | raw memory   | Mapped to the memory on deck 3                           |
| deck 4 base addr  | raw memory   | Mapped to the memory on deck 4                           |


### Info section memory layout

| Address | Type             | Description            |
|---------|------------------|------------------------|
| 0x0000  | uint8            | Version, currently 1   |
| 0x0001  | Deck memory info | Information for deck 1 |
| 0x0021  | Deck memory info | Information for deck 2 |
| 0x0041  | Deck memory info | Information for deck 3 |
| 0x0061  | Deck memory info | Information for deck 4 |


### Deck memory info memory layout

Addresses relative to the deck memory info base address

| Address | Type        | Description                                                        |
|---------|-------------|--------------------------------------------------------------------|
| 0x0000  | uint8       | A bitfield describing the properties of the deck memory, see below |
| 0x0001  | uint32      | required hash - the hash for the reuired firmware                  |
| 0x0005  | uint32      | required length - the length of the required firmware              |
| 0x0009  | uint32      | base address - the start address of the deck memory                |
| 0x000D  | uint8\[19\] | name - zero terminated string, max 19 bytes in total.              |


### Deck memory info bit field

| Bit | Property          | 0                                                          | 1                                                   |
|-----|-------------------|------------------------------------------------------------|-----------------------------------------------------|
| 0   | Is valid          | no deck is installed or does not support memory operations | a deck is installed and the data is valid           |
| 1   | Is started        | the deck is in start up mode, data is not reliable yet     | deck has started, data is reliable                  |
| 3   | Supports write    | write not supported                                        | memory is writeable                                 |
| 2   | Supports read     | read not supported                                         | memory is readable                                  |
| 4   | Supports upgrade  | no upgradable firmware                                     | firmware upgrades possible                          |
| 5   | Upgrade required  | the firmware is up to date                                 | the firmware needs to be upgraded                   |
| 6   | Bootloader active | the deck is running FW                                     | the deck is in bootloader mode, ready to receive FW |
| 7   | Reserved          |                                                            |                                                     |
