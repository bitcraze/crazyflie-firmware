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

The firmware that is required by a deck is uniquely identified through the tuple `(required hash, required length, name)`.

A deck driver may implement read and/or write operations to data on a deck, a camera deck could for
instance proved image data through a memory read operation. The deck driver can freely choose the addresses
where data is mapped. From a client point of view the address will be relative to the base address of the deck,
the base address is aquired by a client from the information section.

If a deck has firmware upgrade capabilities, the write address for firmware upgrades must allways be at 0 (relative
to the base address).

## Memory layout

| Address           | Type         | Description                                              |
|-------------------|--------------|----------------------------------------------------------|
| 0x0000            | Info section | Information on installed decks and the mapping to memory |
| deck 1 base addr  | raw memory   | Mapped to the memory on deck 1                           |
| deck 2 base addr  | raw memory   | Mapped to the memory on deck 2                           |
| ...               | raw memory   | ...                                                      |


### Info section memory layout

| Address | Type             | Description            |
|---------|------------------|------------------------|
| 0x0000  | Deck memory info | Information for deck 1 |
| 0x001C  | Deck memory info | Information for deck 2 |
| 0x0038  | Deck memory info | Information for deck 3 |
| 0x0054  | Deck memory info | Information for deck 4 |


### Deck memory info memory layout

Addresses relative to the deck memory info base address

| Address | Type        | Description                                                        |
|---------|-------------|--------------------------------------------------------------------|
| 0x0000  | uint8       | A bitfield describing the properties of the deck memory, see below |
| 0x0001  | uint32      | required hash - the hash for the reuired firmware                  |
| 0x0005  | uint32      | required length - the length of the required firmware              |
| 0x0009  | uint32      | base address - the start address of the deck memory                |
| 0x000D  | uint8\[15\] | name - zero terminated string, max 15 bytes in total.              |


### Deck memory info bit field

| Bit | Property         | Description                                                                                       |
|-----|------------------|---------------------------------------------------------------------------------------------------|
| 0   | Is valid         | 0 = no deck is installed and no more data is valid, 1 = a deck is installed and the data is valid |
| 1   | supports read    | 0 = read not supported, 1 = memory is readable                                                    |
| 2   | supports write   | 0 = write not supported, 1 = memory is writeable                                                  |
| 3   | supports upgrade | 0 = no upgradable firmware , 1 = firmware upgrades possible                                       |
| 4   | upgrade required | 0 = the firmware is up to date , 1 = the firmware needs to be upgraded                            |
| 5   | -                | reserved                                                                                          |
| 6   | -                | reserved                                                                                          |
| 7   | -                | reserved                                                                                          |
