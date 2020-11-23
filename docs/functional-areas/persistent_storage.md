---
title: Persistent storage
page_id: persistent_storage
---

# Persistent storage

The crazyflie has a persistent storage subsystem that is intended to be used for configuration and other rarely written data.
The 7kB of the internal EEPROM is used for storage.
Fetching data should be fairly fast, storing data can be very slow if the storage space needs to be defragmented/garbage collected.

The API is documented in the [storage.h](https://github.com/bitcraze/crazyflie-firmware/blob/master/src/hal/interface/storage.h).
It currently only implements basic store/fetch/delete functions.
The data stored are buffers and are stored and fetched using a key string.
Care must be taken to not use generic keys in order to avoid collision.


## Embedded KV format

This is low level information about the format used to store data in the EEPROM.
These information are not needed to use the storage api but can be useful if one want to modify or port the memory storage.

The stored binary format is based on TLV (Type Length Value) but modified for the need of a dynamic storage.

The format assumes it is working on a EEPROM since it does not implement a proper wear leveling.
However, it can be noted that the format is already prepared to be used in flash allowing to append and discard entries without erasing the page: entries can be added and holes created by only writing zeros to a all-one memory. So, if modification are implemented using copy-on-write, this is effectively becoming a log-format and would fit a flash.

### Basic format

Each KV couple is written as:

 - **Length** uint16_t: Length of the item, includes length, keylength, key and value.
 - **KeyLength** uint8_t: Length of the key
 - **Key** char*: Key
 - **Value** void*: Data buffer

KV couples are writen one after each other. If one KV is at memory position *n*, the next one will be at memory position *n + length*.

If the key has a length of 0, it indicates a hole and the length indicates
the offset to the next entry.

A lenght of 0xffff means the end of the table.

Holes are created when an entry is deleted or modified entries.

New entries can be added either at the end of the table or in a hole that can fit the new buffer.

When there is no more space for new entries, the memory should be defragmented by moving all items into the holes, packing all the items at the begining of the table.
