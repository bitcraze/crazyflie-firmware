# Embedded KV format

The stored binary format is based on TLV (Type Lenght Value) but modified for the need of a dynamic storage.

The format assumes it is working on a EEPROM since it does not implement a proper wear leveling.
However, it can be noted that the format is already prepared to be used in flash allowing to append and discard entries without erasing the page: entries can be added and holes created by only writing zeros to a all-one memory. So, if modification are implemented using copy-on-write, this is effectively becoming a log-format and would fit a flash.

## Basic format

Each KV couple is written as:

 - **Lenght** uint16_t: Lenght of the item, includes length, keylength, key and value.
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
