
#include "kve/kve.h"
#include "kve/kve_storage.h"

#include "debug.h"

#include <stdbool.h>
#include <string.h>



#define VERSION_ADDRESS (0)
#define FIRST_ITEM_ADDRESS (1)

// Current version of the KVE table is 1
#define KVE_VERSION (1)

static size_t min(size_t a, size_t b)
{
    if (a < b) {
        return a;
    } else {
        return b;
    }
}

// Utility function
static bool appendItemToEnd(kveMemory_t *kve, size_t address, const char* key, const void* buffer, size_t length) {
    size_t itemAddress = kveStorageFindEnd(kve, address);

    // If it is over the end of the memory, table corrupted
    // Do not write anything ...
    if (KVE_STORAGE_IS_VALID(itemAddress) == false) {
        DEBUG_PRINT("Error: table corrupted!\n");
        return false;
    }

    // Test that there is enough space to write the item
    if ((itemAddress + sizeof(kveItemHeader_t) + strlen(key) + length + KVE_END_TAG_LENDTH) < kve->memorySize) {
        itemAddress += kveStorageWriteItem(kve, itemAddress, key, buffer, length);
        kveStorageWriteEnd(kve, itemAddress);
    } else {
        // Otherwise, defrag and try to insert again!
        kveDefrag(kve);

        itemAddress = kveStorageFindEnd(kve, FIRST_ITEM_ADDRESS);

        if ((itemAddress + sizeof(kveItemHeader_t) + strlen(key) + length + KVE_END_TAG_LENDTH) < kve->memorySize) {
            itemAddress += kveStorageWriteItem(kve, itemAddress, key, buffer, length);
            kveStorageWriteEnd(kve, itemAddress);
        } else {
            // Memory full!
            DEBUG_PRINT("Error: memory full!");
            return false;
        }
    }

    return true;
}

// Public API

void kveDefrag(kveMemory_t *kve) {
    size_t holeAddress = kveStorageFindHole(kve, FIRST_ITEM_ADDRESS);
    size_t itemAddress;
    size_t nextHoleAddress;

    while(KVE_STORAGE_IS_VALID(holeAddress)) {
        itemAddress = kveStorageFindNextItem(kve, holeAddress);

        if (KVE_STORAGE_IS_VALID(itemAddress) == false) {
            // This hole is at the end, lets crop it
            kveStorageWriteEnd(kve, holeAddress);
            break;
        }

        nextHoleAddress = kveStorageFindHole(kve, itemAddress);

        if(KVE_STORAGE_IS_VALID(nextHoleAddress) == false) {
            // If there is the end after this group of item, lets copy up to the end
            nextHoleAddress = kveStorageFindEnd(kve, itemAddress);
        }

        size_t lenghtToMove = nextHoleAddress - itemAddress;

        kveStorageMoveMemory(kve, itemAddress, holeAddress, lenghtToMove);

        kveStorageWriteHole(kve, holeAddress + lenghtToMove, itemAddress - holeAddress);

        holeAddress = holeAddress + lenghtToMove;
    }
}

bool kveStore(kveMemory_t *kve, const char* key, const void* buffer, size_t length) {
    size_t itemAddress;

    // Search if the key is already present in the table
    itemAddress = kveStorageFindItemByKey(kve, FIRST_ITEM_ADDRESS, key);
    if (KVE_STORAGE_IS_VALID(itemAddress) == false) {
        // Item does not exit, find the end of the table to insert it
        return appendItemToEnd(kve, FIRST_ITEM_ADDRESS, key, buffer, length);
    } else {
        // Item exist, verify that the data has the same size
        kveItemHeader_t currentItem = kveStorageGetItemInfo(kve, itemAddress);
        uint16_t newLength = length + 3 + strlen(key);
        if (currentItem.full_length != newLength) {
            // If not, delete the item and find the end of the table
            kveStorageWriteHole(kve, itemAddress, currentItem.full_length);
            return appendItemToEnd(kve, FIRST_ITEM_ADDRESS, key, buffer, length);
        } else {
            kveStorageWriteItem(kve, itemAddress, key, buffer, length);
        }
    }

    return true;
}

//
// We will use the function kveStorageFindItemByPrefix to find the first item
// with a key that matches our prefix.
// The function will return the size of that item, which we can use to then
// look for the next item that after that that matches our prefix.
//
// The function kveStorageFindItemByPrefix also gives us the key and item
// address as out-arguments, which we can use as arguments to the supplied
// user function that will be run for each item found.
//
// We keep going until we get an invalid address back, which mean we have
// reached the end.
//
bool kveForeach(kveMemory_t *kve, const char *prefix, kveFunc_t func)
{
    static char keyBuffer[64] = { 0, };
    size_t itemAddress;
    size_t itemSize = kveStorageFindItemByPrefix(kve, FIRST_ITEM_ADDRESS, prefix, keyBuffer, &itemAddress);

    while (KVE_STORAGE_IS_VALID(itemAddress)) {
        const int bufferLength = 8;
        char buffer[bufferLength];

        kveItemHeader_t header = kveStorageGetItemInfo(kve, itemAddress);
        const size_t storeSize = header.full_length - header.key_length - sizeof(header);
        size_t readLength = min(storeSize, bufferLength);

        if (kveStorageGetBuffer(kve, itemAddress, header, buffer, readLength)) {
            if (!func((const char *) keyBuffer, buffer, readLength)) {
                return false;
            }
        } else {
            return false;
        }
        itemSize = kveStorageFindItemByPrefix(kve, itemAddress + itemSize, prefix, keyBuffer, &itemAddress);
    }
    return true;
}


size_t kveFetch(kveMemory_t *kve, const char* key, void* buffer, size_t bufferLength)
{
    size_t itemAddress = kveStorageFindItemByKey(kve, FIRST_ITEM_ADDRESS, key);

    if (KVE_STORAGE_IS_VALID(itemAddress)) {
        kveItemHeader_t header = kveStorageGetItemInfo(kve, itemAddress);
        const size_t storeSize = header.full_length - header.key_length - sizeof(header);
        size_t readLength = min(storeSize, bufferLength);
        return kveStorageGetBuffer(kve, itemAddress, header, buffer, readLength);
    }
    return 0;
}

bool kveDelete(kveMemory_t *kve, const char* key) {
    size_t itemAddress = kveStorageFindItemByKey(kve, FIRST_ITEM_ADDRESS, key);

    if (KVE_STORAGE_IS_VALID(itemAddress)) {
        kveItemHeader_t itemInfo = kveStorageGetItemInfo(kve, itemAddress);
        kveStorageWriteHole(kve, itemAddress, itemInfo.full_length);
        return true;
    }

    return false;
}

void kveFormat(kveMemory_t *kve) {
    uint8_t version = KVE_VERSION;
    kve->write(VERSION_ADDRESS, &version, 1);
    kveStorageWriteEnd(kve, FIRST_ITEM_ADDRESS);
}

bool kveCheck(kveMemory_t *kve) {

    // Check version
    uint8_t version;
    kve->read(VERSION_ADDRESS, &version, 1);
    if (version != KVE_VERSION) {
        return false;
    }

    // Check table consistency
    size_t endAddress = kveStorageFindEnd(kve, FIRST_ITEM_ADDRESS);

    // If it is not possible to get to the end tag, the table is corupted
    if (!KVE_STORAGE_IS_VALID(endAddress)) {
        return false;
    }

    return true;
}

void kveGetStats(kveMemory_t *kve, kveStats_t *stats) {
    size_t item_address = FIRST_ITEM_ADDRESS;

    size_t end_address = kveStorageFindEnd(kve, FIRST_ITEM_ADDRESS);

    size_t total_size = 0;
    size_t total_items = 0;
    size_t hole_size = 0;
    size_t item_size = 0;
    size_t data_size = 0;
    size_t key_size = 0;
    size_t metadata_size = 0;

    while (item_address < end_address) {
        kveItemHeader_t itemInfo = kveStorageGetItemInfo(kve, item_address);

        if (itemInfo.full_length == KVE_END_TAG) {
            break;
        }

        total_size += itemInfo.full_length;

        if (itemInfo.key_length == 0) {
            hole_size += itemInfo.full_length;
        } else {
            item_size += itemInfo.full_length;
            total_items++;

            key_size += itemInfo.key_length;
            metadata_size += sizeof(itemInfo);
            data_size += itemInfo.full_length - itemInfo.key_length - sizeof(itemInfo);
        }

        item_address = item_address + itemInfo.full_length;
    }

    stats->totalSize = kve->memorySize;
    stats->totalItems = total_items;
    stats->itemSize = item_size;
    stats->keySize = key_size;
    stats->dataSize = data_size;
    stats->metadataSize = metadata_size;
    stats->holeSize = hole_size;
    stats->freeSpace = kve->memorySize - item_size;
    stats->fragmentation = (hole_size * 100) / (kve->memorySize - item_size);
    stats->spaceLeftUntilForcedDefrag = kve->memorySize - total_size;
}
