---
title: DeckCtrl protocol specification
page_id: deckctrl_protocol
---

# DeckCtrl protocol specification

The DeckCtrl backend implements an I2C-based deck and control discovery mechanism that uses microcontrollers on deck boards (_deck controllers_) to enable dynamic enumeration of multiple decks.
The microcontroller implementing the DeckCtrl protocol is exclusively used for that purpose, deck functionalities are implemented on
another chip/microcontroller.

## Discovery Sequence

The DeckCtrl backend performs the following sequence during initialization:

### 1. Reset Phase (Address 0x41)

All deck controllers on the bus listen to the reset address. A read operation to this address causes all controllers to reset their I2C configuration and return to default state.

```c
i2cdevReadReg16(I2C1_DEV, 0x41, 0x0000, 2, dummy_buffer);
vTaskDelay(10);  // Wait for controllers to restart
```

### 2. Listening Mode (Address 0x42)

After reset, the unconfigured deck controllers must be told to enter listening mode. In this state, they monitor the bus for address assignment.

```c
i2cdevReadReg16(I2C1_DEV, 0x42, 0x0000, 2, dummy_buffer);
```

### 3. Enumeration

For each deck to be discovered:

#### 3a. Read CPU ID (Address 0x43, Register 0x1900)

We read the next unconfigured deck with the lowest ID.
The other decks will observe a bus collision and back-off, they will then be ready to be put back in listening mode in step 2.

```c
uint8_t cpu_id[12];
i2cdevReadReg16(I2C1_DEV, 0x43, 0x1900, 12, cpu_id);
```

*Note: This is the core of the discovery protocol, it uses the native anticollision behavior of
I2C to be able to detect decks one by one. I2C detects a collision if it tries to let the open-collector data line high
while another deck is pulling it low. The deck that wanted it high will detect the line low, assume a collision, and back-off*

#### 3b. Assign Address (Address 0x43, Register 0x1800)

Write a unique I2C address to the deck controller. The address range starts at 0x44:

```c
uint8_t deck_address = 0x44 + deck_count;
i2cdevWriteReg16(I2C1_DEV, 0x43, 0x1800, 1, &deck_address);
```

After receiving its address, the deck removes itself from the default addresses and begins responding only to its assigned address.

#### 3c. Read Deck Information (Assigned Address, Register 0x0000)

Read the deck's identification and capability information from its newly assigned address:

```c
uint8_t deck_info[21];
i2cdevReadReg16(I2C1_DEV, deck_address, 0x0000, 21, deck_info);
```

### 4. Repeat

Return to step 2 (listening mode) to discover the next deck. Continue until no more decks respond.

## I2C Address Map

| Address | Purpose | Description |
|---------|---------|-------------|
| 0x41 | Reset | Broadcast reset to all deck controllers |
| 0x42 | Listen | Put unconfigured controllers in listening mode |
| 0x43 | Default | Read CPU ID and assign unique address |
| 0x44-0x4F | Assigned | Individual deck addresses (up to 12 decks) |

The maximum number of decks is configured via `CONFIG_DECK_BACKEND_DECKCTRL_MAX_DECKS`.

## Memory Layout

### Register Map

Once configured, the deck-control behaves like an I2C memory with the following memory map:

| Register | Size | Description                                 |
|----------|------|---------------------------------------------|
|   0x0000 |   21 | Deck identification information (Read-only) |
|   0x0020 | 2016 | ROM partitions                              |
|   0x1000 |    4 | GPIO                                        |
|   0x1800 |    1 | Address assignment (write-only)             |
|   0x1900 |   12 | CPU unique ID (read-only)                   |

### Deck Information Format (Register 0x0000)

The 21-byte deck information block contains:

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0x00   |    2 | Magic | Magic number 0xBCDC (big-endian) |
| 0x02   |    1 | Major Version | Firmware major version |
| 0x03   |    1 | Minor Version | Firmware minor version |
| 0x04   |    1 | Vendor ID | Deck vendor ID |
| 0x05   |    1 | Product ID | Deck product ID |
| 0x06   |    1 | Board Revision | Board revision character |
| 0x07   |   14 | Product Name | Null-terminated product name string |

#### Magic Number

All valid DeckCtrl decks must return the magic number **0xBCDC** in the first two bytes. This validates that the device is a proper DeckCtrl deck.

Format: Big-endian (0xBC at offset 0, 0xDC at offset 1)

#### Vendor ID and Product ID

The VID/PID pair uniquely identifies the deck type and is used to match with the appropriate deck driver in firmware. These should match the `vid` and `pid` fields in the `DeckDriver` structure.

#### Product Name

A human-readable name for the deck (up to 14 characters plus null terminator). This is used for logging and debugging.

## Backend Context

Each discovered DeckCtrl deck receives a `DeckCtrlContext` structure that contains backend-specific data:

```c
typedef struct deckCtrlContext_s {
    uint8_t i2cAddress;  // Assigned I2C address (0x44+)
} DeckCtrlContext;
```

Deck drivers can access this context through the `DeckInfo` structure passed to their `init()` function:

```c
void myDeckInit(DeckInfo *info) {
    DeckCtrlContext *ctx = (DeckCtrlContext *)info->backendContext;
    uint8_t address = ctx->i2cAddress;

    // Use address for I2C communication with the deck
}
```

### ROM Partition formats (address 0x0020)

The space is split in partitions with the following format:

| Offset |         Size |   Field | Description                                      |
|--------|--------------|---------|--------------------------------------------------|
| 0x00   |           2  |  Length | Partition full size, 0x0000 if no more partition |
| 0x02   |           4  |    Type | Partition type                                   |
| 0x06   | *Length* - 6 |    Data | Partition data                                   |

At offset *length* the next partition is starting. A length of 0x0000 denote the end of the partition table.
A length between 1 and 5 included is invalid.

There are no partition type defined yet. This mechanism is designed to allow for future expansion.

### GPIO control

|  Offset | Size |     Field | Reset value | Description                               |
|---------|------|-----------|-------------|-------------------------------------------|
|    0x00 |    2 | Direction |        0x00 | GPIO Direction. 0 for input, 1 for output |
|    0x02 |    2 | Value     |           * | GPIO Value                                |

