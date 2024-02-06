---
title: Loco Positioning System 2 - MEM_TYPE_LOCO2
page_id: mem_type_loco2
---

The Loco Positioning System memory implementation provides means to read information about the
Loco Positioning System. There is no write functionality.

## Memory layout

| Address              | Type        | Description                                                                |
|----------------------|-------------|----------------------------------------------------------------------------|
| 0x0000               | uint8       | number of anchors in the system (n)                                        |
| 0x0001 - 0x0000 + n  | uint8       | unordered list of anchor ids in the system                                 |
|                      |             |                                                                            |
| 0x1000               | uint8       | number of active anchors in the system (na)                                |
| 0x1001 - 0x1000 + na | uint8       | unordered list of anchor ids for the anchors in the system that are active |
|                      |             |                                                                            |
| 0x2000               | Anchor data | Data for anchor id 0 (if available)                                        |
| 0x2100               | Anchor data | Data for anchor id 1 (if available)                                        |
| ...                  | Anchor data |                                                                            |
| 0x2000 + 0x100 * i   | Anchor data | Data for anchor id i (if available)                                        |
| ...                  | Anchor data |                                                                            |
| 0x11F00              | Anchor data | Data for anchor id 255 (if available)                                      |

### Anchor data memory layout

Address relative to start address for the anchor data

| Address | Type            | Description                                             |
|---------|-----------------|---------------------------------------------------------|
| 0x0000  | float (4 bytes) | x coordinate of the anchor position                     |
| 0x0004  | float (4 bytes) | y coordinate of the anchor position                     |
| 0x0008  | float (4 bytes) | z coordinate of the anchor position                     |
| 0x000C  | uint8           | Is valid : 0 = the data is not valid, 1 = data is valid |
