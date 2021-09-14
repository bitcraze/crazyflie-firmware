---
title: Lighthouse - MEM_TYPE_LH
page_id: mem_type_lh
---

The lighthouse memory mapping is used to communicate geometry and
calibration data. The implementation supports both read and write
operations.

## Memory layout

| Address | Type             | Description                                     |
|---------|------------------|-------------------------------------------------|
| 0x0000  | Geometry data    | Geometry data for base station on channel 1     |
| 0x0100  | Geometry data    | Geometry data for base station on channel 2     |
| ...     | Geometry data    |                                                 |
| 0x0f00  | Geometry data    | Geometry data for base station on channel 16    |
| 0x1000  | Calibration data | Calibration data for base station on channel 1  |
| 0x1100  | Calibration data | Calibration data for base station on channel 2  |
| ...     | Calibration data |                                                 |
| 0x1f00  | Calibration data | Calibration data for base station on channel 16 |

### Geometry data memory layout

| Address | Type                | Description                                                      |
|---------|---------------------|------------------------------------------------------------------|
| 0x0X00  | float (4 bytes) x 3 | Base station position, 3 element vector of floats (4 bytes)      |
| 0x0X0C  | float (4 bytes) x 9 | Base station attitude, 3 x 3 rotation matrix of floats (4 bytes) |
| 0x0X18  | uint8               | Is valid : 0 = the data is not valid, 1 = data is valid          |

### Calibration data memory layout

| Address | Type                   | Description                                             |
|---------|------------------------|---------------------------------------------------------|
| 0x0X00  | Calibration sweep data | Calibration data for first sweep                        |
| 0x0X1C  | Calibration sweep data | Calibration data for second sweep                       |
| 0x0X38  | uint32                 | Base station UID                                        |
| 0x0X3C  | uint8                  | Is valid : 0 = the data is not valid, 1 = data is valid |

#### Calibration sweep data memory layout

| Address | Type | Description                             |
|---------|------------------------------------------------|
| 0x0X00  | float (4 bytes) | phase                        |
| 0x0X04  | float (4 bytes) | tilt                         |
| 0x0X08  | float (4 bytes) | curve                        |
| 0x0X0C  | float (4 bytes) | gibmag                       |
| 0x0X10  | float (4 bytes) | gibphase                     |
| 0x0X14  | float (4 bytes) | ogeemag (only used in LH2)   |
| 0x0X18  | float (4 bytes) | ogeephase (only used in LH2) |
