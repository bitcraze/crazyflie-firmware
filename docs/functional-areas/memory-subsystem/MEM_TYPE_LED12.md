---
title: LED ring - MEM_TYPE_LED12
page_id: mem_type_led12
---

This memory is mapped to the color/brightness of the LEDs on the LED-ring and supports both read and write operations.

The color/brightness is set using the RGB565 format (2 bytes) and the LEDs are mapped after each other. The number
of LEDs is defined by the `CONFIG_DECK_LEDRING_NBR_LEDS` kbuild flag (12 by default) and the last LED will be mapped to address
`(CONFIG_DECK_LEDRING_NBR_LEDS  - 1) * 2`.

| Address                                 | Type           | LED                              |
|-----------------------------------------|----------------|----------------------------------|
| 0                                       | color (RGB565) | 0                                |
| 2                                       | color (RGB565) | 1                                |
| 4                                       | color (RGB565) | 2                                |
| ...                                     | color (RGB565) | ...                              |
| (CONFIG_DECK_LEDRING_NBR_LEDS  - 1) * 2 | color (RGB565) | CONFIG_DECK_LEDRING_NBR_LEDS - 1 |

### RGB565

Red, green and blue are encoded using 5, 6 and 5 bits, adding up to a total of 16 bits (2 bytes).

{% ditaa --alt "Example diagram" %}
    5 bits       6 bits       5 bits

/-----------+-------------+-----------\
|   cRED    |   cGRE      |   cBLU    |
|    Red    |    Green    |    Blue   |
|    R5     |     G6      |     B5    |
|           |             |           |
\-----------+-------------+-----------/
{% endditaa %}

The encoded values can be converted to 8 bit values (RGB565 to RGB888) like this:
``` C
R8 = (R5 * 527 + 23) >> 6;
G8 = (G6 * 259 + 33) >> 6;
B8 = (B5 * 527 + 23) >> 6;
```

and the other way around, RGB888 to RGB565:
``` C
R5 = ((R8 << 6) - 23) / 527;
G5 = ((G8 << 6) - 33) / 259;
B5 = ((B8 << 6) - 23) / 527;
```
