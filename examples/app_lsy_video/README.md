# LED Patterns Example

This application provides multiple WRGB LED patterns for Crazyflie drones, supporting both top and bottom LED decks. Each pattern has customizable behavior.

---

## Patterns

### Pattern 0: Off / Default
- Leaves the LED decks under normal control (no custom pattern applied).

### Pattern 1: LED Cycle
- Cycles smoothly through Red, Green, Blue, and White.
- Both decks display the same color at all times.

### Pattern 2: XYZ Color Mapping
- Maps drone position to RGB color:
  - **X → Red**
  - **Y → Green**
  - **Z → Blue**
- White is kept off.
- Drone coordinates are clamped to the defined flight area:
  ```c
  MIN_X_BOUND, MAX_X_BOUND
  MIN_Y_BOUND, MAX_Y_BOUND
  MIN_Z_BOUND, MAX_Z_BOUND
  ```


### Pattern 3: Christmas Tree
Height-based pattern:

- Drones above `STAR_HEIGHT` shine Yellow.

- Drones below `STAR_HEIGHT`:

    - Top deck shines Green

    - Bottom deck shines Red

Can create a “tree-like” effect when multiple drones are stacked.


### Pattern 4: Snowflake
White LEDs fade in and out with a sinusoidal effect.

- Brightness scales linearly with height:

    - `max_height` → brightness = 100%

    - `LANDING_Z` → brightness = 0%

    - `max_height` is captured automatically when the pattern is first activated.

Fade period is controlled via:

- `FADE_PERIOD_MS`


### Pattern 5: Flicker
Both decks flicker asynchronously to simulate a random sparkling effect.

- Color is fixed (`COLOR_GOLD` by default).

- Off durations are randomized between `FLICKER_MIN_OFF` and `FLICKER_MAX_OFF`

- Each deck flickers independently.


## Parameters

All patterns can be switched using the `ledpat.pattern` parameter:

| Parameter        | Type   | Description                   | Values |
|------------------|--------|-------------------------------|--------|
| `ledpat.pattern` | uint8  | Select LED pattern to display | 0–5    |