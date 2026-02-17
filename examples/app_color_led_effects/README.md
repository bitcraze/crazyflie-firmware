# Color LED Effects Example

This application provides multiple WRGB LED effects for Crazyflie drones, supporting both top- and bottom-mounted Color LED decks. Each effect has customizable behavior.

---

## Effects

### Effect 0: Off / Default
- Leaves the LED decks under normal control (no custom effect applied).

### Effect 1: LED Cycle
- Cycles smoothly through Red, Green, Blue, and White.
- Both decks display the same color at all times.

### Effect 2: XYZ Color Mapping
- Maps drone position to RGB color:
  - **X -> Red**
  - **Y -> Green**
  - **Z -> Blue**
- White is kept off.
- Drone coordinates are clamped to the defined flight area:
  ```c
  MIN_X_BOUND, MAX_X_BOUND
  MIN_Y_BOUND, MAX_Y_BOUND
  MIN_Z_BOUND, MAX_Z_BOUND
  ```

### Effect 3: Velocity Indicator
- Visualizes the drone’s 3D velocity magnitude using a smooth color gradient.
- The velocity is computed from the logged velocity components:

  vel = sqrt(vx<sup>2</sup> + vy<sup>2</sup> + vz<sup>2</sup>)

- The speed is clamped between 0 and `MAX_VEL` (default 1.0 m/s).

- The LED color transitions smoothly based on normalized velocity (0 -> `MAX_VEL`):

  0% speed -> Blue

  50% speed -> Green

  100% speed -> Red

- The computed color is sent to both the top and bottom LED decks (if attached).


### Effect 4: Snowflake
White LEDs fade in and out with a sinusoidal effect.

- Brightness scales linearly with height:

    - `max_height` -> brightness = 100%

    - `LANDING_Z` -> brightness = 0%

    - `max_height` is captured automatically when the effect is first activated.

Fade period is controlled via:

- `FADE_PERIOD_MS`


### Effect 5: Flicker
Both decks flicker asynchronously to simulate a random sparkling effect.

- Color is fixed (`COLOR_GOLD` by default).

- Off durations are randomized between `FLICKER_MIN_OFF` and `FLICKER_MAX_OFF`

- Each deck flickers independently.


## Parameters

All effects can be switched using the `colorLED.effect` parameter:

| Parameter        | Type   | Description                   | Values |
|------------------|--------|-------------------------------|--------|
| `colorLED.effect`| uint8  | Select LED effect to display  | 0–5    |