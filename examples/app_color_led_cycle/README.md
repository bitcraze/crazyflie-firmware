# LED Color Cycle Example

This example demonstrates how to control the Color LED deck on the Crazyflie by cycling through a smooth color transition pattern. It uses deck-specific features like WRGB control, brightness correction, and thermal monitoring.

For a hardware-agnostic interface that works across different LED deck types, see [app_generic_led_cycle](../app_generic_led_cycle/).

## What it does

The application cycles through 4 distinct color phases, each with 256 steps for smooth transitions:

1. **Red → Green**: Red fades from 255 to 0 while green increases from 0 to 255
2. **Green → Blue**: Green fades from 255 to 0 while blue increases from 0 to 255
3. **Blue → White**: Blue fades from 255 to 0 while white increases from 0 to 255
4. **White → Red**: White fades from 255 to 0 while red increases from 0 to 255

Each color transition takes approximately 768ms (256 steps × 3ms per step), resulting in a complete cycle every ~3 seconds.

## Technical details

- Uses the `colorled.wrgb8888` parameter to control the Color LED deck
- Enables `colorled.brightnessCorr` for perceptually uniform colors by balancing luminance across W/R/G/B channels (disable for maximum brightness per channel)
- Updates colors every 3ms using `vTaskDelayUntil(&lastWakeTime, M2T(3))` for consistent timing
- Color values are packed into a 32-bit format: `0xWWRRGGBB`
- Monitors thermal throttling via the `colorled.deckTemp` and `colorled.throttlePct` log variables
- Prints warnings to the debug console if thermal throttling is detected (checked every 100ms)

## Building and running

From this app folder, run the following commands:

1. Load the default configuration for your platform:
   ```bash
   make cf2_defconfig      # For Crazyflie 2.1
   # or
   make cf21bl_defconfig   # For Crazyflie 2.1 Brushless
   # or the appropriate defconfig for your Crazyflie model
   ```

2. Build the firmware:
   ```bash
   make -j$(nproc)
   ```

3. Flash over Crazyradio:
   ```bash
   make cload
   # or with a specific URI:
   CLOAD_CMDS="-w radio://0/80/2M/E7E7E7E7E7" make cload
   ```

   Using a specific URI avoids needing to manually enter bootloader mode and prevents accidentally flashing someone else's Crazyflie in multi-user environments.

The LED cycling will start automatically when the drone boots up.

To view the debug output (including thermal throttling warnings), connect to the Crazyflie using the cfclient and open the console tab.