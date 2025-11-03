# Generic LED Cycle Example

This example demonstrates how to control LED decks on the Crazyflie using the generic LED API. This API provides a single interface that works across different LED deck types (LED ring, Color LED deck, future LED decks), trading deck-specific features for compatibility.

For most use cases, the deck-specific APIs are recommended as they expose the full capabilities of the hardware. See [app_color_led_cycle](../app_color_led_cycle/) for an example.

## What it does

The application cycles through 3 color phases using the generic RGB888 interface, each with 256 steps for smooth transitions:

1. **Red → Green**: Red fades from 255 to 0 while green increases from 0 to 255
2. **Green → Blue**: Green fades from 255 to 0 while blue increases from 0 to 255
3. **Blue → Red**: Blue fades from 255 to 0 while red increases from 0 to 255

Each color transition takes approximately 768ms (256 steps × 3ms per step), resulting in a complete cycle every ~2.3 seconds.

## Generic LED API vs Deck-Specific APIs

The **generic LED API** (`led_deck_ctrl.rgb888` parameter):

- Works with any LED deck (LED ring, Color LED deck, future LED decks)
- Single parameter controls all attached LED decks simultaneously
- Basic RGB888 color control: `0x00RRGGBB`
- No access to deck-specific features

**Deck-specific APIs** (e.g., `colorled.wrgb8888` for the Color LED deck):

- Access to all hardware-specific features (e.g., thermal feedback for the Color LED deck)
- Individual deck control when multiple decks are attached

**Deck-specific APIs are generally recommended** as they expose the full capabilities of your hardware.

**Note:** You can also check which deck is attached at runtime and use the appropriate deck-specific API. This gives you full hardware control while maintaining flexibility.

## Technical details

- Uses the `led_deck_ctrl.rgb888` parameter to control any LED deck.
- Updates colors every 3ms using `vTaskDelayUntil(&lastWakeTime, M2T(3))` for consistent timing
- Color values are packed into a 32-bit format: `0x00RRGGBB` (RGB888 standard)

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