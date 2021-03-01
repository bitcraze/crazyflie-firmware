# System ID for PWM <-> Thrust


## Setup

* 100g Mini Load Cell, https://www.sparkfun.com/products/14727
* Load Cell Amplifier HX711, https://learn.sparkfun.com/tutorials/load-cell-amplifier-hx711-breakout-hookup-guide
* Prototype deck, https://www.bitcraze.io/prototyping-deck/
* Calibration weights
* M2/M3 Nylon screws
* CF Mount (3D printed)

Mount CF upside down (to avoid downwash)

## Measurement

1. Mount CF
2. Flash firmware from dev-systemId branch
3. Run `python3 collectData.py` (includes prompts to calibrate using calibration weights and writes data.csv)
4. Evaluate using `python3 system_id.py`
