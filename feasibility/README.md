# Feasibility test

Checks that the idea works before building anything on top of it.

- Reads `WHO_AM_I` from all 3 MPU6500s over SPI3. LEDs PG6/PG7/PG8 show which sensors answered.
- Samples the 3 Z axes, learns each sensor's resting value and detects taps.
- After each tap, lights the LED of the sensor that shook the most, so you can see whether the nearest sensor wins.
- No UART. Watch `g_whoami`, `g_z`, `g_peak`, `g_first` and `g_winner` in Keil's Watch window.

Wiring and board jumper settings are at the top of `main.c`.
