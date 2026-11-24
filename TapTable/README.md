# TapTable (final version)

| File | Contents |
|---|---|
| `main.c` | hardware (clock, SPI, I2C, LCD, flash, buzzer), the 4 kHz sampling interrupt, screens, menus, modes |
| `tapf_core.h` | features, kNN and the calibration logic; no hardware code |
| `sktrl_data.h` | the built-in dataset: 600 taps from 5 people, 10 bytes per tap |

## Modes (joystick press opens the menu)

- **Play: 6 zones:** kNN with k = 9; 6 votes are needed. The zone's LED and note play, and the LCD shows the zone and its confidence, e.g. `Z3 confidence 8/9`. With fewer than 6 votes the tap is shown as "unsure" and makes no sound.
- **A/B/C (no ML):** A = Z1, B = Z5, C = Z3, the zones directly above the 3 sensors. A key counts only if the sensor with the most energy also reacted first, at least 0.75 ms before the other two.
- **Password:** a 6-digit PIN typed by tapping. Z1–Z5 = digits 1–5, Z6 = delete, joystick press = Enter. Three wrong PINs in a row lock it for 30 s.
- **Calibrate: merge / full:** 20 taps per zone. Merge checks whether the setup has changed, removes mis-taps and keeps at most 120 taps per zone. The result is saved in flash.
- **Settings:** Change PIN, Factory reset.

**Buttons:**
- TAMPER short press: merge calibration.
- TAMPER held 3 s: full calibration.
- TAMPER held through RESET: factory reset.
- WAKEUP: mute on/off.

## Results (each person tested as a stranger, 6 of 9 votes)

95.9% correct, 2.2% unsure, 1.9% wrong.
A/B/C mode: 97.0% correct, 0 wrong.

## Debugging

There is no UART in this build. To follow the board live, add these variables to Keil's Watch window:

| Variable | Meaning |
|---|---|
| `g_z[3]` | latest reading of each sensor |
| `g_peak[3]`, `g_energy[3]`, `g_onset[3]` | the last tap's measurements |
| `g_feat[9]` | the last tap's features |
| `g_nn_zone[9]` | zones of the 9 nearest stored taps |
| `g_zone_votes[6]` | votes for each zone |
| `g_zone` | the decision (0 = unsure) |
| `g_abc_strong`, `g_abc_lead`, `g_abc` | A/B/C mode's decision |
