# TapTable

TapTable turns a flat pad into a 6-zone touch surface. Three MPU6500 accelerometers sit under the pad. When you tap it, the board works out which zone was hit from how strongly, and how soon, each sensor feels the tap. It then lights that zone's LED and plays its note.

EEE G512 (Embedded Systems Design) course project by Sathwik Kumble and Krishna Meera.

> **Final project code: the [`TapTable/`](TapTable) folder.** The other folders are earlier stages, kept to show how the project developed.

**Hardware:** Keil MCBSTM32F400 board (STM32F407IG, 168 MHz), 3 × MPU6500 on SPI3, buzzer on PF8 (TIM13 PWM), the board's LCD and joystick (STMPE811 on I2C1).

## Folders

| Folder | What it is |
|---|---|
| `feasibility/` | First test: do the 3 sensors answer over SPI, and does the sensor nearest a tap shake the most? |
| `stage_2/` | Data collection: the board captures every tap at 4 kHz and sends its measurements over UART; `logger.py` saves them on the laptop, labelled by zone. |
| `stage_3/` | First on-board model: calibrate with 20 taps per zone (120 taps, stored in flash), then kNN (k = 5, 3 votes needed) picks the zone. |
| `TapTable/` | **Final project code.** 9 features, a built-in 600-tap dataset from 5 people, kNN (6 of 9 votes needed), an A/B/C mode with no machine learning, a 6-digit password typed by tapping, merge/full recalibration, and LCD + joystick menus. |

Each folder is a separate program with its own `README.md`. Every program is flashed on its own.

## Building any stage in Keil µVision 5

1. Create a new project. For the device, choose **STM32F407IGHx**.
2. In Manage Run-Time Environment, tick **CMSIS > CORE** and **Device > Startup**.
   - For `TapTable/` only, also tick **Board Support > Graphic LCD** (MCBSTM32F400) and press **Resolve**, which adds the STM32Cube HAL parts the LCD needs.
3. Add the folder's `main.c` to Source Group 1. The `.h` files in `TapTable/` are included by `main.c`, so you don't add them.
4. Under Options > Debug, choose **ULINK2/ME Cortex Debugger** (or your adapter). Under Utilities > Settings > Flash Download, choose **Erase Sectors**, **Program** and **Verify**.
5. Build (F7), then Download (F8). Press RESET on the board.

The wiring is listed at the top of each `main.c`.
