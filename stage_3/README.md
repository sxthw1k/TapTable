# Stage 3: first on-board model (120-tap calibration)

The first version that recognises zones on the board itself.

- **Calibration** (press TAMPER): the board asks for each of the 6 zones in turn and stores 20 taps per zone, 120 taps in total. They are saved in flash (sector 11), so they survive power-off.
- **Features:** 12 per tap (peak share, energy share, onset delay and peak time for each sensor), standardised.
- **Classifier:** kNN with k = 5. A zone needs at least 3 of the 5 votes; otherwise the tap is ignored.
- **Output:** the zone's LED and note. Harder taps are louder.
- UART output from Stage 2 is still sent, so `logger.py` still works.

The final version (`../TapTable`) replaced the single person's 120 taps with a 600-tap dataset from 5 people. With only 120 taps from 1 person, new users got many more wrong answers.
