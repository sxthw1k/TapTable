# Stage 2: data collection

`main.c` (board):

- Runs at 168 MHz from the 25 MHz crystal.
- TIM2 interrupt at 4 kHz: reads all 3 sensors and keeps the last 64 ms in a ring buffer.
- Threshold = 5 × the measured noise.
- Each tap is captured as 5 ms before + 15 ms after the trigger, then a 150 ms dead time.
- Sends one CSV line per tap over USART1 (DB9, 115200 baud): peak, energy, onset and peak time for each sensor.

`logger.py` (laptop) saves these lines to a CSV file, labelled with the zone you were tapping. This data was used to choose the features and the classifier.

```
pip install pyserial numpy
python logger.py --list
python logger.py --port COM5 --out taps.csv
```

(`--demo` mode was added later and needs `demo_eval.py`, which is not part of this stage.)
