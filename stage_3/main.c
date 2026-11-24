/*
 * TapTable - Stage 3: on-board kNN, calibration in flash, tones per zone
 * Board : Keil MCBSTM32F400 (STM32F407IG). Wiring: final build
 *         (CS1 PF6, CS2 PF7, CS3 PF10, buzzer PF8 via 100 ohm).
 *
 * WHAT IT DOES
 *   - Samples the 3 sensors at 4 kHz and captures each tap (as Stage 2).
 *   - Turns each tap into the same 12 features as analyze.py:
 *       peak share x3, energy share x3, onset (ms after first sensor) x3,
 *       peak time (ms after first onset) x3.
 *   - CALIBRATION (press TAMPER): the board asks for each zone in turn and
 *     stores g_cal_per_zone taps (default 20) per zone. The zone being
 *     asked for has its LED on. Each accepted tap gives a short tick;
 *     a double beep announces the next zone; a rising 3-note tune means
 *     calibration is done and SAVED IN FLASH (survives power-off and
 *     re-flashing). Press TAMPER again during calibration to cancel.
 *   - NORMAL USE: each tap is classified with kNN (k = 5) against the
 *     stored calibration taps (features standardised first). The zone needs
 *     at least 3 of the 5 nearest taps (60 %) or the tap is ignored.
 *     Then: that zone's LED lights and the buzzer plays that zone's note,
 *     louder for harder taps (up to g_volume).
 *   - No calibration stored yet: it beeps the strongest sensor's note
 *     instead (like the hardware test) until you calibrate.
 *
 * ZONES, LEDS AND NOTES
 *   Z1 front left   PG6  C5 523 Hz      Z4 back left    PH3  G5 784 Hz
 *   Z2 front middle PG7  D5 587 Hz      Z5 back middle  PH6  A5 880 Hz
 *   Z3 front right  PG8  E5 659 Hz      Z6 back right   PH2  C6 1047 Hz
 *   ("front" = the long edge with S1 and S3.)  Heartbeat: PH7.
 *
 * BUTTONS
 *   WAKEUP : mute on/off (hold during RESET to start muted)
 *   TAMPER : start calibration / cancel calibration
 *
 * UART (115200, DB9): the usual "T,..." tap lines (logger.py still works)
 *   plus info lines starting with '#', e.g.
 *     # zone 3  votes 5/5  vol 6
 *     # CAL zone 2: 7/20
 *
 * WATCH (optional): g_zone (last zone 1..6, 0 = rejected), g_votes,
 *   g_strength, g_cal_valid, g_cal_n, g_mode (0 run, 1 calibrating),
 *   g_cal_zone, g_cal_count, g_cal_per_zone (10..30), g_volume (max
 *   loudness %, 1..50, default 8), g_mute, g_rejects.
 * ------------------------------------------------------------------
 * Based on the Stage 2 / hardware-test code below.
 */
/*
 * TapTable - Hardware test: 3 sensors + buzzer (quiet)
 *
 * Same as Stage 2 (sampling, tap capture, UART lines) PLUS a short, quiet
 * beep on every tap. Pitch tells you which sensor won:
 *     S1 -> low (523 Hz)   S2 -> middle (659 Hz)   S3 -> high (784 Hz)
 * At start-up it gives one short beep per sensor that answered.
 * The buzzer is NEVER on for more than g_beep_ms (80 ms by default).
 *
 * BUZZER CONTROLS (Keil Watch window, change live):
 *   g_mute      1 = silent (LEDs and UART still work), 0 = beeps on
 *
 * PIN TEST MODE (no multimeter needed):
 *   Press TAMPER during the first 2 seconds after reset (while the three
 *   LEDs are lit), or hold it through reset. All four LEDs then flash
 *   together quickly, and the
 *   chip-select pins play tones in short bursts (0.15 s on, 0.85 s off):
 *       PF6 -> 440 Hz (low)   PF7 -> 660 Hz (middle)   PD3 -> 880 Hz (high)
 *   Pull the PF8 wire out of breadboard row 21, then plug the breadboard
 *   end of the wire you want to test into row 21. If you hear its tone,
 *   that pin, that wire and both of its contacts are good.
 *   Press RESET (without TAMPER) to go back to normal.
 *
 * MUTE BUTTONS (no debugger needed):
 *   Press WAKEUP (PA0) on the board to toggle mute.
 *   Un-muting plays one short beep so you know sound is back.
 *   Hold WAKEUP while pressing RESET to start muted (skips the start-up chirps).
 *   Board wiring (schematic sheet 10): WAKEUP pulls PA0 to 3V3 when pressed
 *   (220k pull-down otherwise). TAMPER (PC13) is not used here.
 *   g_volume    loudness as PWM duty in percent, 1..50 (default 4 = quiet)
 *   g_beep_ms   beep length in ms (default 60; capped at 200)
 *
 * Buzzer wiring: PF8 -> 100 ohm -> buzzer + ; buzzer - -> GND
 * PF8 is TIM13 channel 1 (alternate function 9). Timer ticks at 1 MHz,
 * so the period register ARR = 1,000,000 / frequency - 1 and the
 * on-time CCR1 = (ARR + 1) x volume / 100.
 *
 * ------------------------------------------------------------------
 * (Original Stage 2 description follows.)
 * TapTable - Stage 2: data pipeline + UART logging
 * Board : Keil MCBSTM32F400 (STM32F407IG, 25 MHz crystal)
 * Parts : 3 x MPU6500 on SPI3 (same wiring as the feasibility test)
 *
 * This is a NEW program. The feasibility test (TapTable_Test/main.c) is
 * unchanged; keep it in its own Keil project.
 *
 * ------------------------------------------------------------------
 * WHAT IT DOES
 *   1. Runs the CPU at 168 MHz from the 25 MHz crystal (falls back to the
 *      16 MHz internal clock if the crystal does not start; see g_clock_hz).
 *   2. A timer interrupt (TIM2) fires 4000 times a second. Each time it
 *      reads the Z acceleration of all 3 sensors over SPI and stores it in
 *      a ring buffer that always holds the last 64 ms.
 *   3. At start-up it learns each sensor's resting value, measures the
 *      noise for 1 s and sets the tap threshold to 5 x noise automatically.
 *   4. When any sensor crosses the threshold, it keeps 5 ms BEFORE and
 *      15 ms AFTER the trigger (80 samples per sensor), then ignores new
 *      triggers for 150 ms while the pad stops ringing.
 *   5. The main loop turns each captured tap into numbers (peak, energy,
 *      onset time, peak time, clipping flag per sensor) and sends one CSV
 *      line to the laptop over USART1 (DB9 port, 115200 baud).
 *   6. The LED of the sensor with the most energy lights (PG6/PG7/PG8),
 *      PH2 blinks as a heartbeat.
 *
 * ------------------------------------------------------------------
 * WIRING: identical to the feasibility test
 *   3V3 -> + rail, GND -> - rail, PC10 -> SCL, PC12 -> SDA, PC11 -> AD0,
 *   PF6 -> S1 NCS, PF7 -> S2 NCS, PF10 -> S3 NCS
 *   Serial: board DB9 (USART1, PB6 TX / PB7 RX through the RS-232 chip)
 *           -> USB-RS232 adapter -> laptop
 *
 * BOARD SETTINGS
 *   - No SD card; J13/J14 on TX1/RX1 side (also routes USART1 to the DB9)
 *   - J18 (LED_EN) fitted; BOOT0 to GND
 *
 * ------------------------------------------------------------------
 * UART OUTPUT (one line per tap; lines starting with # are info)
 *   T,n,rng,thr,p1,p2,p3,e1,e2,e3,o1,o2,o3,k1,k2,k3,sat,first,win
 *     n      tap number since reset
 *     rng    range in g (8 or 16)
 *     thr    trigger threshold in counts
 *     p1..3  peak |deviation| per sensor, counts
 *     e1..3  energy per sensor: sum of deviation^2 over the window / 1024
 *     o1..3  onset: first sample (0..79) where |dev| > thr/2, 255 = never
 *     k1..3  sample (0..79) of each sensor's peak
 *            (the trigger itself is at sample 20; one sample = 0.25 ms)
 *     sat    bit mask: bit0 = S1 clipped, bit1 = S2, bit2 = S3
 *     first  sensor (1..3) that crossed the threshold first
 *     win    sensor (1..3) with the largest energy
 *
 * ------------------------------------------------------------------
 * WATCH IN KEIL (optional; the UART log is the main output)
 *   g_clock_hz    168000000 normally; 16000000 = crystal did not start
 *   g_whoami[3]   0x70 per sensor (0x00/0xFF = not answering)
 *   g_cfg[3]      range register read back: 24 (0x18) = 16 g, 16 (0x10) = 8 g,
 *                 0 = the write did not stick (sensor still at 2 g)
 *   g_base[3]     resting reading: about +/-2048 at 16 g, +/-4096 at 8 g,
 *                 +/-16384 would mean the sensor is still at 2 g
 *   g_noise[3]    rest noise per sensor (largest wiggle in 1 s), counts
 *   g_thresh      trigger level in counts (auto = 5 x noise, editable live)
 *   g_range_req   type 8 or 16 to switch range live; it re-learns rest
 *                 values and noise, and sets a new automatic threshold
 *   g_range       range now in use (8 or 16)
 *   g_taps        taps captured
 *   g_sat_taps    taps where at least one sensor clipped
 *   g_isr_max_us  longest sampling interrupt so far, microseconds (< 250)
 *   g_overruns    samples where the interrupt ran late (should stay 0)
 * ------------------------------------------------------------------
 */

#include "stm32f4xx.h"
#include <stdint.h>

/* ------------------------------ settings ---------------------------- */
#define NUM_SENSORS     3
#define FS_HZ           4000u    /* samples per second, per sensor         */
#define RING            256u     /* ring buffer length: 64 ms at 4 kHz     */
#define PRE             20u      /* samples kept before the trigger: 5 ms  */
#define POST            60u      /* samples kept after the trigger: 15 ms  */
#define WIN             (PRE + POST)
#define DEAD_SAMPLES    600u     /* 150 ms with no new trigger             */
#define BASE_SAMPLES    512u     /* 128 ms to learn the resting value      */
#define NOISE_SAMPLES   4000u    /* 1 s to measure noise                   */
#define THRESH_FACTOR   5        /* threshold = 5 x noise                  */
#define THRESH_MIN      200      /* never below this (counts)              */
#define DEFAULT_RANGE_G 16       /* 16 avoids clipping on hard taps; 8 = finer */
#define CLIP_LEVEL      32700    /* |raw| at or above this = clipped       */
#define BAUD            115200u

/* MPU6500 registers */
#define REG_ACCEL_CONFIG   0x1Cu
#define REG_ACCEL_CONFIG2  0x1Du
#define REG_ACCEL_ZOUT_H   0x3Fu
#define REG_USER_CTRL      0x6Au
#define REG_PWR_MGMT_1     0x6Bu
#define REG_PWR_MGMT_2     0x6Cu
#define REG_WHO_AM_I       0x75u

/* ---------------------- variables for the Watch window --------------- */
volatile uint32_t g_ms;
volatile uint32_t g_clock_hz;
volatile uint8_t  g_whoami[NUM_SENSORS];
volatile uint8_t  g_cfg[NUM_SENSORS];       /* range register read back: 24 = 16 g, 16 = 8 g */
volatile int32_t  g_base[NUM_SENSORS];      /* resting value: ~2048 at 16 g, ~4096 at 8 g    */
volatile int32_t  g_noise[NUM_SENSORS];
volatile int32_t  g_thresh = 600;
volatile int32_t  g_range_req = DEFAULT_RANGE_G;
volatile int32_t  g_range;
volatile uint32_t g_taps;
volatile uint32_t g_sat_taps;
volatile uint32_t g_isr_max_us;
volatile uint32_t g_overruns;
volatile int32_t  g_peak[NUM_SENSORS];      /* last tap, for a quick look */
volatile uint8_t  g_winner;
extern volatile uint8_t  g_cal_valid;        /* defined in the Stage 3 section */
extern volatile uint32_t g_cal_n;
volatile uint8_t  g_mute     = 0;           /* 1 = no sound                          */
volatile uint32_t g_volume   = 8;           /* PWM duty %, 1..50 (50 = loudest)      */
volatile uint32_t g_beep_ms  = 80;          /* beep length, ms (max 200)             */

static volatile uint8_t  s_beeping;         /* 1 while a beep is playing             */
static volatile uint32_t s_beep_end;        /* g_ms time when it must stop           */
static const uint32_t    tone_hz[NUM_SENSORS] = { 523u, 659u, 784u };  /* C5, E5, G5 */

/* --------------------------- pin tables ----------------------------- */
static GPIO_TypeDef * const cs_port[NUM_SENSORS] = { GPIOF, GPIOF, GPIOF };
static const uint8_t        cs_pin[NUM_SENSORS]  = { 6u, 7u, 10u };   /* PF6, PF7, PF10 */
static const uint8_t        led_pin[NUM_SENSORS] = { 6u, 7u, 8u };

/* ------------------------ clocks (filled at start) ------------------ */
static uint32_t pclk1_hz, pclk2_hz, tim2_hz;
static uint32_t spi_div_cfg, spi_div_read;   /* SPI BR codes */

/* ------------------- sampling state (shared with ISR) --------------- */
enum { ST_OFF, ST_BASE, ST_NOISE, ST_IDLE, ST_POST, ST_DEAD };

static volatile uint8_t  s_state = ST_OFF;
static volatile uint32_t s_count;                 /* samples since start  */
static int16_t           s_ring[NUM_SENSORS][RING];
static int32_t           s_base_fx[NUM_SENSORS];  /* rest value x 64      */
static int32_t           s_base_sum[NUM_SENSORS];
static int32_t           s_noise_max[NUM_SENSORS];
static uint32_t          s_phase_n;               /* samples in this phase */
static uint32_t          s_trig_count;
static uint8_t           s_trig_first;
static int32_t           s_trig_base[NUM_SENSORS];
static uint8_t           s_sensor_ok[NUM_SENSORS];

/* captured tap, handed from ISR to main loop */
static volatile uint8_t  s_cap_ready;
static int16_t           s_cap[NUM_SENSORS][WIN];
static int32_t           s_cap_base[NUM_SENSORS];
static uint8_t           s_cap_first;
static int32_t           s_cap_thresh;

/* ============================ time base ============================= */
void SysTick_Handler(void) { g_ms++; }

static void delay_ms(uint32_t ms)
{
    uint32_t t0 = g_ms;
    while ((g_ms - t0) < ms) { }
}

/* ============================== clock =============================== */
/* 25 MHz crystal -> PLL -> 168 MHz. APB1 = 42 MHz, APB2 = 84 MHz.
   If the crystal does not start, stay on the 16 MHz internal clock.   */
static void clock_init(void)
{
    uint32_t t;

    RCC->CR |= RCC_CR_HSEON;
    for (t = 0; t < 500000u && !(RCC->CR & RCC_CR_HSERDY); t++) { }

    if (RCC->CR & RCC_CR_HSERDY) {
        RCC->APB1ENR |= RCC_APB1ENR_PWREN;
        (void)RCC->APB1ENR;
        PWR->CR |= PWR_CR_VOS;                          /* full-speed mode */

        FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN |
                     FLASH_ACR_LATENCY_5WS;

        RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2))
                  | RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;

        RCC->PLLCFGR = (25u  << RCC_PLLCFGR_PLLM_Pos) |   /* 25 MHz / 25 = 1 MHz */
                       (336u << RCC_PLLCFGR_PLLN_Pos) |   /* x 336 = 336 MHz     */
                       (0u   << RCC_PLLCFGR_PLLP_Pos) |   /* / 2  = 168 MHz      */
                       (7u   << RCC_PLLCFGR_PLLQ_Pos) |   /* / 7  = 48 MHz (USB) */
                       RCC_PLLCFGR_PLLSRC_HSE;

        RCC->CR |= RCC_CR_PLLON;
        for (t = 0; t < 500000u && !(RCC->CR & RCC_CR_PLLRDY); t++) { }

        if (RCC->CR & RCC_CR_PLLRDY) {
            RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
            while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) { }
            g_clock_hz   = 168000000u;
            pclk1_hz     = 42000000u;
            pclk2_hz     = 84000000u;
            tim2_hz      = 84000000u;   /* APB1 divided, so timers run at 2 x */
            spi_div_cfg  = 6u;          /* 42 MHz / 128 = 328 kHz (< 1 MHz)   */
            spi_div_read = 4u;          /* 42 MHz / 32  = 1.3 MHz, safe on long wires */
            return;
        }
        /* PLL failed: fall back below with safe dividers */
        RCC->CFGR &= ~(RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
    }

    g_clock_hz   = 16000000u;           /* internal clock */
    pclk1_hz     = 16000000u;
    pclk2_hz     = 16000000u;
    tim2_hz      = 16000000u;
    spi_div_cfg  = 5u;                  /* 16 MHz / 64 = 250 kHz */
    spi_div_read = 3u;                  /* 16 MHz / 16 = 1 MHz   */
}

/* =============================== GPIO =============================== */
/* Microsecond delay using the CPU cycle counter (DWT), enabled in main().
   At 168 MHz the CPU is so fast that a chip-select could go high and low
   again within a few tens of nanoseconds; on long breadboard wires the
   sensor may miss that gap and merge two commands into one. These short
   pauses give every chip-select edge time to settle. */
static void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (g_clock_hz / 1000000u);
    while ((DWT->CYCCNT - start) < ticks) { }
}

static void cs_low(int s)  { cs_port[s]->BSRR = (1UL << (cs_pin[s] + 16u)); delay_us(1); }
static void cs_high(int s) { cs_port[s]->BSRR = (1UL << cs_pin[s]); delay_us(2); }

static void led_set(int s, int on)
{
    if (on) GPIOG->BSRR = (1UL << led_pin[s]);
    else    GPIOG->BSRR = (1UL << (led_pin[s] + 16u));
}

static void gpio_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN |
                    RCC_AHB1ENR_GPIOFEN | RCC_AHB1ENR_GPIOGEN |
                    RCC_AHB1ENR_GPIOHEN;
    (void)RCC->AHB1ENR;

    for (int s = 0; s < NUM_SENSORS; s++) {          /* CS high, then output */
        cs_high(s);
        cs_port[s]->MODER &= ~(3UL << (cs_pin[s] * 2u));
        cs_port[s]->MODER |=  (1UL << (cs_pin[s] * 2u));
    }
    for (int s = 0; s < NUM_SENSORS; s++) {          /* LEDs PG6-PG8 */
        GPIOG->MODER &= ~(3UL << (led_pin[s] * 2u));
        GPIOG->MODER |=  (1UL << (led_pin[s] * 2u));
    }
    for (uint32_t p = 2u; p <= 7u; p++) {            /* PH2, PH3, PH6, PH7 LEDs */
        if (p == 4u || p == 5u) continue;
        GPIOH->MODER &= ~(3UL << (p * 2u));
        GPIOH->MODER |=  (1UL << (p * 2u));
    }

    for (uint32_t p = 10u; p <= 12u; p++) {          /* PC10-12 = SPI3, AF6 */
        GPIOC->MODER   &= ~(3UL   << (p * 2u));
        GPIOC->MODER   |=  (2UL   << (p * 2u));
        GPIOC->OSPEEDR &= ~(3UL   << (p * 2u));
        GPIOC->OSPEEDR |=  (2UL   << (p * 2u));
        GPIOC->AFR[1]  &= ~(0xFUL << ((p - 8u) * 4u));
        GPIOC->AFR[1]  |=  (6UL   << ((p - 8u) * 4u));
    }
    GPIOC->PUPDR &= ~(3UL << (11u * 2u));            /* pull-up on MISO */
    GPIOC->PUPDR |=  (1UL << (11u * 2u));

    for (uint32_t p = 6u; p <= 7u; p++) {            /* PB6 TX, PB7 RX = USART1, AF7 */
        GPIOB->MODER  &= ~(3UL   << (p * 2u));
        GPIOB->MODER  |=  (2UL   << (p * 2u));
        GPIOB->AFR[0] &= ~(0xFUL << (p * 4u));
        GPIOB->AFR[0] |=  (7UL   << (p * 4u));
    }
}

/* =============================== SPI ================================ */
static void spi_wait_idle(void) { while (SPI3->SR & SPI_SR_BSY) { } }

static void spi_set_divider(uint32_t code)
{
    spi_wait_idle();
    SPI3->CR1 &= ~SPI_CR1_SPE;
    SPI3->CR1 = (SPI3->CR1 & ~SPI_CR1_BR) | (code << SPI_CR1_BR_Pos);
    SPI3->CR1 |= SPI_CR1_SPE;
}

static void spi_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
    (void)RCC->APB1ENR;
    SPI3->CR1 = 0;
    SPI3->CR2 = 0;
    SPI3->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI |
                SPI_CR1_CPOL | SPI_CR1_CPHA | (spi_div_cfg << SPI_CR1_BR_Pos);
    SPI3->CR1 |= SPI_CR1_SPE;
}

static uint8_t spi_xfer(uint8_t out)
{
    while (!(SPI3->SR & SPI_SR_TXE)) { }
    SPI3->DR = out;
    while (!(SPI3->SR & SPI_SR_RXNE)) { }
    return (uint8_t)SPI3->DR;
}

/* ============================= MPU6500 ============================== */
static uint8_t mpu_read(int s, uint8_t reg)
{
    uint8_t v;
    cs_low(s);
    spi_xfer((uint8_t)(reg | 0x80u));
    v = spi_xfer(0x00u);
    spi_wait_idle();
    cs_high(s);
    return v;
}

static void mpu_write(int s, uint8_t reg, uint8_t val)
{
    cs_low(s);
    spi_xfer((uint8_t)(reg & 0x7Fu));
    spi_xfer(val);
    spi_wait_idle();
    cs_high(s);
}

static int16_t mpu_read_z(int s)
{
    uint8_t hi, lo;
    cs_low(s);
    spi_xfer((uint8_t)(REG_ACCEL_ZOUT_H | 0x80u));
    hi = spi_xfer(0x00u);
    lo = spi_xfer(0x00u);
    spi_wait_idle();
    cs_high(s);
    return (int16_t)(((uint16_t)hi << 8) | lo);
}

static uint8_t range_bits(int32_t g)   /* ACCEL_CONFIG value for a range */
{
    return (g >= 16) ? 0x18u : 0x10u;  /* 0x18 = +/-16 g, 0x10 = +/-8 g */
}

/* Write the range and read it back; retry up to 5 times. g_cfg shows the result. */
static void mpu_set_range_verified(int s, int32_t range_g)
{
    uint8_t want = range_bits(range_g);
    uint8_t got  = 0;
    for (int tries = 0; tries < 5; tries++) {
        mpu_write(s, REG_ACCEL_CONFIG, want);
        delay_ms(2);
        got = mpu_read(s, REG_ACCEL_CONFIG);
        if ((got & 0x18u) == want) break;
    }
    g_cfg[s] = got;
}

static void mpu_init(int s, int32_t range_g)
{
    mpu_write(s, REG_PWR_MGMT_1, 0x80u);    /* reset                        */
    delay_ms(100);
    mpu_write(s, REG_USER_CTRL, 0x10u);     /* SPI only                     */
    mpu_write(s, REG_PWR_MGMT_1, 0x01u);    /* wake, best clock             */
    delay_ms(10);
    mpu_write(s, REG_PWR_MGMT_2, 0x07u);    /* gyro off, accel on           */
    mpu_write(s, REG_ACCEL_CONFIG2, 0x08u); /* 4 kHz output, no filter      */
    mpu_set_range_verified(s, range_g);     /* +/-16 g (or 8 g), checked    */
    delay_ms(20);
}

/* ============================== UART ================================ */
static void uart_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    (void)RCC->APB2ENR;
    USART1->CR1 = 0;
    USART1->BRR = (pclk2_hz + BAUD / 2u) / BAUD;   /* 84 MHz -> 729 */
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

static void uart_putc(char c)
{
    while (!(USART1->SR & USART_SR_TXE)) { }
    USART1->DR = (uint8_t)c;
}

static void uart_puts(const char *s) { while (*s) uart_putc(*s++); }

static void uart_putu(uint32_t v)
{
    char b[11];
    int  i = 0;
    do { b[i++] = (char)('0' + (v % 10u)); v /= 10u; } while (v && i < 10);
    while (i) uart_putc(b[--i]);
}

static void uart_puti(int32_t v)
{
    if (v < 0) { uart_putc('-'); uart_putu((uint32_t)(-v)); }
    else       { uart_putu((uint32_t)v); }
}

static void uart_field(int32_t v) { uart_putc(','); uart_puti(v); }

/* ====================== sampling timer (TIM2) ======================= */
static void tim2_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    (void)RCC->APB1ENR;
    TIM2->CR1  = 0;
    TIM2->PSC  = 0;
    TIM2->ARR  = tim2_hz / FS_HZ - 1u;         /* 84 MHz / 4000 = 21000 */
    TIM2->EGR  = TIM_EGR_UG;
    TIM2->SR   = 0;
    TIM2->DIER = TIM_DIER_UIE;
    NVIC_SetPriority(TIM2_IRQn, 0);            /* highest priority */
    NVIC_EnableIRQ(TIM2_IRQn);
    TIM2->CR1  = TIM_CR1_CEN;
}

/* Runs 4000 times a second: read all sensors, detect and capture taps */
void TIM2_IRQHandler(void)
{
    int16_t  z[NUM_SENSORS];
    int32_t  dev[NUM_SENSORS];
    uint32_t i = s_count & (RING - 1u);
    uint32_t t_start = TIM2->CNT;

    TIM2->SR = ~TIM_SR_UIF;                    /* clear the interrupt flag */

    for (int s = 0; s < NUM_SENSORS; s++) {
        z[s] = s_sensor_ok[s] ? mpu_read_z(s) : 0;
        s_ring[s][i] = z[s];
        dev[s] = (int32_t)z[s] - (s_base_fx[s] / 64);
        if (dev[s] < 0) dev[s] = -dev[s];
    }

    switch (s_state) {
    case ST_BASE:                              /* learn resting values */
        for (int s = 0; s < NUM_SENSORS; s++) s_base_sum[s] += z[s];
        if (++s_phase_n >= BASE_SAMPLES) {
            for (int s = 0; s < NUM_SENSORS; s++) {
                s_base_fx[s]   = (s_base_sum[s] / (int32_t)BASE_SAMPLES) * 64;
                s_noise_max[s] = 0;
            }
            s_phase_n = 0;
            s_state   = ST_NOISE;
        }
        break;

    case ST_NOISE:                             /* measure noise for 1 s */
        for (int s = 0; s < NUM_SENSORS; s++) {
            s_base_fx[s] += (int32_t)z[s] - (s_base_fx[s] / 64);
            if (dev[s] > s_noise_max[s]) s_noise_max[s] = dev[s];
        }
        if (++s_phase_n >= NOISE_SAMPLES) {
            int32_t worst = 0;
            for (int s = 0; s < NUM_SENSORS; s++) {
                g_noise[s] = s_noise_max[s];
                g_base[s]  = s_base_fx[s] / 64;
                if (s_noise_max[s] > worst) worst = s_noise_max[s];
            }
            worst *= THRESH_FACTOR;
            g_thresh  = (worst > THRESH_MIN) ? worst : THRESH_MIN;
            s_phase_n = 0;
            s_state   = ST_IDLE;
        }
        break;

    case ST_IDLE: {                            /* wait for a tap */
        int first = 0;
        for (int s = 0; s < NUM_SENSORS; s++) {
            s_base_fx[s] += (int32_t)z[s] - (s_base_fx[s] / 64);
            if (first == 0 && s_sensor_ok[s] && dev[s] > g_thresh) first = s + 1;
        }
        if (first && !s_cap_ready) {
            s_trig_count = s_count;
            s_trig_first = (uint8_t)first;
            for (int s = 0; s < NUM_SENSORS; s++) s_trig_base[s] = s_base_fx[s] / 64;
            s_state = ST_POST;
        }
        break;
    }

    case ST_POST:                              /* keep sampling 15 ms */
        if ((s_count - s_trig_count) >= (POST - 1u)) {
            uint32_t start = s_trig_count - PRE;
            for (int s = 0; s < NUM_SENSORS; s++) {
                for (uint32_t k = 0; k < WIN; k++) {
                    s_cap[s][k] = s_ring[s][(start + k) & (RING - 1u)];
                }
                s_cap_base[s] = s_trig_base[s];
            }
            s_cap_first  = s_trig_first;
            s_cap_thresh = g_thresh;
            s_cap_ready  = 1;
            s_phase_n    = 0;
            s_state      = ST_DEAD;
        }
        break;

    case ST_DEAD:                              /* let the pad stop ringing */
        if (++s_phase_n >= DEAD_SAMPLES) { s_phase_n = 0; s_state = ST_IDLE; }
        break;

    default:
        break;
    }

    s_count++;

    /* timing check: how long did this interrupt take? */
    {
        uint32_t t_end = TIM2->CNT;
        uint32_t ticks = (t_end >= t_start) ? (t_end - t_start)
                                            : (t_end + TIM2->ARR + 1u - t_start);
        uint32_t us = ticks / (tim2_hz / 1000000u);
        if (us > g_isr_max_us) g_isr_max_us = us;
        if (TIM2->SR & TIM_SR_UIF) g_overruns++;   /* next tick already due */
    }
}

/* ======================= start / restart sampling =================== */
static void start_learning(void)
{
    NVIC_DisableIRQ(TIM2_IRQn);
    for (int s = 0; s < NUM_SENSORS; s++) { s_base_sum[s] = 0; s_base_fx[s] = 0; }
    s_phase_n   = 0;
    s_cap_ready = 0;
    s_state     = ST_BASE;
    NVIC_EnableIRQ(TIM2_IRQn);
}

static void set_range(int32_t g)
{
    NVIC_DisableIRQ(TIM2_IRQn);
    spi_set_divider(spi_div_cfg);
    for (int s = 0; s < NUM_SENSORS; s++) {
        if (s_sensor_ok[s]) mpu_set_range_verified(s, g);
    }
    spi_set_divider(spi_div_read);
    g_range = (g >= 16) ? 16 : 8;
    NVIC_EnableIRQ(TIM2_IRQn);
}

static void print_info(void)
{
    uart_puts("# TapTable stage3 clock_hz="); uart_putu(g_clock_hz);
    uart_puts(" fs="); uart_putu(FS_HZ);
    uart_puts(" range_g="); uart_puti(g_range);
    uart_puts(" whoami=");
    for (int s = 0; s < NUM_SENSORS; s++) {
        if (s) uart_putc('/');
        uart_putu(g_whoami[s]);
    }
    uart_puts(" cfg=");
    for (int s = 0; s < NUM_SENSORS; s++) {
        if (s) uart_putc('/');
        uart_putu(g_cfg[s]);
    }
    uart_puts(" base=");
    for (int s = 0; s < NUM_SENSORS; s++) {
        if (s) uart_putc('/');
        uart_puti(g_base[s]);
    }
    uart_puts(" noise=");
    for (int s = 0; s < NUM_SENSORS; s++) {
        if (s) uart_putc('/');
        uart_puti(g_noise[s]);
    }
    uart_puts(" thresh="); uart_puti(g_thresh);
    uart_puts(g_cal_valid ? " calibration=loaded taps=" : " calibration=NONE (press TAMPER) taps=");
    uart_putu(g_cal_n);
    uart_puts("\r\n# T,n,rng,thr,p1,p2,p3,e1,e2,e3,o1,o2,o3,k1,k2,k3,sat,first,win\r\n");
}

/* ============================ tap features ========================== */
/* ============================== buzzer ============================== */
static void buzzer_init(void)
{
    /* PF8 = TIM13_CH1, alternate function 9 */
    GPIOF->MODER  &= ~(3UL   << (8u * 2u));
    GPIOF->MODER  |=  (2UL   << (8u * 2u));
    GPIOF->AFR[1] &= ~(0xFUL << ((8u - 8u) * 4u));
    GPIOF->AFR[1] |=  (9UL   << ((8u - 8u) * 4u));

    RCC->APB1ENR |= RCC_APB1ENR_TIM13EN;
    (void)RCC->APB1ENR;
    TIM13->CR1   = 0;
    TIM13->PSC   = tim2_hz / 1000000u - 1u;      /* same APB1 timer clock: tick = 1 us */
    TIM13->ARR   = 999u;
    TIM13->CCR1  = 0;                            /* 0 = output stays low = silent */
    TIM13->CCMR1 = (6u << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;   /* PWM mode 1 */
    TIM13->CCER  = TIM_CCER_CC1E;                /* connect channel 1 to the pin */
    TIM13->CR1   = TIM_CR1_ARPE;
    TIM13->EGR   = TIM_EGR_UG;
}

static void buzzer_off(void)
{
    TIM13->CCR1 = 0;                             /* duty 0: pin low, no sound */
    TIM13->EGR  = TIM_EGR_UG;
    TIM13->CR1 &= ~TIM_CR1_CEN;
    s_beeping = 0;
}

/* Start a short beep; it is switched off by buzzer_service() */
static void buzzer_beep(uint32_t hz, uint32_t ms)
{
    uint32_t vol = g_volume, arr;
    if (g_mute || hz == 0u) return;
    if (vol < 1u)  vol = 1u;
    if (vol > 50u) vol = 50u;                    /* 50 % is the loudest a piezo gets */
    if (ms > 200u) ms = 200u;                    /* never a long tone */
    arr = 1000000u / hz - 1u;
    TIM13->CR1 &= ~TIM_CR1_CEN;
    TIM13->ARR  = arr;
    TIM13->CCR1 = ((arr + 1u) * vol) / 100u;
    TIM13->EGR  = TIM_EGR_UG;                    /* load ARR and CCR1 now */
    TIM13->CR1 |= TIM_CR1_CEN;
    s_beep_end  = g_ms + ms;
    s_beeping   = 1;
}

/* Call often from the main loop: stops the beep when its time is up */
static void buzzer_service(void)
{
    if (s_beeping && (int32_t)(g_ms - s_beep_end) >= 0) buzzer_off();
    if (s_beeping && g_mute) buzzer_off();
}

/* ============================ mute buttons ========================== */
/* Mute uses the WAKEUP button only. TAMPER (PC13) is left free for later
   (for example to start calibration). */
static int button_pressed(void)
{
    return (GPIOA->IDR & (1UL << 0u)) != 0u;          /* PA0 high = WAKEUP pressed */
}

/* Call often from the main loop. Checks the buttons every 5 ms and needs
   6 equal readings in a row (30 ms) before accepting a change, so contact
   bounce cannot toggle mute twice. Each new press toggles g_mute. */
static int tamper_pressed(void) { return (GPIOC->IDR & (1UL << 13u)) == 0u; }  /* PC13 low */

typedef struct { int last, stable, same; } debounce_t;

/* 6 equal reads 5 ms apart = 30 ms; returns 1 once per new press */
static int debounce(debounce_t *d, int raw)
{
    if (raw == d->last) { if (d->same < 6) d->same++; }
    else                { d->same = 0; d->last = raw; }
    if (d->same >= 6 && raw != d->stable) { d->stable = raw; return raw; }
    return 0;
}

static void calibration_toggle(void);   /* defined further down */

static void buttons_service(void)
{
    static uint32_t   t_last;
    static debounce_t wk, tp;
    if ((g_ms - t_last) < 5u) return;
    t_last = g_ms;
    if (debounce(&wk, button_pressed())) {          /* WAKEUP: mute */
        g_mute = g_mute ? 0u : 1u;
        if (g_mute) buzzer_off();
        else        buzzer_beep(659u, 40u);          /* "sound is on" beep */
    }
    if (debounce(&tp, tamper_pressed())) {          /* TAMPER: calibration */
        calibration_toggle();
    }
}

/* Blocking short beep, only used at start-up */
static void buzzer_chirp(uint32_t hz)
{
    buzzer_beep(hz, 50u);
    while (s_beeping) buzzer_service();
    delay_ms(120);
}

/* ======================= Stage 3: features ========================= */
#define NF          12                 /* features per tap                */
#define NZ          6                  /* zones                           */
#define KNN_K       5
#define MIN_VOTES   3                  /* 3 of 5 = 60 % or the tap is ignored */
#define MAXCAL      (NZ * 30)          /* up to 30 taps per zone          */
#define CAL_ADDR    0x080E0000u        /* flash sector 11 (last 128 KB)   */
#define CAL_MAGIC   0x54415033u        /* "TAP3"                          */

static const uint32_t zone_hz[NZ] = { 523u, 587u, 659u, 784u, 880u, 1047u };
static GPIO_TypeDef * const zone_led_port[NZ] = { GPIOG, GPIOG, GPIOG, GPIOH, GPIOH, GPIOH };
static const uint32_t       zone_led_pin[NZ]  = { 6u, 7u, 8u, 3u, 6u, 2u };

volatile uint8_t  g_mode;              /* 0 = run, 1 = calibrating        */
volatile uint8_t  g_zone;              /* last zone 1..6, 0 = rejected    */
volatile uint8_t  g_votes;             /* votes for that zone (of 5)      */
volatile uint32_t g_strength;          /* sum of the three peaks          */
volatile uint8_t  g_cal_valid;         /* 1 = a calibration is loaded     */
volatile uint32_t g_cal_n;             /* taps in the loaded calibration  */
volatile uint8_t  g_cal_zone;          /* zone being calibrated (1..6)    */
volatile uint32_t g_cal_count;         /* taps so far in that zone        */
volatile uint32_t g_cal_per_zone = 20; /* taps per zone (10..30)          */
volatile uint32_t g_rejects;           /* taps ignored as unsure          */

typedef struct {
    uint32_t magic;
    uint32_t n;                        /* number of stored taps           */
    float    x[MAXCAL][NF];            /* their features                  */
    uint8_t  y[MAXCAL];                /* their zones 0..5 (MAXCAL is a multiple of 4,
                                          so the struct is a whole number of words) */
} cal_t;

static cal_t s_cal;                    /* calibration in use              */
static cal_t s_new;                    /* calibration being recorded      */
static float s_mu[NF], s_sd[NF];       /* mean and spread of each feature */

/* The same 12 features as analyze.py */
static void make_features(const int32_t *p, const uint32_t *e, const int32_t *o,
                          const int32_t *k, float *f)
{
    float ps = (float)(p[0] + p[1] + p[2]);
    float es = (float)e[0] + (float)e[1] + (float)e[2];
    float on[NUM_SENSORS], t0;
    if (ps < 1.0f) ps = 1.0f;
    if (es < 1.0f) es = 1.0f;
    for (int s = 0; s < NUM_SENSORS; s++) on[s] = (o[s] >= 255) ? (float)WIN : (float)o[s];
    t0 = on[0];
    if (on[1] < t0) t0 = on[1];
    if (on[2] < t0) t0 = on[2];
    for (int s = 0; s < NUM_SENSORS; s++) {
        f[s]     = (float)p[s] / ps;                   /* peak share     */
        f[3 + s] = (float)e[s] / es;                   /* energy share   */
        f[6 + s] = (on[s] - t0) * 0.25f;               /* onset, ms      */
        f[9 + s] = ((float)k[s] - t0) * 0.25f;         /* peak time, ms  */
    }
}

/* Small maths helpers (no math library needed) */
static float sqrt_f(float x)                 /* square root, FPU instruction */
{
    return (x > 0.0f) ? __builtin_sqrtf(x) : 0.0f;
}

static float log2_f(float x)                 /* rough log2, good to ~0.1 */
{
    union { float f; uint32_t u; } v;
    v.f = x;
    return (float)((int32_t)((v.u >> 23) & 0xFFu) - 127) +       /* exponent */
           (float)(v.u & 0x7FFFFFu) / 8388608.0f;                 /* + mantissa */
}

/* mean and standard deviation of each feature over the calibration taps */
static void compute_stats(const cal_t *c)
{
    for (int j = 0; j < NF; j++) {
        float m = 0.0f, v = 0.0f;
        for (uint32_t i = 0; i < c->n; i++) m += c->x[i][j];
        m /= (float)c->n;
        for (uint32_t i = 0; i < c->n; i++) { float d = c->x[i][j] - m; v += d * d; }
        v /= (float)c->n;
        s_mu[j] = m;
        s_sd[j] = (v > 1e-12f) ? sqrt_f(v) : 1.0f;
    }
}

/* =========================== LEDs and sound ========================= */
static void zone_leds(int z)          /* z = 0..5, or -1 for all off */
{
    for (int i = 0; i < NZ; i++) {
        if (i == z) zone_led_port[i]->BSRR = (1UL << zone_led_pin[i]);
        else        zone_led_port[i]->BSRR = (1UL << (zone_led_pin[i] + 16u));
    }
}

/* harder tap -> louder, on a log scale, between 1 % and g_volume % */
static uint32_t volume_for(uint32_t strength)
{
    float lo = 10.0f, hi = 15.0f;     /* log2 of 1,000 and 32,000 counts */
    float x  = (strength > 1u) ? log2_f((float)strength) : 0.0f;
    float t  = (x - lo) / (hi - lo);
    uint32_t vmax = (g_volume > 50u) ? 50u : g_volume;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    if (vmax < 1u) vmax = 1u;
    return 1u + (uint32_t)(t * (float)(vmax - 1u) + 0.5f);
}

static void play(uint32_t hz, uint32_t vol, uint32_t ms)
{
    uint32_t keep = g_volume;
    g_volume = vol;
    buzzer_beep(hz, ms);
    g_volume = keep;
}

static void wait_beep(void) { while (s_beeping) buzzer_service(); }

/* ============================== flash =============================== */
static int cal_ok(const cal_t *c)
{
    if (c->magic != CAL_MAGIC || c->n < NZ * 5u || c->n > MAXCAL) return 0;
    for (uint32_t i = 0; i < c->n; i++) if (c->y[i] >= NZ) return 0;
    return 1;
}

static void cal_load(void)
{
    const cal_t *f = (const cal_t *)CAL_ADDR;
    g_cal_valid = 0;
    if (!cal_ok(f)) return;
    s_cal = *f;
    compute_stats(&s_cal);
    g_cal_n = s_cal.n;
    g_cal_valid = 1;
}

/* Erase sector 11 and write the calibration. Takes about 1-2 s; all
   interrupts are paused, so no taps are sampled meanwhile. */
static int cal_save(const cal_t *c)
{
    const uint32_t *src = (const uint32_t *)c;
    uint32_t words = sizeof(cal_t) / 4u;
    int ok = 1;

    __disable_irq();
    FLASH->KEYR = 0x45670123u;                     /* unlock */
    FLASH->KEYR = 0xCDEF89ABu;
    while (FLASH->SR & FLASH_SR_BSY) { }
    FLASH->SR = 0x000000F3u;                       /* clear EOP and all error flags
                                                      (bits 0,1,4,5,6,7; names differ
                                                      between header versions) */
    FLASH->CR = FLASH_CR_PSIZE_1 | FLASH_CR_SER | (11u << 3);   /* SNB = sector 11 */
    FLASH->CR |= FLASH_CR_STRT;                    /* erase sector 11 */
    while (FLASH->SR & FLASH_SR_BSY) { }
    FLASH->CR = FLASH_CR_PSIZE_1 | FLASH_CR_PG;    /* program 32-bit words */
    for (uint32_t i = 0; i < words; i++) {
        *(volatile uint32_t *)(CAL_ADDR + 4u * i) = src[i];
        while (FLASH->SR & FLASH_SR_BSY) { }
    }
    FLASH->CR = 0;
    FLASH->CR = FLASH_CR_LOCK;
    FLASH->ACR &= ~FLASH_ACR_DCEN;                 /* flush the data cache */
    FLASH->ACR |=  FLASH_ACR_DCRST;
    FLASH->ACR &= ~FLASH_ACR_DCRST;
    FLASH->ACR |=  FLASH_ACR_DCEN;
    __enable_irq();

    for (uint32_t i = 0; i < words; i++) {         /* verify */
        if (((const volatile uint32_t *)CAL_ADDR)[i] != src[i]) { ok = 0; break; }
    }
    return ok;
}

/* ============================ calibration =========================== */
static void cal_announce_zone(void)
{
    zone_leds((int)g_cal_zone - 1);
    uart_puts("# CAL tap zone "); uart_putu(g_cal_zone);
    uart_puts(", "); uart_putu(g_cal_per_zone); uart_puts(" taps\r\n");
    wait_beep();
    play(zone_hz[g_cal_zone - 1u], 6u, 60u); wait_beep(); delay_ms(80);
    play(zone_hz[g_cal_zone - 1u], 6u, 60u);
}

static void calibration_toggle(void)
{
    if (g_mode == 1u) {                            /* cancel */
        g_mode = 0;
        zone_leds(-1);
        uart_puts("# CAL cancelled, previous calibration kept\r\n");
        play(330u, 6u, 150u);
        return;
    }
    if (g_cal_per_zone < 10u) g_cal_per_zone = 10u;
    if (g_cal_per_zone > 30u) g_cal_per_zone = 30u;
    s_new.magic = CAL_MAGIC;
    s_new.n     = 0;
    g_cal_zone  = 1;
    g_cal_count = 0;
    g_mode      = 1;
    uart_puts("# CAL started\r\n");
    cal_announce_zone();
}

static void calibration_add(const float *f)
{
    for (int j = 0; j < NF; j++) s_new.x[s_new.n][j] = f[j];
    s_new.y[s_new.n] = (uint8_t)(g_cal_zone - 1u);
    s_new.n++;
    g_cal_count++;
    play(zone_hz[g_cal_zone - 1u], 3u, 25u);      /* tick */
    uart_puts("# CAL zone "); uart_putu(g_cal_zone); uart_puts(": ");
    uart_putu(g_cal_count); uart_putc('/'); uart_putu(g_cal_per_zone); uart_puts("\r\n");

    if (g_cal_count < g_cal_per_zone) return;
    g_cal_count = 0;
    if (g_cal_zone < NZ) {                         /* next zone */
        g_cal_zone++;
        delay_ms(300);
        cal_announce_zone();
        return;
    }
    /* all zones done: use it and save it */
    g_mode = 0;
    zone_leds(-1);
    s_cal = s_new;
    compute_stats(&s_cal);
    g_cal_n = s_cal.n;
    g_cal_valid = 1;
    wait_beep();
    uart_puts("# CAL done, saving to flash...\r\n");
    if (cal_save(&s_cal)) {
        uart_puts("# CAL saved\r\n");
        play(523u, 6u, 80u); wait_beep(); delay_ms(40);   /* rising tune */
        play(659u, 6u, 80u); wait_beep(); delay_ms(40);
        play(784u, 6u, 120u);
    } else {
        uart_puts("# CAL save FAILED (still used until reset)\r\n");
        play(220u, 6u, 300u);
    }
    start_learning();                              /* re-learn rest after the pause */
}

/* ============================== kNN ================================= */
static void classify_and_play(const float *f, int win)
{
    float    bestd[KNN_K];
    int      besty[KNN_K];
    int      votes[NZ] = { 0 };
    float    z[NF];
    int      zone = 0;

    if (!g_cal_valid) {                            /* no calibration yet */
        play(zone_hz[win == 0 ? 0 : (win == 1 ? 4 : 2)], volume_for(g_strength), g_beep_ms);
        for (int s = 0; s < NUM_SENSORS; s++) led_set(s, s == win);
        return;
    }

    for (int j = 0; j < NF; j++) z[j] = (f[j] - s_mu[j]) / s_sd[j];
    for (int i = 0; i < KNN_K; i++) { bestd[i] = 1e30f; besty[i] = -1; }

    for (uint32_t i = 0; i < s_cal.n; i++) {       /* distance to every stored tap */
        float d = 0.0f;
        for (int j = 0; j < NF; j++) {
            float t = (s_cal.x[i][j] - s_mu[j]) / s_sd[j] - z[j];
            d += t * t;
        }
        if (d < bestd[KNN_K - 1]) {                /* keep the 5 nearest, sorted */
            int p = KNN_K - 1;
            while (p > 0 && bestd[p - 1] > d) {
                bestd[p] = bestd[p - 1]; besty[p] = besty[p - 1]; p--;
            }
            bestd[p] = d; besty[p] = s_cal.y[i];
        }
    }
    for (int i = 0; i < KNN_K; i++) if (besty[i] >= 0) votes[besty[i]]++;
    for (int k = 1; k < NZ; k++) if (votes[k] > votes[zone]) zone = k;

    g_votes = (uint8_t)votes[zone];
    if (votes[zone] < MIN_VOTES) {                 /* unsure: ignore */
        g_zone = 0;
        g_rejects++;
        zone_leds(-1);
        uart_puts("# unsure, ignored (votes "); uart_putu((uint32_t)votes[zone]);
        uart_puts("/5)\r\n");
        return;
    }
    g_zone = (uint8_t)(zone + 1);
    zone_leds(zone);
    {
        uint32_t vol = volume_for(g_strength);
        play(zone_hz[zone], vol, g_beep_ms);
        uart_puts("# zone "); uart_putu(g_zone);
        uart_puts("  votes "); uart_putu(g_votes); uart_puts("/5");
        uart_puts("  vol "); uart_putu(vol); uart_puts("\r\n");
    }
}

static void process_tap(void)
{
    int32_t  peak[NUM_SENSORS], onset[NUM_SENSORS], ptime[NUM_SENSORS];
    uint32_t energy[NUM_SENSORS];
    uint32_t sat = 0;
    int      win = 0;
    int32_t  half = s_cap_thresh / 2;

    for (int s = 0; s < NUM_SENSORS; s++) {
        uint64_t e = 0;
        peak[s] = 0; onset[s] = 255; ptime[s] = 0;
        for (uint32_t k = 0; k < WIN; k++) {
            int32_t raw = s_cap[s][k];
            int32_t d   = raw - s_cap_base[s];
            int32_t a   = (d < 0) ? -d : d;
            if (raw >= CLIP_LEVEL || raw <= -CLIP_LEVEL) sat |= (1u << s);
            e += (uint64_t)((int64_t)d * d);
            if (a > peak[s])               { peak[s] = a; ptime[s] = (int32_t)k; }
            if (onset[s] == 255 && a > half) onset[s] = (int32_t)k;
        }
        energy[s] = (uint32_t)(e >> 10);
        g_peak[s] = peak[s];
    }
    for (int s = 1; s < NUM_SENSORS; s++) if (energy[s] > energy[win]) win = s;

    g_taps++;
    if (sat) g_sat_taps++;
    g_winner = (uint8_t)(win + 1);

    uart_puts("T");
    uart_field((int32_t)g_taps);
    uart_field(g_range);
    uart_field(s_cap_thresh);
    for (int s = 0; s < NUM_SENSORS; s++) uart_field(peak[s]);
    for (int s = 0; s < NUM_SENSORS; s++) uart_field((int32_t)energy[s]);
    for (int s = 0; s < NUM_SENSORS; s++) uart_field(onset[s]);
    for (int s = 0; s < NUM_SENSORS; s++) uart_field(ptime[s]);
    uart_field((int32_t)sat);
    uart_field(s_cap_first);
    uart_field(win + 1);
    uart_puts("\r\n");

    {
        float f[NF];
        make_features(peak, energy, onset, ptime, f);
        g_strength = (uint32_t)(peak[0] + peak[1] + peak[2]);
        if (g_mode == 1u) calibration_add(f);
        else              classify_and_play(f, win);
    }
}

/* ================================ main ============================== */
int main(void)
{
    uint32_t t_beat;
    int      info_pending = 1;

    clock_init();
    SystemCoreClockUpdate();
    SysTick_Config(g_clock_hz / 1000u);

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;   /* cycle counter for delay_us */
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    gpio_init();
    spi_init();
    uart_init();
    buzzer_init();
    if (button_pressed()) g_mute = 1;   /* held at reset: start muted */
    delay_ms(100);

    /* which sensors answer? LEDs on for 2 s, as in the feasibility test */
    for (int s = 0; s < NUM_SENSORS; s++) {
        g_whoami[s]    = mpu_read(s, REG_WHO_AM_I);
        s_sensor_ok[s] = (g_whoami[s] != 0x00u) && (g_whoami[s] != 0xFFu);
        led_set(s, s_sensor_ok[s]);
    }
    g_range = (g_range_req >= 16) ? 16 : 8;
    for (int s = 0; s < NUM_SENSORS; s++) {
        if (!s_sensor_ok[s]) continue;
        for (int tries = 0; tries < 3; tries++) {        /* re-init until the range sticks */
            mpu_init(s, g_range);
            if ((g_cfg[s] & 0x18u) == range_bits(g_range)) break;
        }
    }
    /* buzzer self-test: one short beep per sensor that answered */
    for (int s = 0; s < NUM_SENSORS; s++) {
        if (s_sensor_ok[s]) buzzer_chirp(tone_hz[s]);
    }
    delay_ms(1700);
    for (int s = 0; s < NUM_SENSORS; s++) led_set(s, 0);

    cal_load();
    spi_set_divider(spi_div_read);
    tim2_init();
    start_learning();               /* keep the pad still for about 1.2 s */
    t_beat = g_ms;

    while (1) {
        /* range change requested from the Watch window */
        int32_t want = (g_range_req >= 16) ? 16 : 8;
        if (want != g_range) {
            set_range(want);
            delay_ms(20);
            start_learning();
            info_pending = 1;
        }

        /* once learning has finished, report settings to the laptop */
        if (info_pending && s_state == ST_IDLE) {
            print_info();
            info_pending = 0;
        }

        if (s_cap_ready) {
            process_tap();
            s_cap_ready = 0;
        }

        buzzer_service();
        buttons_service();

        if ((g_ms - t_beat) >= 500u) {
            GPIOH->ODR ^= (1UL << 7u);              /* heartbeat PH7 */
            t_beat += 500u;
        }
    }
}
