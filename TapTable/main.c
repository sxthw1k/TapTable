/*
 * TapTable - final version
 * Board : Keil MCBSTM32F400 (STM32F407IG).
 *         (CS1 PF6, CS2 PF7, CS3 PF10, buzzer PF8 via 100 ohm).
 *
 * 3 x MPU6500 for sensing, LCD and Joystick for user interface.
 *   - 9 features (peak times dropped) and the 5-person S K T R L dataset
 *   - votes needed fixed at 6 of 9 in every mode (3 of 5 while a zone has
 *     fewer than 30 taps, i.e. after a full recalibration until a merge)
 *   - A/B/C mode (no machine learning)
 *   - the board's response can be followed live in Keil's Watch window
 *     (see WATCH WINDOW below)
 *
 * MODES (menu: joystick press in Play)
 *   PLAY 6 ZONES  kNN: the zone's LED and note, and on the LCD the zone and
 *                 its confidence (votes of the 9 nearest stored taps, e.g.
 *                 "Z3  8/9"); fewer than 6 votes = "unsure", no sound
 *   A/B/C         no machine learning: A = Z1, B = Z5, C = Z3 (the zones on
 *                 top of S1, S2, S3). The sensor with the most energy must
 *                 also be the first to react, at least 0.75 ms (3 samples)
 *                 before the other two; otherwise no output
 *   PASSWORD      a 6-digit PIN typed by tapping: Z1-Z5 = digits 1-5,
 *                 Z6 = delete, joystick press = Enter. Shown as * when
 *                 entering the PIN; shown as digits when setting a new one.
 *                 3 wrong PINs in a row = locked for 30 s
 *   CALIBRATE     merge or full, both 20 taps per zone (120)
 *   SETTINGS      change PIN, factory reset
 *   Loudness follows tap strength; WAKEUP = mute on/off
 *
 * FLASH
 *   sector 10  calibrated dataset (saved after each calibration)
 *   sector 8   settings and PIN
 *   Factory reset (Settings, or TAMPER held during RESET) erases sector 10
 *   and goes back to the built-in dataset.
 *   Keil flash download on "Erase Sectors" keeps calibrations and the PIN
 *   when the program is flashed again; "Erase Full Chip" clears them.
 *
 * FILES:
 *   main.c        this file: hardware, LCD screens, joystick, modes
 *   tapf_core.h   dataset logic and kNN
 *   sktrl_data.h  the built-in dataset (600 taps)
 *
 * CONTROLS
 *   Joystick (STMPE811 on I2C1, PB8/PB9), as seen by the user:
 *     Play, A/B/C press = menu
 *     Menu       up/down = move, press or right = open, left = back to Play
 *     Password   press = Enter, left = leave
 *     Calibrate  left = cancel
 *     Settings   up/down = move, press = open, left = back
 *   TAMPER short / held 3 s : merge / full calibration (not in Password)
 *   TAMPER held during RESET: factory reset (sector 10 erased)
 *   WAKEUP: mute on/off (held during RESET: start muted)
 *
 * ZONES, LEDS AND NOTES   (LCD: Z1-Z3 top row, Z4-Z6 bottom row)
 *   Z1 PH3 C5 523 Hz   Z2 PH6 D5 587 Hz   Z3 PH7 E5 659 Hz
 *   Z4 PG6 G5 784 Hz   Z5 PG7 A5 880 Hz   Z6 PG8 C6 1047 Hz
 *   Heartbeat: PH2. Start-up sensor check (2 s): PG6/PG7/PG8 = S1/S2/S3.
 *
 * WATCH WINDOW (Keil: Ctrl+F5, F5, View > Watch 1, and turn on
 *   View > Periodic Window Update). Arrays are per sensor S1, S2, S3.
 *   Live, all modes:
 *     g_z[3]        latest Z reading, counts (2048 = 1 g; about -1950 at rest)
 *     g_base[3]     rest value learned at start-up
 *     g_thresh      trigger level (5 x noise); g_noise[3] the noise
 *     g_taps        taps detected so far
 *   Last tap, all modes:
 *     g_peak[3]     largest distance from rest in the 20 ms window
 *     g_energy[3]   sum of squared distances (/1024)
 *     g_onset[3]    sample (0..79, 4 per ms) where each sensor passed half
 *                   the threshold: who felt it first (255 = never)
 *     g_first       sensor that triggered the capture (1..3)
 *   6 zones (kNN):
 *     g_feat[9]     peak shares S1-S3, energy shares S1-S3, onset delays (ms)
 *     g_nn_zone[9]  zones of the 9 nearest stored taps
 *     g_zone_votes[6]  votes for Z1..Z6;  g_votes = the winner's votes
 *     g_zone        decision 1..6, 0 = unsure (fewer than 6 votes)
 *   A/B/C (no ML):
 *     g_abc_strong  sensor with the most energy (1..3)
 *     g_abc_lead    how many samples before the other two it reacted
 *                   (needs 3 = 0.75 ms)
 *     g_abc         'A', 'B', 'C', or '-' = no output
 */

#include "stm32f4xx.h"
#include <stdint.h>
#include "Board_GLCD.h"            /* Keil Board Support: Graphic LCD API */
#include "GLCD_Config.h"           /* GLCD_WIDTH, GLCD_HEIGHT, colours    */
#include "tapf_core.h"

extern GLCD_FONT GLCD_Font_6x8;    /* the only font used: large text is this
                                      font drawn twice as big (12 x 16), which
                                      saves the 5.4 KB of Keil's 16x24 font */

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
#define DEFAULT_RANGE_G 16
#define CLIP_LEVEL      32700
#define HOLD_MS         3000u    /* TAMPER held this long = full recalibration */
#define ABC_LEAD        3        /* A/B/C: the strongest sensor must react at least
                                    3 samples (0.75 ms) before the other two */

#define DATA_ADDR       0x080C0000u   /* flash sector 10 (128 KB)          */
#define DATA_SECTOR     10u
#define DATA_MAGIC      0x54504639u   /* "TPF9": 9-feature dataset         */
#define SET_ADDR        0x08080000u   /* flash sector 8: settings and PIN  */
#define SET_SECTOR      8u
#define SET_MAGIC       0x54504653u   /* "TPFS"                            */

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
volatile uint8_t  g_cfg[NUM_SENSORS];
volatile int32_t  g_base[NUM_SENSORS];
volatile int32_t  g_noise[NUM_SENSORS];
volatile int32_t  g_thresh = 600;
volatile int32_t  g_range;
volatile uint32_t g_taps;
volatile uint32_t g_sat_taps;
volatile uint32_t g_isr_max_us;
volatile uint32_t g_overruns;
volatile uint8_t  g_mute     = 0;
volatile uint32_t g_volume   = 8;      /* max loudness, PWM duty %, 1..50   */
volatile uint32_t g_beep_ms  = 80;

volatile uint8_t  g_mode;              /* 0 = run, 1 = calibrating          */
volatile uint8_t  g_cal_full;          /* 1 = this calibration is a full one*/
volatile uint8_t  g_cal_zone;          /* zone being calibrated (1..6)      */
volatile uint32_t g_cal_count;         /* taps so far in that zone          */
volatile uint8_t  g_zone;              /* last zone 1..6, 0 = unsure        */
volatile uint8_t  g_votes;
volatile uint32_t g_t_us;              /* last classification time, us      */
volatile uint32_t g_strength;
volatile uint32_t g_rejects;
volatile uint8_t  g_from_flash;        /* 1 = dataset loaded from sector 10 */
volatile uint16_t g_joy_id;            /* 0x0811 = joystick chip answered   */
volatile uint32_t g_joy_us;            /* one joystick read, us             */
volatile uint32_t g_i2c_err;           /* failed joystick reads             */

/* ---- for the Keil Watch window (see WATCH WINDOW at the top) ---- */
volatile int32_t  g_z[NUM_SENSORS];      /* latest Z reading of each sensor   */
volatile int32_t  g_peak[NUM_SENSORS];   /* last tap: peak                    */
volatile uint32_t g_energy[NUM_SENSORS]; /* last tap: energy                  */
volatile int32_t  g_onset[NUM_SENSORS];  /* last tap: onset sample            */
volatile uint8_t  g_first;               /* last tap: sensor that triggered   */
volatile float    g_feat[NF];            /* last tap: the 9 features          */
volatile uint8_t  g_abc_strong;          /* A/B/C: sensor with most energy    */
volatile int32_t  g_abc_lead;            /* A/B/C: lead in samples (need 3)   */
volatile char     g_abc = '-';           /* A/B/C: decision                   */

typedef struct {                       /* saved in flash sector 8           */
    uint8_t vol;                       /* 0 off, 1..6 fixed, 7 by tap strength */
    uint8_t pin[6];                    /* digits 1..5                       */
} settings_t;
static settings_t S;

static volatile uint8_t  s_beeping;
static volatile uint32_t s_beep_end;
static const uint32_t    tone_hz[NUM_SENSORS] = { 523u, 659u, 784u };

/* --------------------------- pin tables ----------------------------- */
static GPIO_TypeDef * const cs_port[NUM_SENSORS] = { GPIOF, GPIOF, GPIOF };
static const uint8_t        cs_pin[NUM_SENSORS]  = { 6u, 7u, 10u };   /* PF6, PF7, PF10 */
static const uint8_t        chk_pin[NUM_SENSORS] = { 6u, 7u, 8u };    /* start-up check PG6-8 */

static const uint32_t       zone_hz[NZ] = { 523u, 587u, 659u, 784u, 880u, 1047u };
static GPIO_TypeDef * const zone_led_port[NZ] = { GPIOH, GPIOH, GPIOH, GPIOG, GPIOG, GPIOG };
static const uint32_t       zone_led_pin[NZ]  = { 3u, 6u, 7u, 6u, 7u, 8u };
#define HEART_PORT  GPIOH
#define HEART_PIN   2u

/* ------------------------ clocks (filled at start) ------------------ */
static uint32_t pclk1_hz, tim2_hz;
static uint32_t spi_div_cfg, spi_div_read;

/* ------------------- sampling state (shared with ISR) --------------- */
enum { ST_OFF, ST_BASE, ST_NOISE, ST_IDLE, ST_POST, ST_DEAD };

static volatile uint8_t  s_state = ST_OFF;
static volatile uint32_t s_count;
static int16_t           s_ring[NUM_SENSORS][RING];
static int32_t           s_base_fx[NUM_SENSORS];
static int32_t           s_base_sum[NUM_SENSORS];
static int32_t           s_noise_max[NUM_SENSORS];
static uint32_t          s_phase_n;
static uint32_t          s_trig_count;
static uint8_t           s_trig_first;
static int32_t           s_trig_base[NUM_SENSORS];
static uint8_t           s_sensor_ok[NUM_SENSORS];

static volatile uint8_t  s_cap_ready;
static int16_t           s_cap[NUM_SENSORS][WIN];
static int32_t           s_cap_base[NUM_SENSORS];
static uint8_t           s_cap_first;
static int32_t           s_cap_thresh;


/* ============================ time base ============================= */
void SysTick_Handler(void) { g_ms++; }

/* Keil's LCD driver waits with HAL_Delay(), which asks HAL_GetTick() for
   the time; this replaces the HAL's own version with our 1 ms counter. */
uint32_t HAL_GetTick(void) { return g_ms; }

static void delay_ms(uint32_t ms)
{
    uint32_t t0 = g_ms;
    while ((g_ms - t0) < ms) { }
}

/* ============================== clock =============================== */
static void clock_init(void)
{
    uint32_t t;

    RCC->CR |= RCC_CR_HSEON;
    for (t = 0; t < 500000u && !(RCC->CR & RCC_CR_HSERDY); t++) { }

    if (RCC->CR & RCC_CR_HSERDY) {
        RCC->APB1ENR |= RCC_APB1ENR_PWREN;
        (void)RCC->APB1ENR;
        PWR->CR |= PWR_CR_VOS;

        FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN |
                     FLASH_ACR_LATENCY_5WS;

        RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2))
                  | RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;

        RCC->PLLCFGR = (25u  << RCC_PLLCFGR_PLLM_Pos) |   /* 25 MHz / 25 = 1 MHz */
                       (336u << RCC_PLLCFGR_PLLN_Pos) |   /* x 336 = 336 MHz     */
                       (0u   << RCC_PLLCFGR_PLLP_Pos) |   /* / 2  = 168 MHz      */
                       (7u   << RCC_PLLCFGR_PLLQ_Pos) |   /* / 7  = 48 MHz       */
                       RCC_PLLCFGR_PLLSRC_HSE;

        RCC->CR |= RCC_CR_PLLON;
        for (t = 0; t < 500000u && !(RCC->CR & RCC_CR_PLLRDY); t++) { }

        if (RCC->CR & RCC_CR_PLLRDY) {
            RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
            while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) { }
            g_clock_hz   = 168000000u;
            pclk1_hz     = 42000000u;
            tim2_hz      = 84000000u;
            spi_div_cfg  = 6u;          /* 42 MHz / 128 = 328 kHz */
            spi_div_read = 4u;          /* 42 MHz / 32  = 1.3 MHz */
            return;
        }
        RCC->CFGR &= ~(RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
    }

    g_clock_hz   = 16000000u;
    pclk1_hz     = 16000000u;
    tim2_hz      = 16000000u;
    spi_div_cfg  = 5u;
    spi_div_read = 3u;
}

/* =============================== GPIO =============================== */
static void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (g_clock_hz / 1000000u);
    while ((DWT->CYCCNT - start) < ticks) { }
}

static void cs_low(int s)  { cs_port[s]->BSRR = (1UL << (cs_pin[s] + 16u)); delay_us(1); }
static void cs_high(int s) { cs_port[s]->BSRR = (1UL << cs_pin[s]); delay_us(2); }

static void chk_led(int s, int on)            /* start-up sensor check LEDs */
{
    if (on) GPIOG->BSRR = (1UL << chk_pin[s]);
    else    GPIOG->BSRR = (1UL << (chk_pin[s] + 16u));
}

static void gpio_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN |
                    RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOFEN | RCC_AHB1ENR_GPIOGEN |
                    RCC_AHB1ENR_GPIOHEN;
    (void)RCC->AHB1ENR;

    for (int s = 0; s < NUM_SENSORS; s++) {          /* CS high, then output */
        cs_high(s);
        cs_port[s]->MODER &= ~(3UL << (cs_pin[s] * 2u));
        cs_port[s]->MODER |=  (1UL << (cs_pin[s] * 2u));
    }
    for (uint32_t p = 6u; p <= 8u; p++) {            /* LEDs PG6-PG8 */
        GPIOG->MODER &= ~(3UL << (p * 2u));
        GPIOG->MODER |=  (1UL << (p * 2u));
    }
    for (uint32_t p = 2u; p <= 7u; p++) {            /* LEDs PH2, PH3, PH6, PH7 */
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

static uint8_t range_bits(int32_t g) { return (g >= 16) ? 0x18u : 0x10u; }

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
    mpu_write(s, REG_PWR_MGMT_1, 0x80u);    /* reset              */
    delay_ms(100);
    mpu_write(s, REG_USER_CTRL, 0x10u);     /* SPI only           */
    mpu_write(s, REG_PWR_MGMT_1, 0x01u);    /* wake, best clock   */
    delay_ms(10);
    mpu_write(s, REG_PWR_MGMT_2, 0x07u);    /* gyro off           */
    mpu_write(s, REG_ACCEL_CONFIG2, 0x08u); /* 4 kHz, no filter   */
    mpu_set_range_verified(s, range_g);     /* +/-16 g, checked   */
    delay_ms(20);
}

/* ====================== sampling timer (TIM2) ======================= */
static void tim2_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    (void)RCC->APB1ENR;
    TIM2->CR1  = 0;
    TIM2->PSC  = 0;
    TIM2->ARR  = tim2_hz / FS_HZ - 1u;
    TIM2->EGR  = TIM_EGR_UG;
    TIM2->SR   = 0;
    TIM2->DIER = TIM_DIER_UIE;
    NVIC_SetPriority(TIM2_IRQn, 0);
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

    TIM2->SR = ~TIM_SR_UIF;

    for (int s = 0; s < NUM_SENSORS; s++) {
        z[s] = s_sensor_ok[s] ? mpu_read_z(s) : 0;
        s_ring[s][i] = z[s];
        g_z[s] = z[s];                               /* for the Watch window */
        dev[s] = (int32_t)z[s] - (s_base_fx[s] / 64);
        if (dev[s] < 0) dev[s] = -dev[s];
    }

    switch (s_state) {
    case ST_BASE:
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

    case ST_NOISE:
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

    case ST_IDLE: {
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

    case ST_POST:
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

    case ST_DEAD:
        if (++s_phase_n >= DEAD_SAMPLES) { s_phase_n = 0; s_state = ST_IDLE; }
        break;

    default:
        break;
    }

    s_count++;

    {
        uint32_t t_end = TIM2->CNT;
        uint32_t ticks = (t_end >= t_start) ? (t_end - t_start)
                                            : (t_end + TIM2->ARR + 1u - t_start);
        uint32_t us = ticks / (tim2_hz / 1000000u);
        if (us > g_isr_max_us) g_isr_max_us = us;
        if (TIM2->SR & TIM_SR_UIF) g_overruns++;
    }
}

static void start_learning(void)
{
    NVIC_DisableIRQ(TIM2_IRQn);
    for (int s = 0; s < NUM_SENSORS; s++) { s_base_sum[s] = 0; s_base_fx[s] = 0; }
    s_phase_n   = 0;
    s_cap_ready = 0;
    s_state     = ST_BASE;
    NVIC_EnableIRQ(TIM2_IRQn);
}



/* ============================== buzzer ============================== */
static void buzzer_init(void)
{
    GPIOF->MODER  &= ~(3UL   << (8u * 2u));          /* PF8 = TIM13_CH1, AF9 */
    GPIOF->MODER  |=  (2UL   << (8u * 2u));
    GPIOF->AFR[1] &= ~(0xFUL << ((8u - 8u) * 4u));
    GPIOF->AFR[1] |=  (9UL   << ((8u - 8u) * 4u));

    RCC->APB1ENR |= RCC_APB1ENR_TIM13EN;
    (void)RCC->APB1ENR;
    TIM13->CR1   = 0;
    TIM13->PSC   = tim2_hz / 1000000u - 1u;          /* tick = 1 us */
    TIM13->ARR   = 999u;
    TIM13->CCR1  = 0;
    TIM13->CCMR1 = (6u << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
    TIM13->CCER  = TIM_CCER_CC1E;
    TIM13->CR1   = TIM_CR1_ARPE;
    TIM13->EGR   = TIM_EGR_UG;
}

static void buzzer_off(void)
{
    TIM13->CCR1 = 0;
    TIM13->EGR  = TIM_EGR_UG;
    TIM13->CR1 &= ~TIM_CR1_CEN;
    s_beeping = 0;
}

static void buzzer_beep(uint32_t hz, uint32_t ms)
{
    uint32_t vol = g_volume, arr;
    if (g_mute || hz == 0u) return;
    if (vol < 1u)  vol = 1u;
    if (vol > 50u) vol = 50u;
    if (ms > 300u) ms = 300u;
    arr = 1000000u / hz - 1u;
    TIM13->CR1 &= ~TIM_CR1_CEN;
    TIM13->ARR  = arr;
    TIM13->CCR1 = ((arr + 1u) * vol) / 100u;
    TIM13->EGR  = TIM_EGR_UG;
    TIM13->CR1 |= TIM_CR1_CEN;
    s_beep_end  = g_ms + ms;
    s_beeping   = 1;
}

static void buzzer_service(void)
{
    if (s_beeping && (int32_t)(g_ms - s_beep_end) >= 0) buzzer_off();
    if (s_beeping && g_mute) buzzer_off();
}

static void play(uint32_t hz, uint32_t vol, uint32_t ms)
{
    uint32_t keep = g_volume;
    g_volume = vol;
    buzzer_beep(hz, ms);
    g_volume = keep;
}

static void wait_beep(void) { while (s_beeping) buzzer_service(); }

static void tune(const uint32_t *hz, int n, uint32_t ms)     /* blocking */
{
    for (int i = 0; i < n; i++) {
        wait_beep();
        play(hz[i], 6u, (i == n - 1) ? ms + 40u : ms);
        wait_beep();
        delay_ms(40);
    }
}

/* harder tap -> louder, log scale, 1 % .. g_volume % */
static float log2_f(float x)
{
    union { float f; uint32_t u; } v;
    v.f = x;
    return (float)((int32_t)((v.u >> 23) & 0xFFu) - 127) +
           (float)(v.u & 0x7FFFFFu) / 8388608.0f;
}

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


/* ================================ LEDs ============================== */
static void zone_leds(int z)          /* z = 0..5, or -1 for all off */
{
    for (int i = 0; i < NZ; i++) {
        if (i == z) zone_led_port[i]->BSRR = (1UL << zone_led_pin[i]);
        else        zone_led_port[i]->BSRR = (1UL << (zone_led_pin[i] + 16u));
    }
}

/* ============================== flash =============================== */
/* Sector 10 layout (32-bit words):
     [0] magic "TPF9"  [1] n  [2] next session  [3] checksum
     then x: n x 9 floats, y: (n+3)/4 words, s: (n+1)/2 words
   The magic word is written LAST, so a save cut short by a power loss
   is simply not recognised and the built-in dataset is used instead.
   (Older 12-feature saves have another magic word and are ignored.)
   Sector 8 (settings): [0] magic "TPFS" [1] volume
     [2] PIN digits 1-4 [3] PIN digits 5-6 [4] checksum                   */
static uint32_t img_words(uint32_t n) { return n * NF + (n + 3u) / 4u + (n + 1u) / 2u; }

static uint32_t img_word(uint32_t i, uint32_t n)       /* i-th data word of D */
{
    uint32_t nx = n * NF, ny = (n + 3u) / 4u;
    if (i < nx)      return ((const uint32_t *)&D.x[0][0])[i];
    i -= nx;
    if (i < ny)      return ((const uint32_t *)D.y)[i];
    return ((const uint32_t *)D.s)[i - ny];
}

static uint32_t img_checksum(uint32_t n)
{
    uint32_t c = 0x12345678u, w = img_words(n);
    for (uint32_t i = 0; i < w; i++) c = ((c << 5) | (c >> 27)) ^ img_word(i, n);
    return c;
}

static void flash_unlock_erase(uint32_t sector)    /* call with IRQs off */
{
    FLASH->KEYR = 0x45670123u;
    FLASH->KEYR = 0xCDEF89ABu;
    while (FLASH->SR & FLASH_SR_BSY) { }
    FLASH->SR = 0x000000F3u;                       /* clear EOP and error flags */
    FLASH->CR = FLASH_CR_PSIZE_1 | FLASH_CR_SER | (sector << 3);
    FLASH->CR |= FLASH_CR_STRT;
    while (FLASH->SR & FLASH_SR_BSY) { }
    FLASH->CR = FLASH_CR_PSIZE_1 | FLASH_CR_PG;    /* ready to program words */
}

static void flash_word(uint32_t addr, uint32_t v)
{
    *(volatile uint32_t *)addr = v;
    while (FLASH->SR & FLASH_SR_BSY) { }
}

static void flash_lock(void)
{
    FLASH->CR = 0;
    FLASH->CR = FLASH_CR_LOCK;
    FLASH->ACR &= ~FLASH_ACR_DCEN;                 /* flush the data cache */
    FLASH->ACR |=  FLASH_ACR_DCRST;
    FLASH->ACR &= ~FLASH_ACR_DCRST;
    FLASH->ACR |=  FLASH_ACR_DCEN;
}

/* Erase sector 10 and write D. About 1-2 s; interrupts paused meanwhile. */
static int data_save(void)
{
    uint32_t n = D.n, w = img_words(n), sum = img_checksum(n);
    const volatile uint32_t *f = (const volatile uint32_t *)DATA_ADDR;

    __disable_irq();
    flash_unlock_erase(DATA_SECTOR);
    for (uint32_t i = 0; i < w; i++) flash_word(DATA_ADDR + 16u + 4u * i, img_word(i, n));
    flash_word(DATA_ADDR + 4u,  n);
    flash_word(DATA_ADDR + 8u,  D.next_session);
    flash_word(DATA_ADDR + 12u, sum);
    flash_word(DATA_ADDR + 0u,  DATA_MAGIC);       /* last: marks the save complete */
    flash_lock();
    __enable_irq();

    if (f[0] != DATA_MAGIC || f[1] != n) return 0;   /* verify */
    for (uint32_t i = 0; i < w; i++) if (f[4u + i] != img_word(i, n)) return 0;
    return 1;
}

static void data_erase(void)                      /* factory reset */
{
    __disable_irq();
    flash_unlock_erase(DATA_SECTOR);
    flash_lock();
    __enable_irq();
}

/* Load sector 10 into D if it holds a complete, valid dataset */
static int data_load(void)
{
    const volatile uint32_t *f = (const volatile uint32_t *)DATA_ADDR;
    uint32_t n, w, cnt[NZ];
    if (f[0] != DATA_MAGIC) return 0;
    n = f[1];
    if (n < NZ * 5u || n > MAXN || f[2] < 1u || f[2] > 60000u) return 0;
    w = img_words(n);
    for (uint32_t i = 0; i < w; i++) {             /* copy into D */
        uint32_t v = f[4u + i], nx = n * NF, ny = (n + 3u) / 4u;
        if (i < nx)           ((uint32_t *)&D.x[0][0])[i] = v;
        else if (i < nx + ny) ((uint32_t *)D.y)[i - nx] = v;
        else                  ((uint32_t *)D.s)[i - nx - ny] = v;
    }
    if (img_checksum(n) != f[3]) return 0;
    for (uint32_t i = 0; i < n; i++) if (D.y[i] >= NZ) return 0;
    zone_counts(D.y, n, cnt);
    for (int z = 0; z < NZ; z++) if (cnt[z] < 5u) return 0;
    D.n = n;
    D.next_session = f[2];
    dataset_refresh();
    return 1;
}

static void data_init(void)
{
    if (data_load()) { g_from_flash = 1; return; }
    g_from_flash = 0;
    load_builtin();                                /* tapf_core.h */
}

/* ----------------------------- settings ----------------------------- */
static void settings_default(void)
{
    static const uint8_t pin[6] = { 1, 2, 3, 4, 5, 1 };
    S.vol = 7;                                    /* 7 = loudness by tap strength */
    for (int i = 0; i < 6; i++) S.pin[i] = pin[i];
}

static void settings_words(uint32_t *w)
{
    w[1] = S.vol;
    w[2] = (uint32_t)S.pin[0] | ((uint32_t)S.pin[1] << 8) | ((uint32_t)S.pin[2] << 16) | ((uint32_t)S.pin[3] << 24);
    w[3] = (uint32_t)S.pin[4] | ((uint32_t)S.pin[5] << 8);
    w[4] = 0x5A5A5A5Au ^ (w[1] << 3) ^ w[2] ^ (w[3] << 9);
}

static void settings_load(void)
{
    const volatile uint32_t *f = (const volatile uint32_t *)SET_ADDR;
    uint32_t w[5];
    int ok = (f[0] == SET_MAGIC);
    if (ok) {
        S.vol = (uint8_t)f[1];
        for (int i = 0; i < 4; i++) S.pin[i] = (uint8_t)(f[2] >> (8 * i));
        S.pin[4] = (uint8_t)f[3]; S.pin[5] = (uint8_t)(f[3] >> 8);
        settings_words(w);
        ok = (w[4] == f[4]);
        for (int i = 0; i < 6; i++) if (S.pin[i] < 1 || S.pin[i] > 5) ok = 0;
    }
    if (!ok) settings_default();
    S.vol = 7;                       /* loudness always by tap strength (no volume setting) */
}

static void settings_save(void)                   /* about 1 s, IRQs paused */
{
    uint32_t w[5];
    settings_words(w);
    __disable_irq();
    flash_unlock_erase(SET_SECTOR);
    for (int i = 1; i < 5; i++) flash_word(SET_ADDR + 4u * (uint32_t)i, w[i]);
    flash_word(SET_ADDR, SET_MAGIC);
    flash_lock();
    __enable_irq();
}

/* =================== I2C1 joystick (STMPE811, 0x44) ================= */
#define JOY_ADDR    0x44u
#define JOY_UP      (1u << 0)
#define JOY_RIGHT   (1u << 1)
#define JOY_LEFT    (1u << 2)
#define JOY_DOWN    (1u << 3)
#define JOY_CENTER  (1u << 4)

static int i2c_wait(uint32_t flag)
{
    uint32_t t0 = g_ms;
    while (!(I2C1->SR1 & flag)) {
        if (I2C1->SR1 & (I2C_SR1_AF | I2C_SR1_BERR | I2C_SR1_ARLO)) {
            I2C1->SR1 &= ~(I2C_SR1_AF | I2C_SR1_BERR | I2C_SR1_ARLO);
            return -1;
        }
        if ((g_ms - t0) > 3u) return -2;
    }
    return 0;
}

static void i2c_init(void)            /* PB8 SCL, PB9 SDA, AF4, 100 kHz */
{
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    (void)RCC->APB1ENR;
    /* bus recovery: up to 9 clocks by hand if SDA is held low, then STOP */
    GPIOB->OTYPER |=  (1UL << 8u) | (1UL << 9u);
    GPIOB->PUPDR   = (GPIOB->PUPDR & ~((3UL << 16u) | (3UL << 18u))) | (1UL << 16u) | (1UL << 18u);
    GPIOB->BSRR    =  (1UL << 8u) | (1UL << 9u);
    GPIOB->MODER   = (GPIOB->MODER & ~((3UL << 16u) | (3UL << 18u))) | (1UL << 16u) | (1UL << 18u);
    delay_us(10);
    for (int i = 0; i < 9 && !(GPIOB->IDR & (1UL << 9u)); i++) {
        GPIOB->BSRR = (1UL << 24u); delay_us(5);
        GPIOB->BSRR = (1UL << 8u);  delay_us(5);
    }
    GPIOB->BSRR = (1UL << 25u); delay_us(5);
    GPIOB->BSRR = (1UL << 8u);  delay_us(5);
    GPIOB->BSRR = (1UL << 9u);  delay_us(5);
    for (uint32_t p = 8u; p <= 9u; p++) {
        GPIOB->MODER  = (GPIOB->MODER & ~(3UL << (p * 2u))) | (2UL << (p * 2u));
        GPIOB->AFR[1] = (GPIOB->AFR[1] & ~(0xFUL << ((p - 8u) * 4u))) | (4UL << ((p - 8u) * 4u));
    }
    RCC->APB1RSTR |=  RCC_APB1RSTR_I2C1RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;
    I2C1->CR1   = 0;
    I2C1->CR2   = pclk1_hz / 1000000u;
    I2C1->CCR   = pclk1_hz / (2u * 100000u);
    I2C1->TRISE = pclk1_hz / 1000000u + 1u;
    I2C1->CR1   = I2C_CR1_PE;
}

static int i2c_fail(void)
{
    I2C1->CR1 |= I2C_CR1_STOP;
    g_i2c_err++;
    if (g_i2c_err % 8u == 0u) i2c_init();
    return -1;
}

static int i2c_idle(void)             /* wait (max 3 ms) for the last STOP to finish */
{
    uint32_t t0 = g_ms;
    while (I2C1->SR2 & I2C_SR2_BUSY) { if ((g_ms - t0) > 3u) return -1; }
    return 0;
}

static int i2c_start(uint8_t addr_rw)
{
    I2C1->CR1 |= I2C_CR1_START;
    if (i2c_wait(I2C_SR1_SB)) return -1;
    I2C1->DR = addr_rw;
    return i2c_wait(I2C_SR1_ADDR);
}

static int stmpe_write(uint8_t reg, uint8_t val)
{
    if (i2c_idle()) return i2c_fail();
    if (i2c_start((uint8_t)(JOY_ADDR << 1))) return i2c_fail();
    (void)I2C1->SR2;
    I2C1->DR = reg;
    if (i2c_wait(I2C_SR1_TXE)) return i2c_fail();
    I2C1->DR = val;
    if (i2c_wait(I2C_SR1_BTF)) return i2c_fail();
    I2C1->CR1 |= I2C_CR1_STOP;
    return 0;
}

static int stmpe_read(uint8_t reg, uint8_t *val)
{
    if (i2c_idle()) return i2c_fail();
    if (i2c_start((uint8_t)(JOY_ADDR << 1))) return i2c_fail();
    (void)I2C1->SR2;
    I2C1->DR = reg;
    if (i2c_wait(I2C_SR1_BTF)) return i2c_fail();
    if (i2c_start((uint8_t)((JOY_ADDR << 1) | 1u))) return i2c_fail();
    I2C1->CR1 &= ~I2C_CR1_ACK;                     /* one byte: NACK it */
    __disable_irq();
    (void)I2C1->SR2;
    I2C1->CR1 |= I2C_CR1_STOP;
    __enable_irq();
    if (i2c_wait(I2C_SR1_RXNE)) return i2c_fail();
    *val = (uint8_t)I2C1->DR;
    return 0;
}

static void joy_init(void)
{
    uint8_t hi = 0, lo = 0;
    i2c_init();
    stmpe_write(0x03u, 0x02u);                     /* SYS_CTRL1: soft reset   */
    delay_ms(10);
    stmpe_write(0x04u, 0x03u);                     /* SYS_CTRL2: GPIO clock on */
    stmpe_write(0x13u, 0x00u);                     /* GPIO_DIR: inputs        */
    stmpe_write(0x17u, 0xFFu);                     /* GPIO_AF: plain GPIO     */
    stmpe_read(0x00u, &hi);
    stmpe_read(0x01u, &lo);
    g_joy_id = (uint16_t)((hi << 8) | lo);         /* 0x0811 = found          */
}

/* JOY_ bits pressed now, -1 if no answer. The joystick is turned 90 degrees
   from Keil's names on this board (checked 18 Nov): pin 3 RIGHT, 4 DOWN,
   5 UP, 6 LEFT, 7 CENTER (each 0 while pressed). */
static int32_t joy_get(void)
{
    uint8_t v;
    uint32_t s = 0, t0 = DWT->CYCCNT;
    if (stmpe_read(0x12u, &v)) return -1;
    g_joy_us = (DWT->CYCCNT - t0) / (g_clock_hz / 1000000u);
    if (!(v & (1u << 3))) s |= JOY_RIGHT;
    if (!(v & (1u << 4))) s |= JOY_DOWN;
    if (!(v & (1u << 5))) s |= JOY_UP;
    if (!(v & (1u << 6))) s |= JOY_LEFT;
    if (!(v & (1u << 7))) s |= JOY_CENTER;
    return (int32_t)s;
}

/* =========================== text helpers =========================== */
static char *ap_s(char *d, const char *s) { while (*s) *d++ = *s++; *d = 0; return d; }
static char *ap_u(char *d, uint32_t v)
{
    char b[11]; int i = 0;
    do { b[i++] = (char)('0' + v % 10u); v /= 10u; } while (v);
    while (i) *d++ = b[--i];
    *d = 0;
    return d;
}
/* ============================ LCD basics ============================ */
#define TITLE_H   28u
#define STATUS_Y  (GLCD_HEIGHT - 20u)
#define BIG_W     12u                 /* large text: 12 x 16 pixels per char */
#define BIG_N     26                  /* large characters per screen width   */
#define GRID_X    40u
#define GRID_Y    36u
#define ZW        80u
#define ZH        50u
#define C_BG      GLCD_COLOR_BLACK

static const uint32_t zone_col[NZ] = { GLCD_COLOR_RED, GLCD_COLOR_GREEN, GLCD_COLOR_BLUE,
                                       GLCD_COLOR_YELLOW, GLCD_COLOR_CYAN, GLCD_COLOR_MAGENTA };

static void lcd_fill(uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint32_t c)
{
    GLCD_SetForegroundColor(c);
    for (uint32_t k = 0; k < h; k++) GLCD_DrawHLine(x, y + k, w);
}

/* one large character: the 6x8 glyph with every pixel doubled. The 6x8
   font keeps one byte per row, bit 0 = leftmost pixel; GLCD_DrawBitmap
   takes 16-bit pixels, bottom row first. */
static void big_char(uint32_t x, uint32_t y, char ch, uint16_t fg, uint16_t bg)
{
    static uint16_t px[16u * BIG_W];
    uint32_t c = ((uint8_t)ch >= 32u && (uint8_t)ch < 144u) ? (uint8_t)ch - 32u : 0u;
    const uint8_t *g = (const uint8_t *)GLCD_Font_6x8.bitmap + c * 8u;
    for (uint32_t r = 0; r < 16u; r++) {
        uint8_t bits = g[r >> 1];
        for (uint32_t k = 0; k < BIG_W; k++)
            px[(15u - r) * BIG_W + k] = ((bits >> (k >> 1)) & 1u) ? fg : bg;
    }
    GLCD_DrawBitmap(x, y, BIG_W, 16u, (const uint8_t *)px);
}

/* text in the large (12x16) or small (6x8) font, padded with spaces to
   'width' characters so it also clears what was there before */
static void lcd_text(uint32_t x, uint32_t y, const char *s, uint32_t fg, uint32_t bg, int big, int width)
{
    char b[54];
    int  i = 0;
    while (s[i] && i < 53) { b[i] = s[i]; i++; }
    while (i < width && i < 53) b[i++] = ' ';
    b[i] = 0;
    if (big) {
        for (i = 0; b[i] && x + BIG_W <= GLCD_WIDTH; i++, x += BIG_W)
            big_char(x, y, b[i], (uint16_t)fg, (uint16_t)bg);
        return;
    }
    GLCD_SetFont(&GLCD_Font_6x8);
    GLCD_SetForegroundColor(fg);
    GLCD_SetBackgroundColor(bg);
    GLCD_DrawString(x, y, b);
}

static void lcd_status(const char *s, uint32_t fg) { lcd_text(4u, STATUS_Y + 2u, s, fg, C_BG, 1, BIG_N); }
static void lcd_line(uint32_t row, const char *s)    /* small font body line */
{
    lcd_text(4u, TITLE_H + 6u + row * 12u, s, GLCD_COLOR_WHITE, C_BG, 0, 52);
}

static void lcd_screen(const char *title, int info)  /* clear, title bar */
{
    char b[24], *d = b;
    lcd_fill(0u, TITLE_H, GLCD_WIDTH, GLCD_HEIGHT - TITLE_H, C_BG);
    lcd_fill(0u, 0u, GLCD_WIDTH, TITLE_H, GLCD_COLOR_NAVY);
    lcd_text(6u, 6u, title, GLCD_COLOR_WHITE, GLCD_COLOR_NAVY, 1, 0);
    if (!info) return;
    d = ap_s(d, g_from_flash ? "calibrated" : FACT_NAME);
    *d++ = ' ';
    d = ap_u(d, D.n);
    lcd_text(GLCD_WIDTH - 6u * 17u - 4u, 10u, b, GLCD_COLOR_LIGHT_GREY, GLCD_COLOR_NAVY, 0, 17);
}

static void draw_zone(int z, int lit)               /* Z1-Z3 top, Z4-Z6 bottom */
{
    uint32_t x = GRID_X + (uint32_t)(z % 3) * ZW, y = GRID_Y + ((z < 3) ? 0u : ZH);
    char t[3] = { 'Z', (char)('1' + z), 0 };
    uint32_t bg = lit ? zone_col[z] : GLCD_COLOR_DARK_GREY;
    lcd_fill(x + 2u, y + 2u, ZW - 4u, ZH - 4u, bg);
    lcd_text(x + ZW / 2u - BIG_W, y + ZH / 2u - 8u, t, lit ? GLCD_COLOR_BLACK : GLCD_COLOR_WHITE, bg, 1, 0);
}

static void draw_grid(int lit)
{
    for (int z = 0; z < NZ; z++) draw_zone(z, z == lit);
}

/* ============================ UI state ============================== */
enum { UI_PLAY, UI_MENU, UI_PWD, UI_CAL, UI_CALDONE, UI_ABC, UI_SET, UI_CONFIRM };
static uint8_t s_ui = UI_PLAY;
static uint8_t s_sel;                 /* selected row in Menu / Settings  */
static int8_t  s_lit = -1;            /* zone lit on the Play / Cal grid  */
static uint8_t s_wait_ready;          /* waiting for rest values (status) */

/* =============================== Play =============================== */
/* the decision and its confidence: "Z3  confidence 8/9" or "unsure 4/9" */
static void play_result(int zone, int votes)
{
    char b[28], *d = b;
    if (zone >= 0) { *d++ = 'Z'; *d++ = (char)('1' + zone); d = ap_s(d, "  confidence "); }
    else           { d = ap_s(d, "unsure  confidence "); }
    d = ap_u(d, (uint32_t)votes); *d++ = '/'; d = ap_u(d, (uint32_t)g_k);
    lcd_text(8u, 146u, b, zone >= 0 ? GLCD_COLOR_GREEN : GLCD_COLOR_YELLOW, C_BG, 1, 25);
    d = ap_s(b, "votes of the "); d = ap_u(d, (uint32_t)g_k); d = ap_s(d, " nearest stored taps; ");
    d = ap_u(d, (uint32_t)g_need); d = ap_s(d, " needed");
    lcd_text(8u, 178u, b, GLCD_COLOR_LIGHT_GREY, C_BG, 0, 50);
}

static void scr_play(void)
{
    s_ui  = UI_PLAY;
    s_lit = -1;
    lcd_screen("Play: 6 zones", 1);
    draw_grid(-1);
    if (D.n == 0u) lcd_status("No data: calibrate", GLCD_COLOR_RED);
    else           lcd_status(s_wait_ready ? "Hands off: wait" : "Press: menu", GLCD_COLOR_WHITE);
}

/* ============================== A/B/C =============================== */
/* No machine learning. Keys A, B, C are the zones on top of S1, S2, S3
   (Z1, Z5, Z3). The sensor with the most vibration energy must also be
   the first to react, at least ABC_LEAD samples (0.75 ms) before both
   others. Taps elsewhere on the pad fail this test and give no output. */
static const uint8_t abc_zone[NUM_SENSORS] = { 0u, 4u, 2u };   /* A=Z1 B=Z5 C=Z3 */

static int abc_decide(const uint32_t *e, const int32_t *o)  /* 0..2 = A..C, -1 = none */
{
    int32_t on[NUM_SENSORS], other = 1000;
    int w = 0;
    for (int s = 1; s < NUM_SENSORS; s++) if (e[s] > e[w]) w = s;      /* most energy */
    for (int s = 0; s < NUM_SENSORS; s++) on[s] = (o[s] >= 255) ? 80 : o[s];
    for (int s = 0; s < NUM_SENSORS; s++) if (s != w && on[s] < other) other = on[s];
    g_abc_strong = (uint8_t)(w + 1);                 /* for the Watch window */
    g_abc_lead   = other - on[w];
    return (other - on[w] >= ABC_LEAD) ? w : -1;      /* first by at least 0.75 ms */
}

static void draw_key(int i, int lit)               /* key i drawn on its zone's place */
{
    int z = abc_zone[i];
    uint32_t x = GRID_X + (uint32_t)(z % 3) * ZW, y = GRID_Y + ((z < 3) ? 0u : ZH);
    char t[2] = { (char)('A' + i), 0 };
    uint32_t bg = lit ? zone_col[z] : GLCD_COLOR_DARK_GREY;
    lcd_fill(x + 2u, y + 2u, ZW - 4u, ZH - 4u, bg);
    lcd_text(x + ZW / 2u - BIG_W / 2u, y + ZH / 2u - 8u, t, lit ? GLCD_COLOR_BLACK : GLCD_COLOR_WHITE, bg, 1, 0);
}

static void scr_abc(void)
{
    s_ui  = UI_ABC;
    s_lit = -1;
    zone_leds(-1);
    lcd_screen("A/B/C (no ML)", 0);
    for (int i = 0; i < NUM_SENSORS; i++) draw_key(i, 0);
    lcd_text(8u, 178u, "Tap A (Z1), B (Z5) or C (Z3)", GLCD_COLOR_LIGHT_GREY, C_BG, 0, 50);
    lcd_status("Press: menu", GLCD_COLOR_WHITE);
}

static void abc_tap(const uint32_t *e, const int32_t *o)
{
    int k = abc_decide(e, o);
    if (s_lit >= 0) draw_key(s_lit, 0);
    s_lit = (int8_t)k;
    g_abc = (k < 0) ? '-' : (char)('A' + k);
    if (k < 0) {
        zone_leds(-1);
        lcd_text(8u, 146u, "-  (not on A, B or C)", GLCD_COLOR_YELLOW, C_BG, 1, 25);
        return;
    }
    {
        uint32_t vol = volume_for(g_strength);
        char t[2] = { (char)('A' + k), 0 };
        zone_leds(abc_zone[k]);
        play(zone_hz[abc_zone[k]], vol, g_beep_ms);
        draw_key(k, 1);
        lcd_text(8u, 146u, t, GLCD_COLOR_GREEN, C_BG, 1, 25);
    }
}

/* =============================== Menu =============================== */
static const char * const menu_txt[6] = { "Play: 6 zones", "A/B/C (no ML)", "Password",
                                          "Calibrate: merge", "Calibrate: full", "Settings" };

static void menu_row(int i, int sel, const char *txt)
{
    lcd_text(8u, TITLE_H + 6u + (uint32_t)i * 28u, txt,
             sel ? GLCD_COLOR_BLACK : GLCD_COLOR_WHITE, sel ? GLCD_COLOR_CYAN : C_BG, 1, 25);
}

static void scr_menu(void)
{
    s_ui = UI_MENU;
    lcd_screen("Menu", 1);
    for (int i = 0; i < 6; i++) menu_row(i, i == s_sel, menu_txt[i]);
    lcd_status("Press: open", GLCD_COLOR_WHITE);
}

/* ============================= Settings ============================= */
static void set_row_text(int i, char *b)
{
    char *d = b;
    switch (i) {
    case 0: d = ap_s(d, "Change PIN"); break;
    case 1: d = ap_s(d, "Factory reset"); break;
    default: d = ap_s(d, "Back"); break;
    }
}

static void set_row(int i)
{
    char b[24];
    set_row_text(i, b);
    menu_row(i, i == s_sel, b);
}

static void scr_settings(void)
{
    s_ui = UI_SET;
    lcd_screen("Settings", 0);
    for (int i = 0; i < 3; i++) set_row(i);
    lcd_status("Press: open  Left: back", GLCD_COLOR_WHITE);
}

static void settings_leave(void)
{
    s_sel = 0;
    scr_menu();
}

/* ============================= Password ============================= */
enum { PW_ENTER, PW_OLD, PW_NEW, PW_AGAIN };
static uint8_t  pw_step, pw_len, pw_buf[6], pw_new[6], pw_wrong, pw_from_set, pw_locked;
static uint32_t pw_lock_end, pw_lock_shown;

static void pw_msg(const char *s, uint32_t c) { lcd_text(8u, 140u, s, c, C_BG, 1, 25); }

static void pw_boxes(void)
{
    for (uint32_t i = 0; i < 6u; i++) {
        uint32_t x = 16u + i * 50u;
        char t[2] = { (i >= pw_len) ? '_' :                 /* new PIN: digits shown */
                      (pw_step == PW_NEW || pw_step == PW_AGAIN) ? (char)('0' + pw_buf[i]) : '*', 0 };
        lcd_fill(x, 80u, 40u, 44u, GLCD_COLOR_DARK_GREY);
        lcd_text(x + 14u, 94u, t, GLCD_COLOR_WHITE, GLCD_COLOR_DARK_GREY, 1, 0);
    }
}

static void pw_prompt(void)
{
    static const char * const p[4] = { "Enter PIN", "Old PIN", "New PIN", "New PIN again" };
    lcd_text(8u, 40u, p[pw_step], GLCD_COLOR_WHITE, C_BG, 1, 25);
    pw_len = 0;
    pw_boxes();
}

static void scr_pwd(uint8_t step, uint8_t from_set)
{
    s_ui = UI_PWD;
    pw_step = step;
    pw_from_set = from_set;
    zone_leds(-1);
    lcd_screen(from_set ? "Change PIN" : "Password", 0);
    pw_prompt();
    lcd_text(8u, 180u, "Z1-Z5 = digits 1-5   Z6 = delete", GLCD_COLOR_LIGHT_GREY, C_BG, 0, 0);
    lcd_text(8u, 192u, "Joystick press = Enter", GLCD_COLOR_LIGHT_GREY, C_BG, 0, 0);
    lcd_status("Press=OK  Left=exit", GLCD_COLOR_WHITE);
    if (pw_locked) pw_lock_shown = 0;
}

static int pw_same(const uint8_t *a, const uint8_t *b)
{
    for (int i = 0; i < 6; i++) if (a[i] != b[i]) return 0;
    return 1;
}

static void pw_tap(int zone, int votes)
{
    if (pw_locked) return;
    if (zone == 5) {                                    /* Z6 = delete */
        if (pw_len) pw_len--;
    } else if (pw_len < 6u) {
        pw_buf[pw_len++] = (uint8_t)(zone + 1);
        } else {
        pw_msg("6 digits: press OK", GLCD_COLOR_YELLOW);
        return;
    }
    play(1200u, 3u, 15u);                               /* same click for every zone */
    pw_msg("", C_BG);
    pw_boxes();
}

static void pw_enter(void)
{
    static const uint32_t t_ok[3] = { 523u, 659u, 1047u };
    if (pw_locked) return;
    if (pw_len < 6u) { pw_msg("Need 6 digits", GLCD_COLOR_YELLOW); return; }
    if (pw_step == PW_NEW) {
        for (int i = 0; i < 6; i++) pw_new[i] = pw_buf[i];
        pw_step = PW_AGAIN; pw_prompt(); pw_msg("", C_BG);
        return;
    }
    if (pw_step == PW_AGAIN) {
        if (pw_same(pw_buf, pw_new)) {
            for (int i = 0; i < 6; i++) S.pin[i] = pw_new[i];
            pw_msg("Saving...", GLCD_COLOR_YELLOW);
            settings_save();
            pw_step = PW_ENTER; pw_prompt();
            pw_msg("PIN changed", GLCD_COLOR_GREEN);
            tune(t_ok, 3, 80u);
        } else {
            pw_step = PW_NEW; pw_prompt();
            pw_msg("Did not match", GLCD_COLOR_RED);
            play(220u, 6u, 300u);
        }
        return;
    }
    if (pw_same(pw_buf, S.pin)) {                       /* PW_ENTER or PW_OLD */
        pw_wrong = 0;
        if (pw_step == PW_OLD) { pw_step = PW_NEW; pw_prompt(); pw_msg("", C_BG); return; }
        pw_prompt();
        pw_msg("UNLOCKED", GLCD_COLOR_GREEN);
        tune(t_ok, 3, 80u);
        return;
    }
    pw_wrong++;
    pw_prompt();
    pw_msg("WRONG PIN", GLCD_COLOR_RED);
    play(220u, 6u, 300u);
    if (pw_wrong >= 3u) {
        pw_wrong  = 0;
        pw_locked = 1;
        pw_lock_end = g_ms + 30000u;
        pw_lock_shown = 0;
    }
}

static void pw_service(void)                            /* lock-out countdown */
{
    uint32_t left;
    if (!pw_locked) return;
    if ((int32_t)(g_ms - pw_lock_end) >= 0) {
        pw_locked = 0;
        if (s_ui == UI_PWD) pw_msg("Try again", GLCD_COLOR_WHITE);
        return;
    }
    left = (pw_lock_end - g_ms + 999u) / 1000u;
    if (s_ui == UI_PWD && left != pw_lock_shown) {
        char b[20];
        ap_s(ap_u(ap_s(b, "Locked: "), left), " s");
        pw_msg(b, GLCD_COLOR_RED);
        pw_lock_shown = left;
    }
}

/* =========================== calibration ============================ */
static uint32_t cal_per_zone(void) { return g_cal_full ? CAL_FULL_PER_ZONE : CAL_MERGE_PER_ZONE; }

static void cal_progress(void)
{
    char b[24], *d = b;
    uint32_t per = cal_per_zone(), done = NN, total = per * NZ;
    d = ap_s(d, "Tap Z"); *d++ = (char)('0' + g_cal_zone); d = ap_s(d, ": ");
    d = ap_u(d, g_cal_count); *d++ = '/'; ap_u(d, per);
    lcd_text(8u, 146u, b, GLCD_COLOR_WHITE, C_BG, 1, 25);
    lcd_fill(20u, 180u, 280u, 12u, GLCD_COLOR_DARK_GREY);
    if (done) lcd_fill(20u, 180u, 280u * done / total, 12u, GLCD_COLOR_GREEN);
}

static void cal_announce_zone(void)
{
    zone_leds((int)g_cal_zone - 1);
    if (s_lit >= 0) draw_zone(s_lit, 0);
    s_lit = (int8_t)(g_cal_zone - 1u);
    draw_zone(s_lit, 1);
    cal_progress();
    wait_beep();
    play(zone_hz[g_cal_zone - 1u], 6u, 60u); wait_beep(); delay_ms(80);
    play(zone_hz[g_cal_zone - 1u], 6u, 60u);
}

static void calibration_start(int full)
{
    NN          = 0;
    g_cal_full  = (uint8_t)(full ? 1 : 0);
    g_cal_zone  = 1;
    g_cal_count = 0;
    g_mode      = 1;
    s_ui        = UI_CAL;
    s_lit       = -1;
    lcd_screen(full ? "Calibrate: full" : "Calibrate: merge", 0);
    draw_grid(-1);
    lcd_status("Left/TAMPER: cancel", GLCD_COLOR_WHITE);
    cal_announce_zone();
}

static void calibration_cancel(void)
{
    g_mode = 0;
    NN     = 0;
    zone_leds(-1);
    play(330u, 6u, 150u);
    scr_play();
    lcd_status("Cancelled", GLCD_COLOR_YELLOW);
}

static void calibration_finish(void)
{
    static const uint32_t t_merge[3] = { 523u, 659u, 784u };
    static const uint32_t t_full[4]  = { 523u, 659u, 784u, 1047u };
    calres_t r;
    char b[56], *d;
    uint32_t c[NZ];
    int saved;

    g_mode = 0;
    zone_leds(-1);
    wait_beep();
    lcd_status("Updating dataset...", GLCD_COLOR_YELLOW);

    cal_apply(g_cal_full, &r);                       /* tap4_core.h */

    lcd_status("Saving...", GLCD_COLOR_YELLOW);
    saved = data_save();

    s_ui = UI_CALDONE;                               /* results screen */
    lcd_screen("Calibration done", 0);
    if (r.checked) {
        d = ap_s(b, "Old data -> new taps: "); d = ap_u(d, r.on_ok); *d++ = '/'; ap_u(d, r.nnew);
        lcd_line(0, b);
        d = ap_s(b, "New taps -> each other: "); d = ap_u(d, r.nn_ok); *d++ = '/'; ap_u(d, r.nnew);
        lcd_line(1, b);
        d = ap_s(b, "Gap ");
        if (r.gap10 < 0) { *d++ = '-'; d = ap_u(d, (uint32_t)(-r.gap10) / 10u); *d++ = '.'; d = ap_u(d, (uint32_t)(-r.gap10) % 10u); }
        else             { d = ap_u(d, (uint32_t)r.gap10 / 10u); *d++ = '.'; d = ap_u(d, (uint32_t)r.gap10 % 10u); }
        ap_s(d, r.full ? " points: setup changed -> FULL" : " points: MERGE");
        lcd_line(2, b);
    } else {
        lcd_line(0, "Full recalibration: old taps replaced");
    }
    d = ap_s(b, "Mis-taps removed "); d = ap_u(d, r.zv_removed); d = ap_s(d, "   oldest trimmed "); ap_u(d, r.trimmed);
    lcd_line(4, b);
    zone_counts(D.y, D.n, c);
    d = ap_s(b, "Dataset "); d = ap_u(d, D.n); d = ap_s(d, " taps: ");
    for (int z = 0; z < NZ; z++) { if (z) *d++ = '/'; d = ap_u(d, c[z]); }
    lcd_line(5, b);
    d = ap_s(b, "k "); d = ap_u(d, (uint32_t)g_k); d = ap_s(d, "   new session "); ap_u(d, r.session);
    lcd_line(6, b);
    lcd_text(4u, TITLE_H + 6u + 8u * 12u, saved ? "Saved to flash" : "SAVE FAILED (used until RESET)",
             saved ? GLCD_COLOR_GREEN : GLCD_COLOR_RED, C_BG, 0, 52);

    if (saved) {
        g_from_flash = 1;
        if (r.full) tune(t_full, 4, 80u);
        else        tune(t_merge, 3, 80u);
    } else {
        play(220u, 6u, 300u); wait_beep();
    }
    lcd_status("Hands off: wait", GLCD_COLOR_YELLOW);
    start_learning();
    s_wait_ready   = 1;
}

static void calibration_add(const float *f)
{
    if (NN >= MAXNEW) return;
    for (int j = 0; j < NF; j++) NX[NN][j] = f[j];
    NY[NN] = (uint8_t)(g_cal_zone - 1u);
    NN++;
    g_cal_count++;
    play(zone_hz[g_cal_zone - 1u], 3u, 25u);       /* tick */
    cal_progress();

    if (g_cal_count < cal_per_zone()) return;
    g_cal_count = 0;
    if (g_cal_zone < NZ) {
        g_cal_zone++;
        delay_ms(300);
        cal_announce_zone();
        return;
    }
    calibration_finish();
}

/* ========================== factory reset =========================== */
static void factory_reset(void)
{
    static const uint32_t t_fall[3] = { 784u, 659u, 523u };
    data_erase();
    g_from_flash   = 0;
    load_builtin();
    tune(t_fall, 3, 100u);
}

/* ============================== buttons ============================= */
static int wakeup_pressed(void) { return (GPIOA->IDR & (1UL << 0u)) != 0u;  }   /* PA0 high  */
static int tamper_pressed(void) { return (GPIOC->IDR & (1UL << 13u)) == 0u; }   /* PC13 low  */

typedef struct { int last, stable, same; } debounce_t;

static void debounce(debounce_t *d, int raw)      /* 6 equal reads, 5 ms apart */
{
    if (raw == d->last) { if (d->same < 6) d->same++; }
    else                { d->same = 0; d->last = raw; }
    if (d->same >= 6) d->stable = raw;
}

static debounce_t s_wk, s_tp;
static uint8_t    s_tp_armed;      /* press started outside calibration      */
static uint8_t    s_tp_ignore;     /* ignore TAMPER until it is released     */
static uint32_t   s_tp_down;       /* g_ms when it was pressed               */

static void buttons_service(void)
{
    static uint32_t t_last;
    int was;
    if ((g_ms - t_last) < 5u) return;
    t_last = g_ms;

    was = s_wk.stable;                                   /* WAKEUP: mute */
    debounce(&s_wk, wakeup_pressed());
    if (!was && s_wk.stable) {
        g_mute = g_mute ? 0u : 1u;
        if (g_mute) { buzzer_off(); }
        else        { buzzer_beep(659u, 40u); }
    }

    was = s_tp.stable;                                   /* TAMPER */
    debounce(&s_tp, tamper_pressed());
    if (!was && s_tp.stable) {                           /* pressed */
        if (s_tp_ignore || s_ui == UI_PWD) {
            s_tp_ignore = 1;                             /* not in Password */
        } else if (g_mode == 1u) {
            calibration_cancel();
            s_tp_ignore = 1;
        } else {
            s_tp_armed = 1;
            s_tp_down  = g_ms;
        }
    }
    if (s_tp.stable && s_tp_armed && (g_ms - s_tp_down) >= HOLD_MS) {   /* held 3 s */
        s_tp_armed = 0;
        play(262u, 6u, 200u); wait_beep(); delay_ms(120);
        play(262u, 6u, 200u); wait_beep();
        calibration_start(1);
    }
    if (was && !s_tp.stable) {                           /* released */
        if (s_tp_armed) { s_tp_armed = 0; calibration_start(0); }   /* short press */
        s_tp_ignore = 0;
    }
}

/* Blocking short beep, only used at start-up */
static void buzzer_chirp(uint32_t hz)
{
    buzzer_beep(hz, 50u);
    while (s_beeping) buzzer_service();
    delay_ms(120);
}

/* ============================ a new tap ============================= */
static void classify_and_act(const float *f)
{
    int      votes, zone;
    int      pwd = (s_ui == UI_PWD);
    uint32_t t0 = DWT->CYCCNT;

    zone   = classify(f, &votes);                      /* tapf_core.h: 6 of 9 */
    g_t_us = (DWT->CYCCNT - t0) / (g_clock_hz / 1000000u);
    g_votes = (uint8_t)votes;

    if (pwd) {                                         /* no LEDs, no notes */
        if (zone < 0) {
            g_rejects++;
            if (!pw_locked) pw_msg("Unsure: tap again", GLCD_COLOR_YELLOW);
        } else {
            pw_tap(zone, votes);
        }
        return;
    }

    if (zone < 0) {                                  /* unsure: ignore */
        g_zone = 0;
        g_rejects++;
        zone_leds(-1);
    } else {
        uint32_t vol = volume_for(g_strength);
        g_zone = (uint8_t)(zone + 1);
        zone_leds(zone);
        play(zone_hz[zone], vol, g_beep_ms);         /* sound first, then text */
    }
    if (s_ui == UI_PLAY) {
        if (s_lit >= 0 && s_lit != zone) draw_zone(s_lit, 0);
        if (zone >= 0 && s_lit != zone) draw_zone(zone, 1);
        s_lit = (int8_t)zone;
        play_result(zone, votes);
    }
}

static void process_tap(void)
{
    int32_t  peak[NUM_SENSORS], onset[NUM_SENSORS];
    uint32_t energy[NUM_SENSORS];
    uint32_t sat = 0;
    int32_t  half = s_cap_thresh / 2;

    for (int s = 0; s < NUM_SENSORS; s++) {
        uint64_t e = 0;
        peak[s] = 0; onset[s] = 255;
        for (uint32_t k = 0; k < WIN; k++) {
            int32_t raw = s_cap[s][k];
            int32_t d   = raw - s_cap_base[s];
            int32_t a   = (d < 0) ? -d : d;
            if (raw >= CLIP_LEVEL || raw <= -CLIP_LEVEL) sat |= (1u << s);
            e += (uint64_t)((int64_t)d * d);
            if (a > peak[s])               peak[s] = a;
            if (onset[s] == 255 && a > half) onset[s] = (int32_t)k;
        }
        energy[s] = (uint32_t)(e >> 10);
    }

    g_taps++;
    if (sat) g_sat_taps++;
    for (int s = 0; s < NUM_SENSORS; s++) {           /* for the Watch window */
        g_peak[s] = peak[s]; g_energy[s] = energy[s]; g_onset[s] = onset[s];
    }
    g_first = (uint8_t)s_cap_first;

    g_strength = (uint32_t)(peak[0] + peak[1] + peak[2]);
    if (g_mode != 1u && s_ui == UI_ABC) {             /* A/B/C: no kNN at all */
        abc_tap(energy, onset);
        return;
    }
    {
        float f[NF];
        make_features(peak, energy, onset, f);         /* tapf_core.h */
        for (int j = 0; j < NF; j++) g_feat[j] = f[j];  /* for the Watch window */
        if (g_mode == 1u) calibration_add(f);
        else              classify_and_act(f);
    }
}

/* ============================ joystick UI =========================== */
static void menu_move(int n, int dir, void (*row)(int))
{
    int old = s_sel;
    s_sel = (uint8_t)((s_sel + n + dir) % n);
    row(old);
    row(s_sel);
}
static void menu_row_i(int i) { menu_row(i, i == s_sel, menu_txt[i]); }

static void menu_open(int i)
{
    switch (i) {
    case 0: scr_play(); break;
    case 1: scr_abc(); break;
    case 2: scr_pwd(PW_ENTER, 0); break;
    case 3: calibration_start(0); break;
    case 4: calibration_start(1); break;
    default: s_sel = 0; scr_settings(); break;
    }
}

static void ui_joy(uint32_t b)                       /* one newly pressed direction */
{
    switch (s_ui) {
    case UI_PLAY:
        if (b == JOY_CENTER) { s_sel = 0; scr_menu(); }
        break;
    case UI_ABC:
        if (b == JOY_CENTER) { s_sel = 1; zone_leds(-1); scr_menu(); }
        break;
    case UI_MENU:
        if (b == JOY_UP)        menu_move(6, -1, menu_row_i);
        else if (b == JOY_DOWN) menu_move(6, +1, menu_row_i);
        else if (b == JOY_LEFT) scr_play();
        else                    menu_open(s_sel);          /* press or right */
        break;
    case UI_PWD:
        if (b == JOY_CENTER) pw_enter();
        else if (b == JOY_LEFT) { if (pw_from_set) { s_sel = 0; scr_settings(); } else { s_sel = 2; scr_menu(); } }
        break;
    case UI_CAL:
        if (b == JOY_LEFT) calibration_cancel();
        break;
    case UI_CALDONE:
        if (!s_wait_ready) scr_play();
        break;
    case UI_SET:
        if (b == JOY_UP)        menu_move(3, -1, set_row);
        else if (b == JOY_DOWN) menu_move(3, +1, set_row);
        else if (b == JOY_RIGHT) { }
        else if (b == JOY_LEFT || s_sel == 2u) settings_leave();
        else if (s_sel == 0u) scr_pwd(PW_OLD, 1);
        else if (s_sel == 1u) {
            s_ui = UI_CONFIRM;
            lcd_screen("Factory reset", 0);
            lcd_text(8u, 60u, "Erase calibrations", GLCD_COLOR_WHITE, C_BG, 1, 0);
            lcd_text(8u, 88u, "and use the built-in", GLCD_COLOR_WHITE, C_BG, 1, 0);
            lcd_text(8u, 116u, "dataset?", GLCD_COLOR_WHITE, C_BG, 1, 0);
            lcd_status("Press=yes  Left=no", GLCD_COLOR_YELLOW);
        }
        break;
    case UI_CONFIRM:
        if (b == JOY_CENTER) {
            lcd_status("Erasing...", GLCD_COLOR_YELLOW);
            factory_reset();
            start_learning();
            s_wait_ready   = 1;
            scr_play();
        } else if (b == JOY_LEFT) { s_sel = 1; scr_settings(); }
        break;
    default:
        break;
    }
}

static void joy_service(void)                        /* every 20 ms, debounced */
{
    static uint32_t t_last, last, stable, t_retry;
    int32_t r;
    if ((g_ms - t_last) < 20u) return;
    t_last = g_ms;
    if (g_joy_id != 0x0811u && (g_ms - t_retry) >= 1000u) {   /* id not read yet: */
        t_retry = g_ms;                                         /* set up again,    */
        joy_init();                                             /* once a second    */
    }
    r = joy_get();
    if (r < 0) return;
    if ((uint32_t)r == last && (uint32_t)r != stable) {
        uint32_t pressed = (uint32_t)r & ~stable;
        stable = (uint32_t)r;
        for (uint32_t b = JOY_UP; b <= JOY_CENTER; b <<= 1) if (pressed & b) { ui_joy(b); break; }
    }
    last = (uint32_t)r;
}

/* ================================ main ============================== */
int main(void)
{
    uint32_t t_beat;
    int      factory = 0;

    clock_init();
    SystemCoreClockUpdate();
    SysTick_Config(g_clock_hz / 1000u);

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;   /* cycle counter */
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    gpio_init();
    spi_init();
    buzzer_init();
    if (wakeup_pressed()) g_mute = 1;   /* WAKEUP held at reset: start muted */
    if (tamper_pressed()) factory = 1;  /* TAMPER held at reset: factory reset */
    settings_load();

    GLCD_Initialize();
    lcd_screen("TapTable", 0);
    lcd_text(8u, 60u, "Starting...", GLCD_COLOR_WHITE, C_BG, 1, 0);
    lcd_text(8u, 92u, "Keep hands off", GLCD_COLOR_YELLOW, C_BG, 1, 0);
    lcd_text(8u, 120u, "the pad", GLCD_COLOR_YELLOW, C_BG, 1, 0);
    joy_init();

    /* which sensors answer? PG6/PG7/PG8 = S1/S2/S3 for 2 s */
    for (int s = 0; s < NUM_SENSORS; s++) {
        g_whoami[s]    = mpu_read(s, REG_WHO_AM_I);
        s_sensor_ok[s] = (g_whoami[s] != 0x00u) && (g_whoami[s] != 0xFFu);
        chk_led(s, s_sensor_ok[s]);
    }
    g_range = DEFAULT_RANGE_G;
    for (int s = 0; s < NUM_SENSORS; s++) {
        if (!s_sensor_ok[s]) continue;
        for (int tries = 0; tries < 3; tries++) {
            mpu_init(s, g_range);
            if ((g_cfg[s] & 0x18u) == range_bits(g_range)) break;
        }
    }
    for (int s = 0; s < NUM_SENSORS; s++) if (s_sensor_ok[s]) buzzer_chirp(tone_hz[s]);
    delay_ms(1700);
    for (int s = 0; s < NUM_SENSORS; s++) chk_led(s, 0);

    if (factory) {                      /* TAMPER held through RESET */
        static const uint32_t t_fall[3] = { 784u, 659u, 523u };
        data_erase();
        tune(t_fall, 3, 100u);
        s_tp.last = s_tp.stable = 1; s_tp.same = 6;   /* this press is not a calibration */
        s_tp_ignore = 1;
    }
    data_init();

    spi_set_divider(spi_div_read);
    tim2_init();
    start_learning();                   /* keep the pad still for about 1.2 s */
    s_wait_ready = 1;
    scr_play();
    t_beat = g_ms;

    while (1) {
        if (s_wait_ready && s_state == ST_IDLE) {      /* rest values learned */
            s_wait_ready = 0;
            if (s_ui == UI_PLAY && D.n) lcd_status("Press: menu", GLCD_COLOR_WHITE);
            if (s_ui == UI_CALDONE)     lcd_status("Press: continue", GLCD_COLOR_GREEN);
        }

        if (s_cap_ready) {
            process_tap();
            s_cap_ready = 0;
        }

        buzzer_service();
        buttons_service();
        joy_service();
        pw_service();

        if ((g_ms - t_beat) >= 500u) {
            HEART_PORT->ODR ^= (1UL << HEART_PIN);     /* heartbeat PH2 */
            t_beat += 500u;
        }
    }
}
