/*
 * TapTable feasibility test
 * Board : Keil MCBSTM32F400 (STM32F407IG)
 * Parts : 3 x MPU6500 motion sensors on SPI3
 *
 * ------------------------------------------------------------------
 * WIRING
 *   STM32 pin          signal        MPU6500 pin (all 3 sensors)
 *   3V3 (header)       +3V3          VCC
 *   GND (header)       GND           GND
 *   PC10               SPI3_SCK      SCL  (SCLK)
 *   PC12               SPI3_MOSI     SDA  (SDI)
 *   PC11               SPI3_MISO     AD0  (SDO)
 *   PF6                CS1           NCS of S1 only
 *   PF7                CS2           NCS of S2 only
 *   PF10               CS3           NCS of S3 only
 *   Leave EDA, ECL, INT unconnected. FSYNC to GND is optional.
 *
 * BOARD SETTINGS
 *   - No SD card in the slot (PC10..PC12 are also the SD-card lines)
 *   - Jumpers J13/J14 on the TX1/RX1 side (not TX3/RX3)
 *   - Jumper J18 (LED_EN) fitted, otherwise the LEDs stay dark
 *
 * ------------------------------------------------------------------
 * WHAT THE LEDS MEAN
 *   PH2 (blinking)      : program is running (heartbeat, 1 blink per second)
 *   PG6 / PG7 / PG8     : first 2 seconds after reset:
 *                           sensor S1 / S2 / S3 answered over SPI
 *                         after that:
 *                           the sensor that shook the MOST in the last tap
 *
 * WHAT TO WATCH IN KEIL (Debug session -> View -> Watch Windows -> Watch 1,
 * and tick View -> Periodic Window Update)
 *   g_whoami[3]  sensor IDs. 0x70 = genuine MPU6500. 0x00 or 0xFF = no answer
 *   g_cfg[3]     read-back of the range setting. 0x10 = our write worked
 *   g_z[3]       live vertical acceleration (4096 = 1 g, about 4096 or -4096 at rest)
 *   g_base[3]    resting value of each sensor (slowly tracked)
 *   g_peak[3]    how hard each sensor shook in the LAST tap (counts above rest)
 *   g_first      sensor (1..3) that crossed the trigger level first in the last tap
 *   g_winner     sensor (1..3) with the largest peak in the last tap
 *   g_taps       number of taps detected since reset
 *   g_wins[3]    how many taps each sensor has "won"
 *   g_rate       how many times per second all 3 sensors are read
 *   g_thresh     trigger level in counts. You may change it live in the
 *                Watch window (start 600; raise if it triggers by itself,
 *                lower if light taps are missed)
 *   g_log[32][3] peaks of the last 32 taps (oldest is overwritten)
 *   g_noise[3]   biggest wiggle of each sensor while the pad is STILL,
 *                over the last second. Set g_thresh to about 5x the
 *                largest of these.
 *                (If g_taps goes up while nobody taps, those are false
 *                triggers: raise g_thresh.)
 *   g_clear      type 1 here in the Watch window to zero g_taps, g_wins
 *                and g_log before each new test run (goes back to 0 itself)
 * ------------------------------------------------------------------
 */

#include "stm32f4xx.h"
#include <stdint.h>

#define NUM_SENSORS   3
#define CAPTURE_MS    30u     /* look for the peak this long after a trigger      */
#define DEADTIME_MS   200u    /* then ignore triggers this long (board still rings) */
#define LOG_SIZE      32

/* MPU6500 register addresses (from the MPU6500 register map) */
#define REG_ACCEL_CONFIG   0x1Cu
#define REG_ACCEL_CONFIG2  0x1Du
#define REG_ACCEL_ZOUT_H   0x3Fu
#define REG_USER_CTRL      0x6Au
#define REG_PWR_MGMT_1     0x6Bu
#define REG_PWR_MGMT_2     0x6Cu
#define REG_WHO_AM_I       0x75u

/* SPI clock divider codes for the BR field: SPI clock = APB1 clock / 2^(code+1) */
#define SPI_DIV_64   5u        /* slow: used while writing settings (must stay < 1 MHz) */
#define SPI_DIV_16   3u        /* fast: used for reading data                            */

/* ------------------------- values to watch -------------------------- */
volatile uint32_t g_ms = 0;
volatile uint8_t  g_whoami[NUM_SENSORS];
volatile uint8_t  g_cfg[NUM_SENSORS];
volatile int16_t  g_z[NUM_SENSORS];
volatile int32_t  g_base[NUM_SENSORS];
volatile int32_t  g_peak[NUM_SENSORS];
volatile int32_t  g_thresh = 600;
volatile uint8_t  g_first;
volatile uint8_t  g_winner;
volatile uint32_t g_taps;
volatile uint32_t g_wins[NUM_SENSORS];
volatile uint32_t g_rate;
volatile int32_t  g_log[LOG_SIZE][NUM_SENSORS];
volatile int32_t  g_noise[NUM_SENSORS];
volatile uint8_t  g_clear;

/* --------------------------- pin tables ----------------------------- */
static GPIO_TypeDef * const cs_port[NUM_SENSORS] = { GPIOF, GPIOF, GPIOF };
static const uint8_t        cs_pin[NUM_SENSORS]  = { 6u, 7u, 10u };   /* PF6, PF7, PF10 */
static const uint8_t        led_pin[NUM_SENSORS] = { 6u, 7u, 8u };    /* PG6, PG7, PG8  */

static uint8_t sensor_ok[NUM_SENSORS];

/* ------------------------- 1 ms time base --------------------------- */
void SysTick_Handler(void)
{
    g_ms++;
}

static void delay_ms(uint32_t ms)
{
    uint32_t t0 = g_ms;
    while ((g_ms - t0) < ms) { }
}

/* ----------------------------- GPIO -------------------------------- */
static void cs_low(int s)  { cs_port[s]->BSRR = (1UL << (cs_pin[s] + 16u)); }
static void cs_high(int s) { cs_port[s]->BSRR = (1UL << cs_pin[s]); }

static void led_set(int s, int on)
{
    if (on) GPIOG->BSRR = (1UL << led_pin[s]);
    else    GPIOG->BSRR = (1UL << (led_pin[s] + 16u));
}

static void leds_all_off(void)
{
    for (int s = 0; s < NUM_SENSORS; s++) led_set(s, 0);
}

static void gpio_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIOFEN |
                    RCC_AHB1ENR_GPIOGEN | RCC_AHB1ENR_GPIOHEN;
    (void)RCC->AHB1ENR;                      /* small delay after enabling clocks */

    /* Chip-selects PF6, PF7, PF10: set HIGH first (sensor not selected),
       then make them outputs, so no sensor is ever selected by accident. */
    for (int s = 0; s < NUM_SENSORS; s++) {
        cs_high(s);
        cs_port[s]->MODER &= ~(3UL << (cs_pin[s] * 2u));
        cs_port[s]->MODER |=  (1UL << (cs_pin[s] * 2u));     /* 01 = output */
    }

    /* LEDs PG6, PG7, PG8 as outputs */
    for (int s = 0; s < NUM_SENSORS; s++) {
        GPIOG->MODER &= ~(3UL << (led_pin[s] * 2u));
        GPIOG->MODER |=  (1UL << (led_pin[s] * 2u));
    }
    /* Heartbeat LED PH2 as output */
    GPIOH->MODER &= ~(3UL << (2u * 2u));
    GPIOH->MODER |=  (1UL << (2u * 2u));

    /* PC10 (SCK), PC11 (MISO), PC12 (MOSI): alternate function 6 = SPI3 */
    for (uint32_t p = 10u; p <= 12u; p++) {
        GPIOC->MODER   &= ~(3UL   << (p * 2u));
        GPIOC->MODER   |=  (2UL   << (p * 2u));               /* 10 = alternate fn */
        GPIOC->OSPEEDR &= ~(3UL   << (p * 2u));
        GPIOC->OSPEEDR |=  (2UL   << (p * 2u));               /* fast              */
        GPIOC->AFR[1]  &= ~(0xFUL << ((p - 8u) * 4u));
        GPIOC->AFR[1]  |=  (6UL   << ((p - 8u) * 4u));        /* AF6               */
    }
    /* Weak pull-up on MISO: with no sensor answering we read 0xFF, not noise */
    GPIOC->PUPDR &= ~(3UL << (11u * 2u));
    GPIOC->PUPDR |=  (1UL << (11u * 2u));
}

/* ------------------------------ SPI -------------------------------- */
static void spi_wait_idle(void)
{
    while (SPI3->SR & SPI_SR_BSY) { }
}

static void spi_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
    (void)RCC->APB1ENR;

    SPI3->CR1 = 0;
    SPI3->CR2 = 0;
    /* Master, software chip-select, mode 3 (CPOL=1, CPHA=1), 8-bit, MSB first */
    SPI3->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI |
                SPI_CR1_CPOL | SPI_CR1_CPHA |
                (SPI_DIV_64 << SPI_CR1_BR_Pos);
    SPI3->CR1 |= SPI_CR1_SPE;
}

static void spi_set_divider(uint32_t code)
{
    spi_wait_idle();
    SPI3->CR1 &= ~SPI_CR1_SPE;
    SPI3->CR1 = (SPI3->CR1 & ~SPI_CR1_BR) | (code << SPI_CR1_BR_Pos);
    SPI3->CR1 |= SPI_CR1_SPE;
}

/* Send one byte and return the byte received at the same time */
static uint8_t spi_xfer(uint8_t out)
{
    while (!(SPI3->SR & SPI_SR_TXE)) { }
    SPI3->DR = out;
    while (!(SPI3->SR & SPI_SR_RXNE)) { }
    return (uint8_t)SPI3->DR;
}

/* ---------------------------- MPU6500 ------------------------------ */
static uint8_t mpu_read(int s, uint8_t reg)
{
    uint8_t v;
    cs_low(s);
    spi_xfer((uint8_t)(reg | 0x80u));      /* top bit 1 = read */
    v = spi_xfer(0x00u);                   /* dummy byte clocks the answer out */
    spi_wait_idle();
    cs_high(s);
    return v;
}

static void mpu_write(int s, uint8_t reg, uint8_t val)
{
    cs_low(s);
    spi_xfer((uint8_t)(reg & 0x7Fu));      /* top bit 0 = write */
    spi_xfer(val);
    spi_wait_idle();
    cs_high(s);
}

/* Read the vertical (Z) acceleration: two bytes, high byte first */
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

static void mpu_init(int s)
{
    mpu_write(s, REG_PWR_MGMT_1, 0x80u);   /* reset the chip                         */
    delay_ms(100);
    mpu_write(s, REG_USER_CTRL, 0x10u);    /* disable I2C so it stays in SPI mode    */
    mpu_write(s, REG_PWR_MGMT_1, 0x01u);   /* wake up, use the best clock            */
    delay_ms(10);
    mpu_write(s, REG_PWR_MGMT_2, 0x07u);   /* gyroscope off, accelerometer on        */
    mpu_write(s, REG_ACCEL_CONFIG, 0x10u); /* range +/- 8 g -> 4096 counts per g     */
    mpu_write(s, REG_ACCEL_CONFIG2, 0x08u);/* fastest output (4 kHz), no filtering   */
    delay_ms(20);
}

/* ------------------------------ main ------------------------------- */
int main(void)
{
    int32_t  base_fx[NUM_SENSORS];      /* resting value x 64, for smooth tracking */
    int32_t  peak[NUM_SENSORS];
    int32_t  dev[NUM_SENSORS];
    enum { IDLE, CAPTURE, DEAD } state = IDLE;
    uint32_t t_event = 0, t_sec, t_beat, loops = 0;
    uint32_t log_i = 0;
    int32_t  noise_max[NUM_SENSORS] = {0};  /* rest wiggle in the current second */

    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000u);   /* interrupt every 1 ms */

    gpio_init();
    spi_init();
    delay_ms(100);                             /* let the sensors power up */

    /* Step 1: which sensors answer? Light their LED. */
    for (int s = 0; s < NUM_SENSORS; s++) {
        g_whoami[s]  = mpu_read(s, REG_WHO_AM_I);
        sensor_ok[s] = (g_whoami[s] != 0x00u) && (g_whoami[s] != 0xFFu);
        led_set(s, sensor_ok[s]);
    }

    /* Step 2: configure the ones that answered, read a setting back */
    for (int s = 0; s < NUM_SENSORS; s++) {
        if (sensor_ok[s]) {
            mpu_init(s);
            g_cfg[s] = mpu_read(s, REG_ACCEL_CONFIG);
        }
    }

    delay_ms(2000);                            /* LEDs show who answered for 2 s */
    leds_all_off();

    spi_set_divider(SPI_DIV_16);               /* faster clock for data reads */

    /* Step 3: measure each sensor's resting value (keep the pad still!) */
    for (int s = 0; s < NUM_SENSORS; s++) base_fx[s] = 0;
    for (int n = 0; n < 256; n++) {
        for (int s = 0; s < NUM_SENSORS; s++) {
            if (sensor_ok[s]) base_fx[s] += mpu_read_z(s);
        }
        delay_ms(1);
    }
    for (int s = 0; s < NUM_SENSORS; s++) {
        g_base[s]  = base_fx[s] / 256;
        base_fx[s] = g_base[s] * 64;
        peak[s]    = 0;
    }

    t_sec  = g_ms;
    t_beat = g_ms;

    /* Step 4: main loop - read, detect taps, show the winner */
    while (1) {
        for (int s = 0; s < NUM_SENSORS; s++) {
            if (sensor_ok[s]) {
                g_z[s] = mpu_read_z(s);
                dev[s] = (int32_t)g_z[s] - g_base[s];
                if (dev[s] < 0) dev[s] = -dev[s];
            } else {
                g_z[s] = 0;
                dev[s] = 0;
            }
        }
        loops++;

        switch (state) {
        case IDLE: {
            int first = 0;
            for (int s = 0; s < NUM_SENSORS; s++) {
                if (sensor_ok[s]) {
                    /* follow slow drift of the resting value */
                    base_fx[s] += (int32_t)g_z[s] - (base_fx[s] / 64);
                    g_base[s]   = base_fx[s] / 64;
                    if (dev[s] > noise_max[s]) noise_max[s] = dev[s];
                }
                if (first == 0 && dev[s] > g_thresh) first = s + 1;
            }
            if (first != 0) {                   /* a tap has started */
                g_first = (uint8_t)first;
                for (int s = 0; s < NUM_SENSORS; s++) peak[s] = dev[s];
                leds_all_off();
                t_event = g_ms;
                state   = CAPTURE;
            }
            break;
        }

        case CAPTURE:
            for (int s = 0; s < NUM_SENSORS; s++) {
                if (dev[s] > peak[s]) peak[s] = dev[s];
            }
            if ((g_ms - t_event) >= CAPTURE_MS) {
                int w = 0;
                for (int s = 1; s < NUM_SENSORS; s++) {
                    if (peak[s] > peak[w]) w = s;
                }
                for (int s = 0; s < NUM_SENSORS; s++) {
                    g_peak[s]       = peak[s];
                    g_log[log_i][s] = peak[s];
                }
                log_i = (log_i + 1u) % LOG_SIZE;
                g_winner = (uint8_t)(w + 1);
                g_wins[w]++;
                g_taps++;
                led_set(w, 1);                  /* light the winner's LED */
                t_event = g_ms;
                state   = DEAD;
            }
            break;

        case DEAD:
        default:
            if ((g_ms - t_event) >= DEADTIME_MS) state = IDLE;
            break;
        }

        /* heartbeat LED PH2: toggle every 500 ms */
        if ((g_ms - t_beat) >= 500u) {
            GPIOH->ODR ^= (1UL << 2u);
            t_beat += 500u;
        }

        /* loop rate, updated once per second */
        if ((g_ms - t_sec) >= 1000u) {
            g_rate = loops;
            loops  = 0;
            t_sec += 1000u;
            for (int s = 0; s < NUM_SENSORS; s++) {
                g_noise[s]   = noise_max[s];
                noise_max[s] = 0;
            }
        }

        /* g_clear = 1 from the Watch window: start a fresh test run */
        if (g_clear) {
            g_taps = 0;
            for (int s = 0; s < NUM_SENSORS; s++) g_wins[s] = 0;
            for (int i = 0; i < LOG_SIZE; i++)
                for (int s = 0; s < NUM_SENSORS; s++) g_log[i][s] = 0;
            log_i   = 0;
            g_clear = 0;
        }
    }
}
