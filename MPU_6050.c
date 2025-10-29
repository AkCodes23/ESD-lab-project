#include "LPC17xx.h"
#include <stdio.h>
#include <stdint.h>

#define MPU6050_ADDR         (0x68 << 1)     // I2C address: 0xD0 write / 0xD1 read
#define LED_PIN              (1 << 4)        // LED on P0.4
#define PWR_MGMT_1_REG       0x6B
#define WHO_AM_I_REG         0x75
#define ACCEL_XOUT_H         0x3B

/* LCD control pins (P0.23–P0.28) */
#define DT_CTRL              0x07800000UL    // P0.23–P0.26 ? D4–D7
#define RS_CTRL              0x08000000UL    // P0.27 ? RS
#define EN_CTRL              0x10000000UL    // P0.28 ? EN

#define MAX_ABS_G            17000
#define DEBOUNCE_SAMPLES     4
#define LCD_WIDTH            16
#define THRESHOLD            400

/* initialization sequence (nibble/byte values) */
unsigned long init_command[] = {0x30,0x30,0x30,0x20,0x28,0x0C,0x06,0x01,0x80};

char buffer[LCD_WIDTH + 1];
unsigned int flag1, temp1, temp2;
int16_t accel_x = 0;
int16_t prev_accel_x = 0;
int32_t accel_history[DEBOUNCE_SAMPLES] = {0};
uint8_t history_index = 0;
uint8_t step_debounce = 0;
uint32_t step_count = 0;

/* Prototypes */
void delay_ms(uint32_t ms);
void lcd_pulse(void);
void lcd_write_nibble(uint8_t nibble);
void lcd_write_byte(uint8_t data, uint8_t rs);
void lcd_init(void);
void lcd_set_cursor(uint8_t line);
void lcd_print(const char *str, uint8_t line);
void GPIO_Init(void);
void I2C1_Init(void);
static void I2C1_Start(void);
static void I2C1_Stop(void);
static void I2C1_Write(uint8_t data);
static uint8_t I2C1_Read(uint8_t ack);
uint8_t read_MPU6050_register(uint8_t reg);
int read_MPU6050_accel_x(int16_t *accel_x);

/* Simple blocking delay (calibrated for your clock — may be approximate) */
void delay_ms(uint32_t ms) {
    volatile uint32_t i;
    for (i = 0; i < ms * 8500; i++) __NOP();
}

/* ===== LCD Functions ===== */
void lcd_pulse(void) {
    /* pulse EN: EN = 1, small delay, EN = 0 */
    LPC_GPIO0->FIOSET = EN_CTRL;
    delay_ms(1);
    LPC_GPIO0->FIOCLR = EN_CTRL;
    delay_ms(1);
}

/* Write 4-bit nibble to data pins mapped at P0.23..P0.26 */
void lcd_write_nibble(uint8_t nibble) {
    uint32_t val;
    /* clear data pins (mask) and set new nibble shifted into bits 23..26 */
    LPC_GPIO0->FIOCLR = DT_CTRL;
    val = ((uint32_t)(nibble & 0x0F) << 23) & DT_CTRL;
    LPC_GPIO0->FIOSET = val;
    lcd_pulse();
}

/* Write full byte in 4-bit mode. rs==1 => data, rs==0 => command */
void lcd_write_byte(uint8_t data, uint8_t rs) {
    if (rs)
        LPC_GPIO0->FIOSET = RS_CTRL;
    else
        LPC_GPIO0->FIOCLR = RS_CTRL;

    /* high nibble */
    lcd_write_nibble((uint8_t)(data >> 4));
    /* low nibble */
    lcd_write_nibble((uint8_t)(data & 0x0F));
}

void lcd_init(void) {
    int i;
    /* According to HD44780 4-bit init sequence */
    for (i = 0; i < 3; i++) {
        /* send 0x03 as nibble */
        lcd_write_nibble(0x03);
        delay_ms(5);
    }
    /* set to 4-bit mode */
    lcd_write_nibble(0x02);
    delay_ms(2);

    /* function set: 4-bit, 2 lines, 5x8 font */
    lcd_write_byte(0x28, 0);
    /* display on, cursor off, blink off */
    lcd_write_byte(0x0C, 0);
    /* entry mode set: increment, no shift */
    lcd_write_byte(0x06, 0);
    /* clear display */
    lcd_write_byte(0x01, 0);
    delay_ms(2);
}

void lcd_set_cursor(uint8_t line) {
    if (line == 1)
        lcd_write_byte(0x80, 0);
    else
        lcd_write_byte(0xC0, 0);
}

/* Print exactly LCD_WIDTH characters on the specified line (pad with spaces) */
void lcd_print(const char *str, uint8_t line) {
    int idx;
    lcd_set_cursor(line);
    for (idx = 0; idx < LCD_WIDTH; idx++) {
        char c = (str[idx] != '\0') ? str[idx] : ' ';
        lcd_write_byte((uint8_t)c, 1);
    }
}

/* ===== GPIO & I2C Init ===== */
void GPIO_Init(void) {
    /* Make sure P0.4 is GPIO for LED */
    LPC_PINCON->PINSEL0 &= ~(3 << 8);   /* P0.4 -> GPIO */
    LPC_GPIO0->FIODIR |= LED_PIN;

    /* Configure LCD pins as outputs (P0.23..P0.28) */
    LPC_GPIO0->FIODIR |= DT_CTRL | RS_CTRL | EN_CTRL;
}

void I2C1_Init(void) {
    /* Enable power to I2C1 */
    LPC_SC->PCONP |= (1 << 19);
    /* Use default peripheral clock */
    LPC_SC->PCLKSEL1 &= ~(3 << 6);
    /* Configure P0.19 SDA1, P0.20 SCL1 */
    LPC_PINCON->PINSEL1 |= (3 << 6) | (3 << 8);
    /* I2C timing (approx) */
    LPC_I2C1->I2SCLH = 60;
    LPC_I2C1->I2SCLL = 60;
    /* Clear and enable */
    LPC_I2C1->I2CONCLR = 0x6C;
    LPC_I2C1->I2CONSET = (1 << 6); /* I2EN */
}

/* ===== I2C Core (blocking) ===== */
static void I2C1_Start(void) {
    LPC_I2C1->I2CONSET = (1 << 5); /* STA = 1 */
    /* wait for SI */
    while (!(LPC_I2C1->I2CONSET & (1 << 3)));
    LPC_I2C1->I2CONCLR = (1 << 5); /* clear STA */
}

static void I2C1_Stop(void) {
    LPC_I2C1->I2CONSET = (1 << 4); /* STO = 1 */
    LPC_I2C1->I2CONCLR = (1 << 3); /* clear SI */
}

static void I2C1_Write(uint8_t data) {
    LPC_I2C1->I2DAT = data;
    LPC_I2C1->I2CONCLR = (1 << 3); /* clear SI to start transfer */
    while (!(LPC_I2C1->I2CONSET & (1 << 3))); /* wait for SI */
}

static uint8_t I2C1_Read(uint8_t ack) {
    if (ack)
        LPC_I2C1->I2CONSET = (1 << 2); /* AA = 1 -> ACK */
    else
        LPC_I2C1->I2CONCLR = (1 << 2); /* AA = 0 -> NACK */
    LPC_I2C1->I2CONCLR = (1 << 3); /* clear SI to start read */
    while (!(LPC_I2C1->I2CONSET & (1 << 3)));
    return (uint8_t)(LPC_I2C1->I2DAT & 0xFF);
}

/* ===== MPU6050 Functions ===== */
uint8_t read_MPU6050_register(uint8_t reg) {
    uint8_t val;
    I2C1_Start();
    I2C1_Write(MPU6050_ADDR);    /* write address */
    I2C1_Write(reg);             /* register */
    I2C1_Start();                /* repeated start */
    I2C1_Write(MPU6050_ADDR | 1);/* read address */
    val = I2C1_Read(0);          /* read with NACK (single byte) */
    I2C1_Stop();
    return val;
}

int read_MPU6050_accel_x(int16_t *accel_x) {
    uint8_t msb, lsb;
    I2C1_Start();
    I2C1_Write(MPU6050_ADDR);
    I2C1_Write(ACCEL_XOUT_H);
    I2C1_Start();
    I2C1_Write(MPU6050_ADDR | 1);
    msb = I2C1_Read(1); /* ACK for MSB */
    lsb = I2C1_Read(0); /* NACK for LSB */
    I2C1_Stop();
    *accel_x = (int16_t)(((int16_t)msb << 8) | lsb);
    if (*accel_x == -1 || *accel_x == 0 || *accel_x > MAX_ABS_G || *accel_x < -MAX_ABS_G)
        return -1;
    return 0;
}

/* ===== Main ===== */
int main(void) {
    uint8_t k;
    uint8_t who;
    int valid;
    int idx;
    int32_t avg;
    int i;

    SystemInit();
    SystemCoreClockUpdate();

    GPIO_Init();
    I2C1_Init();
    lcd_init();

    /* Wake up MPU6050 (clear sleep) */
    I2C1_Start();
    I2C1_Write(MPU6050_ADDR);
    I2C1_Write(PWR_MGMT_1_REG);
    I2C1_Write(0x00);
    I2C1_Stop();
    delay_ms(100);

    /* WHO_AM_I check */
    who = read_MPU6050_register(WHO_AM_I_REG);
    if (who != 0x68) {
        lcd_print("MPU6050 ERR!", 1);
        while (1) {
            /* Halt here - error shown on LCD */
        }
    }

    /* Initialize accel_history */
    for (k = 0; k < DEBOUNCE_SAMPLES; k++) accel_history[k] = 0;
    history_index = 0;
    prev_accel_x = 0;
    step_debounce = 0;
    step_count = 0;

    while (1) {
        valid = read_MPU6050_accel_x(&accel_x);
        if (valid != 0) {
            lcd_print("Acc Error!     ", 1);
            LPC_GPIO0->FIOCLR = LED_PIN;
            delay_ms(200);
            continue;
        }

        /* Moving average */
        accel_history[history_index] = accel_x;
        history_index++;
        if (history_index >= DEBOUNCE_SAMPLES) history_index = 0;

        avg = 0;
        for (idx = 0; idx < DEBOUNCE_SAMPLES; idx++) {
            avg += accel_history[idx];
        }
        avg /= DEBOUNCE_SAMPLES;

        /* Step detection: detect crossing large positive/negative threshold */
        if (((prev_accel_x < -THRESHOLD) && (avg > THRESHOLD)) ||
            ((prev_accel_x > THRESHOLD) && (avg < -THRESHOLD))) {
            step_debounce++;
            if (step_debounce >= DEBOUNCE_SAMPLES) {
                step_count++;
                step_debounce = 0;
            }
        } else {
            step_debounce = 0;
        }
        prev_accel_x = (int16_t)avg;

        /* Display */
        sprintf(buffer, "Accel:%ld      ", (long)avg);
        lcd_print(buffer, 1);

        sprintf(buffer, "Steps:%lu      ", (unsigned long)step_count);
        lcd_print(buffer, 2);

        LPC_GPIO0->FIOSET = LED_PIN;
        delay_ms(200);
    }
    /* No return — main loops forever */
}
