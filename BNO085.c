#include "LPC17xx.h"
#include <stdio.h>
#include <stdint.h>

// ------ Device/Protocol Settings ------
#define BNO085_ADDR        (0x4A << 1)  // Default addr
#define LED_PIN            (1 << 4)     // P0.4 for LED
#define MAX_STEPS          9999          // Max LCD-stable step count

unsigned long init_command[] = {0x30, 0x30, 0x30, 0x20, 0x28, 0x0C, 0x06, 0x01, 0x80};
unsigned int flag1, temp1, temp2, flag2;
unsigned int i;
char buffer[16];
volatile int16_t accel_x;
uint8_t rx_buffer[32];
int16_t prev_accel_x = 0;
unsigned int step_count = 0;
unsigned int threshold = 400;

// LCD Control Macros
#define RS_CTRL  0x08000000 // P0.27
#define EN_CTRL  0x10000000 // P0.28
#define DT_CTRL  0x07800000 // P0.23–P0.26

// ------- Helper Function Prototypes --------
void lcd_write(void);
void port_write(void);
void delay_lcd(unsigned int);
void delay_ms(uint32_t);
void I2C1_Init(void);
void I2C1_Start(void);
void I2C1_Stop(void);
void I2C1_Write(uint8_t data);
uint8_t I2C1_Read(uint8_t ack);
void GPIO_Init(void);
void BNO085_Init(void);
int BNO085_SetAccelerometerFeature(void);
int BNO085_WaitForReport(uint8_t* rx, uint8_t* report_id);
int BNO085_ReadAccel(int16_t* accel_x);

// ------------ LCD Write/Handling -----------
void lcd_write(void) {
    flag2 = (flag1 == 1) ? 0 : ((temp1 == 0x30) || (temp1 == 0x20)) ? 1 : 0;
    temp2 = (temp1 & 0xF0) << 19;
    port_write();
    if (flag2 == 0) {
        temp2 = (temp1 & 0x0F) << 23;
        port_write();
    }
}

void port_write(void) {
    LPC_GPIO0->FIOPIN = temp2;
    (flag1 == 0) ? (LPC_GPIO0->FIOCLR = RS_CTRL) : (LPC_GPIO0->FIOSET = RS_CTRL);
    LPC_GPIO0->FIOSET = EN_CTRL;
    delay_lcd(1500); // Calibrate delay for your LCD
    LPC_GPIO0->FIOCLR = EN_CTRL;
    delay_lcd(1500);
}

void delay_lcd(unsigned int r1) {
    volatile unsigned int r;
    for (r = 0; r < r1; r++);
}

void delay_ms(uint32_t ms) {
    volatile uint32_t i;
    for (i = 0; i < ms * 9000; i++) { __NOP(); }
}

// ----------- I2C Helpers ----------
void I2C1_Init(void) {
    LPC_SC->PCONP |= (1 << 19);
    LPC_SC->PCLKSEL1 &= ~(3 << 6);
    LPC_PINCON->PINSEL1 |= (3 << 6) | (3 << 8);  // SDA1, SCL1
    LPC_PINCON->PINMODE1 &= ~((3 << 6) | (3 << 8));
    LPC_I2C1->I2SCLH = 60;
    LPC_I2C1->I2SCLL = 60;
    LPC_I2C1->I2CONCLR = 0x6C;
    LPC_I2C1->I2CONSET = (1 << 6); // Enable I2C1
}

void I2C1_Start(void) {
    LPC_I2C1->I2CONSET = (1 << 5);
    while (!(LPC_I2C1->I2CONSET & (1 << 3)));
    LPC_I2C1->I2CONCLR = (1 << 5);
}

void I2C1_Stop(void) {
    LPC_I2C1->I2CONSET = (1 << 4);
    LPC_I2C1->I2CONCLR = (1 << 3);
    while (LPC_I2C1->I2CONSET & (1 << 4));
}

void I2C1_Write(uint8_t data) {
    LPC_I2C1->I2DAT = data;
    LPC_I2C1->I2CONCLR = (1 << 3);
    while (!(LPC_I2C1->I2CONSET & (1 << 3)));
}

uint8_t I2C1_Read(uint8_t ack) {
    if (ack)
        LPC_I2C1->I2CONSET = (1 << 2);
    else
        LPC_I2C1->I2CONCLR = (1 << 2);
    LPC_I2C1->I2CONCLR = (1 << 3);
    while (!(LPC_I2C1->I2CONSET & (1 << 3)));
    return LPC_I2C1->I2DAT;
}

// --------- GPIO/LED Initialization --------
void GPIO_Init(void) {
    LPC_PINCON->PINSEL0 &= ~(3 << 8);  // P0.4 = GPIO
    LPC_GPIO0->FIODIR |= LED_PIN;
}

// ----------- BNO085 SHTP/Feature Setup ----------
void BNO085_Init(void) {
    delay_ms(700);

    // Soft Reset SHTP (Channel 1 Command) -- recommended but optional for minimum use
    I2C1_Start();
    I2C1_Write(BNO085_ADDR);
    I2C1_Write(0x00); // SHTP header
    I2C1_Write(0x06); // length
    I2C1_Write(0x00); // length
    I2C1_Write(0x01); // Channel 1
    I2C1_Write(0x00); // Sequence
    I2C1_Write(0xF2); // Command: Reset
    I2C1_Stop();
    delay_ms(500);

    // Enable Accelerometer feature--send feature report to SHTP Channel 2 (minimum viable code, normally use vendor library)
    BNO085_SetAccelerometerFeature();
    delay_ms(50);
}

// Minimum SHTP enable feature: Channel 2, Report ID 0xFD (Set Feature Command)
int BNO085_SetAccelerometerFeature(void) {
    I2C1_Start();
    I2C1_Write(BNO085_ADDR);
    // Sample SHTP header+report: Adjust as needed for feature
    I2C1_Write(0x0B); // Packet length LSB
    I2C1_Write(0x00); // Packet length MSB
    I2C1_Write(0x02); // Channel 2
    I2C1_Write(0x00); // Sequence # (0 for single)
    I2C1_Write(0xFD); // Set Feature Command
    I2C1_Write(0x01); // Report ID 1 = accelerometer
    I2C1_Write(0x00); // Reserved
    I2C1_Write(0x01); // Feature enable (1=on)
    I2C1_Write(0x00); // time between reports (optional, less urgent)
    I2C1_Write(0x00); // further params
    I2C1_Write(0x00); // further params
    I2C1_Stop();
    return 0;
}

// Wait for incoming packet and extract report
int BNO085_WaitForReport(uint8_t* rx, uint8_t* report_id) {
    I2C1_Start();
    I2C1_Write(BNO085_ADDR | 1); // READ mode

    // Adjust for actual BNO085 packet structure
    for (i = 0; i < 14; i++)
        rx[i] = I2C1_Read(i < 13 ? 1 : 0);

    I2C1_Stop();
    *report_id = rx[2]; // In practice, extract correct report id from packet header
    return 0;
}

// Read latest accel_x (assuming last valid new report, not just buffer repeat)
int BNO085_ReadAccel(int16_t* accel_x) {
    uint8_t report_id;
    BNO085_WaitForReport(rx_buffer, &report_id);
    // Check for correct report_id
    if (report_id != 0x05) return -1;

    // Data bytes: accel_x typically at rx_buffer[8:9] for BNO085 report 0x05 (check your datasheet)
    *accel_x = (int16_t)((rx_buffer[9] << 8) | rx_buffer[8]);

    // Data validity check
    if (*accel_x == -1 || *accel_x == 0xFFFF || *accel_x == 0x0000)
        return -1;
    return 0;
}

// ----------- Main Application ---------
int main(void) {
    SystemInit();
    SystemCoreClockUpdate();

    LPC_GPIO0->FIODIR = DT_CTRL | RS_CTRL | EN_CTRL;
    flag1 = 0;

    for (i = 0; i < 9; i++) {
        temp1 = init_command[i];
        lcd_write();
    }

    GPIO_Init();
    I2C1_Init();
    BNO085_Init();

    step_count = 0;
    prev_accel_x = 0;
    uint32_t last_step_time = 0;
    const uint32_t min_step_ms = 250;
    static int16_t filt_accel_x = 0;

    while (1) {
        int err = BNO085_ReadAccel(&accel_x);

        // --- Error Handling ---
        if (err != 0) {
            flag1 = 0; temp1 = 0x80; lcd_write();
            flag1 = 1; sprintf(buffer, "ERR ACCEL"); i = 0;
            while (buffer[i] != '\0') { temp1 = buffer[i++]; lcd_write(); }
            for (i = 0; i < 16; i++) buffer[i] = 0;
            LPC_GPIO0->FIOCLR = LED_PIN;
            delay_ms(200);
            continue;
        }

        // --- Acceleration display ---
        sprintf(buffer, "%d", accel_x);
        flag1 = 0; temp1 = 0x80; lcd_write();
        flag1 = 1; i = 0;
        while (buffer[i] != '\0') { temp1 = buffer[i++]; lcd_write(); }
        for (i = 0; i < 16; i++) buffer[i] = '\0';

        // --- Step detection (with filtering and debounce) ---
        filt_accel_x = (filt_accel_x * 3 + accel_x) / 4;
        uint32_t curr_time = 0; // TODO: Use SysTick (add hardware timer) or increment in delay_ms
        if (
            ((prev_accel_x < -threshold && filt_accel_x > threshold) ||
            (prev_accel_x > threshold && filt_accel_x < -threshold))
            && ((curr_time - last_step_time) > min_step_ms)
        ) {
            if (step_count < MAX_STEPS) step_count++;
            last_step_time = curr_time;
        }
        prev_accel_x = filt_accel_x;

        // --- Step count display ---
        flag1 = 0; temp1 = 0xC0; lcd_write();
        flag1 = 1; sprintf(buffer, "Steps:%d", step_count); i = 0;
        while (buffer[i] != '\0') { temp1 = buffer[i++]; lcd_write(); }
        for (i = 0; i < 16; i++) buffer[i] = '\0';

        // --- LED movement indication ---
        if (err == 0) LPC_GPIO0->FIOSET = LED_PIN;
        else LPC_GPIO0->FIOCLR = LED_PIN;

        delay_ms(200);
    }
}
