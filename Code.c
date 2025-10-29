#include "LPC17xx.h"
#include <stdio.h>  // For sprintf()

// Changed LED pin from P1.18 to P0.4
#define BNO055_ADDR      (0x28 << 1)
#define LED_PIN          (1 << 4)  // Changed: P0.4

unsigned long int init_command[] = {0x30, 0x30, 0x30, 0x20, 0x28, 0x0C, 0x06, 0x01, 0x80};
unsigned int flag1, temp1, temp2, flag2;
unsigned int i;
char buffer[16];
int16_t accel_x;
uint8_t lsb;
uint8_t msb;

int16_t prev_accel_x = 0;
unsigned int step_count = 0;
unsigned int threshold = 400;

#define RS_CTRL 0x08000000 // P0.27, 1<<27
#define EN_CTRL 0x10000000 // P0.28, 1<<28
#define DT_CTRL 0x07800000 // P0.23 to P0.26 data lines, F<<23

void lcd_write(void);
void port_write(void);
void delay_lcd(unsigned int);

void lcd_write(void) {
    flag2 = (flag1 == 1) ? 0 : ((temp1 == 0x30) || (temp1 == 0x20)) ? 1 : 0;
    temp2 = temp1 & 0xF0;
    temp2 = temp2 << 19;
    port_write();

    if (flag2 == 0) {
        temp2 = temp1 & 0x0F;
        temp2 = temp2 << 23;
        port_write();
    }
}

void port_write(void) {
    LPC_GPIO0->FIOPIN = temp2;
    if (flag1 == 0)
        LPC_GPIO0->FIOCLR = RS_CTRL;
    else
        LPC_GPIO0->FIOSET = RS_CTRL;

    LPC_GPIO0->FIOSET = EN_CTRL;
    delay_lcd(1000000);
    LPC_GPIO0->FIOCLR = EN_CTRL;
    delay_lcd(1000000);
}

void delay_lcd(unsigned int r1) {
    unsigned int r;
    for (r = 0; r < r1; r++);
    return;
}

void delay_ms(uint32_t ms) {
    uint32_t i;
    for (i = 0; i < ms * 10000; i++) {
        __NOP();
    }
}

void I2C1_Init(void) {  // Changed: I2C0_Init → I2C1_Init
    LPC_SC->PCONP |= (1 << 19);        // Power up I2C1 (bit 19 instead of bit 7)
    LPC_SC->PCLKSEL1 &= ~(3 << 6);     // Use PCLKSEL1, bits 7:6 for I2C1
    LPC_PINCON->PINSEL1 |= (3 << 6) | (3 << 8);     // P0.19 SDA1 (11), P0.20 SCL1 (11)
    LPC_PINCON->PINMODE1 &= ~((3 << 6) | (3 << 8)); // Enable pull-ups

    LPC_I2C1->I2SCLH = 60;
    LPC_I2C1->I2SCLL = 60;

    LPC_I2C1->I2CONCLR = 0x6C; // Clear flags
    LPC_I2C1->I2CONSET = (1 << 6); // Enable I2C
}

void I2C1_Start(void) {
    LPC_I2C1->I2CONSET = (1 << 5); // STA = 1
    while (!(LPC_I2C1->I2CONSET & (1 << 3))) {
        // Wait for SI
    }
    LPC_I2C1->I2CONCLR = (1 << 5); // Clear STA
}

void I2C1_Stop(void) {
    LPC_I2C1->I2CONSET = (1 << 4); // STO = 1
    LPC_I2C1->I2CONCLR = (1 << 3); // Clear SI
    while (LPC_I2C1->I2CONSET & (1 << 4)) {
        // Wait for stop to finish
    }
}

void I2C1_Write(uint8_t data) {
    LPC_I2C1->I2DAT = data;
    LPC_I2C1->I2CONCLR = (1 << 3); // Clear SI
    while (!(LPC_I2C1->I2CONSET & (1 << 3))) {
        // Wait for SI
    }
}

uint8_t I2C1_Read(uint8_t ack) {
    uint8_t result;
    if (ack) {
        LPC_I2C1->I2CONSET = (1 << 2); // AA = 1
    } else {
        LPC_I2C1->I2CONCLR = (1 << 2); // AA = 0
    }

    LPC_I2C1->I2CONCLR = (1 << 3); // Clear SI
    while (!(LPC_I2C1->I2CONSET & (1 << 3))) {
        // Wait for SI
    }

    result = LPC_I2C1->I2DAT;
    return result;
}

void GPIO_Init(void) {
    // Changed: Set P0.4 as GPIO
    LPC_PINCON->PINSEL0 &= ~(3 << 8);   // Clear bits 9:8 for P0.4 = GPIO
    LPC_GPIO0->FIODIR |= LED_PIN;       // Changed: Use GPIO0 instead of GPIO1
}

int main(void) {
    SystemInit();
    SystemCoreClockUpdate();

    LPC_GPIO0->FIODIR = DT_CTRL | RS_CTRL | EN_CTRL;
    flag1 = 0; // Command

    for (i = 0; i < 9; i++) {
        temp1 = init_command[i];
        lcd_write();
    }

    GPIO_Init();
    I2C1_Init(); // Changed: call I2C1_Init instead of I2C0_Init
    delay_ms(700);

    // Set BNO055 to NDOF mode (0x0C) by writing to 0x3D
    I2C1_Start();
    I2C1_Write(BNO055_ADDR);     // Write mode
    I2C1_Write(0x3D);            // Register address
    I2C1_Write(0x0C);            // NDOF mode
    I2C1_Stop();
    delay_ms(100);

    while (1) {
        // Read 2 bytes from 0x08 and 0x09 for ACCEL_DATA_X
        I2C1_Start();
        I2C1_Write(BNO055_ADDR); // Write address
        I2C1_Write(0x08);        // ACCEL_DATA_X_LSB
        I2C1_Stop();

        I2C1_Start();
        I2C1_Write(BNO055_ADDR | 1); // Read address
        lsb = I2C1_Read(1);  // ACK
        msb = I2C1_Read(0);  // NACK
        I2C1_Stop();

        accel_x = (int16_t)((msb << 8) | lsb);

        // Convert to string
        sprintf(buffer, "%d", accel_x);
        flag1 = 0;
        temp1 = 0x80;
        lcd_write();

        flag1 = 1;
        i = 0;
        while (buffer[i++] != '\0') {
            temp1 = buffer[i - 1];
            lcd_write();
        }

        for (i = 0; i < 16; i++)
            buffer[i] = 0;

        // --- STEP DETECTION LOGIC ---
        if ((prev_accel_x < -threshold && accel_x > threshold) ||
            (prev_accel_x > threshold && accel_x < -threshold)) {
            step_count++;
        }
        prev_accel_x = accel_x;

        // Display step count on LCD second row
        flag1 = 0;
        temp1 = 0xC0;  // Move cursor to second line
        lcd_write();

        flag1 = 1;
        sprintf(buffer, "Steps: %d", step_count);
        i = 0;
        while (buffer[i++] != '\0') {
            temp1 = buffer[i - 1];
            lcd_write();
        }

        if (accel_x != 0xFFFF && accel_x != 0x0000) {
            LPC_GPIO0->FIOSET = LED_PIN; // Changed: Use GPIO0
        } else {
            LPC_GPIO0->FIOCLR = LED_PIN; // Changed: Use GPIO0
        }

        delay_ms(200);
    }
}
