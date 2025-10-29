#include "LPC17xx.h"
#include <stdio.h>

#define MPU6050_ADDR       (0x68 << 1)   // MPU-6050 I2C address shifted left
#define LED_PIN            (1 << 4)      // P0.4 for LED

#define PWR_MGMT_1_REG     0x6B
#define WHO_AM_I_REG       0x75
#define ACCEL_XOUT_H       0x3B

#define RS_CTRL            0x08000000    // P0.27
#define EN_CTRL            0x10000000    // P0.28
#define DT_CTRL            0x07800000    // P0.23-P0.26 LCD data pins

unsigned long init_command[] = {0x30,0x30,0x30,0x20,0x28,0x0c,0x06,0x01,0x80};

char buffer[16];
unsigned int flag1, temp1, temp2, flag2;
unsigned int i;

int16_t accel_x;
int16_t prev_accel_x = 0;
unsigned int step_count = 0;
const unsigned int threshold = 400;

uint8_t lsb, msb;

// Basic delay with approximate calibration for 100MHz CPU clock
void delay_ms(uint32_t ms) {
    volatile uint32_t i;
    for(i = 0; i < ms * 8500; i++) {
        __NOP();
    }
}

// LCD low-level functions
void lcd_write(void) {
    flag2 = (flag1 == 1) ? 0 : ((temp1 == 0x30) || (temp1 == 0x20)) ? 1 : 0;
    temp2 = (temp1 & 0xF0) << 19;
    LPC_GPIO0->FIOPIN = temp2;

    if(flag1 == 0)
        LPC_GPIO0->FIOCLR = RS_CTRL;
    else
        LPC_GPIO0->FIOSET = RS_CTRL;

    LPC_GPIO0->FIOSET = EN_CTRL;
    delay_ms(1);
    LPC_GPIO0->FIOCLR = EN_CTRL;
    delay_ms(1);

    if(flag2 == 0) {
        temp2 = (temp1 & 0x0F) << 23;
        LPC_GPIO0->FIOPIN = temp2;

        if(flag1 == 0)
            LPC_GPIO0->FIOCLR = RS_CTRL;
        else
            LPC_GPIO0->FIOSET = RS_CTRL;

        LPC_GPIO0->FIOSET = EN_CTRL;
        delay_ms(1);
        LPC_GPIO0->FIOCLR = EN_CTRL;
        delay_ms(1);
    }
}

void GPIO_Init(void) {
    // Configure LCD pins as GPIO
    LPC_PINCON->PINSEL0 &= ~(3 << 8);  // P0.4 GPIO for LED
    LPC_GPIO0->FIODIR |= LED_PIN;

    // LCD control pins and data pins write direction
    LPC_GPIO0->FIODIR |= DT_CTRL | RS_CTRL | EN_CTRL;
}

void I2C1_Init(void) {
    LPC_SC->PCONP |= (1 << 19);            // Power on I2C1
    LPC_SC->PCLKSEL1 &= ~(3 << 6);         // Clear peripheral clock bits for I2C1

    LPC_PINCON->PINSEL1 |= (3 << 6) | (3 << 8); // Set P0.19 SDA1 and P0.20 SCL1 pins for I2C1
    LPC_PINCON->PINMODE1 &= ~((3 << 6) | (3 << 8));  // Enable pull-ups

    LPC_I2C1->I2SCLH = 60;  // Set I2C clock high time
    LPC_I2C1->I2SCLL = 60;  // Set I2C clock low time

    LPC_I2C1->I2CONCLR = 0x6C; 
    LPC_I2C1->I2CONSET = (1 << 6); // Enable I2C1 interface
}

// Blocking wait with timeout helper
int I2C1_Wait_For_SI(void) {
    uint32_t timeout = 100000;
    while(!(LPC_I2C1->I2CONSET & (1 << 3))) {
        if(--timeout == 0) return -1;
    }
    return 0;
}

int I2C1_Start(void) {
    LPC_I2C1->I2CONSET = (1 << 5); // Set START
    if(I2C1_Wait_For_SI()) return -1;
    LPC_I2C1->I2CONCLR = (1 << 5); // Clear START
    return 0;
}

int I2C1_Stop(void) {
    LPC_I2C1->I2CONSET = (1 << 4); // Set STOP
    LPC_I2C1->I2CONCLR = (1 << 3); // Clear SI
    uint32_t timeout = 100000;
    while(LPC_I2C1->I2CONSET & (1 << 4)) {
        if(--timeout == 0) return -1;
    }
    return 0;
}

int I2C1_Write(uint8_t data) {
    LPC_I2C1->I2DAT = data;
    LPC_I2C1->I2CONCLR = (1 << 3);
    if(I2C1_Wait_For_SI()) return -1;
    if((LPC_I2C1->I2STAT & 0xF8) != 0x28)  // Check for ACK after data byte sent
        return -1;
    return 0;
}

int I2C1_Read(uint8_t ack, uint8_t *data) {
    if(ack)
        LPC_I2C1->I2CONSET = (1 << 2);
    else
        LPC_I2C1->I2CONCLR = (1 << 2);

    LPC_I2C1->I2CONCLR = (1 << 3);
    if(I2C1_Wait_For_SI()) return -1;

    *data = LPC_I2C1->I2DAT;
    return 0;
}

int I2C1_WriteRegister(uint8_t reg, uint8_t data) {
    if(I2C1_Start()) return -1;
    if(I2C1_Write(MPU6050_ADDR)) return -1;
    if(I2C1_Write(reg)) return -1;
    if(I2C1_Write(data)) return -1;
    if(I2C1_Stop()) return -1;
    return 0;
}

int I2C1_ReadBytes(uint8_t reg, uint8_t* buffer, uint8_t length) {
    if(I2C1_Start()) return -1;
    if(I2C1_Write(MPU6050_ADDR)) return -1;
    if(I2C1_Write(reg)) return -1;
    
    if(I2C1_Start()) return -1;
    if(I2C1_Write(MPU6050_ADDR | 1)) return -1;

    for(i=0; i<length; i++) {
        uint8_t ack = (i < (length - 1)) ? 1 : 0;
        if(I2C1_Read(ack, &buffer[i])) return -1;
    }
    if(I2C1_Stop()) return -1;
    return 0;
}

void DisplayStringAtLine(const char* str, uint8_t line) {
    flag1 = 0;
    if(line == 1)
        temp1 = 0x80; // LCD line 1 start address
    else
        temp1 = 0xC0; // LCD line 2 start address
    lcd_write();

    flag1 = 1;
    i = 0;
    while(str[i] != '\0') {
        temp1 = str[i++];
        lcd_write();
    }
}

int main(void) {
    SystemInit();
    SystemCoreClockUpdate();

    GPIO_Init();
    LPC_GPIO0->FIODIR |= DT_CTRL | RS_CTRL | EN_CTRL; // LCD control/data pins as output
    I2C1_Init();

    // Initialize LCD display with commands (same as your original init_command usage)
    flag1 = 0;
    for(i=0; i<9; i++) {
        temp1 = init_command[i];
        lcd_write();
    }

    delay_ms(100);

    // Wake up MPU-6050
    if(I2C1_WriteRegister(PWR_MGMT_1_REG, 0x00) != 0) {
        DisplayStringAtLine("MPU Init Fail", 1);
        while(1);
    }

    delay_ms(100);

    uint8_t who_am_i = 0;
    if(I2C1_ReadBytes(WHO_AM_I_REG, &who_am_i, 1) != 0 || who_am_i != 0x68) {
        DisplayStringAtLine("MPU Not Found", 1);
        while(1);
    }

    while(1) {
        uint8_t accel_data[6] = {0};

        if(I2C1_ReadBytes(ACCEL_XOUT_H, accel_data, 6) != 0) {
            DisplayStringAtLine("Read Err", 1);
            continue;
        }

        // Combine high and low bytes
        accel_x = (int16_t)((accel_data[0] << 8) | accel_data[1]);

        // Validate accel_x range
        if(accel_x == -1 || accel_x == 0xFFFF || accel_x == 0 || accel_x > 20000 || accel_x < -20000) {
            DisplayStringAtLine("Acc Err", 1);
            LPC_GPIO0->FIOCLR = LED_PIN;
            delay_ms(200);
            continue;
        }

        // Show accel_x value
        sprintf(buffer, "Accel X:%d  ", accel_x);
        DisplayStringAtLine(buffer, 1);

        // Step Detection threshold crossing logic
        if((prev_accel_x < -threshold && accel_x > threshold) ||
           (prev_accel_x > threshold && accel_x < -threshold)) {
            step_count++;
        }
        prev_accel_x = accel_x;

        // Display step count
        sprintf(buffer, "Steps: %d   ", step_count);
        DisplayStringAtLine(buffer, 2);

        if(accel_x != 0xFFFF && accel_x != 0x0000)
            LPC_GPIO0->FIOSET = LED_PIN;
        else
            LPC_GPIO0->FIOCLR = LED_PIN;

        delay_ms(200);
    }
}
