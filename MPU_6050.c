#include "LPC17xx.h"
#include <stdio.h>

#define MPU6050_ADDR       (0x68 << 1)   // 7-bit I2C address for MPU6050 shifted left
#define LED_PIN            (1 << 4)      // P0.4 for LED
#define PWR_MGMT_1_REG     0x6B
#define WHO_AM_I_REG       0x75
#define ACCEL_XOUT_H       0x3B

#define RS_CTRL            0x08000000    // P0.27
#define EN_CTRL            0x10000000    // P0.28
#define DT_CTRL            0x07800000    // P0.23-P0.26

unsigned long init_command[] = {0x30,0x30,0x30,0x20,0x28,0x0c,0x06,0x01,0x80};

char buffer[16];
unsigned int flag1, temp1, temp2, flag2;
unsigned int i;

int16_t accel_x;
int16_t prev_accel_x = 0;
unsigned int step_count = 0;
const unsigned int threshold = 400;

// Basic delay for ~100MHz clock
void delay_ms(uint32_t ms) {
    volatile uint32_t i;
    for(i = 0; i < ms * 8500; i++) {
        __NOP();
    }
}

void lcd_write(void) {
    flag2 = (flag1 == 1) ? 0 : ((temp1==0x30)||(temp1==0x20)) ? 1 : 0;
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
    LPC_PINCON->PINSEL0 &= ~(3 << 8);
    LPC_GPIO0->FIODIR |= LED_PIN;
    LPC_GPIO0->FIODIR |= DT_CTRL | RS_CTRL | EN_CTRL;
}

void I2C1_Init(void) {
    LPC_SC->PCONP |= (1 << 19);
    LPC_SC->PCLKSEL1 &= ~(3<<6);
    LPC_PINCON->PINSEL1 |= (3 << 6) | (3 << 8); // P0.19 SDA1, P0.20 SCL1
    LPC_PINCON->PINMODE1 &= ~((3<<6) | (3<<8));
    LPC_I2C1->I2SCLH = 60;
    LPC_I2C1->I2SCLL = 60;
    LPC_I2C1->I2CONCLR = 0x6C;
    LPC_I2C1->I2CONSET = (1 << 6);
}

void I2C1_Start(void) {
    LPC_I2C1->I2CONSET = (1 << 5);
    while(!(LPC_I2C1->I2CONSET & (1 << 3)));
    LPC_I2C1->I2CONCLR = (1 << 5);
}

void I2C1_Stop(void) {
    LPC_I2C1->I2CONSET = (1 << 4);
    LPC_I2C1->I2CONCLR = (1 << 3);
    while(LPC_I2C1->I2CONSET & (1 << 4));
}

void I2C1_Write(uint8_t data) {
    LPC_I2C1->I2DAT = data;
    LPC_I2C1->I2CONCLR = (1 << 3);
    while(!(LPC_I2C1->I2CONSET & (1 << 3)));
}

uint8_t I2C1_Read(uint8_t ack) {
    if(ack)
        LPC_I2C1->I2CONSET = (1 << 2);
    else
        LPC_I2C1->I2CONCLR = (1 << 2);
    LPC_I2C1->I2CONCLR = (1 << 3);
    while(!(LPC_I2C1->I2CONSET & (1 << 3)));
    return LPC_I2C1->I2DAT;
}

// Read register value from MPU6050 (for WHO_AM_I check)
uint8_t read_MPU6050_register(uint8_t reg) {
    uint8_t val;
    I2C1_Start();
    I2C1_Write(MPU6050_ADDR);
    I2C1_Write(reg);
    I2C1_Start(); // repeated start
    I2C1_Write(MPU6050_ADDR | 1);
    val = I2C1_Read(0);
    I2C1_Stop();
    return val;
}

// Read accel_x using correct repeated start and pointer setting
int read_MPU6050_accel_x(int16_t* accel_x) {
    uint8_t msb, lsb;
    I2C1_Start();
    I2C1_Write(MPU6050_ADDR);          // Write address
    I2C1_Write(ACCEL_XOUT_H);          // Set reg pointer
    I2C1_Start();                      // Repeated start
    I2C1_Write(MPU6050_ADDR | 1);      // Read address
    msb = I2C1_Read(1);                // Read MSB, ACK
    lsb = I2C1_Read(0);                // Read LSB, NACK
    I2C1_Stop();
    *accel_x = (int16_t)((msb << 8) | lsb);
    // Range check
    if(*accel_x == -1 || *accel_x == 0 || *accel_x > 20000 || *accel_x < -20000)
        return -1;
    return 0;
}

void DisplayStringAtLine(const char* str, uint8_t line) {
    flag1 = 0;
    temp1 = (line == 1) ? 0x80 : 0xC0;
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
    I2C1_Init();
    flag1 = 0;
    for(i = 0; i < 9; i++) {
        temp1 = init_command[i];
        lcd_write();
    }
    delay_ms(100);

    // Wake up MPU-6050
    I2C1_Start();
    I2C1_Write(MPU6050_ADDR);
    I2C1_Write(PWR_MGMT_1_REG);
    I2C1_Write(0x00);
    I2C1_Stop();
    delay_ms(100);

    // Verify sensor is present
    uint8_t who = read_MPU6050_register(WHO_AM_I_REG);
    if(who != 0x68) {
        DisplayStringAtLine("MPU Err!", 1);
        while(1);
    }

    while(1) {
        int valid = read_MPU6050_accel_x(&accel_x);

        if(valid != 0) {
            DisplayStringAtLine("Acc Err", 1);
            LPC_GPIO0->FIOCLR = LED_PIN;
            delay_ms(200);
            continue;
        }

        sprintf(buffer, "%d         ", accel_x);
        DisplayStringAtLine(buffer, 1);

        if((prev_accel_x < -threshold && accel_x > threshold) ||
           (prev_accel_x > threshold && accel_x < -threshold)) {
            step_count++;
        }
        prev_accel_x = accel_x;

        sprintf(buffer, "Steps: %d   ", step_count);
        DisplayStringAtLine(buffer, 2);

        LPC_GPIO0->FIOSET = LED_PIN;
        delay_ms(200);
    }
}
