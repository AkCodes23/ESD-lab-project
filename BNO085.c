#include "LPC17xx.h"
#include <stdio.h>

#define BNO085_ADDR       (0x4A << 1) // Check datasheet; usually use 0x4A (7-bit) then shift for R/W
#define LED_PIN           (1 << 4)    // P0.4 for LED

unsigned long init_command[] = {0x30, 0x30, 0x30, 0x20, 0x28, 0x0C, 0x06, 0x01, 0x80};
unsigned int flag1, temp1, temp2, flag2;
unsigned int i;
char buffer[16];
int16_t accel_x;
uint8_t rx_buffer[32];

int16_t prev_accel_x = 0;
unsigned int step_count = 0;
unsigned int threshold = 400;

#define RS_CTRL 0x08000000 // P0.27
#define EN_CTRL 0x10000000 // P0.28
#define DT_CTRL 0x07800000 // P0.23–P0.26

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
int BNO085_ReadAccel(void); // return int (status)

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
    delay_lcd(1000); // Shorter delay
    LPC_GPIO0->FIOCLR = EN_CTRL;
    delay_lcd(1000); // Shorter delay
}

void delay_lcd(unsigned int r1) {
    unsigned int r;
    for (r = 0; r < r1; r++);
}

void delay_ms(uint32_t ms) {
    uint32_t i;
    for (i = 0; i < ms * 9000; i++) { // Tweak loop count for real timing!
        __NOP();
    }
}

void I2C1_Init(void) {
    LPC_SC->PCONP |= (1 << 19);
    LPC_SC->PCLKSEL1 &= ~(3 << 6);

    LPC_PINCON->PINSEL1 |= (3 << 6) | (3 << 8);  // SDA1, SCL1
    LPC_PINCON->PINMODE1 &= ~((3 << 6) | (3 << 8));

    LPC_I2C1->I2SCLH = 60;
    LPC_I2C1->I2SCLL = 60;

    LPC_I2C1->I2CONCLR = 0x6C;
    LPC_I2C1->I2CONSET = (1 << 6);
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

void GPIO_Init(void) {
    LPC_PINCON->PINSEL0 &= ~(3 << 8);  // P0.4 = GPIO
    LPC_GPIO0->FIODIR |= LED_PIN;
}

// Real SHTP usage for BNO085 requires transaction sequence—see SHTP protocol/datasheet!

void BNO085_Init(void) {
    delay_ms(700);
    // Typically send SHTP "reset" over I2C here
}

int BNO085_ReadAccel(void) {
    // Replace below with actual SHTP report reading! This is placeholder.
    I2C1_Start();
    I2C1_Write(BNO085_ADDR | 1);
    for (i = 0; i < 14; i++) {
        rx_buffer[i] = I2C1_Read(i < 13 ? 1 : 0);
    }
    I2C1_Stop();
    accel_x = (int16_t)((rx_buffer[5] << 8) | rx_buffer[4]);
    return 0; // Return error/status if needed
}

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

    while (1) {
        if (BNO085_ReadAccel() != 0) continue;

        sprintf(buffer, "%d", accel_x);
        flag1 = 0;
        temp1 = 0x80;
        lcd_write();

        flag1 = 1;
        i = 0;
        while (buffer[i] != '\0') {
            temp1 = buffer[i++];
            lcd_write();
        }

        for (i = 0; i < 16; i++) buffer[i] = 0;

        // Improved step logic: Use threshold crossing, avoid noise
        if ((prev_accel_x < -threshold && accel_x > threshold) ||
            (prev_accel_x > threshold && accel_x < -threshold)) {
            step_count++;
        }

        prev_accel_x = accel_x;

        flag1 = 0;
        temp1 = 0xC0;
        lcd_write();

        flag1 = 1;
        sprintf(buffer, "Steps:%d", step_count);
        i = 0;
        while (buffer[i] != '\0') {
            temp1 = buffer[i++];
            lcd_write();
        }

        if (accel_x != 0xFFFF && accel_x != 0x0000)
            LPC_GPIO0->FIOSET = LED_PIN;
        else
            LPC_GPIO0->FIOCLR = LED_PIN;

        delay_ms(200);
    }
}
