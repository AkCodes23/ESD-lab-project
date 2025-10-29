#include "LPC17xx.h"
#include <stdio.h>

#define BNO055_ADDR       (0x28 << 1)   // BNO055 default 7-bit address shifted left
#define LED_PIN           (1 << 4)      // P0.4 pin for LED indication

unsigned long init_command[] = {0x30, 0x30, 0x30, 0x20, 0x28, 0x0c, 0x06, 0x01, 0x80};
unsigned int flag1, temp1, temp2, flag2;
unsigned int i;
char buffer[16];
int16_t accel_x;
uint8_t lsb, msb;
int16_t prev_accel_x = 0;
unsigned int step_count = 0;
unsigned int threshold = 400;

#define RS_CTRL 0x08000000  // P0.27
#define EN_CTRL 0x10000000  // P0.28
#define DT_CTRL 0x07800000  // P0.23–P0.26

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
    if (flag1 == 0) 
        LPC_GPIO0->FIOCLR = RS_CTRL;
    else 
        LPC_GPIO0->FIOSET = RS_CTRL;
    LPC_GPIO0->FIOSET = EN_CTRL;
    delay_lcd(1500);  // Adjust to your LCD timing
    LPC_GPIO0->FIOCLR = EN_CTRL;
    delay_lcd(1500);
}

void delay_lcd(unsigned int r1) {
    volatile unsigned int r;
    for (r = 0; r < r1; r++);
}

void delay_ms(uint32_t ms) {
    volatile uint32_t i;
    for (i = 0; i < ms * 8500; i++) __NOP();
}

void I2C1_Init(void) {
    LPC_SC->PCONP |= (1 << 19);  // Power on I2C1
    LPC_SC->PCLKSEL1 &= ~(3 << 6); // Peripheral clock for I2C1
    LPC_PINCON->PINSEL1 |= (3 << 6) | (3 << 8);  // P0.19 SDA1, P0.20 SCL1
    LPC_PINCON->PINMODE1 &= ~((3 << 6) | (3 << 8)); // Pull-ups enabled
    LPC_I2C1->I2SCLH = 60;
    LPC_I2C1->I2SCLL = 60;
    LPC_I2C1->I2CONCLR = 0x6C;
    LPC_I2C1->I2CONSET = (1 << 6);
}

void I2C1_Start(void) {
    LPC_I2C1->I2CONSET = (1 << 5);  // STA set
    while (!(LPC_I2C1->I2CONSET & (1 << 3)));  // Wait SI
    LPC_I2C1->I2CONCLR = (1 << 5);  // Clear STA
}

void I2C1_Stop(void) {
    LPC_I2C1->I2CONSET = (1 << 4);  // STO set
    LPC_I2C1->I2CONCLR = (1 << 3);  // Clear SI
    while (LPC_I2C1->I2CONSET & (1 << 4));  // Wait stop
}

void I2C1_Write(uint8_t data) {
    LPC_I2C1->I2DAT = data;
    LPC_I2C1->I2CONCLR = (1 << 3);  // Clear SI
    while (!(LPC_I2C1->I2CONSET & (1 << 3)));  // Wait SI
}

uint8_t I2C1_Read(uint8_t ack) {
    if (ack)
        LPC_I2C1->I2CONSET = (1 << 2);
    else
        LPC_I2C1->I2CONCLR = (1 << 2);
    LPC_I2C1->I2CONCLR = (1 << 3);  // Clear SI
    while (!(LPC_I2C1->I2CONSET & (1 << 3)));
    return LPC_I2C1->I2DAT;
}

void GPIO_Init(void) {
    LPC_PINCON->PINSEL0 &= ~(3 << 8);  // P0.4 GPIO
    LPC_GPIO0->FIODIR |= LED_PIN;
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
    delay_ms(700);

    // Write 0x0C to BNO055 mode register 0x3D for NDOF mode
    I2C1_Start();
    I2C1_Write(BNO055_ADDR);  // Write mode
    I2C1_Write(0x3D);         // Mode register
    I2C1_Write(0x0C);         // NDOF mode
    I2C1_Stop();

    delay_ms(100);  // Delay for mode stabilization

    while (1) {
        // Set register pointer to 0x08 (ACCEL_DATA_X_LSB)
        I2C1_Start();
        I2C1_Write(BNO055_ADDR);
        I2C1_Write(0x08);
        I2C1_Stop();

        // Read 2 bytes for accel_x
        I2C1_Start();
        I2C1_Write(BNO055_ADDR | 1);
        lsb = I2C1_Read(1);
        msb = I2C1_Read(0);
        I2C1_Stop();

        accel_x = (int16_t)((msb << 8) | lsb);

        // Display accel_x on LCD first line
        sprintf(buffer, "%d", accel_x);
        flag1 = 0;
        temp1 = 0x80;  // LCD line 1
        lcd_write();

        flag1 = 1;
        i = 0;
        while (buffer[i] != '\0') {
            temp1 = buffer[i++];
            lcd_write();
        }
        for (i = 0; i < 16; i++) buffer[i] = 0;

        // Step counting logic
        if ((prev_accel_x < -threshold && accel_x > threshold) ||
            (prev_accel_x > threshold && accel_x < -threshold)) {
            step_count++;
        }
        prev_accel_x = accel_x;

        // Display step count second line of LCD
        flag1 = 0;
        temp1 = 0xC0;  // LCD line 2
        lcd_write();

        flag1 = 1;
        sprintf(buffer, "Steps: %d", step_count);
        i = 0;
        while (buffer[i] != '\0') {
            temp1 = buffer[i++];
            lcd_write();
        }
        for (i = 0; i < 16; i++) buffer[i] = 0;

        // LED indication for valid data
        if (accel_x != 0xFFFF && accel_x != 0x0000)
            LPC_GPIO0->FIOSET = LED_PIN;
        else
            LPC_GPIO0->FIOCLR = LED_PIN;

        delay_ms(200);
    }
}
