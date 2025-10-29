#include "LPC17xx.h"
#include <stdio.h>
#include <stdint.h>

// ------ Device/Protocol Settings ------
#define BNO085_ADDR        (0x4A << 1)  // Default addr (verify pin config)
#define LED_PIN            (1 << 4)     // P0.4 for LED
#define MAX_STEPS          9999         // Max step count for display
#define REPORT_ID_ACCEL    0x05         // Accel report as per datasheet

unsigned long init_command[] = {0x30, 0x30, 0x30, 0x20, 0x28, 0x0C, 0x06, 0x01, 0x80};
unsigned int flag1, temp1, temp2, flag2;
volatile int16_t accel_x, prev_accel_x;
unsigned int i;
char buffer[16];
uint8_t rx_buffer[32];
unsigned int step_count = 0;
unsigned int threshold = 400;

// LCD Control Macros
#define RS_CTRL  0x08000000 // P0.27
#define EN_CTRL  0x10000000 // P0.28
#define DT_CTRL  0x07800000 // P0.23–P0.26

// Timebase variables for debounce
volatile uint32_t g_millis = 0;
uint32_t last_step_time = 0;
const uint32_t min_step_ms = 250;

// --------- Helper Routine Prototypes --------
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
void SysTick_Handler(void);
void setup_systick(uint32_t);
void BNO085_Init(void);
void print_uart(const char* msg); // Stub for debugging
int BNO085_SetAccelerometerFeature(void);
int BNO085_WaitForReport(uint8_t* rx, uint8_t* report_id, uint8_t* length);
int BNO085_ReadAccel(int16_t* accel_x);

// ---------- LCD Write/Handling ----------
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
    delay_lcd(1500); // Calibrate for display model!
    LPC_GPIO0->FIOCLR = EN_CTRL;
    delay_lcd(1500);
}
void delay_lcd(unsigned int r1) {
    volatile unsigned int r;
    for (r = 0; r < r1; r++);
}
void delay_ms(uint32_t ms) {
    volatile uint32_t i;
    for (i = 0; i < ms * 8500; i++) { __NOP(); }
}
// --------- SysTick Timebase Setup (1ms tick) --------
void SysTick_Handler(void) { g_millis++; }
void setup_systick(uint32_t system_clk_hz) {
    SysTick_Config(system_clk_hz / 1000);
    g_millis = 0;
}
uint32_t get_millis(void) { return g_millis; }
// --------- I2C Helpers ----------
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
// --------- UART Print (debug only) ---------
void print_uart(const char* msg) {
    // Implement as needed for your UART debug/trace.
}

// --------- GPIO/LED Initialization --------
void GPIO_Init(void) {
    LPC_PINCON->PINSEL0 &= ~(3 << 8);  // P0.4 = GPIO
    LPC_GPIO0->FIODIR |= LED_PIN;
}
// --------- BNO085 SHTP/Feature Setup ----------
void BNO085_Init(void) {
    delay_ms(700);
    BNO085_SetAccelerometerFeature();
    delay_ms(50);
}
int BNO085_SetAccelerometerFeature(void) {
    // Sample SHTP feature command (for report 0x01 = accelerometer)
    I2C1_Start();
    I2C1_Write(BNO085_ADDR);
    I2C1_Write(0x0B); // Packet length LSB
    I2C1_Write(0x00); // Packet length MSB
    I2C1_Write(0x02); // Channel 2
    I2C1_Write(0x00); // Sequence #
    I2C1_Write(0xFD); // Set Feature Command
    I2C1_Write(0x01); // Accelerometer
    I2C1_Write(0x00); // Reserved
    I2C1_Write(0x01); // Feature enable
    I2C1_Write(0x00); // Interval
    I2C1_Write(0x00); // Additional
    I2C1_Write(0x00); // Additional
    I2C1_Stop();
    return 0;
}
// SHTP report parser with length & ID return
int BNO085_WaitForReport(uint8_t* rx, uint8_t* report_id, uint8_t* length) {
    I2C1_Start();
    I2C1_Write(BNO085_ADDR | 1); // READ mode
    for (i = 0; i < 14; i++)
        rx[i] = I2C1_Read(i < 13 ? 1 : 0);
    I2C1_Stop();
    // Print report for debug
    char debug[64];
    sprintf(debug, "SHTP Data: ");
    print_uart(debug);
    for (i = 0; i < 14; ++i) {
        sprintf(debug, "0x%02X ", rx[i]);
        print_uart(debug);
    }
    print_uart("\n");
    *length = rx[0]; // SHTP packet length LSB
    *report_id = rx[2]; // SHTP report id
    return 0;
}
// Parse accel_x based on report
int BNO085_ReadAccel(int16_t* accel_x) {
    uint8_t report_id, pkt_len;
    BNO085_WaitForReport(rx_buffer, &report_id, &pkt_len);
    if (report_id != REPORT_ID_ACCEL || pkt_len < 12) return -1;
    // Use BNO085 datasheet offsets! Example: accel_x at rx_buffer[8],[9]
    *accel_x = (int16_t)((rx_buffer[9] << 8) | rx_buffer[8]);
    if (*accel_x == -1 || *accel_x == 0xFFFF || *accel_x == 0x0000)
        return -1;
    return 0;
}
// ---------- Main Application ----------
int main(void) {
    SystemInit();
    SystemCoreClockUpdate();
    setup_systick(SystemCoreClock); // For debounce
    LPC_GPIO0->FIODIR = DT_CTRL | RS_CTRL | EN_CTRL;
    flag1 = 0;
    for (i = 0; i < 9; i++) { temp1 = init_command[i]; lcd_write(); }
    GPIO_Init();
    I2C1_Init();
    BNO085_Init();
    step_count = 0;
    prev_accel_x = 0;
    static int16_t filt_accel_x = 0;
    uint32_t error_counter = 0;
    while (1) {
        int err = BNO085_ReadAccel(&accel_x);
        // ---- Error Reporting & Freeze Step Counter if Comm Fails Repeatedly ----
        if (err != 0) {
            flag1 = 0; temp1 = 0x80; lcd_write();
            flag1 = 1; sprintf(buffer, "ERR ACCEL"); i = 0;
            while (buffer[i] != '\0') { temp1 = buffer[i++]; lcd_write(); }
            for (i = 0; i < 16; i++) buffer[i] = '\0';
            LPC_GPIO0->FIOCLR = LED_PIN;
            error_counter++;
            if (error_counter > 10) { step_count = 0; prev_accel_x = 0; }
            delay_ms(200);
            continue;
        }
        error_counter = 0; // Comm good, reset error counter
        // ---- Accel value display ----
        sprintf(buffer, "%d", accel_x);
        flag1 = 0; temp1 = 0x80; lcd_write();
        flag1 = 1; i = 0;
        while (buffer[i] != '\0') { temp1 = buffer[i++]; lcd_write(); }
        for (i = 0; i < 16; i++) buffer[i] = '\0';
        // ---- Step filtering + debounce ----
        filt_accel_x = (filt_accel_x * 3 + accel_x) / 4;
        uint32_t curr_time = get_millis();
        if (
            ((prev_accel_x < -threshold && filt_accel_x > threshold) ||
            (prev_accel_x > threshold && filt_accel_x < -threshold))
            && ((curr_time - last_step_time) > min_step_ms)
        ) {
            step_count = (step_count + 1) % MAX_STEPS;
            last_step_time = curr_time;
        }
        prev_accel_x = filt_accel_x;
        // ---- Step count display ----
        flag1 = 0; temp1 = 0xC0; lcd_write();
        flag1 = 1; sprintf(buffer, "Steps:%d", step_count); i = 0;
        while (buffer[i] != '\0') { temp1 = buffer[i++]; lcd_write(); }
        for (i = 0; i < 16; i++) buffer[i] = '\0';
        // ---- LED -- Valid sensor movement ----
        if (err == 0) LPC_GPIO0->FIOSET = LED_PIN; else LPC_GPIO0->FIOCLR = LED_PIN;
        delay_ms(200);
    }
}
