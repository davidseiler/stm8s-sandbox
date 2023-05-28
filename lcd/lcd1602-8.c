#include <stdint.h>
#include "lcd1602-8.h"

// Pin mappings
/* LCD1602 ---- STM8S   
RS -> PC4
RW -> PC6
E  -> PC7

D0 -> PB0
D1 -> PB1
D2 -> PB2
D3 -> PB3
D4 -> PB4
D5 -> PB5
D6 -> PF4 (A6)
D7 -> PC5
*/

#define F_CPU 2000000UL   
#define _SFR_(mem_addr)     (*(volatile uint8_t *)(0x5000 + (mem_addr))) // Macro function for calculating register locations. 0x5000 on the STM8 is the start of the registers

// Port B
#define PB_ODR      _SFR_(0x05)  
#define PB_IDR      _SFR_(0x06)
#define PB_DDR      _SFR_(0x07)    
#define PB_CR1      _SFR_(0x08)     
#define PB_CR2      _SFR_(0x09)

// Port C
#define PC_ODR      _SFR_(0x0A)  
#define PC_IDR      _SFR_(0x0B)
#define PC_DDR      _SFR_(0x0C)    
#define PC_CR1      _SFR_(0x0D)     
#define PC_CR2      _SFR_(0x0E)
#define RS_PIN 4
#define RW_PIN 6
#define E_PIN 7

// Port F
#define PF_ODR      _SFR_(0x19)  
#define PF_IDR      _SFR_(0x1A)
#define PF_DDR      _SFR_(0x1B)    
#define PF_CR1      _SFR_(0x1C)     
#define PF_CR2      _SFR_(0x1D)


static inline void delay_ms(uint16_t ms) {
    uint32_t i;
    for (i = 0; i < ((F_CPU / 12000UL) * ms); i++)  // 110 nop instructions per ms
        __asm__("nop");    
}

static inline void delay_us(uint16_t us) {      // minimum is 6us, delay will be to the nearest factor of 6 us
    for (uint32_t i = 0; i < ((us / 6) - 1); i++) {
        __asm__("nop");
    }
}

static inline void set_data_byte(uint8_t data) {
    PB_ODR &= 0b11000000;
    PF_ODR &= 0b11101111;
    PC_ODR &= 0b11011111;
    PB_ODR |= (data & 0b00111111);
    PF_ODR |= (((data & 0b01000000) >> 6) << 4);
    PC_ODR |= ((data >> 7) << 5);
}

void LCD_init() {
// Setup Ports
    PB_DDR |= 0b00111111;     // set as output
    PB_CR1 |= 0b00111111;     // enable as push pull

    PC_DDR |= 0b11110000;
    PC_CR1 |= 0b11110000;

    PF_DDR |= (1 << 4);
    PF_CR1 |= (1 << 4);

    delay_ms(10);   // LCD takes 10 ms to initialize itself after power on

    // RS RW 7 6 5 4 3 2 1 0
    // clear display 0000000001
    PC_ODR &= ~(1 << RS_PIN);
    PC_ODR &= ~(1 << RW_PIN);
    set_data_byte(0b00000001);
    PC_ODR |= (1 << E_PIN);
    PC_ODR &= ~(1 << E_PIN);
    delay_ms(2);    // max lcd instruction time

    // Function set 0000111000
    PC_ODR &= ~(1 << RS_PIN);
    PC_ODR &= ~(1 << RW_PIN);
    set_data_byte(0b00111000);
    PC_ODR |= (1 << E_PIN);
    PC_ODR &= ~(1 << E_PIN);
    delay_ms(2);

    // Display on/off control 0000001111
    PC_ODR &= ~(1 << RS_PIN);
    PC_ODR &= ~(1 << RW_PIN);
    set_data_byte(0b00001111);
    PC_ODR |= (1 << E_PIN);
    PC_ODR &= ~(1 << E_PIN);
    delay_ms(2);

    // Entry mode set 0000000110
    PC_ODR &= ~(1 << RS_PIN);
    PC_ODR &= ~(1 << RW_PIN);
    set_data_byte(0b00000110);
    PC_ODR |= (1 << E_PIN);
    PC_ODR &= ~(1 << E_PIN);
    delay_ms(2);
}

void LCD_clear() {
    // clear display 0000000001
    PC_ODR &= ~(1 << RS_PIN);
    PC_ODR &= ~(1 << RW_PIN);
    set_data_byte(0b00000001);
    PC_ODR |= (1 << E_PIN);
    PC_ODR &= ~(1 << E_PIN);
    delay_ms(2);    // max lcd instruction time
}


void LCD_write_character(char c, uint16_t display_speed_ms) {
    // Write data to CGRAM/DDRAM
    PC_ODR |= (1 << RS_PIN);
    PC_ODR &= ~(1 << RW_PIN);
    set_data_byte(c);
    PC_ODR |= (1 << E_PIN);
    PC_ODR &= ~(1 << E_PIN);
    delay_ms(2);
    delay_ms(display_speed_ms);
}

void LCD_write(char* data, uint8_t size, uint16_t display_speed_ms) {
    for (uint8_t i = 0; i < size; i++) {
        // Write data to CGRAM/DDRAM
        LCD_write_character(data[i], display_speed_ms);
    }
}

void LCD_write_full(char data[32], uint16_t display_speed_ms) {
    LCD_init();
    delay_ms(10);
    for (uint8_t i = 0; i < 32; i++) {
        if (i == 16) { 
            // LCD_write(0x00, 24, display_speed_ms);   // pad with garbage from the top of RAM
            // Set address to next line 0x40: 001
            PC_ODR &= ~(1 << RS_PIN);
            PC_ODR &= ~(1 << RW_PIN);
            set_data_byte(0b11000000);
            PC_ODR |= (1 << E_PIN);
            PC_ODR &= ~(1 << E_PIN);
            delay_ms(2);
        }
        LCD_write_character(data[i], display_speed_ms);
    }
}

void main() {
    LCD_init();
    char data[32] = "11111111111111112222222222222222";
    LCD_write_full(data, 0);
    LCD_clear();
    delay_ms(500);
    LCD_write_full(data, 0);
}