#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define F_CPU 2000000UL   

#define _SFR_(mem_addr)     (*(volatile uint8_t *)(0x5000 + (mem_addr))) // Macro function for calculating register locations. 0x5000 on the STM8 is the start of the registers

// UART 
#define UART_SR     _SFR_(0x240)
#define UART_TXE    7
#define UART_TC     6
#define UART_RXNE   5

#define UART_DR     _SFR_(0x241)
#define UART_BRR1   _SFR_(0x242)
#define UART_BRR2   _SFR_(0x243)
#define UART_CR1    _SFR_(0x244)
#define UART_CR2    _SFR_(0x245)
#define UART_TEN    3
#define UART_REN    2

#define USE_UART 1

#if USE_UART
   #define PRINTF(...)     printf(__VA_ARGS__)
#else
   #define PRINTF(...)    __asm__("nop")
#endif

// PortB
#define PB_ODR      _SFR_(0x05)  
#define PB_IDR      _SFR_(0x06)
#define PB_DDR      _SFR_(0x07)    
#define PB_CR1      _SFR_(0x08)     
#define PB_CR2      _SFR_(0x09)
#define ONE_WIRE_BUS    5

#define NUM_SENSORS 3


void uart_init() {
    UART_BRR2 = 0x00;
    UART_BRR1 = 0x0D;
    UART_CR2 = (1 << UART_TEN) | (1 << UART_REN);
}

void uart_write(uint8_t data) {
    UART_DR = data;
    while (!(UART_SR & (1 << UART_TC)));
}

uint8_t uart_read() {
    while (!(UART_SR & (1 << UART_RXNE)));
    return UART_DR;
}

int putchar(int c) {
    uart_write(c);
    return 0;
}

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

static void write_zero() {
    PB_ODR &= ~(1 << ONE_WIRE_BUS);
    delay_us(60);
    PB_ODR |= (1 << ONE_WIRE_BUS);
    delay_us(6);
}

static void write_one() {
    PB_ODR &= ~(1 << ONE_WIRE_BUS);
    delay_us(12);
    PB_ODR |= (1 << ONE_WIRE_BUS);
    delay_us(56);   // must be at least 1 us between messages and delay_us is 6 us minimum
}

static uint8_t read_bit() {
    // pull low for one clock to enable read slot
    PB_ODR &= ~(1 << ONE_WIRE_BUS);
    delay_us(6);     // Sometimes the delay is only 0.5 us when it is required to be at least 1 us, works without this just gives a warning. 2 nops may be enough
    PB_ODR |= (1 << ONE_WIRE_BUS);
    PB_DDR &= ~(1 << ONE_WIRE_BUS);     // enable as input
    delay_us(12);   // sample within the first 15 us
    uint8_t res = ((PB_IDR & (1 << ONE_WIRE_BUS)) >> ONE_WIRE_BUS);
    PB_DDR |= (1 << ONE_WIRE_BUS);
    delay_us(56);
    return res;    // 0x01 or 0x00
}

static inline void reset() {
    while (1) {
        // Send initilization pulse (pull down for min of 480us)
        PB_ODR &= ~(1 << ONE_WIRE_BUS);    // pull down
        delay_us(550);

        PB_ODR |= (1 << ONE_WIRE_BUS);    // pull up
        PB_DDR &= ~(1 << ONE_WIRE_BUS);   // enable as input

        delay_us(60);   // wait for one window for DS18B20 to respond

        if (!(PB_IDR & (1 << ONE_WIRE_BUS))) {
            delay_us(430); // wait for the end of slave pulse
            break;
        } 
        else {
            PRINTF("ERROR: Temperature sensor not detected error\n");
            delay_ms(500);
        }         
    }
}

static inline void write_byte(uint8_t data) {
    PB_DDR |= (1 << ONE_WIRE_BUS);
    for (int i = 0; i < 8; i++) {
        if ((data & (1 << i)) >> i) {
            write_one();
        } else {
            write_zero();
        }
    }
}

// Function will read as many bytes as will fit into the buffer, it is the caller's responsibility to allocate and free the buffer
static void read_bytes(void *buf, uint8_t num_bytes) {
    for (uint8_t i = num_bytes; i >= 1; i--) {
        for (uint8_t j = 0; j < 8; j++) {
           *((uint8_t*)buf + i - 1) |= (read_bit() << j);    // bits come LSB first 
        }
    }
}

static void print_buf(uint8_t* buf, uint8_t num_bytes) {
    PRINTF("%d Bytes at %p:\n", num_bytes, buf);
    for (uint8_t i = 0; i < num_bytes; i++) {
        PRINTF("0x%02x\n", buf[i]);
    }
}

static void print_temp(uint8_t* temp_bytes) {       // only the first 2 bytes will be read in the order MSB, LSB
    // For now assume the default 12 bit precision
    int8_t whole = (((temp_bytes[1] & 0b11110000) >> 4) | ((temp_bytes[0] & 0b00001111) << 4));
    if (temp_bytes[0] >> 7 == 1) whole += 1;    // negative number
    
    // manual 4 bit precision floating point
    uint8_t floating = ((temp_bytes[1] & 0b00001111));
    uint16_t decimal = 0;

    if ((floating & 0b00001000) >> 3) decimal += 5000;
    if ((floating & 0b00000100) >> 2) decimal += 2500;
    if ((floating & 0b00000010) >> 1) decimal += 1250;
    if (floating & 0b00000001) decimal += 625;

    PRINTF("%d.%04d degrees Celsius\n", whole, decimal);
}

static uint8_t check_crc(uint8_t* data, uint8_t data_size) {    // CRC of zero is a successful message
    // 8 bit shift register with XOR's at bit 0, 4, and 6
    uint8_t crc = 0x00;
    // start from the least significant byte of the data
    for (uint8_t i = data_size; i >= 1; i--) {
        for (uint8_t j = 0; j < 8; j++) {
            uint8_t next_bit = (data[i - 1] & (1 << j)) >> j;

            // LSB XOR with input
            next_bit ^= (crc & 0b00000001);

            // Input XOR with bit 3 and bit 4
            crc ^= next_bit << 3;           // 0b0000x000
            crc ^= next_bit << 4;           // 0b000x0000

            // Shift shift register
            crc >>= 1;
            crc |= next_bit << 7;
        }
    }
    return crc;
}

void searchROMs(uint8_t *buf) {
    write_byte(0xF0);   // search ROM

    // The general idea is to by the process of elimination is to get all 64 bits from the ROM at once
    uint8_t conflict_flag = 0;
    for (uint32_t i = NUM_SENSORS * 64; i >= 1 ; i--) {
        // once the full 64 bits have been received reset and run again
        if (i < (NUM_SENSORS * 64) && i % 64 == 0) {
            reset();
            write_byte(0xF0);       // search ROM
        }
        uint8_t position = 7 - ((i - 1) % 8);

        // Read the next 2 bits
        uint8_t b1 = read_bit();        // each sensor sends the LSB of its ROM code at once, resulting in a logical AND of all the bits            
        uint8_t b2 = read_bit();        // each sensor sends the ~complement of its LSB of its rom code at once

        // 00 -> indicates device conflict (there is a zero and a one in the position)
        // 10 -> all devices have one in the current position
        // 01 -> all devices have zero in the current position
        // 11 -> impossible

        // master writes either a one or a zero to select devices (0 selects devices with 0 at position and 1 selects devies with 1 at position). Once deselected reset required to make device respond.
        if (b1 && !b2) {        // 10
            write_one();
            buf[(i - 1) / 8] |= (1 << position);
        }
        else if (!b1 && b2) {   // 01
            write_zero();
        }
        else {                  // 00
            // check the last successfully written ROM and write the opposite bit (this will start as zero)
            // For each conflict check the last conflicts and choose the XOR of all the bit choices
            for (uint8_t j = 0; j < NUM_SENSORS; j++) {
                conflict_flag ^= ((buf[((i - 1) / 8) + ((8 * j) - 1)] & (1 << (position))) >> position);
            }
            if (conflict_flag) {
                write_one();
                buf[(i - 1) / 8] |= (1 << position);
            } 
            else {
                write_zero();
            }
        }
    }
}

void main() {
    #if USE_UART
         uart_init();
    #endif

    // Setup PIN A4 for input/output for 1-wire bus
    PB_DDR |= (1 << ONE_WIRE_BUS);    // enable as output
    PB_CR1 |= (1 << ONE_WIRE_BUS);    // enable as push pull when output and pull up when input
    PB_ODR |= (1 << ONE_WIRE_BUS);    // pull up

    // Fetch the ROM's of all the devices on the bus
    uint8_t* rom_bytes = calloc(NUM_SENSORS * 8, sizeof(uint8_t));
    reset();
    searchROMs(rom_bytes);

    // expected ROM serial numbers (from closest to STM8 on circuit to furthest)
    // 0xc1a967770e64ff28
    // 0xe29815740e64ff28
    // 0x3f760e770e64ff28
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        PRINTF("\nROM%d SERIAL NUMBER: 0x", i); 
        for (uint8_t j = 0; j < 8; j++) {
            PRINTF("%02x", rom_bytes[j + i * 8]);
        }
        PRINTF(" -> CRC check ROM%d:%x", i, check_crc(rom_bytes, 8));
    }
    PRINTF("\n");
    
    // Convert and Fetch temperature from each sensor
    reset();
    
    write_byte(0xCC);   // skip rom command: 0xCC
    write_byte(0x44);   // Convert temperature request: 0x44
    while(!read_bit()); // wait for sensor(s) to finish the temp conversion

    // address each sensor and fetch temperature
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        reset();
        delay_us(450); // wait till end of slave pulse
        write_byte(0x55);
        write_byte(rom_bytes[(i * 8) + 7]);
        write_byte(rom_bytes[(i * 8) + 6]);
        write_byte(rom_bytes[(i * 8) + 5]);
        write_byte(rom_bytes[(i * 8) + 4]);
        write_byte(rom_bytes[(i * 8) + 3]);
        write_byte(rom_bytes[(i * 8) + 2]);
        write_byte(rom_bytes[(i * 8) + 1]);
        write_byte(rom_bytes[(i * 8) + 0]);

        write_byte(0xBE);   // read scratch pad: 0xBE

        uint8_t* scratchpad = calloc(9, sizeof(uint8_t));
        read_bytes(scratchpad, 9);

        PRINTF("CRC for SCRATCHPAD:%x\n", check_crc(scratchpad, 9));
        
        print_temp(&(scratchpad[7]));
        free(scratchpad);
    }
    free(rom_bytes);
}