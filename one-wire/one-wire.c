#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define F_CPU 2000000UL     // this value is to calculate time based on clock cycles, it should be set to the clock speed of the MCU

#define _SFR_(mem_addr)     (*(volatile uint8_t *)(0x5000 + (mem_addr))) // Macro function for calculating register locations. 0x5000 on the STM8 is the start of the registers

/* UART */
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

static void send_zero() {
    PB_ODR &= ~(1 << ONE_WIRE_BUS);
    delay_us(60);
    PB_ODR |= (1 << ONE_WIRE_BUS);
    delay_us(6);
}

static void send_one() {
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

static inline uint8_t reset() {
    // Send initilization pulse (pull down for min of 480us)
    PB_ODR &= ~(1 << ONE_WIRE_BUS);    // pull down
    delay_us(550);

    PB_ODR |= (1 << ONE_WIRE_BUS);    // pull up
    PB_DDR &= ~(1 << ONE_WIRE_BUS);   // enable as input

    delay_us(60);   // wait for one window for DS18B20 to respond

    return !(PB_IDR & (1 << ONE_WIRE_BUS));
}

static inline void write_byte(uint8_t data) {
    PB_DDR |= (1 << ONE_WIRE_BUS);
    for (int i = 0; i < 8; i++) {
        if ((data & (1 << i)) >> i) {
            send_one();
        } else {
            send_zero();
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

void main() {
    #if USE_UART
         uart_init();     // UART forces the device to reset once every 8.5 ms and I am not sure why
    #endif
    // May 19th
    // 12:13 wired up ds18b20 to 5v and common ground, pin 2 DQ (data line) is connected to A4/PB5 (i2c sda) 
    // 12:45 setup port for 1-wire bus and configured for the recommended pullup
    // 2:38 PM updated delay methods to match the actual timings, reset pulse and response from bus
    // 4:27 PM reviewed bitwise operations and setup delay for determining DS18b20 responding
    // 7:45 PM request rom serial number and store in memory 
    // May 20th 
    // 7:44 PM Bit banged temperature request 
    // 8:33 PM functions for writing bytes
    // 9:59 PM manual temperature fetch and conversion from bits to readable stuff
    // May 22nd
    // 9:50 AM: read bytes implemented 
    // 9:15 PM: add uart toggle macros for easily turning uart off and on
    // 9:42 PM: fixed bug of inconsistent RAM storing (was using calloc incorrectly)
    // May 23rd
    // 10:23 AM got test whole places working properly
    // 10:52 AM fixed bytes going into the wrong place in memory, spaced by zeros
    // 11:44 AM implemented manual 4 bit precision floating point printing
    // 12:18 PM fixed byte ordering from read so that MSB is always correctly in front of LSB in memory
    // 7:52 PM CRC generation and checking for the ROM and SCRATCHPAD reads
    // 9:46 PM CRC verification on fresh breadboard configuration with line interrupts
    // May 24
    //

    // TODO: next up setup multiple sensors (3) on the same circuit
        // setup LCD control
    

    // Setup PIN A4 for input/output for 1-wire bus
    PB_DDR |= (1 << ONE_WIRE_BUS);    // enable as output
    PB_CR1 |= (1 << ONE_WIRE_BUS);    // enable as push pull when output and pull up when input
    PB_ODR |= (1 << ONE_WIRE_BUS);    // pull up

    uint8_t* rom_bytes = calloc(NUM_SENSORS * 8, sizeof(uint8_t));   // start of RAM
    
    // fetch the ROM's of all the devices on the bus
    if (reset()) {      // Sensors responds
        delay_us(430);;     // wait till end of slave pulse
        // fetch temperature sensor serial number: Read ROM command (Ox33)
        // *IMPORTANT tranport LSB first
        write_byte(0x33);

        // *IMPORTANT read LSB first
        // read 64 bit serial number from the rom
        
        read_bytes(rom_bytes, 8);

        // expected ROM serial numbers (from closest to STM8 on circuit to furthest)
        // 0xc1a967770e64ff28
        // 0xe29815740e64ff28
        // 0x3f760e770e64ff28
        PRINTF("\nROM SERIAL NUMBER: 0x"); 
        for (uint8_t i = 0; i < NUM_SENSORS * 8; i++) {
            PRINTF("%02x", rom_bytes[i]);
        }
        PRINTF("\n");

        PRINTF("CRC for ROM:%x\n", check_crc(rom_bytes, 8));
        
    } else {
        // No DS18B20 temperature sensor was detected on the line
        PRINTF("Temperature sensor not detected error\n");
    }


    // send convert temperature command
    if (reset()) {
        PB_DDR |= (1 << ONE_WIRE_BUS);  // set as output
        delay_us(450);;     // wait till end of slave pulse
        // skip rom command: 0xCC
        write_byte(0xCC);

        // Convert temperature request: 0x44
        write_byte(0x44);   // ~30 ms with 12bit precision set on the sensor

        while(!read_bit());        // wait for sensor to finish the conversion

        reset();
        delay_us(450); // wait till end of slave pulse

        // skip rom command
        write_byte(0xCC);

        // read scratch pad: 0xBE
        write_byte(0xBE);   // this will return 85 degrees if no values set

        uint8_t* scratchpad = calloc(9, sizeof(uint8_t));
        read_bytes(scratchpad, 9);

        PRINTF("CRC for SCRATCHPAD:%x\n", check_crc(scratchpad, 9));
        
        print_buf(scratchpad, 9);
        print_temp(&(scratchpad[7]));

        free(scratchpad);

        // temperature display testing
        // uint8_t* test = calloc(2, sizeof(uint8_t));

        // // -25.0625
        // test[1] = 0b01101111;
        // test[0] = 0b11111110;

        // 25.0625
        // test[1] = 0b10010001;
        // test[0] = 0b00000001;

        // 10.125
        // test[1] = 0b10100010;
        // test[0] = 0b00000000;

        // 125
        // test[1] = 0b11010000;
        // test[0] = 0b00000111;

        // 85
        // test[1] = 0b01010000;
        // test[0] = 0b00000101;
        // uint16_t test =  // -25.0625//0b0101000000000101;   
        // print_buf(test, 2);
        // print_temp(test);
        // free(test);    
    }

    free(rom_bytes);
}