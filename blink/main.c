#include <stdint.h>

#define F_CPU 2000000UL     // this value is to calculate time based on clock cycles, it should be set to the clock speed of the MCU

#define _SFR_(mem_addr)     (*(volatile uint8_t *)(0x5000 + (mem_addr))) // Macro function for calculating register locations. 0x5000 on the STM8 is 

/* PORT D */
#define PD_ODR      _SFR_(0x0F)     // PD_ODR PortD Data Output Latch Register: for writing pin state
#define PD_DDR      _SFR_(0x11)     // PD_DDR PortD Data Direction Register (8 bit machine so registers are 8 bit): located at 0x5011 in memory so 0x5000 + 0x11 = 0x5011
#define PD_CR1      _SFR_(0x12)     // PD_CR1 PortD Control Register 1: for configuring internal pull ups, output speed, and selecting bteween push-pull or pseudo open-drain TODO figure out what this means

#define LED_PIN     4      // PIN definition for bit operations: on my board this maps to D10

static inline void delay_ms(uint16_t ms) {
    uint32_t i;
    for (i = 0; i < ((F_CPU / 12000UL) * ms); i++)  // 110 nop instructions per ms, TODO right now these feels like magic numbers but I have a feeling it is a magic number for a ballpark estimate of time. More accurate delays should use one of the TIMER's built into STM8
        __asm__("nop");     // This is a feature of SDCC; you can put assembly instructions in C code
}

void main() {

    PD_DDR |= (1 << LED_PIN); // configure PD4 as output, equivalent to PD_DDR = PD_DDR | (1 << LED_PIN)
    PD_CR1 |= (1 << LED_PIN); // push-pull mode

    while (1) {
        // /* toggle pin every 1000ms */
        PD_ODR ^= (1 << LED_PIN);   // how does the PIN number in this instruction determine which pin on the board has power???
        delay_ms(1000);
    }    
}