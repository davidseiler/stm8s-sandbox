#include <stdint.h>

#define F_CPU 2000000UL     // this value is to calculate time based on clock cycles, it should be set to the clock speed of the MCU

#define _SFR_(mem_addr)     (*(volatile uint8_t *)(0x5000 + (mem_addr))) // Macro function for calculating register locations. 0x5000 on the STM8 is 

/* PORT D */
#define PD_ODR      _SFR_(0x0F)     // PD_ODR PortD Data Output Latch Register: for writing pin state
#define PD_DDR      _SFR_(0x11)     // PD_DDR PortD Data Direction Register (8 bit machine so registers are 8 bit): located at 0x5011 in memory so 0x5000 + 0x11 = 0x5011
#define PD_CR1      _SFR_(0x12)     // PD_CR1 PortD Control Register 1: for configuring internal pull ups, output speed, and selecting bteween push-pull or pseudo open-drain TODO figure out what this means

// TIM2 registers
#define TIM2_ARRH    _SFR_(0x30D)
#define TIM2_ARRL    _SFR_(0x30E)
#define TIM2_CCR1H   _SFR_(0x30F)
#define TIM2_CCR1L   _SFR_(0x310)

#define TIM2_CCMR1   _SFR_(0x305)
#define TIM2_CCER1   _SFR_(0x308)
#define TIM2_PSCR    _SFR_(0x30C)
#define TIM2_IER     _SFR_(0x301)
#define TIM2_EGR     _SFR_(0x304)
#define TIM2_CR1     _SFR_(0x300)
#define PWM_PERIOD   100

static inline void delay_ms(uint16_t ms) {  // TODO: 1000ms delay is in reality creating ~668ms of delay based on logic analyzer
    uint32_t i;
    for (i = 0; i < ((F_CPU / 18000UL) * ms); i++)  // 110 nop instructions per ms
        __asm__("nop");     // This is a feature of SDCC; you can put assembly instructions in C code
}

void main() {

    // TIM2 SET prescaler to change the frequency
    // PWM frequency = (MASTER / 2^(Prescale value)) / Auto Reload Register
    // Given values currently this is 10 kHz
    TIM2_PSCR = 0b00000001;
    TIM2_ARRH = (PWM_PERIOD >> 8);
    TIM2_ARRL = PWM_PERIOD;

    // Configure Channel 1 PWM
    TIM2_CCMR1 |= 0b01101000;       // enable pwm mode, OC1PE set to 1 since preload register is set in TIM2_CR1, set channel as output

    TIM2_EGR |= 0x01;       // enable the preload registers for output channel

    TIM2_CCER1 |= 0b00000001;      // Set channel as output to pin TIM2_OC1 channel is pin 29 or D10 on my eval board
    
    TIM2_CR1 |= 0b10000001; // enable ARPE and start timer
    
    while (1) {
        // Duty Cycle 25%
        TIM2_CCR1H = ((PWM_PERIOD/4) >> 8);
        TIM2_CCR1L = (PWM_PERIOD/4);
        delay_ms(1000);

        // Duty Cycle 50%
        TIM2_CCR1H = ((PWM_PERIOD/2) >> 8);     
        TIM2_CCR1L = (PWM_PERIOD/2);
        delay_ms(1000);

        // Duty Cycle 75%
        TIM2_CCR1H = ((PWM_PERIOD*3/4) >> 8);     
        TIM2_CCR1L = (PWM_PERIOD*3/4);
        delay_ms(1000);

        // Duty Cycle 100%
        TIM2_CCR1H = (PWM_PERIOD >> 8);     
        TIM2_CCR1L = (PWM_PERIOD);
        delay_ms(1000);
    }  
}