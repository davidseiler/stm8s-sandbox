#include <stdint.h>
#include <stdio.h>

#define F_CPU 2000000UL     // this value is to calculate time based on clock cycles, it should be set to the clock speed of the MCU

#define _SFR_(mem_addr)     (*(volatile uint8_t *)(0x5000 + (mem_addr))) // Macro function for calculating register locations. 0x5000 on the STM8 is 

/* PORT D */
#define PD_IDR      _SFR_(0x10)     // PD_IDR PortD Data Input Latch Register: for reading pin state
#define PD_DDR      _SFR_(0x11)     // PD_DDR PortD Data Direction Register (8 bit machine so registers are 8 bit): located at 0x5011 in memory so 0x5000 + 0x11 = 0x5011
#define PD_CR1      _SFR_(0x12)     // PD_CR1 PortD Control Register 1: for configuring internal pull ups, output speed, and selecting bteween push-pull or pseudo open-drain TODO figure out what this means
#define PD_CR2      _SFR_(0x13)
#define PD_ISR      6   // EXTI3 interrupt irq for port D external
#define SENSE_PIN   3

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

#define PWM_PERIOD   0x0050 

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


volatile int counter = 0;

/*
 * PD5 -> TX
 * PD6 -> RX
 */
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
        __asm__("nop");     // This is a feature of SDCC; you can put assembly instructions in C code
}

void sense_counter(void) __interrupt(PD_ISR) {      // SDCC generates the interrupt table for us
    // 2 interrupts per rotation of the fan
    // TODO this happens so fast that it takes up 100% of the CPU time, I think this might be because the fan tacho doesn't 
    counter++;
    printf("interrupt count: %d, RPM: %d\n", counter);
}

void main() {
    __asm__("rim");         // Enable interrupts
    // TIM2 SET prescaler to change the frequency
    // PWM frequency = (MASTER / 2^(Prescale value)) / Auto Reload Register
    // To control the 4-wire fan a frequency of 25 kHz is required (within 21 kHz and 28 kHz is allowed)
    TIM2_PSCR = 0b0000;
    TIM2_ARRH = (PWM_PERIOD >> 8);
    TIM2_ARRL = PWM_PERIOD;

    // Configure Channel 1 PWM
    TIM2_CCMR1 |= 0b01101000;       // enable pwm mode, OC1PE set to 1 since preload register is set in TIM2_CR1, set channel as output

    TIM2_EGR |= 0x01;       // enable the preload registers for output channel

    TIM2_CCER1 |= 0b00000001;      // Set channel as output to pin TIM2_OC1 channel is pin 29 or D10 on my eval board
    
    TIM2_CR1 |= 0b10000001; // enable ARPE and start timer

    // Setup Sense wire inputs
    PD_DDR |= (0 << SENSE_PIN);     // enable as input
    PD_CR1 |= (1 << SENSE_PIN);     // enable as pull up
    PD_CR2 |= (1 << SENSE_PIN);     // enable interrupts

    uart_init();

    
    printf("Main Loop Starting\n");
    while (1) {
        printf("Looping\n");
        int rpm = counter * 30;     // Hertz to RPM
        // Duty Cycle 25%
        TIM2_CCR1H = ((PWM_PERIOD/4) >> 8);
        TIM2_CCR1L = (PWM_PERIOD/4);
        delay_ms(3000);

        // Duty Cycle 50%
        TIM2_CCR1H = ((PWM_PERIOD/2) >> 8);     
        TIM2_CCR1L = (PWM_PERIOD/2);
        delay_ms(3000);

        // Duty Cycle 75%
        TIM2_CCR1H = ((PWM_PERIOD*3/4) >> 8);     
        TIM2_CCR1L = (PWM_PERIOD*3/4);
        delay_ms(3000);

        // Duty Cycle 100%
        TIM2_CCR1H = (PWM_PERIOD >> 8);     
        TIM2_CCR1L = (PWM_PERIOD);
        delay_ms(3000);
    }  
}