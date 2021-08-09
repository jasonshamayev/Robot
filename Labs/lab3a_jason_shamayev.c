

/**
 * main.c
 */

#include <stdint.h>
#include <stdbool.h>
#include "clock.h"
#include "tm4c123gh6pm.h"

//Bitband alias
#define BLUE_LED (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))

// PortF masks
#define BLUE_LED_MASK 4

void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable clocks
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
    _delay_cycles(3);

    // Configure LED pin
    GPIO_PORTF_DIR_R |= BLUE_LED_MASK;  // make bit an output
    GPIO_PORTF_DR2R_R |= BLUE_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R |= BLUE_LED_MASK;  // enable LED
}

void wait1Second(void){
        __asm(".const");
        __asm("US .field 8000000");
        __asm("             LDR R0, US");
        __asm("LOOP1:  SUB  R0, #1");
        __asm("             NOP");
        __asm("             CBZ  R0, DONE");
        __asm("             B    LOOP1");
        __asm("DONE:                    ");

}
int main(void)
{
    initHw();
    while(true){
        BLUE_LED ^= 1;
        wait1Second();
    }
}
