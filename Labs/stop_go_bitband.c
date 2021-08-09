 // Stop Go C Example (Bitbanding)
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Red LED:
//   PE1 drives an NPN transistor that powers the red LED
// Green LED:
//   PB2 drives an NPN transistor that powers the green LED
// Blue LED:
//   PE0 drives an XXX transistor that powers the blue LED
// Yellow LED:
//   PB3 drives an XXX transistor that powers the yellow LED
// SW1:
//   PD0
// SW2:
//   PD1
// Pushbutton:
//   SW1 pulls pin PF4 low (internal pull-up is used)

// NPN 3904
// PNP 3906
//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "clock.h"
#include "tm4c123gh6pm.h"

// Bitband aliases USE APB
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 2*4)))
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 0*4)))
#define YELLOW_LED   (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 3*4)))
//#define PUSH_BUTTON  (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))
#define SWITCH_ONE   (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 0*4)))
#define SWITCH_TWO   (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 1*4)))

// PortF masks // masks are just 2^ # of pin connected to. Red is on pin 1 so 2^1 = 2
#define GREEN_LED_MASK 4
#define RED_LED_MASK 2
#define BLUE_LED_MASK 1
#define YELLOW_LED_MASK 8
//#define PUSH_BUTTON_MASK 16
#define SWITCH_ONE_MASK 1
#define SWITCH_TWO_MASK 2
//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Blocking function that returns only when SW1 is pressed
void waitPbPress(void)
{
	//while(PUSH_BUTTON);
}

// Initialize Hardware
void initHw(void)
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable clocks
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4 | SYSCTL_RCGCGPIO_R1 | SYSCTL_RCGCGPIO_R3;
    _delay_cycles(3);

    GPIO_PORTE_DIR_R |= RED_LED_MASK | BLUE_LED_MASK;
    GPIO_PORTE_DR2R_R |= RED_LED_MASK | BLUE_LED_MASK;
    GPIO_PORTE_DEN_R |= RED_LED_MASK | BLUE_LED_MASK;

    GPIO_PORTB_DIR_R |= YELLOW_LED_MASK | GREEN_LED_MASK;
    GPIO_PORTB_DR2R_R |= YELLOW_LED_MASK | GREEN_LED_MASK;
    GPIO_PORTB_DEN_R |= YELLOW_LED_MASK | GREEN_LED_MASK;

    GPIO_PORTD_DIR_R &= ~(SWITCH_ONE_MASK | SWITCH_TWO_MASK);
    GPIO_PORTD_DR2R_R |= SWITCH_ONE_MASK | SWITCH_TWO_MASK;
    GPIO_PORTD_DEN_R |= SWITCH_ONE_MASK | SWITCH_TWO_MASK;

    GPIO_PORTD_PUR_R |= SWITCH_ONE_MASK; // pull up resistor
    GPIO_PORTD_PDR_R |= SWITCH_TWO_MASK; // pull down resistor

    //GPIO_PORTF_PUR_R |= PUSH_BUTTON_MASK;                // enable internal pull-up for push button
}

void waitMicrosecond(uint32_t us)
{
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*2 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             NOP");                  // 1
    __asm("             B    WMS_LOOP0");       // 1*2 (speculative, so P=1)
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}

void waitSw1Pressed()
{
    while(SWITCH_ONE);
}

void waitSw2Pressed()
{
    while(!SWITCH_TWO);
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
	// Initialize hardware
	initHw();

    RED_LED = 0;
    YELLOW_LED = 1;
    BLUE_LED = 0;
    GREEN_LED = 1;
    waitSw1Pressed();
    RED_LED = 1;
    waitSw2Pressed();
    RED_LED = 0;
    GREEN_LED = 0;
    waitMicrosecond(1000000);
    BLUE_LED = 1;
    waitSw1Pressed();
    while(1){
    waitMicrosecond(500000);
    YELLOW_LED ^= 1;
    }

    // Endless loop
    while(true);
}
