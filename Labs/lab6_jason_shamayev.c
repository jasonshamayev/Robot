// Hardware configuration:
// Green LED:
//   PF3 drives an NPN transistor that powers the green LED
// Blue LED:
//   PF2 drives an NPN transistor that powers the blue LED
// Pushbutton:
//   SW1 pulls pin PF4 low (internal pull-up is used)
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1
// Frequency counter and timer input:
//   FREQ_IN on PC6 (WT1CCP0)
//   TIMER2 on PC4

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "clock.h"
#include "wait.h"
#include "uart0.h"
#include "tm4c123gh6pm.h"

#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
#define PUSH_BUTTON  (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))

// PortC masks
#define FREQ_IN_MASK 64
#define TIMER2_MASK 16

// PortF masks
#define BLUE_LED_MASK 4
#define GREEN_LED_MASK 8
#define PUSH_BUTTON_MASK 16

// Global variables
//-----------------------------------------------------------------------------
bool timeMode = false;
uint32_t frequency = 0;
uint32_t time = 0;


void setCounterMode()
{
    WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER1_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER1_TAMR_R = TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR; // configure for edge count mode, count up
    WTIMER1_CTL_R = 0;                               //
    WTIMER1_IMR_R = 0;                               // turn-off interrupts
    WTIMER1_TAV_R = 0;                               // zero counter for first period
    WTIMER1_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
    //NVIC_EN3_R &= ~(1 << (INT_WTIMER1A-16-96));      // turn-off interrupt 112 (WTIMER1A)
}

void setDecrementMode()
{
    WTIMER0_CTL_R &= ~TIMER_CTL_TAEN;
    WTIMER0_CFG_R = 4;
    WTIMER0_TAMR_R = TIMER_TAMR_TAMR_CAP; // configure for edge count mode, count down
    WTIMER0_CTL_R = 0;                               //
    WTIMER0_IMR_R = 0;                               // turn-off interrupts
    WTIMER0_TAV_R = 40000;                               // zero counter for first period
    WTIMER0_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter

}

// id = 0 is left wheel, id = 1 is right wheel
void setEncoderPosition(uint8_t id, int32_t position)
{
    if(id == 0){
        WTIMER0_TAV_R = position;
    }
    if(id == 1){
        WTIMER1_TAV_R = position;
    }

}

int32_t getEncoderPosition(uint8_t id)
{
    if(id == 0){
        return WTIMER0_TAV_R;
    }
    if(id == 1){
        return WTIMER1_TAV_R;
    }
    return 0;
}

//
void selectEncoderIncMode(uint8_t id)
{
    WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;
    WTIMER0_CTL_R &= ~TIMER_CTL_TAEN;
    if(id == 0){
        WTIMER0_TAMR_R |= TIMER_TAMR_TACDIR;
    }
    if(id == 1){
        WTIMER1_TAMR_R |= TIMER_TAMR_TACDIR;
    }
    WTIMER0_CTL_R |= TIMER_CTL_TAEN;
    WTIMER1_CTL_R |= TIMER_CTL_TAEN;
}

void selectEncoderDecMode(uint8_t id)
{
    WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;
    WTIMER0_CTL_R &= ~TIMER_CTL_TAEN;
    if(id == 0){
        WTIMER0_TAMR_R &= ~TIMER_TAMR_TACDIR;
    }
    if(id == 1){
        WTIMER1_TAMR_R &= ~TIMER_TAMR_TACDIR;
    }
    WTIMER0_CTL_R |= TIMER_CTL_TAEN;
    WTIMER1_CTL_R |= TIMER_CTL_TAEN;
}

// Initialize Hardware
void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable clocks
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1 | SYSCTL_RCGCTIMER_R0 ;
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R1 | SYSCTL_RCGCWTIMER_R0;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2 | SYSCTL_RCGCGPIO_R5;
    _delay_cycles(3);

    // Configure LED and pushbutton pins
    GPIO_PORTF_DIR_R |= GREEN_LED_MASK | BLUE_LED_MASK;  // bits 1 and 2 are outputs, other pins are inputs
    GPIO_PORTF_DIR_R &= ~PUSH_BUTTON_MASK;               // bit 4 is an input
    GPIO_PORTF_DR2R_R |= GREEN_LED_MASK | BLUE_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R |= PUSH_BUTTON_MASK | GREEN_LED_MASK | BLUE_LED_MASK;
                                                         // enable LEDs and pushbuttons
    GPIO_PORTF_PUR_R |= PUSH_BUTTON_MASK;                // enable internal pull-up for push button

    // Configure FREQ_IN for frequency counter
    GPIO_PORTC_AFSEL_R |= FREQ_IN_MASK;              // select alternative functions for FREQ_IN pin
    GPIO_PORTC_PCTL_R &= ~GPIO_PCTL_PC6_M;           // map alt fns to FREQ_IN
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC6_WT1CCP0;
    GPIO_PORTC_DEN_R |= FREQ_IN_MASK;                // enable bit 6 for digital input

    //Configure Timer2
    GPIO_PORTC_AFSEL_R |= TIMER2_MASK;              // select alternative functions for FREQ_IN pin
    GPIO_PORTC_PCTL_R &= ~GPIO_PCTL_PC4_M;           // map alt fns to FREQ_IN
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC4_WT0CCP0;
    GPIO_PORTC_DEN_R |= TIMER2_MASK;                // enable bit 6 for digital input

    setCounterMode();
    setDecrementMode();
    // Configure Wide Timer 1 as counter
    /*if (timeMode)
        setTimerMode();
    else
        setCounterMode();

    // Configure Timer 1 as the time base
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER1_TAILR_R = 40000000;                       // set load value to 40e6 for 1 Hz interrupt rate
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer */
}


void wideTimer1Isr()
{
    if(timeMode)
    {
        time = WTIMER1_TAV_R;                        // read counter input
        WTIMER1_TAV_R = 0;                              // zero counter for next edge
        GREEN_LED ^= 1;                              // flash on/off green led
    }
}

int main()
{
    // Initialize hardware
       initHw();
       initUart0();

       // Setup UART0 baud rate
       setUart0BaudRate(115200, 40e6);

       // Use blue LED to show mode
       BLUE_LED = timeMode;
       uint32_t c1 = 0;
       uint32_t c2 = 0;
       setEncoderPosition(0, 0);
       setEncoderPosition(1,0);
       selectEncoderIncMode(0);
       selectEncoderIncMode(1);
       while(true){
           c1 = WTIMER1_TAV_R; // should count up by 1 each loop
           c2 = WTIMER0_TAV_R; // should count down by 1 each loop
           char str[50];
           sprintf(str, "c1 = %d, c2 = %d\n", c1, c2);
           putsUart0(str);
           //printf("c1: ", c1);
           //printf("c2: ", c2);


           if(c1 == 10 || c2 == 10){
               selectEncoderDecMode(1);
               selectEncoderDecMode(0);
           }
           if( c1 == 0|| c2  == 0){
               selectEncoderIncMode(0);
               selectEncoderIncMode(1);
           }
       }


}
