// Hardware configuration:
// Right Motor 1:
//   M0PWM3 (PB5) drives an NPN transistor that powers RM1
// Right Motor 2:
//   M0PWM2 (PB4) drives an NPN transistor that powers RM2
// Left Motor 1:
//   M0PWM5 (PE5) drives an NPN transistor that powers LM1
// Left Motor 2:
//   M0PWM4 (PE4) drives an NPN transistor that powers LM2
// Wide Timers:
//   WTIMER0 on PC6 (WT1CCP0)
//   WTIMER1 on PC4
// SLEEP on PD7


#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "clock.h"
#include "graphics_lcd.h"
#include "backlight.h"
#include "wait.h"
#include "tm4c123gh6pm.h"
#include "uart0.h"

#define SLEEP   (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 7*4))) //PD7
#define WAIT_PB (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4))) //PF4

// PortB masks
#define RM1_MASK 32
#define RM2_MASK 16

// PortC masks
#define WTIMER0_MASK 64
#define WTIMER1_MASK 16

// PortE masks
#define LM1_MASK 32
#define LM2_MASK 16
#define SLEEP_MASK 128

// PortF masks
#define PB_MASK 16

// Global variables
//-----------------------------------------------------------------------------
bool timeMode = false;
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
        return WTIMER1_TAV_R;
    }
    if(id == 1){
        return WTIMER0_TAV_R;
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

void wideTimer1Isr()
{
    if(timeMode)
    {
        time = WTIMER1_TAV_R;                        // read counter input
        WTIMER1_TAV_R = 0;                              // zero counter for next edge
        //GREEN_LED ^= 1;                              // flash on/off green led
    }
}

void setPwmDutyCycle(uint8_t id, uint16_t pwmA, uint16_t pwmB){
    // compare/load * 100

    if(id == 1){
        PWM0_1_CMPA_R = (pwmA * 1024) /100;
        PWM0_1_CMPB_R = (pwmB * 1024) /100;
    }
    if(id == 0){
        PWM0_2_CMPA_R = (pwmA * 1024) /100;
        PWM0_2_CMPB_R = (pwmB * 1024) /100;
    }

}
// 46 rotations is 2pir 1 cm is 46/2pir
//x cm is 46/2pir(C) * x
// distance/circum * 46
// C = 19 cm
void forward(uint16_t dist_cm){
    uint32_t c1 = 0;
    setEncoderPosition(0,0);
    setEncoderPosition(1,0);
    uint16_t rotations = (dist_cm/19 * 46);
    setPwmDutyCycle(0, 90, 0);
    setPwmDutyCycle(1, 0, 90);
    selectEncoderIncMode(0);
    selectEncoderIncMode(1);
    bool b = true;
    while(b){
        c1 = getEncoderPosition(1);
        if(c1 > rotations){
            setPwmDutyCycle(0, 0, 0);
            setPwmDutyCycle(1, 0, 0);
            b = false;
        }
    }
}

void reverse(uint16_t dist_cm){

    uint32_t c1 = 0;
    setEncoderPosition(0,0);
    setEncoderPosition(1,0);
    uint16_t rotations = ((dist_cm/19) * 46);
    setPwmDutyCycle(0, 0, 90);
    setPwmDutyCycle(1, 90,0);
        selectEncoderIncMode(0);
        selectEncoderIncMode(1);
        bool b = true;
        while(b) {
            c1=getEncoderPosition(1);
            if(c1 >rotations){
               setPwmDutyCycle(0, 0, 0);
               setPwmDutyCycle(1, 0, 0);
               b = false;
            }
        }



}

// y = mx - b
// m =
// x = degrees
// b =
// 0.32*degrees - 11
void cw(uint16_t degrees){
    uint32_t c1 = 0;
    setEncoderPosition(0,0);
    setEncoderPosition(1,0);
    uint16_t rotations = ((.33*degrees) - 15);
    setPwmDutyCycle(0,80,0);
    setPwmDutyCycle(1,80,0);
    selectEncoderIncMode(0);
    selectEncoderIncMode(1);
    bool b = true;
    while(b){
        c1 = getEncoderPosition(1);
        if(c1 > rotations){
            setPwmDutyCycle(0,0,0);
            setPwmDutyCycle(1,0,0);
            b = false;

        }
    }


}

void ccw(uint16_t degrees){
    uint32_t c1 = 0;
        setEncoderPosition(0,0);
        setEncoderPosition(1,0);
        uint16_t rotations = ((.82*degrees) - 15);
        setPwmDutyCycle(0,0,40);
        setPwmDutyCycle(1,0,40);
        selectEncoderIncMode(0);
        selectEncoderIncMode(1);
        bool b = true;
        while(b){
            c1 = getEncoderPosition(1);
            if(c1 > rotations){
                setPwmDutyCycle(0,0,0);
                setPwmDutyCycle(1,0,0);
                b = false;

            }
        }

}

void stop(){
    SLEEP = 0;
}

void wait()
{
    while(GPIO_PORTF_DATA_R & PB_MASK);
}

void pause(uint32_t us)
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

void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable clocks
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R1 | SYSCTL_RCGCWTIMER_R0;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2 | SYSCTL_RCGCGPIO_R5 | SYSCTL_RCGCGPIO_R3;
    _delay_cycles(3);

    // Configure WTIMER0
    GPIO_PORTC_AFSEL_R |= WTIMER0_MASK;              // select alternative functions for WTIMER0 pin
    GPIO_PORTC_PCTL_R &= ~GPIO_PCTL_PC6_M;           // map alt fns to WTIMER0 pin
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC6_WT1CCP0;
    GPIO_PORTC_PUR_R |= WTIMER0_MASK;                //pullup resistor
    GPIO_PORTC_DEN_R |= WTIMER0_MASK;                // enable bit 6 for digital input

    //Configure WTIMER1
    GPIO_PORTC_AFSEL_R |= WTIMER1_MASK;              // select alternative functions for WTIMER1 pin
    GPIO_PORTC_PCTL_R &= ~GPIO_PCTL_PC4_M;           // map alt fns to WTIMER1
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC4_WT0CCP0;
    GPIO_PORTC_PUR_R |= WTIMER1_MASK;
    GPIO_PORTC_DEN_R |= WTIMER1_MASK;                // enable bit 4 for digital input

    //Configure SLEEP
    GPIO_PORTD_DIR_R |= SLEEP_MASK;
    GPIO_PORTD_DR2R_R |= SLEEP_MASK;
    GPIO_PORTD_DEN_R |= SLEEP_MASK;



    GPIO_PORTF_DIR_R &= ~PB_MASK;               // bit 4 is an input
    GPIO_PORTF_DEN_R |= PB_MASK;
    GPIO_PORTF_PUR_R |= PB_MASK;                // enable internal pull-up for push button

    setCounterMode();
    setDecrementMode();
    // Configure Wide Timer 1 as counter
    /*if (timeMode)
        setTimerMode();
    else
        setCounterMode(); */
}

void initPwm(void)
{
    // Enable clocks
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R0;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1 | SYSCTL_RCGCGPIO_R4;
    _delay_cycles(3);

    // Configure three backlight LEDs
    GPIO_PORTB_DIR_R |= RM1_MASK | RM2_MASK;                       // make bits 4 and 5 outputs
    GPIO_PORTB_DR2R_R |= RM1_MASK | RM2_MASK;                      // set drive strength to 2mA
    GPIO_PORTB_DEN_R |= RM1_MASK | RM2_MASK;                       // enable digital
    GPIO_PORTB_AFSEL_R |= RM1_MASK | RM2_MASK;                     // select auxilary function
    GPIO_PORTB_PCTL_R &= GPIO_PCTL_PB5_M | GPIO_PCTL_PB4_M;                    // enable PWM
    GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB5_M0PWM3 | GPIO_PCTL_PB4_M0PWM2;

    GPIO_PORTE_DIR_R |= LM1_MASK | LM2_MASK;     // make bits 4 and 5 outputs
    GPIO_PORTE_DR2R_R |= LM1_MASK | LM2_MASK;    // set drive strength to 2mA
    GPIO_PORTE_DEN_R |= LM1_MASK | LM2_MASK;     // enable digital
    GPIO_PORTE_AFSEL_R |= LM1_MASK | LM2_MASK;   // select auxilary function
    GPIO_PORTE_PCTL_R &= GPIO_PCTL_PE4_M | GPIO_PCTL_PE5_M;    // enable PWM
    GPIO_PORTE_PCTL_R |= GPIO_PCTL_PE4_M0PWM4 | GPIO_PCTL_PE5_M0PWM5;

    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R0;                // reset PWM0 module
    SYSCTL_SRPWM_R = 0;                              // leave reset state
    PWM0_1_CTL_R = 0;                                // turn-off PWM0 generator 1 (drives outs 2 and 3)
    PWM0_2_CTL_R = 0;                                // turn-off PWM0 generator 2 (drives outs 4 and 5)
    PWM0_1_GENB_R = PWM_0_GENB_ACTCMPBD_ZERO | PWM_0_GENB_ACTLOAD_ONE;
                                                     // output 3 on PWM0, gen 1b, cmpb
    PWM0_1_GENA_R = PWM_0_GENA_ACTCMPAD_ZERO | PWM_0_GENA_ACTLOAD_ONE;
    PWM0_2_GENA_R = PWM_0_GENA_ACTCMPAD_ZERO | PWM_0_GENA_ACTLOAD_ONE;
                                                     // output 4 on PWM0, gen 2a, cmpa
    PWM0_2_GENB_R = PWM_0_GENB_ACTCMPBD_ZERO | PWM_0_GENB_ACTLOAD_ONE;
                                                     // output 5 on PWM0, gen 2b, cmpb
    PWM0_1_LOAD_R = 1024;                            // set frequency to 40 MHz sys clock / 2 / 1024 = 19.53125 kHz
    PWM0_2_LOAD_R = 1024;
    PWM0_INVERT_R = PWM_INVERT_PWM3INV | PWM_INVERT_PWM4INV | PWM_INVERT_PWM5INV | PWM_INVERT_PWM2INV;
                                                     // invert outputs so duty cycle increases with increasing compare values
    PWM0_1_CMPB_R = 0;                               // RM1 off (0=always low, 1023=always high)
    PWM0_2_CMPB_R = 0;                               // LM2 off
    PWM0_2_CMPA_R = 0;                               // LM1 off
    PWM0_1_CMPA_R = 0;                               // RM2 off

    PWM0_1_CTL_R = PWM_0_CTL_ENABLE;                 // turn-on PWM0 generator 1
    PWM0_2_CTL_R = PWM_0_CTL_ENABLE;                 // turn-on PWM0 generator 2
    PWM0_ENABLE_R = PWM_ENABLE_PWM3EN | PWM_ENABLE_PWM4EN | PWM_ENABLE_PWM5EN | PWM_ENABLE_PWM2EN;
                                                     // enable outputs
}


int main()
{
    // Initialize hardware
       initHw();
       initUart0();
       initPwm();
       SLEEP = 1;
       // Setup UART0 baud rate
      // setUart0BaudRate(115200, 40e6);

       // Use blue LED to show mode
       /*BLUE_LED = timeMode;
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
       } */
       wait();
       forward(30);
       //pause(1000);
       //reverse(100);
       //ccw(90);
       cw(90);
       forward(30);
       cw(90);
       forward(30);
       cw(90);
       stop();
      /* wait();
       forward(100);
       reverse(100); */

//       setPwmDutyCycle(0, 90, 0);
//       setPwmDutyCycle(1, 0, 90);
//       forward(100);
//       reverse(100);

//       PWM0_1_CMPA_R = 1024;
//       PWM0_1_CMPB_R = 1024;

      while(true);



}

