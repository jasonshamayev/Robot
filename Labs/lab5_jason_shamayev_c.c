// Serial C/ASM Mix Example
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Red LED:
//   PF1 drives an NPN transistor that powers the red LED
// Green LED:
//   PF3 drives an NPN transistor that powers the green LED
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "clock.h"
#include "tm4c123gh6pm.h"
#include "uart0.h"

#define MAX_CHARS 80
#define MAX_FIELDS 5

typedef struct _USER_DATA
{
    char buffer[MAX_CHARS+1];
    uint8_t fieldCount;
    uint8_t fieldPosition[MAX_FIELDS];
    char fieldType[MAX_FIELDS];
} USER_DATA;

// Bitband aliases
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))

// PortF masks
#define GREEN_LED_MASK 8
#define RED_LED_MASK 2

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable clocks
    SYSCTL_RCGCGPIO_R = SYSCTL_RCGCGPIO_R5;
    _delay_cycles(3);

    // Configure LED pins
    GPIO_PORTF_DIR_R |= GREEN_LED_MASK | RED_LED_MASK;  // bits 1 and 3 are outputs
    GPIO_PORTF_DR2R_R |= GREEN_LED_MASK | RED_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R |= GREEN_LED_MASK | RED_LED_MASK;  // enable LEDs
}

char* getFieldString(USER_DATA* data, uint8_t fieldNumber){
    if(data->fieldType[fieldNumber] == 'a'){
        return &data->buffer[data->fieldPosition[fieldNumber]];
    }
    return 0;
}
int32_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber){
    if(fieldNumber < MAX_FIELDS && (data->fieldType[fieldNumber] == 'n')){
        int sum = 0;
        char* temp = &data->buffer[data->fieldPosition[fieldNumber]];
        while(!temp['\0']){
        int c = '1' - 48;
        sum = sum*10 + c;
    }
        return sum;
    }
    else{
        return 0;
    }
}

bool isCommand(USER_DATA* data, const char strCommand[], uint8_t minArguments)
{
    if(minArguments >= data->fieldCount) return false;
    uint8_t i = 0;
    char* cmd = &data->buffer[data->fieldPosition[0]];
    for(i = 0; strCommand[i] != '\0'; i++)
    {
        if(cmd[i] != strCommand[i])
            return false;
    }
    return true;
}

void parseFields(USER_DATA* data){
    data->fieldCount = 0;
    uint8_t position = 0;
    bool found = false;
    while(data->buffer[position] != '\0' && data->fieldCount < MAX_FIELDS){
        char c = data->buffer[position];
        if(!found && (c >= '0' && c <= '9'))
        {
            found = true;
            data->fieldPosition[data->fieldCount] = position;
            data->fieldType[data->fieldCount] = 'n'; //numeric
            data->fieldCount++;
        }
        else if(!found && ((c >= 'a' && c <= 'z') || c >= 'A' && c <= 'Z'))
        {
            found = true;
            data->fieldPosition[data->fieldCount] = position;
            data->fieldType[data->fieldCount] = 'a';
            data->fieldCount++;
        }
        else if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9')))
        {
            data->buffer[position] = '\0'; //delimiter
            found = false;
        }
        position++;
    }
}

void getsUart0(USER_DATA* data) {
    int count = 0;
    while(1)
    {
        char c = getcUart0();
        if( (c == 8 || c == 127) && count > 0){
            //count--;
        }
        if(c == 13){
            data->buffer[count] = '\0';
            return;
        }
        if(c >= 32){
            data->buffer[count] = c;
            count++;
            if(count == MAX_CHARS){
                data->buffer[count] = '\0';
                return;
            }
        }
    }
}
//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    initHw();
    initUart0();
    setUart0BaudRate(115200, 40e6);
    // Initialize hardware
    USER_DATA data;
    bool valid;
    while(true)
    {
        putsUart0(">");
        getsUart0(&data);
        parseFields(&data);
        putsUart0(data.buffer);
        uint8_t i;
        for (i = 0; i < data.fieldCount; i++)
        {
        putcUart0(data.fieldType[i]);
        putcUart0('\t');
        putsUart0(&data.buffer[data.fieldPosition[i]]);
        putcUart0('\n');
        }
        //#endif
        // Command evaluation
        // set add, data  add and data are integers
        if (isCommand(&data, "set", 2))
        {
        int32_t add = getFieldInteger(&data, 1);
        int32_t data = getFieldInteger(&data, 2);
        valid = true;
        // do something with this information
        }
        // alert ON|OFF  alert ON or alert OFF are the expected commands
        if (isCommand(&data, "alert", 1))
        {
        char* str = getFieldString(&data, 1);
        valid = true;
        // process the string with your custom strcmp instruction, then do something
        }
        // Process other commands here
        // Look for error
        if (!valid){
        putsUart0("Invalid Command\n");
        }

    }
    while (true);
}
