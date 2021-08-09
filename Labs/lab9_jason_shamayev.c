// Serial C/ASM Mix Example
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "clock.h"
#include "tm4c123gh6pm.h"
#include "uart0.h"
#include "lab8.h"

#define MAX_CHARS 80
#define MAX_FIELDS 20

typedef struct _USER_DATA
{
    char buffer[MAX_CHARS+1];
    uint8_t fieldCount;
    uint8_t fieldPosition[MAX_FIELDS];
    char fieldType[MAX_FIELDS];
} USER_DATA;

typedef struct _instruction
{
    uint8_t command;
    uint16_t argument;
} instruction;

instruction list[MAX_FIELDS];

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware


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

bool stringComp(char* array, const char strCommand[])
{
    uint8_t i = 0;
    for(i = 0; strCommand[i] != '\0'; i++)
    {
        if(array[i] != strCommand[i])
            return false;
    }
    return true;
}

void parseFields(USER_DATA* data){
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

void listFunction(USER_DATA data, int size){
    char uart[MAX_FIELDS];
    int i = 0;
    for(i = 0; i < size; i++){
                //char type[20];
                switch(list[i].command){
                case '0':
                    //type = "forward";
                    sprintf(uart,"%d forward %ld", i, &data.buffer[data.fieldPosition[1]]);
                    break;
                case '1':
                    //type = 'reverse';
                    sprintf(uart,"%d reverse %ld", i, &data.buffer[data.fieldPosition[1]]);
                    break;
                case '2':
                    //type = 'cw';
                    sprintf(uart,"%d cw %ld", i, &data.buffer[data.fieldPosition[1]]);
                    break;
                case '3':
                    //type = 'ccw';
                    sprintf(uart,"%d ccw %ld", i, &data.buffer[data.fieldPosition[1]]);
                    break;
                case '4':
                    //type = 'wait';
                    sprintf(uart,"%d wait pb", i);
                    break;
                case '5':
                    //type = 'pause';
                    sprintf(uart,"%d pause %ld", i, &data.buffer[data.fieldPosition[1]]);
                    break;
                case '6':
                    //type = 'stop';
                    sprintf(uart,"%d stop", i);
                    break;
                }
                putsUart0(uart);
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
    instruction instr;
    int instructionCount = 0;
    //bool valid;
    int size = 0;
    while(true){
        putsUart0(">");
        getsUart0(&data);
        parseFields(&data);
        //putsUart0(data.buffer);
        //        uint8_t i;
        //char array[10] = "forward";
        //char* cmd = &data.buffer[data.fieldPosition[0]];
//        if(stringComp(cmd, "forward")){
//            putsUart0("works");
//        }
//        else
//            putsUart0("fail");
        if (stringComp(&data.buffer[data.fieldPosition[0]], "forward"))
        {
            putsUart0("going forward...");
            list[size].command = 0;
            list[size].argument = atoi( &data.buffer[data.fieldPosition[1]]);
            forward(list[size].argument);
        }
        else if(stringComp(&data.buffer[data.fieldPosition[0]], "reverse"))
        {
            putsUart0("going reverse...");
            list[size].command = 1;
            list[size].argument = atoi( &data.buffer[data.fieldPosition[1]]);
            reverse(list[size].argument);
        }
        else if(stringComp(&data.buffer[data.fieldPosition[0]], "cw"))
        {
            putsUart0("going cw...");
            list[size].command = 2;
            list[size].argument = atoi( &data.buffer[data.fieldPosition[1]]);
            cw(list[size].argument);
        }
        else if(stringComp(&data.buffer[data.fieldPosition[0]], "ccw"))
                {
                    putsUart0("going ccw...");
                    list[size].command = 3;
                    list[size].argument = atoi( &data.buffer[data.fieldPosition[1]]);
                    ccw(list[size].argument);
                }
        else if(stringComp(&data.buffer[data.fieldPosition[0]], "wait"))
                {
                    putsUart0("going wait...");
                    list[size].command = 4;
                    list[size].argument = atoi( &data.buffer[data.fieldPosition[1]]);
                }
        else if(stringComp(&data.buffer[data.fieldPosition[0]], "pause"))
                {
                    putsUart0("going pause...");
                    list[size].command = 5;
                    list[size].argument = atoi( &data.buffer[data.fieldPosition[1]]);
                    pause(list[size].argument);
                }
        else if(stringComp(&data.buffer[data.fieldPosition[0]], "stop"))
                {
                    putsUart0("going stop...");
                    list[size].command = 6;
                    list[size].argument = atoi( &data.buffer[data.fieldPosition[1]]);
                }
        else if(stringComp(&data.buffer[data.fieldPosition[0]], "list")){
                putsUart0("listing");
                list[size].command = 7;
                listFunction(data, size);
        }
        size++;
    }

        /*switch(list[size].command){
            case '0':
                list[instructionCount].command = 0;
                list[instructionCount].argument = &data.buffer[data.fieldPosition[1]];
                forward(list[instructionCount].argument);
                instructionCount++;
                break;
            case '1':
                reverse(instr.argument);
                list[instructionCount].command = 1;
                list[instructionCount].argument = instr.argument;
                instructionCount++;
                break;
            case '2':
                cw(instr.argument);
                list[instructionCount].command = 2;
                list[instructionCount].argument = instr.argument;
                instructionCount++;
                break;
            case '3':
                ccw(instr.argument);
                list[instructionCount].command = 3;
                list[instructionCount].argument = instr.argument;
                instructionCount++;
                break;
            case '4':
                wait(); // pb
                list[instructionCount].command = 4;
                instructionCount++;
                break;
            case '5':
                pause(instr.argument); // TIME_MS
                list[instructionCount].command = 5;
                list[instructionCount].argument = instr.argument;
                instructionCount++;
                break;
            case '6':
                stop();
                list[instructionCount].command = 6;
                instructionCount++;
                break;
            } */
        // print list
        //size_t size = sizeof(list)/sizeof(list[0]);
        int i;
        char uart[MAX_FIELDS];
        for(i = 0; i < size; i++){
            //char type[20];
            switch(list[i].command){
            case '0':
                //type = "forward";
                sprintf(uart,"%d forward %ld", i, &data.buffer[data.fieldPosition[1]]);
                break;
            case '1':
                //type = 'reverse';
                sprintf(uart,"%d reverse %ld", i, &data.buffer[data.fieldPosition[1]]);
                break;
            case '2':
                //type = 'cw';
                sprintf(uart,"%d cw %ld", i, &data.buffer[data.fieldPosition[1]]);
                break;
            case '3':
                //type = 'ccw';
                sprintf(uart,"%d ccw %ld", i, &data.buffer[data.fieldPosition[1]]);
                break;
            case '4':
                //type = 'wait';
                sprintf(uart,"%d wait pb", i);
                break;
            case '5':
                //type = 'pause';
                sprintf(uart,"%d pause %ld", i, &data.buffer[data.fieldPosition[1]]);
                break;
            case '6':
                //type = 'stop';
                sprintf(uart,"%d stop", i);
                break;
            }
            putsUart0(uart);
    }

//    while(true)
//    {
//        putsUart0(">");
//        getsUart0(&data);
//        parseFields(&data);
//        putsUart0(data.buffer);
//        uint8_t i;
//        for (i = 0; i < data.fieldCount; i++)
//        {
//        putcUart0(data.fieldType[i]);
//        putcUart0('\t');
//        putsUart0(&data.buffer[data.fieldPosition[i]]);
//        putcUart0('\n');
//        }
//        //#endif
//        // Command evaluation
//        // set add, data  add and data are integers
//        if (isCommand(&data, "set", 2))
//        {
//        int32_t add = getFieldInteger(&data, 1);
//        int32_t data = getFieldInteger(&data, 2);
//        valid = true;
//        // do something with this information
//        }
//        // alert ON|OFF  alert ON or alert OFF are the expected commands
//        if (isCommand(&data, "alert", 1))
//        {
//        char* str = getFieldString(&data, 1);
//        valid = true;
//        // process the string with your custom strcmp instruction, then do something
//        }
//        // Process other commands here
//        // Look for error
//        if (!valid){
//        putsUart0("Invalid Command\n");
//        }
//
//    }
    while (true);
}
