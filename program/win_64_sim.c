#ifndef programTarget
#define programTarget "sim"
#include "./programDeclarations.h"

// Includes
#include <stdio.h>
#include <conio.h> // MS dependent

// declarations
struct accelometerReadingStruct
{
    /* data */
};

struct gyroscopeReadingStruct
{
    /* data */
};

// Routine implementations
int setup()
{
    int returnCode = 0;
    printf("Serial interupts initialized\n");
    return returnCode;
}

int preRoutine()
{
    int returnCode = 0;

    programCycle++;

    return returnCode;
}

int sensorRoutine()
{
    int returnCode = 0;
}

// Checks for input high flag then reads a buffer.

int getch_noblock()
{
    if (_kbhit())
        return _getch();
    else
        return -1;
}

long int cycleOfLastInput;
int physicalInputRoutine()
{
    int returnCode = 0;
    char userInput[64];
    int keystroke = getch_noblock();

    if (keystroke > 0)
    {
        printf("\33[2K\r%c", keystroke);

        switch (keystroke)
        {
        case 27:
            persistFlag = false;
            break;

        default:

            break;
        }

        printf("\t\tNumeric=%3i cycles since last input:%d", keystroke, (programCycle - cycleOfLastInput));
        cycleOfLastInput = programCycle;
    }

    return returnCode;
}

int motorRoutine()
{
    int returnCode = 0;
    return returnCode;
}

#endif