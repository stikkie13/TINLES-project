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

    programCycles++;
    
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

char keystroke;
long int cyclesSinceLastInput;
bool resetCycles = false;
int physicalInputRoutine()
{
    int returnCode = 0;
    char userInput[64];
    int newkeystroke = getch_noblock();

    if (newkeystroke < 0)
    {
        cyclesSinceLastInput += 1;
        return newkeystroke;
    }
    else
    {
        keystroke = newkeystroke;
        resetCycles = 1;
    }

    printf("\33[2K\r%c", keystroke);

    switch (keystroke)
    {
    case 27:
        persistFlag = false;
        break;

    default:

        break;
    }
    printf("\t\tNumeric=%3i cycles since last input:%d", keystroke, cyclesSinceLastInput);

    if (resetCycles)
    {
        resetCycles = 0;
        cyclesSinceLastInput = 0;
    }
    return returnCode;
}

int motorRoutine()
{
    int returnCode = 0;
    return returnCode;
}

#endif