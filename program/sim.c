#ifndef programTarget
#define programTarget "sim"
#include "./programDeclarations.h"

// Includes
#include <stdio.h>
#include <conio.h> // MS dependent

// declarations for Simulation in this file
void initializeInterupts()
{
    printf("Serial interupts initialized\n");
}

void sensorRoutine()
{
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
void physicalInputRoutine()
{
    char userInput[64];
    int newkeystroke = getch_noblock(); // Blocking

    if (newkeystroke < 0)
    {
        cyclesSinceLastInput += 1;
        return;
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
}

void motorRoutine()
{
}

#endif