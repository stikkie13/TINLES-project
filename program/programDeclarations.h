#pragma once // Prevents header from being implemented more then once. https://en.cppreference.com/w/cpp/preprocessor/impl

// Concerns
/*This file sets the standards that the implementations must comply with.*/

#include <stdbool.h>

// Utils
#define NL printf("\n")
//      Program variables
/*Used to keep track of the amount of loops that have occured. Never write to this variable outside of incrementing it in the preroutine*/
unsigned long int programCycle = 0;
//  Used to shut down the main thread.
bool persistFlag = true;

// routines
/*
        Rules
    Routines are seperated. Perhaps in the future we can assign process priority(run this process realtime, the others only once each 60ms) to optimize power use.
    Routines return 0 if all is well.
*/
/*Used to set hardware and software interupts.
Loop routine order goes:
    0: preRoutine
    1: physicalInputRoutine
    2: sensorRoutine
    3: motorRoutine
    */
int setup();
/*Runs before the other routines each loop, increments program cycle. Free use.*/
int preRoutine();
/*Reads the sensors.*/
int sensorRoutine();
/*Checks if hardware interupts occured.*/
int physicalInputRoutine();
/*Used to control the motors.*/
int motorRoutine();