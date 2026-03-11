#pragma once // Prevents header from being implemented more then once. https://en.cppreference.com/w/cpp/preprocessor/impl

#include <stdbool.h>

// Utils
#define NL printf("\n")
// Program variables
#define maxAccelometerReadings 24
#define maxGyroscopeReadings 24

// Operation
/*
The idea is to use void pointers and structs.
*/

// Holds
struct accelometerReadingStruct
{
    /* data */
};

struct gyroscopeReadingStruct
{
    /* data */
};

// Arrays for sensor readings
struct accelometerReadingStruct accelometerReadings[maxAccelometerReadings];
struct gyroscopeReadingStruct gryroscopeReadings[maxGyroscopeReadings];

// routines
/*Used to set hardware and software interupts.*/
void initializeInterupts();
/*Updates the sensors.*/
void sensorRoutine();
/*Checks if hardware interupts occured.*/
void physicalInputRoutine();
/*Used to control the motors*/
void motorRoutine();