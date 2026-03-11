#pragma once // Prevents header from being implemented more then once. https://en.cppreference.com/w/cpp/preprocessor/impl

//Concerns
/*This file sets the standards that the implementations must comply with.*/

#include <stdbool.h>

// Utils
#define NL printf("\n")
// Program variables
#define maxAccelometerReadings 24
#define maxGyroscopeReadings 24
//  Used to shut down the main thread.
bool persistFlag = true;

// Definitions
struct accelometerReadingStruct;
struct gyroscopeReadingStruct;

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