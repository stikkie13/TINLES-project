#pragma once //Prevents header from being implemented more then once. https://en.cppreference.com/w/cpp/preprocessor/impl

#define NL printf("\n")

// Operation
/*
The idea is to use void pointers and structs.
*/

/*Updates the sensors.*/
void sensorRoutine();
/*Checks if hardware interupts occured.*/
void physicalInputRoutine();
/*Used to control the motors*/
void motorRoutine();