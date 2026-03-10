#pragma once //Prevents header from being implemented more then once. https://en.cppreference.com/w/cpp/preprocessor/impl

// Operation
/*
The idea is to use void pointers and structs.
*/

/*Updates the sensors.*/
void sensorRoutine(void* pointer);
/*Checks if hardware interupts occured.*/
void physicalInputRoutine(void* pointer);
/*Used to control the motors*/
void motorRoutine(void* pointer);