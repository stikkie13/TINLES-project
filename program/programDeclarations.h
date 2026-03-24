#pragma once // Prevents header from being implemented more then once. https://en.cppreference.com/w/cpp/preprocessor/impl

#include <stdbool.h>

// Utils
#define NL printf("\n")
// Program variables
#define maxAccelometerReadings 24
#define maxGyroscopeReadings 24

// Outputs
/*unit: Floating point number
Range: 0-1
*/
typedef double motorOutput;
/*Unit:
Output: 0-1
Description: To use this variable multiply max_pulse_width. If this is 0 no pulse should occur, if this is 1 the pin should be writing high without pause.
*/
motorOutput motorOutputs[4];

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

typedef struct routineReturnStruct
{
    int code;
} routineReturn;

/*Used to set hardware and software interupts.*/
routineReturn initializeInterupts();
/*Updates the sensors.*/
routineReturn sensorRoutine();
/*Checks if hardware interupts occured.*/
routineReturn physicalInputRoutine();
/*Used to control the motors*/
routineReturn motorRoutine();