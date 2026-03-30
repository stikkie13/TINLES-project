#ifndef controllerTarg
#define controllerTarg "example"

#include "./simulator.c"
#include "./pid.ambigious.c"

// TODO:
/*
Set PID weights that ensure all pids on max have at most 1 duty cycle
Slow down pids verrryyyyy much
*/

void setAltitude(double altitude);

#define orrientationPropertionalGain 0.025
#define orrientationIntegralGain 0.025
#define orrientationDerivatigeGain 0.5

struct pidStruct rollPid = {
    orrientationPropertionalGain, // Propertional gain constant
    orrientationIntegralGain,     // Integral gain constant
    orrientationDerivatigeGain,   // Derivative gain constant
    0,                            // Anti-windup constant
    1,                            // Time constant for deprivative filtering
    Δt,                           // Timestep
    1,                            // Max command
    -1,                           // Min command
    40,                           // Max rate of change of the command
    0,                            // Integral term
    0,                            // Previous error
    0,                            // Previous derivative
    0,                            // Previous saturated command
    0                             // Previous command
};

struct pidStruct pitchPid = {
    orrientationPropertionalGain, // Propertional gain constant
    orrientationIntegralGain,     // Integral gain constant
    orrientationDerivatigeGain,   // Derivative gain constant
    0,                            // Anti-windup constant
    1,                            // Time constant for deprivative filtering
    Δt,                           // Timestep
    1,                            // Max command
    -1,                           // Min command
    40,                           // Max rate of change of the command
    0,                            // Integral term
    0,                            // Previous error
    0,                            // Previous derivative
    0,                            // Previous saturated command
    0                             // Previous command
};

struct pidStruct altitudePID = {
    0.2,   // Propertional gain constant
    0.025, // Integral gain constant
    0.20,  // Derivative gain constant
    0,     // Anti-windup constant
    1,     // Time constant for deprivative filtering
    Δt,    // Timestep
    1,     // Max command
    0,    // Min command
    40,    // Max rate of change of the command
    0,     // Integral term
    0,     // Previous error
    0,     // Previous derivative
    0,     // Previous saturated command
    0      // Previous command
};

const double pitchMask[4] = {1, 0, -1, 0};
const double rollMask[4] = {0, -1, 0, 1};

pidNumber rollTarget = 0;
pidNumber pitchTarget = 0;
pidNumber altitudeTarget = 5;

extern double state_previous[];

// proportional controller for acro/roll mode.
// CONTROLLER_P_ACRO calculates the desired rates from the sticks and applies a
// proportional controller to calculate the motors' duty cycles
// Duty cycle for each motor, 0 is the one pointing towards the camera initially, order then goes counter clockwise.
// sticks x,y,x,y?
// Gyro XYZ rot?
void controller_p_acro(double duty_cycle[4], double sticks[4], double gyro[3], double accel[3], double altitude)
{
    // pidNumber motorSpeed[4] = {0.65, 0.68, 0.65, 0.65};

    double *absoluteGyro = &state_previous[5];

    // Step
    // {1,0,-1,0}
    pidNumber rollCommand = PID_Step(&rollPid, absoluteGyro[0], rollTarget);
    // {0,1,0,-1}
    pidNumber pitchCommand = PID_Step(&pitchPid, absoluteGyro[1], pitchTarget);

    pidNumber altitudeCommand = PID_Step(&altitudePID, altitude, altitudeTarget);

    // setAltitude(4);

    for (int i = 0; i < 4; i++)
    {
        duty_cycle[i] = rollCommand * rollMask[i] * 0.2 + pitchCommand * pitchMask[i] * 0.2 + altitudeCommand * 1;
    }
    printf("roll:%f\t", absoluteGyro[0]);
    printf("rollCommand:%f\t", rollCommand);

    printf("pitch:%f\t", absoluteGyro[1]);
    printf("pitchCommand:%f\t", pitchCommand);

    printf("altitude:%f\t", altitude);
    printf("altitudeCommand:%f\t", altitudeCommand);

    printf("\n");
}

void setAltitude(double altitude)
{
    state_previous[10] = altitude; // Altitude lock
}
#endif