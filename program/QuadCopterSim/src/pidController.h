#ifndef controllerTarg
#define controllerTarg "example"

#include "./simulator.c"
#include "./pid.ambigious.c"

struct pidStruct rollPid = {
    1,   // Propertional gain constant
    0.1, // Integral gain constant
    5,   // Derivative gain constant
    0.1, // Anti-windup constant
    1,   // Time constant for deprivative filtering
    Δt,  // Timestep
    1,   // Max command
    0,   // Min command
    40,  // Max rate of change of the command
    0,   // Integral term
    0,   // Previous error
    0,   // Previous derivative
    0,   // Previous saturated command
    0    // Previous command
};

struct pidStruct pitchPid = {
    1,   // Propertional gain constant
    0.1, // Integral gain constant
    5,   // Derivative gain constant
    0.1, // Anti-windup constant
    1,   // Time constant for deprivative filtering
    Δt,  // Timestep
    1,   // Max command
    0,   // Min command
    40,  // Max rate of change of the command
    0,   // Integral term
    0,   // Previous error
    0,   // Previous derivative
    0,   // Previous saturated command
    0    // Previous command
};

pidNumber rollTarget = 0;
pidNumber pitchTarget = 0;

// proportional controller for acro/roll mode.
// CONTROLLER_P_ACRO calculates the desired rates from the sticks and applies a
// proportional controller to calculate the motors' duty cycles
// Duty cycle for each motor?
// sticks x,y,x,y?
// Gyro XYZ rot?
void controller_p_acro(double duty_cycle[4], double sticks[4], double gyro[3], double accel[3], double altitude)
{
    pidNumber motorSpeed[4];

    // Step
    // asuming front left is [0]. {1,-1,-1,1}
    pidNumber rollCommand = PID_Step(&rollPid, gyro[0], rollTarget);
    // {1,1,-1,-1}
    pidNumber pitchCommand = PID_Step(&pitchPid, gyro[1], pitchTarget);

    for (int i = 0; i < 4; i++)
    {

        duty_cycle[i] = motorSpeed[i];
    }
}

#endif