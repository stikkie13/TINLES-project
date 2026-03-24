#pragma once
// This header file is implemented by different PID implementations

// Table of contents
/*
    Declarations of core variables
    External variables from the Gyroscope
    Declarations of core functions
*/

/**/
struct PIDReturn
{
    /**/
    double PID;
    /**/
    double integral;
    /**/
    double lastError;
};

/*Ubicoders*/
double kP, kI, kD;
/**/
struct PIDReturn pitchPID, rollPID;
/**/
double targetPitch, targetRoll;
/*unit:
range:1150?*/
int hover;
/**/
int motorInputNW, motorInputNE, motorInputSE, motorInputSW;

// External variables from the gyroscope
/**/
extern float rollAngle;
/**/
extern float pitchAngle;
/**/
extern float alpha;
/*(presumedly) Delta Time*/
extern float dt;

