#ifndef PIDImplementation
#define PIDImplementation "arduino"

#include <Arduino.h>
#include <Wire.h>


#include "./pid.h"

/*Ubicoders*/
kP = 0.5, kI = 0.05, kD = 0.1;
/**/
targetPitch = 0, targetRoll = 0;
/*unit:
range:1150?*/
hover = 1150;

// New average formula (complimentary filter)
void newAverage(float roll, float pitch, float Gx, float Gy)
{
    rollAngle = (1 - alpha) * (rollAngle + Gx * dt) + alpha * roll;
    pitchAngle = (1 - alpha) * (pitchAngle + Gy * dt) + alpha * pitch;
}

struct PIDReturn PID(double target, double current, double integral, double lastError, double kP, double kI, double kD)
{
    struct PIDReturn returnValue;

    double error = target - current;
    integral = integral + error * dt;
    integral = constrain(integral, -400 / kI, 400 / kI);
    double derivative = (error - lastError) / dt;
    double PID = kP * error + kI * integral + kD * derivative;
    PID = constrain(PID, -400, 400);
    lastError = error;

    returnValue.integral = integral;
    returnValue.lastError = lastError;
    returnValue.PID = PID;

    return returnValue;
}

void
stabilize()
{
    // --- PIDs for Pitch & Roll ---
    pitchPID = PID(targetPitch, pitchAngle, pitchPID.integral, pitchPID.lastError, kP, kI, kD);
    rollPID = PID(targetRoll, rollAngle, rollPID.integral, rollPID.lastError, kP, kI, kD);

    // --- calculate motor inputs ---
    motorInputNE = (hover - rollPID.PID + pitchPID.PID); // front right - counter clockwise
    motorInputSE = (hover - rollPID.PID - pitchPID.PID); // rear right - clockwise
    motorInputSW = (hover + rollPID.PID - pitchPID.PID); // rear left  - counter clockwise
    motorInputNW = (hover + rollPID.PID + pitchPID.PID); // front left - clockwise

    motorInputNE = constrain(motorInputNE, 1000, 2000);
    motorInputSE = constrain(motorInputSE, 1000, 2000);
    motorInputSW = constrain(motorInputSW, 1000, 2000);
    motorInputNW = constrain(motorInputNW, 1000, 2000);

    // --- apply to motors ---

    // -- prints ---
    Serial.print("Roll: ");
    Serial.print(rollAngle);
    Serial.print(" | Pitch: ");
    Serial.println(pitchAngle);

    Serial.print("NE: ");
    Serial.print(motorInputNE);
    Serial.print(" | SE: ");
    Serial.print(motorInputSE);
    Serial.print(" | SW: ");
    Serial.print(motorInputSW);
    Serial.print(" | NW: ");
    Serial.println(motorInputNW);
}

#endif