#include <Arduino.h>
#include <Wire.h>
#include "esp_task_wdt.h"

//Gyro
const int MPU_addr = 0x68;
float rollAngle = 0;
float pitchAngle = 0;
float alpha = 0.1; // factor for complimentary filter
const TickType_t period = pdMS_TO_TICKS(10);
TaskHandle_t gyroscopeTaskHandle;
float dt = 0.01;

const int WD_TIMEOUT = 5;

//PID
double kPAngle = 0.6, kIAngle = 3.5, kDAngle = 0.03;
double kPRate = 2, kIRate = 12, kDRate = 0;

struct PIDReturn {
  double PID;
  double integral;
  double lastError;
};

PIDReturn pitchAnglePID, rollAnglePID, pitchRatePID, rollRatePID;
double targetPitch = 0, targetRoll = 0;

int hover = 1150;
int motorInputNW, motorInputNE, motorInputSE, motorInputSW;

// New average formula (complimentary filter)
void newAverage(float roll, float pitch, float Gx, float Gy) {
  rollAngle = (1 - alpha) * (rollAngle  + Gx * dt) + alpha * roll;
  pitchAngle = (1 - alpha) * (pitchAngle + Gy * dt) + alpha * pitch;
}

PIDReturn PID(double target, double current, double integral, double lastError, double kP, double kI, double kD) {
  double error = target - current;
  integral = integral + error * dt;
  integral = constrain(integral, -400 / kI, 400 / kI);
  double derivative = (error - lastError) / dt;
  double PID = kP * error + kI * integral + kD * derivative;
  PID = constrain(PID, -400, 400);
  lastError = error;

  return {PID, integral, lastError};
}

void stabilize(float Gx, float Gy) {
  // --- outer PIDs for Pitch & Roll ---
  pitchAnglePID = PID(targetPitch, pitchAngle, pitchAnglePID.integral, pitchAnglePID.lastError, kPAngle, kIAngle, kDAngle);
  rollAnglePID = PID(targetRoll, rollAngle, rollAnglePID.integral, rollAnglePID.lastError, kPAngle, kIAngle, kDAngle);

  // --- inner PIDs for Pitch & Roll ---
  pitchRatePID = PID(pitchAnglePID.PID, Gy, pitchRatePID.integral, pitchRatePID.lastError, kPRate, kIRate, kDRate);
  rollRatePID = PID(rollAnglePID.PID, Gx, rollRatePID.integral, rollRatePID.lastError, kPRate, kIRate, kDRate);

  /* 
  Single PID for Yaw and Altitude
  */

  // --- calculate motor inputs ---
  motorInputNE = (hover - rollRatePID.PID + pitchRatePID.PID); // front right - counter clockwise
  motorInputSE = (hover - rollRatePID.PID - pitchRatePID.PID); // rear right - clockwise
  motorInputSW = (hover + rollRatePID.PID - pitchRatePID.PID); // rear left  - counter clockwise
  motorInputNW = (hover + rollRatePID.PID + pitchRatePID.PID); // front left - clockwise

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

void setup() {
  //Wire
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // select PWR_MGMT_1 register
  Wire.write(0);     // wake up
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1C);  // select ACCEL_CONFIG register
  Wire.write(0);     // configure accelerometer to 16384 LSB/g
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1B);  // GYRO_CONFIG
  Wire.write(0);     // default 250 deg/s
  Wire.endTransmission(true);

  Serial.begin(115200);

  pitchAnglePID.integral = 0;
  rollAnglePID.integral = 0;
  pitchRatePID.integral = 0; 
  rollRatePID.integral = 0;

  esp_task_wdt_init(WD_TIMEOUT, true); // !! false for debugging

  // Create gyroscopeTask
  xTaskCreatePinnedToCore(
    gyroscopeTask,        // Task function
    "gyroscopeTask",     // Name of the task
    2048,         // Stack size in words
    NULL,         // Parameter to pass to the task
    3,            // Task priority
    &gyroscopeTaskHandle,  // Task handle
    1             // Core (0 for communication, 1 for calculations and control)
  );

  esp_task_wdt_add(gyroscopeTaskHandle);
}

void gyroscopeTask(void *pvParameters) {
  // ------ Timing ------
  TickType_t lastWakeTime = xTaskGetTickCount();

  while (true)
  {
    bool newAvg = true;

    // --- Read accelerometer ---
    int16_t rawAcX, rawAcY, rawAcZ;
    float AcX, AcY, AcZ;
    float roll, pitch;

    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);  // first register of accel measurements
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 6, true); // request 6 registers (x, y and z accel)

    if (Wire.available() == 6) {
      rawAcX = Wire.read() << 8 | Wire.read();  // first read brings highest bits to the left, second fills in the rest with the lower 8 bits
      rawAcY = Wire.read() << 8 | Wire.read();
      rawAcZ = Wire.read() << 8 | Wire.read();
      AcX = (float)rawAcX / 16384.0;  // divide by LSB Sensitivity
      AcY = (float)rawAcY / 16384.0;
      AcZ = (float)rawAcZ / 16384.0;

      roll = AcY * 90;
      pitch = AcX * 90;
    } 
    else {
      newAvg = false;
    }

    // --- Read gyroscope ---
    int16_t rawGyX, rawGyY, rawGyZ;
    float Gx, Gy;

    Wire.beginTransmission(MPU_addr);
    Wire.write(0x43);  // first register of gyro
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 6, true); // request 6 registers (x, y and z gyro)

    if (Wire.available() == 6) {
      rawGyX = Wire.read() << 8 | Wire.read();  // first read brings highest bits to the left, second fills in the rest with the lower 8 bits
      rawGyY = Wire.read() << 8 | Wire.read();
      rawGyZ = Wire.read() << 8 | Wire.read();
    
      Gx = rawGyX / 131.0;  // convert to deg/s
      Gy = rawGyY / 131.0;
    } 
    else {
      newAvg = false;
    }

    if (newAvg) {
      newAverage(roll, pitch, Gx, Gy);
    }

    // --- Stabilisation ---
    stabilize(Gx, Gy);

    // Feed the watchdog
    esp_task_wdt_reset();

    vTaskDelayUntil(&lastWakeTime, period);
  }
}

void loop() {}
