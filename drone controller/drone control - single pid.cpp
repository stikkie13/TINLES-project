
#include "./pid.c"
//Gyro
const int MPU_addr = 0x68;
float rollAngle = 0;
float pitchAngle = 0;
float alpha = 0.1; // factor for complimentary filter
float dt = 0.01;

//PID


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

  pitchPID.integral = 0;
  rollPID.integral = 0;
}

void loop() {
  bool newAvg = true;
  unsigned long startTime = micros(); // for dt calculation

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

  // --- PID ---
  stabilize();

  unsigned long elapsed = micros() - startTime;
  dt = elapsed / 1000000.0; // Update dt dynamically
}

/*
mia
  motoren moeten nog gedefineerd worden
  standard pwm is 1000 - 2000
    our pwm is ?

geen yaw

gyro alpha te groot ?

cascaded PID?
(angle error -> PID -> desired rate -> PID -> motor input) (ep 16 carbon)
*/


/*
tweak kP, kI & kD

kP
te hoog: overshoot de goal snel en met hoge frequentie
te laag: het reageert langzaam op een nieuwe input

kI
te hoog: langzame oscillations (overshoots)
te laag: het drift terug naar de voorheen gecorrigeerde angle. angle wordt niet goed vast gehouden

kD
te hoog: motoren worden twitchy
te laag: overshoots
*/
