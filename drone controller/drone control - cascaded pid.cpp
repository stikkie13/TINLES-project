/*
TO DO
- juiste pins voor de motoren
- yaw PID ?
- measure height ?
- height PID ?
*/

#include <Arduino.h>
#include <Wire.h>
#include "esp_task_wdt.h"
#include <WiFi.h>
#include <WebServer.h>

// Motors
#define motorPinNW 1 // dummy values voor de pins
#define motorChannelNW 0
#define motorPinNE 2
#define motorChannelNE 1
#define motorPinSE 3
#define motorChannelSE 2
#define motorPinSW 4
#define motorChannelSW 3

// Gyro
const int MPU_addr = 0x68;
float rollAngle = 0;
float pitchAngle = 0;
float alpha = 0.1; // factor for complimentary filter
const TickType_t period = pdMS_TO_TICKS(4);
TaskHandle_t gyroscopeTaskHandle;
float dt = 0.01;

const int WD_TIMEOUT = 5;

// PID
double kPAngle = 0.6, kIAngle = 3.5, kDAngle = 0.03;
double kPRate = 2, kIRate = 12, kDRate = 0;

struct PIDReturn {
  double PID;
  double integral;
  double lastError;
};

PIDReturn pitchAnglePID, rollAnglePID, pitchRatePID, rollRatePID;
double targetPitch = 0, targetRoll = 0;

int hover = 150;
int motorInputNW, motorInputNE, motorInputSE, motorInputSW;

// AP
#define button 32
#define interrupt_button_gpio GPIO_NUM_32

RTC_DATA_ATTR int bootCount = 0;

const char *ssid = "ESP32S3-AP";
const char *password = "embeddedgroup4";

WebServer server(80);

TaskHandle_t sleepAndAPTaskHandle;

const char* htmlPage = R"rawliteral(
<!DOCTYPE html>
<head>
  <style>
    body {
      font-family: Arial, sans-serif;
      text-align: center;
      margin-top: 50px;
    }

    #joystickContainer {
      width: 200px;
      height: 200px;
      background: #ddd;
      border-radius: 50%;
      position: relative;
      margin: 20px auto;
      touch-action: none;
    }

    #stick {
      width: 80px;
      height: 80px;
      background: #555;
      border-radius: 50%;
      position: absolute;
      top: 60px;
      left: 60px;
    }

    button {
      padding: 15px 30px;
      font-size: 18px;
      cursor: pointer;
    }
  </style>
</head>
<body>

  <div id="joystickContainer">
    <div id="stick"></div>
  </div>

  <button onclick="location.href='/OFF'">OFF</button>

  <p id="output">X: 0 | Y: 0</p>

  <script>
    const stick = document.getElementById("stick");
    const container = document.getElementById("joystickContainer");
    const output = document.getElementById("output");

    let dragging = false;

    container.addEventListener("mousedown", start);
    container.addEventListener("touchstart", start);

    document.addEventListener("mousemove", move);
    document.addEventListener("touchmove", move);

    document.addEventListener("mouseup", end);
    document.addEventListener("touchend", end);

    function start(e) {
      dragging = true;
    }

    let lastUpdate = 0;

    function move(e) {
      if (!dragging) return;

      let rect = container.getBoundingClientRect();
      let x = (e.touches ? e.touches[0].clientX : e.clientX) - rect.left;
      let y = (e.touches ? e.touches[0].clientY : e.clientY) - rect.top;

      let centerX = rect.width / 2;
      let centerY = rect.height / 2;

      let dx = x - centerX;
      let dy = y - centerY;

      let max = 60;
      let distance = Math.sqrt(dx * dx + dy * dy);

      if (distance > max) {
        dx = (dx / distance) * max;
        dy = (dy / distance) * max;
      }

      stick.style.left = (centerX + dx - 40) + "px";
      stick.style.top = (centerY + dy - 40) + "px";

      dx = dx / 4;
      dy = dy / 4;

      output.innerText = `X: ${Math.round(dx)} | Y: ${Math.round(dy)}`;

      if(Date.now() - lastUpdate > 50) {
        fetch("/joystick?angleX=" + Math.round(dx) + "&angleY=" + Math.round(dy));
        lastUpdate = Date.now();
      }
    }

    function end() {
      dragging = false;
      stick.style.left = "60px";
      stick.style.top = "60px";
      output.innerText = "X: 0 | Y: 0";

      fetch("/joystick?angleX=" + 0 + "&angleY=" + 0);
    }
  </script>

</body>
</html>
)rawliteral";

void handleRoot() {
  server.send(200, "text/html", htmlPage);
}

void joystickInterrupt() {
  int roll = server.arg("angleX").toInt();
  int pitch = server.arg("angleY").toInt();

  Serial.print(roll);
  Serial.print(", ");
  Serial.println(pitch);

  targetPitch = pitch;
  targetRoll = roll;

  handleRoot();
}

void OFFInterrupt() {
  Serial.println("drone asleep (handleOFF())");
  server.close();
  esp_deep_sleep_start();
}

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
  // outer PIDs for Pitch & Roll: Desired angle -> desired rotation speed
  pitchAnglePID = PID(targetPitch, pitchAngle, pitchAnglePID.integral, pitchAnglePID.lastError, kPAngle, kIAngle, kDAngle);
  rollAnglePID = PID(targetRoll, rollAngle, rollAnglePID.integral, rollAnglePID.lastError, kPAngle, kIAngle, kDAngle);

  // inner PIDs for Pitch & Roll: Desired rotation speed -> motor input
  pitchRatePID = PID(pitchAnglePID.PID, Gy, pitchRatePID.integral, pitchRatePID.lastError, kPRate, kIRate, kDRate);
  rollRatePID = PID(rollAnglePID.PID, Gx, rollRatePID.integral, rollRatePID.lastError, kPRate, kIRate, kDRate);

  /* 
  Single PID for Yaw and Altitude
  */

  // calculate motor inputs
  motorInputNW = (hover + rollRatePID.PID + pitchRatePID.PID); // front left - clockwise
  motorInputNE = (hover - rollRatePID.PID + pitchRatePID.PID); // front right - counter clockwise
  motorInputSE = (hover - rollRatePID.PID - pitchRatePID.PID); // rear right - clockwise
  motorInputSW = (hover + rollRatePID.PID - pitchRatePID.PID); // rear left  - counter clockwise

  motorInputNW = constrain(motorInputNW, 0, 1023);
  motorInputNE = constrain(motorInputNE, 0, 1023);
  motorInputSE = constrain(motorInputSE, 0, 1023);
  motorInputSW = constrain(motorInputSW, 0, 1023);

  // apply to motors
  ledcWrite(motorChannelNW, motorInputNW);
  ledcWrite(motorChannelNE, motorInputNE);
  ledcWrite(motorChannelSE, motorInputSE);
  ledcWrite(motorChannelSW, motorInputSW);

//   Serial.print("Roll: ");
//   Serial.print(rollAngle);
//   Serial.print(" | Pitch: ");
//   Serial.println(pitchAngle);
  
//   Serial.print("NE: ");
//   Serial.print(motorInputNE);
//   Serial.print(" | SE: ");
//   Serial.print(motorInputSE);
//   Serial.print(" | SW: ");
//   Serial.print(motorInputSW);
//   Serial.print(" | NW: ");
//   Serial.println(motorInputNW);
}

void gyroscopeTask(void *pvParameters) {
  // Timing
  TickType_t lastWakeTime = xTaskGetTickCount();

  while (true)
  {
    bool newAvg = true;

    // Read accelerometer
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

    // Read gyroscope
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

    // Stabilisation
    stabilize(Gx, Gy);

    // Pet the watchdog
    esp_task_wdt_reset();

    vTaskDelayUntil(&lastWakeTime, period);
  }
}

void sleepAndAPControl(void *pvParameters) {
  TickType_t lastWakeTime = xTaskGetTickCount();

  while(1){
    server.handleClient();

    if(digitalRead(button) == LOW) {
      while(digitalRead(button) == LOW) {
          delay(100);
      }
      Serial.println("drone asleep (loop())");
      esp_deep_sleep_start();
    }

    esp_task_wdt_reset();

    vTaskDelayUntil(&lastWakeTime, period);
  }
}

void setup() {
  // Sleep
  pinMode(button, INPUT_PULLUP);

  bootCount++;

  esp_sleep_enable_ext0_wakeup(interrupt_button_gpio, LOW);

  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  if(bootCount == 1 || wakeup_reason != ESP_SLEEP_WAKEUP_EXT0) {
    Serial.println("drone asleep (setup())");
    esp_deep_sleep_start();
  }

  while(digitalRead(button) == LOW) {
    delay(100);
  }
  Serial.println("drone awake");
  delay(3000);

  // Motors
  int PWMFrequency = 10000;
  int PWMResolution = 10;

  ledcSetup(motorChannelNW, PWMFrequency, PWMResolution);
  ledcAttachPin(motorPinNW, motorChannelNW);

  ledcSetup(motorChannelNE, PWMFrequency, PWMResolution);
  ledcAttachPin(motorPinNE, motorChannelNE);

  ledcSetup(motorChannelSE, PWMFrequency, PWMResolution);
  ledcAttachPin(motorPinSE, motorChannelSE);

  ledcSetup(motorChannelSW, PWMFrequency, PWMResolution);
  ledcAttachPin(motorPinSW, motorChannelSW);

  // AP
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  Serial.print("IP address: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/joystick", joystickInterrupt);
  server.on("/OFF", OFFInterrupt);

  server.begin();

  // Wire
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
    gyroscopeTask,          // Task function
    "gyroscopeTask",        // Name of the task
    2048,                   // Stack size in words
    NULL,                   // Parameter to pass to the task
    3,                      // Task priority
    &gyroscopeTaskHandle,   // Task handle
    1                       // Core (0 for communication, 1 for calculations and control)
  );

  // Create sleep and AP task
  xTaskCreatePinnedToCore(
    sleepAndAPControl,
    "sleepAndAPTaskHandle",
    8192,
    NULL,
    2,
    &sleepAndAPTaskHandle,
    0
  );

  // Enable watchdog for both tasks
  esp_task_wdt_add(gyroscopeTaskHandle);
  esp_task_wdt_add(sleepAndAPTaskHandle);
}

void loop() {}