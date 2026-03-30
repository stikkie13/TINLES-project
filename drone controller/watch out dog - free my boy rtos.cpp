#include <Arduino.h>
#include <Wire.h>
#include "esp_task_wdt.h"

TaskHandle_t gyroscopeTaskHandle;
const TickType_t period = pdMS_TO_TICKS(50);

const int WD_TIMEOUT = 5;

void setup() {
  Serial.begin(115200);

  esp_task_wdt_init(WD_TIMEOUT, false); // !! false for debugging

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
    Serial.printf("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");

    // Feed the watchdog
    esp_task_wdt_reset();

    vTaskDelayUntil(&lastWakeTime, period);
  }
}

void loop() {}
