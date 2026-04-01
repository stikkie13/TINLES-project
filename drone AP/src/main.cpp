/*
  https://www.youtube.com/watch?v=3T1tnY2_NpI
  https://www.instructables.com/Making-a-Joystick-With-HTML-pure-JavaScript/
  https://www.cssscript.com/touch-virtual-joystick/
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

#define button 32
#define interrupt_button_gpio GPIO_NUM_32

RTC_DATA_ATTR int bootCount = 0;

const char *ssid = "ESP32S3-AP";
const char *password = "embeddedgroup4";

WebServer server(80);

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

void handleJoystick() {
  int roll = server.arg("angleX").toInt();
  int pitch = server.arg("angleY").toInt();

  Serial.print(roll);
  Serial.print(", ");
  Serial.println(pitch);

  handleRoot();
}

void handleOFF() {
  Serial.println("OFF");
  handleRoot();
  Serial.println("drone asleep (handleOFF())");
  esp_deep_sleep_start();
}

void setup() {
  pinMode(button, INPUT_PULLUP);
  
  Serial.begin(115200);

  bootCount++;
  Serial.println("Boot number: " + String(bootCount));

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

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  Serial.print("IP address: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/joystick", handleJoystick);
  server.on("/OFF", handleOFF);

  server.begin();
}

void loop() {
  server.handleClient();

  if(digitalRead(button) == LOW) {
    while(digitalRead(button) == LOW) {
        delay(100);
    }
    Serial.println("drone asleep (loop())");
    esp_deep_sleep_start();
  }
}
