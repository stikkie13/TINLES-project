/*
  https://www.youtube.com/watch?v=3T1tnY2_NpI
  https://www.instructables.com/Making-a-Joystick-With-HTML-pure-JavaScript/

  192.168.4.1
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

const char *ssid = "ESP32S3-AP";
const char *password = "embeddedgroup4";

WebServer server(80);

const char* htmlPage = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
body { text-align: center; font-family: Arial; }

#joy {
  width: 220px;
  height: 220px;
  background: lightgray;
  border-radius: 50%;
  position: relative;
  margin: auto;
  touch-action: none;
}

#stick {
  width: 70px;
  height: 70px;
  background: gray;
  border-radius: 50%;
  position: absolute;
  left: 75px;
  top: 75px;
}
</style>
</head>

<body>

<div id="joy">
  <div id="stick"></div>
</div>

<script>
let joy = document.getElementById("joy");
let stick = document.getElementById("stick");

let dragging = false;
let centerX = 110;
let centerY = 110;
let radius = 80;

joy.addEventListener("mousedown", () => dragging = true);
document.addEventListener("mouseup", () => dragging = false);

joy.addEventListener("touchstart", () => dragging = true);
document.addEventListener("touchend", () => dragging = false);

let lastUpdate = 0;

function moveStick(clientX, clientY){
  let rect = joy.getBoundingClientRect();
  let x = clientX - rect.left;
  let y = clientY - rect.top;

  let dx = x - centerX;
  let dy = y - centerY;

  let dist = Math.sqrt(dx*dx + dy*dy);
  if(dist > radius){
    dx = dx * radius / dist;
    dy = dy * radius / dist;
    dist = radius;
  }

  let stickX = centerX + dx;
  let stickY = centerY + dy;

  stick.style.left = (stickX - 35) + "px";
  stick.style.top = (stickY - 35) + "px";

  // 0 graden boven
  let angle = Math.atan2(dx, -dy) * 180 / Math.PI;
  if(angle < 0) angle += 360;

  let power = dist / radius;

  if(Date.now() - lastUpdate > 50) {
    fetch("/joystick?angle=" + Math.round(angle) + "&power=" + power.toFixed(2));
    lastUpdate = Date.now();
  }
}

joy.addEventListener("mousemove", function(e){
  if(!dragging) return;
  moveStick(e.clientX, e.clientY);
});

joy.addEventListener("touchmove", function(e){
  if(!dragging) return;
  moveStick(e.touches[0].clientX, e.touches[0].clientY);
});
</script>

</body>
</html>
)rawliteral";

void handleRoot() {
    server.send(200, "text/html", htmlPage);
}

void handleJoystick() {
    int angle = server.arg("angle").toInt();
    int power = server.arg("power").toInt();

    float percentage, rollAngle, pitchAngle;
    int totalAngle = 15;
    
    float rad = angle * PI / 180.0;
    rollAngle  = totalAngle * sin(rad);
    pitchAngle = -totalAngle * cos(rad);

    Serial.print("Pitch: ");
    Serial.print(pitchAngle);
    Serial.print("  Roll: ");
    Serial.println(rollAngle);

    server.send(200, "text/plain", "OK");
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(115200);

    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password);

    Serial.print("IP address: ");
    Serial.println(WiFi.softAPIP());

    server.on("/", handleRoot);
    server.on("/joystick", handleJoystick);

    server.begin();
}

void loop() {
    server.handleClient();
}
