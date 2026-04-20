# Group 4 embedded systems: Drone

This project is part of [TINLAB Embedded Systems](./assets/Cursushandleiding_TINLES03_2025_2026.pdf)

Project members:
- Ember Durkin
- Jordy van den Bos
- Mia Stikvoort
- Peter Hoogenboezem

<img width="2040" height="1148" alt="image" src="https://github.com/user-attachments/assets/d5177fea-0896-432e-bdb0-7f9de5e27bda" />

## Drone controller folder
This folder includes the PlatformIO project for the controller code of the drone.
To upload the code to the drone, the VSCode PlatformIO extension will need to be installed.

### Uploading the code
- Clone this repository.
- Open VSCode and go to the PlatformIO extension.
- Project tasks > Pick a folder > Go to the folder which the github is cloned in and choose the folder "drone controller".
- Either go to Project tasks > Click on upload, or click the arrow facing right on the bottom bar of the window.
  - Assuming the drone's microcontroller is plugged into your laptop, this will upload the code to the drone.

### Common issue while uploading
While uploading, the error:
"Error: Please specify `upload_port` for environment or use global `--upload-port` option."
Might occur.

If this happens, please ensure the microcontroller is plugged into your laptop and the connection is not being used by another program (e.g. Arduino IDE).
If this has not resolved the issue, please unplug the cable from the drone's microcontroller, hold the boot button indicated by "B" on the microcontroller and plug the cable back in.
This should resolve the issue.

### Turning on the drone
When the code is uploaded, the drone starts in deep sleep mode and is turned off. The drone can be pulled out of this mode by pressing the button on the drone's frame (not the ones one the microcontroller).
When this is pressed the drone will awake from deep sleep and after three seconds it will turn on. This is indicated by an orange light on the microcontroller.

### Controlling the drone
Now that the orange light is on, a new Wi-Fi option is available. This is the access point of the drone and it has the name "ESP32S3-AP". To connect to the point the password "embeddedgroup4" is required. When connected to this access point, you can go to the IP adres 192.168.4.1, where you will find the controlls of the drone.

With the joystick the drone can be steered in any direction, with the angle depending om where the joystick is aimed and how far it is aimed. When the joystick is being held in the middle of the circle or not held at all, the drone will assume a horizontal angle.
The "OFF" button can be used to turn off the drone. When this button is pressed, the drone will turn off immediately. For this reason, it is advised to firstly land the drone of get it to a safe position before turning it off.

## Program folder
For the simulator visit the [quad copter sim folder](./program/QuadCopterSim/README.md)

## Electrical-diagram folder
This folder includes the files for the electrical wiring of the drone.
