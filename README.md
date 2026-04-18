# Groep 4 embedded systems: Drone

Dit is het project van de [TINLAB Embedded Systems](./assets/Cursushandleiding_TINLES03_2025_2026.pdf)

Project leden:
- Ember Durkin
- Jordy van den Bos
- Mia Stikvoort
- Peter Hoogenboezem

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

Firstly, please ensure the microcontroller is plugged into your laptop and the connection is not being used by another program (e.g. Arduino IDE).
If this has not resolved the issue, please unplug the cable from the drone's microcontroller, hold the boot button indicated by "B" on the microcontroller and plug the cable back in.
This should resolve the issue.

## Program folder
For the simulator visit the [quad copter sim folder](./program/QuadCopterSim/README.md)

## Electrical-diagram folder
