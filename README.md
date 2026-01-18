# Motorized Knob Project
## Instructions
Instructions to build the project including the mechanical assembly can be found on instructables:

https://www.instructables.com/Motorized-Knob/

## Create Firmware
Clone the repo and the following command in the firmware folder:
```
pio run
```
This will create the PlatformIO project and install the needed libraries.

## Using volume control on Windows
I have created a python script that automatically connects to the motorized knob (default name is **VolumeKnob**) using BLE. Then it interfaces with the Computer's volume (always the currently active output device).

To have the script always running in the background you can create an executable from it using for example **nuitka**. Then you can add it to your windows task scheduler, which will always run it in the background.
