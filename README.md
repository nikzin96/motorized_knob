# Motorized Knob Project
## Instructions
Instructions to build the project including the mechanical assembly can be found on instructables:

https://www.instructables.com/Motorized-Knob/

## Create Firmware
Clone the repo and run the following command in the firmware folder:
```
pio run
```
This will create the PlatformIO project and install the needed libraries.

## Parameters
There are several parameters in the firmware you can adjust to your liking:
1. **MAX_BRIGHTNESS**: Sets brightness of LED ring
2. **MAX_ANGLE**: Sets the angle where the volume should be 100%
3. **MIN_ANGLE**: Sets the angle where the volume should be 0%
4. **NUM_DETENTS_IN_LIMITS**: Sets the amount of detents in the specified area (between min and max angle)
5. **ANGLE_EPSILON**: Sets the accuracy of the motor when moved to an angle

## Using volume control on Windows
I have created a python script that automatically connects to the motorized knob (default name is **VolumeKnob**) using BLE. Then it interfaces with the Computer's volume (always the currently active output device).

To have the script always running in the background you can create an executable from it using for example **Nuitka**. Then you can add it to your windows task scheduler, which will always run it in the background. Similar methods exist for Linux or Mac, however the cript might need changes for other operating systems to work properly.

## Behaviour
When the knob is powered up it first moves to *MIN_ANGLE* and starts breathing blue to indicate that it is waiting for a BLE connection.

When the script is running, it looks every 5 seconds for a BLE device with the name "VolumeKnob". If it finds one it connects to it which causes the light to change to a solid color and the knob to move to the current volume's position.

Now the knob is controling the devices volume and the volume of the device is controlling the knob's position. 

When the knob is turned it changes the devices volume. The LED ring changes it´s color from green (0% volume) to red (100% volume). If the device changes it´s volume the knob will move to the corresponding position. 

When you press down on the knob it will toggle mute on the device. Mute is indicated by the LED ring which shows a gradient from blue to purple, depending on the volume the knob is set to. <br> 
When you turn the knob while it is muted it will not immediately change the devices volume, but after it is unmuted the new volume will be applied.
