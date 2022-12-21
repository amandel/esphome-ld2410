# esphome-ld2410
Implementation of a HLK-LD2410 component for ESPHome

TODO: A Lot of documentation

Feedback very welcome. To the sensor, code, naming and so on.

# Setup

For now this is "manual":

1. Place ld2410.h into your ESPHome configuration folder
2. Use the sample.yaml for your device the code including `binary_sensor` and below is what you need.

# Wording

The sensor spec uses a lot of alternative wording for the same thing which makes it seem overly complicated. I've tried to clean this up a bit in the code and here in the readme:

- *static* also known as stillness, resting, stationary, rest
- *motion* also known as movement, exercise
- *unmanned* also known as "no time" == unoccupied delay time. This is the delay how long the sensor needs to detect _no presence_ until it gets reported.
- *sensitivity* setting used is more a sensitivity threshold. A value of 100% means no sensitivity and 0% is full sensitivity. (might need to clean this up in the code and sensor names?). 


# Observed version numbers

## V1.07.22082218
My HLK-LD2410 delivered 10-2022 report version V1.07.22082218. It does not include bluetooth functionality.


# Reference Documentation

## Serial Protokoll

I got hold of `HLK-LD2410B Serial communication protocol V1.04.pdf` distributed in a Google Drive archive. The version is last updated 2022-08-22 and covers the bluetooth version. 

## Sensor Behavior

See https://www.youtube.com/watch?v=dAzHXpP3FcI for a detailed review from Andreas Spiess, actually the video triggered me to look into the sensor.

The thread [mmWave Wars: one sensor (module) to rule them all](https://community.home-assistant.io/t/mmwave-wars-one-sensor-module-to-rule-them-all) contains a lot of input and some code that I used as start for this implementation.
