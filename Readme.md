# Whadda WPI430/VMA430 - GPS MODULE U-BLOX NEO-7M 

This repository contains the library and example code to work with Whadda WPI430/VMA430 GPS MODULE with the U-BLOX NEO-7M chipset. 
It can configure the module to send out UBX data packets, and is able to decode the time and location data.

This python version is heavily based on official repository at [https://github.com/Velleman/VMA430_GPS_Module](https://github.com/Velleman/VMA430_GPS_Module)


## Installation

This section needs to be written

Requirements :
- serial


## Wiring
Wire up the GPS module to an Arduino compatible board as shown below:

|GPS Module|GPIO|
|----------|-------------|
|VCC|5V|
|GND|GND|
|TXD|GPIO 14 (TXD)|
|RXD|GPIO 15 (RXD)|


## Todo
- Complete Installation/requirements section
- add a schema/picture showing wiring
- Add support for module configuration
- Add/verify support for GLONASS, QZSS and SBAS