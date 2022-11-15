# 9-axis-IMU
This repository is an implementation of an inertial measurement unit created for private purposes.

## Overview
 1. Hardware
 2. Extern librarys
 3. Current state of project
 4. Future projects

## Hardware
The Discovery kit STM32F3DISCOVERY was used for the implementation. Following parts are used for the implementation:
 1. The microcontroller STM32F303VC
 2. The 3 axis digital gyroscope I3G4250D
 3. The eCompass module LSM303AGR with a 3 axis digital accelerometer and a 3 axis digital magnetometer

The sensor I3G4250D is connected with the microcontroller via SPI and LSM303AGR is connected via I2C.

## Extern librarys
The software was developt with the STM32CubeIDE. Some parts of the main.c and the main.h file are auto generated. The HAL-Library was used for GPIO-Pins, SPI-Communication, I2C-Communication, USB-Communication, Timers, Interrupts, etc. It's files are not included in this repository. Furthermore the library USB_DEVICE from ST was used to create a virtual USB COM port. The USB_DEVICE files are also not included in this repository.

## Current state of project
 - Library for I3G250D was created based on the HAL-library. The sensor data registers are read with DMA via SPI. After the communication finished a interrupt get called. In the interrupt the angular rate get calculated and low-pass-filtered. The functions can be used with a external interrupt from the data-ready-pin of the sensor.
 - Library for LSM303AGR was also created with the HAL-library. The sensor data registers are read with DMA via I2C. After finished communication, sensor value get calculated in interrupt and low-pass filtered. The functions can be used with a external interrupt from the data-ready-pins of the sensor.
 - Calculation of pitch and roll angle based on acceleration and gyroscope data. Sensor fusion of the angles calculated with both sensors with complementary filter. 
 - send inertial measurement unit data as String or as float values via USB to display it on a computer.

## Future projects
 - Fusion of the calculated angles with a extended Kalman filter.
 - In addition, use magnetic field data to obtain long-term stable yaw data and higher accuracy of the other angles.
