# 9-axis-IMU
This repository is an implementation of an inertial measurement unit created for private purposes. I did this to learn about microcontrollers and sensors. I wrote this code before I went to university. Looking back, this code looks like a mess and I would do many things differently.

>[!WARNING]
>The extended Kalman filter contains several errors in the prediction step and does **not** predict the correct state of the Euler angles.
>Due to the update step, the Euler angles will still be close to the actual Euler angles, but worse than the accelerometer data.
>
>If you are looking for an implementation of an extended Kalman filter for these sensors, you can get my functioning template class from [this](https://github.com/feesm/SEFL) repository.

## Overview
 1. Hardware
 2. Extern librarys
 3. Current state of project
 4. Known issues
 5. Future projects

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
 - Calculation of pitch, roll and yaw angle based on acceleration, magnetometer and gyroscope data. Sensor fusion of the angles are calculated with a extended Kalman filter. Gyroscope is used for predict current state and the accelerometer and the magnetometer are used to update the estimation.
 - send inertial measurement unit data as String or as float values via USB to display it on a computer.

## Known issues
- a pitch angle of 90 Degres will result in NaN for all three angles

## Future projects
 - This project is finished, everything is implemented. Maybe I'll cleanup some parts of the code in the future.
