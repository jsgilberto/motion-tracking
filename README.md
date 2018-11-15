# Motion Tracking

This project aims to use MPU9250 sensor with Madgwick's algorithm to achieve motion tracking.

![sensor](sensor.gif)

## Hardware

The sensor and the arduino used is not mandatory, but the code is only tested in the follwing devices:

- MPU9250 - Accelerometer, Gyroscope and Magnetometer
- Arduino Pro Micro

## Dependencies

In order to run this project, you need to have the following installed:

- Arduino IDE
- Python 3
- Python dependencies: Numpy, Pygame, OpenGL

## Usage

To use this module, you need to follow the next steps:
- Load `mpu9250_serial_data.ino` into your arduino (Test it with your serial terminal in your Arduino IDE)
- Once you have the program loaded and running in your Arduino, run `main.py` file with `python main.py` and that should do it!

## About the Algorithm

Madgwick's algorithm is an efficient orientation filter for inertial and inertial/magnetic sensor arrays. You can check his paper [here](http://x-io.co.uk/res/doc/madgwick_internal_report.pdf)

