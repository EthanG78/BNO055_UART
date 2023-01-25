# BNO055_UART
A C driver to communicate with the [BNO055 absolute orientation IMU](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/overview) over a serial UART connection on a Raspberry Pi.

## Why?
The BNO055 presents an easy to use, low cost solution to obtain absolute orientation data on an embedded system. By default this device communicates over I2C, thus it interfaces exceptionally well with the Arduino family of devices. However, there exists a [long standing and well documented bug](https://www.advamation.com/knowhow/raspberrypi/rpi-i2c-bug.html) with the Raspberry Pi ARM processor’s ability to clock stretch that causes the device and the target it is trying to communicate with over I2C to get out of sync. Therefore, the BNO055 is unable to reliably communicate with a Raspberry Pi device. This sucks because the Raspberry Pi family of microcontrollers are an excellent choice for embedded system design! This is where the purpose of this driver stems from. By placing the PS1 pin of the BNO055 to the same potential as the Vin pin (see board pictured below), the SDA and SCL pins effectively become TX and RX pins, respectively. This allows for the BNO055 to communicate over a device's serial port, such as the Raspberry Pi! A software library to handle serial communication over UART with the BNO055 [already exists from Adafruit](https://github.com/adafruit/Adafruit_Python_BNO055), however this is written for Python. If you simply need to get the BNO055 working on your Raspberry Pi, then I highly suggest you check out their library and the examples within it. However, if you require the software running on your Raspberry Pi for data collection be written in C/C++, then this library is for you.

![BNO055](https://cdn-learn.adafruit.com/assets/assets/000/024/585/original/sensors_2472_top_ORIG.jpg)

## Dependencies
This driver uses the [unofficial fork of WiringPi](https://github.com/WiringPi/WiringPi) to handle communication over the Pi's serial port.

## Installation
Once the above version of WiringPi has been installed on your Raspberry Pi, clone this repo and run:
```
> make
> sudo make install
```
You can test your installation by running the examples found in /examples.
## Notice
As of January 2023, this is still a WORK IN PROGRESS.

Author: Ethan Garnier