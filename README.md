# Max30102 spo2 pulse Oximetry sensor driver

## Table on Contents
 - [Overview](#overview)
 - [Features](#features)
 - [Hardware Connections](#hardware-connections)
 - [Device Tree Example](#device-tree-example)
 - [Build and Install](#build-and-install)
 - [Usage](#usage)
 - [Examples](#examples)
 - [Screenshot](#screenshot)
 - [License](#license)

## Overview

 A max30102 spo2 heatrate oximeter sensor driver based on Linux i2c subsystem. Developed and tested on ARM-based
 platfroms such as Raspberry Pi 4 Model B. Exposes sensor data through Linux iio intefaces. Designed and developed for
 learning linux kernel device driver development.

## Features

 - Device tree support
 - Linux kernel I2C Subsystem based communication
 - IIO inteface for data channels
 - Interrupt driven data sample handeling
 - Runtime power management

## Hardware Connection

### Pin Connections

 | Sensor Pin | Connects to | Description |
 |------------|-------------|-------------|
 | VIN        | 3.3v        | Power supply|
 | GND        | GND         | Ground      |
 | SCL        | I2C-SCL     | Clock Line  |
 | SDA        | I2C_SDA     | Data line   |
 | INT        | GPIO-x      | Interupt    |

Note: Sensor pins IR and RED must be left untouched

## Device Tree Example

 Device tree node for max30102 driver
```
spi@0 {
    max30102@57 {
         compatible ="adi,max30102";
         status =  "okay";
         reg = <0x57>;
         interrupt-gpios = <&gpio 17 0x01>;
        vdd-supply = <&reg_3v3>;
        vled-supply = <&reg_3v3>;
     };
};
```

## Build and Install
### Build
    make
### Insert driver
    insmod max30102.ko
### Verify
    dmesg | grep max30102
### Remove
    rmmod max30102

## License

 This driver is license under the GNU General Public License version v2.0 GPL-2.0. For more information
 refer to License file.

