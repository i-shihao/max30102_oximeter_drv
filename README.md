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

