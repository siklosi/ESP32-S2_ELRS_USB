# CRSF to USB Joystick Converter

## Overview
ESP32-S2 firmware that converts CrossFire (CRSF) radio protocol to USB HID joystick, enabling direct use of ELRS Transmitter  in flight simulators and other PC applications.

## Features
- Real-time RC channel conversion
- USB HID gamepad emulation
- Supports up to 8 channels and buttons
- Low-latency UART processing
- Compatible with ExpressLRS and CrossFire protocols



## Wiring

ELRS RX Module   ESP32-S2

--------------   ---------

VCC          →   5V/3.3V

GND          →   GND

RX           →   GPIO 16 (UART RX)

TX           →   GPIO 17 (UART TX)
