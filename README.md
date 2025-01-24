# CRSF to USB Joystick/Keyboard Adapter

## Overview
ESP32-S2 firmware that converts CrossFire (CRSF) radio protocol to USB HID joystick and/or Keyboard, enabling direct use of ELRS Transmitter  in flight simulators and other PC applications.

## Features
- Real-time RC channel conversion
- USB HID gamepad emulation
- Supports up to 16 channels and buttons
- Low-latency UART processing

## Configuration
- Connect to WiFi AP "CRSF Joystick Config" using password conf1234
- Access web Config via 10.0.0.1 address
- Assign Joystick Axes/Buttons or Keyboard keys to Channels
- Save Config (stays saved to device flash)
- Enjoy

![WebConfig Screenshot](https://github.com/siklosi/ESP32-S2_ELRS_USB/blob/main/web_config.png))
## Wiring

ELRS RX Module   ESP32-S2

--------------   ---------

VCC          →   5V/3.3V

GND          →   GND

TX           →   GPIO 16 (UART RX)

RX           →   GPIO 17 (UART TX)

![Adapter Image](https://github.com/siklosi/ESP32-S2_ELRS_USB/blob/main/ESP32_S2_ELRS.jpg)
![Adapter with case Image](https://github.com/siklosi/ESP32-S2_ELRS_USB/blob/main/case.png)
