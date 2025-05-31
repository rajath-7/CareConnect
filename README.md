# CareConnect

This project is a wearable IoT-based fall detection and health monitoring system using:
- ESP32 microcontroller
- MPU6050 for motion sensing
- MAX30102 for heart rate and SpO2 monitoring
- Blynk for real-time alerts

## Features
- Fall detection using accelerometer and gyroscope data
- Heart rate monitoring
- Real-time alerts via Blynk app
- Manual alert cancellation via button

## Hardware
- ESP32 Dev Board
- MPU6050 Accelerometer/Gyroscope
- MAX30102 Pulse Sensor
- Tactile button
- Blynk IoT platform

## Setup
1. Flash the code in `code/main.ino` to your ESP32.
2. Connect sensors as per the circuit diagram.
3. Update your Wi-Fi and Blynk credentials in the code.
4. Open the Blynk app and connect your device.

## License
This project is open-source under the MIT License.
