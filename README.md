# IoT Predictive Maintenance System

## Overview
This project is an IoT-based system that monitors machine health using vibration, temperature, and sound sensors. It detects abnormal behavior and alerts the user in real time.

## Problem Statement
Unexpected machine failures cause downtime and financial loss. Traditional maintenance methods are inefficient and reactive.

## Solution
We built a system using ESP32 that continuously monitors:
- Vibration (MPU6050 + FFT)
- Temperature (DHT22)
- Sound (analog sensor → dB)

It detects faults using threshold and stability logic and alerts using LEDs and buzzer.

## Hardware Used
- ESP32
- MPU6050
- DHT22
- Sound Sensor
- LEDs & Buzzer
- Motor (for simulation)

## Working
Sensor data is collected and processed by ESP32:
- Vibration → FFT → frequency detection
- Temperature → compared with baseline
- Sound → converted to dB

If abnormal values persist, the system triggers alerts.

## Output
- LED indicators for faults
- Buzzer alerts
- Serial + Web monitoring

## Applications
- Industrial machines
- Motors and rotating equipment
- Predictive maintenance systems
