# Pulse Oximeter Project

## Overview
This project aims to develop a wireless pulse oximeter using Arduino and various electronic components. The pulse oximeter measures oxygen saturation levels and pulse rate, transmitting the data wirelessly to a PC for display and analysis.

## Objectives
- Measure oxygen saturation (SpO2) and pulse rate.
- Utilize Arduino and MAX30101 sensor for data acquisition.
- Implement wireless communication using XBee modules.
- Display and analyze sensor data on a PC.

## Components
- Arduino Uno R4
- MAX30101 Sensor
- XBee Modules
- LED and Button Shield
- Breadboard and connecting wires

## Setup and Installation
1. **Hardware Setup:**
   - Connect the MAX30101 sensor to the Arduino using the I2C interface.
   - Attach the XBee modules to the Arduino for wireless communication.
   - Ensure the LED and button shield is properly connected.

2. **Software Setup:**
   - Install the Arduino IDE.
   - Install necessary libraries: `Wire`, `SoftwareSerial`.
   - Upload the provided Arduino sketches to the respective boards.

## Project Structure
- `src/`: Contains all the Arduino code for different stages of the project.
- `pdf/`: Includes datasheets for the MAX30101 sensor and other components.

## Usage
1. **Data Acquisition:**
   - Place your finger on the MAX30101 sensor.
   - The sensor will measure the oxygen saturation and pulse rate.

2. **Data Transmission:**
   - The Arduino will transmit the sensor data wirelessly using XBee modules.

3. **Data Display:**
   - Use the MATLAB scripts to visualize and analyze the received data.


## Acknowledgments
- This project was done on an educational purpose, at TPS (Télécom Physique Strasbourg, my engineering school).
