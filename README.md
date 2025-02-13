# ESP32-Based Smart Helmet System v0.0.1

## Overview
This repository contains the development of an **ESP32-based Smart Helmet System** with the following key features:

- **Live GPS Tracking** using the **NEO-6M GPS module**
- **Accident Detection** using the **MPU6050 Gyroscope & Accelerometer**
- **S.O.S Alert System** via **SMS** using the **SIM800L module**

> **⚠️ Project Status: Under Development**
>
> 🚧 This project is currently in development. Please do not use this code in production yet.

## Features
- 📍 **Real-time GPS tracking** with periodic location updates in Google FireBase Cloud.
- ⚠️ **Accident detection** using impact force analysis (5-6G threshold)
- 📲 **Emergency SMS alerts** with last known location upon crash detection
- 🔔 **Buzzer Alert System** for immediate on-site warning
- 💾 **EEPROM storage** for last 10 GPS coordinates with timestamps
- 💤 **Deep sleep mode** to optimize power consumption
- ⏳ **30-second cooldown** before sending S.O.S message (abort option available)
- 🔵 **Status LED** indicating system state

## Components Used
- **ESP32** (Microcontroller)
- **NEO-6M GPS Module** (Location tracking)
- **MPU6050 Gyroscope & Accelerometer** (Accident detection)
- **SIM800L GSM Module** (SMS communication)
- **5V Buzzer** (Alert system)
- **Push Button** (Abort S.O.S feature)
- **LED Indicator** (System status display)

## Installation & Usage
1. Clone this repository:
   ```sh
   git clone https://github.com/your-username/smart-helmet.git
   cd smart-helmet
   ```
2. Install necessary libraries in Arduino IDE or PlatformIO:
   - `TinyGPS++`
   - `Adafruit MPU6050`
   - `Adafruit Unified Sensor`
   - `SoftwareSerial`
3. Configure your emergency contact numbers in the code.
4. Upload the code to ESP32 and test the functionalities.

## Future Enhancements
- 🔋 Battery optimization with solar charging support
- 📡 Integration with a cloud dashboard for remote monitoring
- 📶 Wi-Fi-based alternative communication option

## Contributing
Contributions are welcome! Feel free to fork this repository and submit pull requests.

## License
This project is open-source and available under the [MIT License](LICENSE).

