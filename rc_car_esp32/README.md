# 🏎️ Global IoT RC Car System

A full-stack Internet of Things (IoT) project that transforms a standard short-range RC toy car into a globally accessible vehicle.

Developed by a Computer Science and Engineering undergraduate at the University of Moratuwa, this project utilizes a dual-core ESP32 architecture, MQTT cloud communication, and low-latency video streaming to allow the car to be driven from anywhere in the world via a web dashboard or a custom mobile app.

## 🔗 Project Ecosystem

This repository contains the embedded C++ (Arduino) code for the ESP32. The client-side interfaces are hosted in their respective repositories:

- **[Web Command Center (HTML/JS/CSS) ➔](https://github.com/madhushan99/ESP32_CAR_WEB.git)**
- **[Mobile Application (Flutter/Dart) ➔](https://github.com/madhushan99/ESP32_CAR_APP.git)**

## ✨ Key Features

- **Global Range:** Connected via HiveMQ (MQTT) over a 4G mobile hotspot, allowing control from any internet-connected device.
- **Dual-Core Processing (FreeRTOS):**
  - **Core 0:** Real-time obstacle avoidance (Ultrasonic sensor + Servo sweep) and a heartbeat watchdog for emergency stopping.
  - **Core 1:** Fast motor control execution and processing incoming MQTT payloads.
- **4-Gear Digital Transmission:** Maps user inputs to PWM signals for smooth, variable speed control.
- **FPV Video Feed (Hardware Hack):** Utilizes a repurposed smartphone mounted to the chassis. The phone provides the hotspot and streams ultra-low-latency video via VDO.Ninja (configured using `scrcpy` via ADB due to a broken screen).
- **Software Steering Patch:** Motor control logic re-mapped via software to compensate for a failed physical steering mechanism.

## 🛠️ Hardware Stack

- **Microcontroller:** ESP32 Development Board
- **Sensors/Actuators:** Ultrasonic Distance Sensor (HC-SR04), 9g Micro Servo
- **Motors:** DC Motors driven by custom PWM logic
- **Network/Camera:** Repurposed smartphone (Hotspot + FPV Camera)

## 🚀 Setup & Installation

1. Clone this repository:
   ```bash
   git clone https://github.com/madhushan99/ESP-32-Car-.git
   ```

````
2. Open the `rc_car_esp32` folder in VS Code.

3. Install the required libraries via the Arduino Library Manager:
   * `PubSubClient` (for MQTT)
   * `ESP32Servo`

4. In the `rc_car_esp32.ino` file, update the WiFi credentials to your actual hotspot (like your Mobitel network) and add your MQTT passwords back in:
   ```cpp
   const char* ssid = "YOUR_MOBITEL_SSID";
   const char* password = "YOUR_WIFI_PASSWORD";
   const char* mqtt_username = "madushan7";
   const char* mqtt_password = "YOUR_MQTT_PASSWORD";
````

5. Compile and upload to your ESP32 board.
