# ESP32 Sound Monitoring and Alert System

This project implements a real-time sound monitoring and alert system using an ESP32 microcontroller. It continuously monitors analog microphone input, detects loud sound events based on a configurable threshold, records a short audio snippet, saves it to SPIFFS, and publishes an alert message with metadata to an MQTT broker.

## Features

* [cite_start]**Continuous Analog Input Monitoring:** Utilizes the ESP32's internal ADC to constantly sample microphone input. [cite: 245, 259]
* [cite_start]**Loud Sound Event Detection:** Configurable threshold (`LOUD_SOUND_THRESHOLD_DEVIATION`) to identify significant sound events. [cite: 246]
* [cite_start]**On-Demand Audio Recording:** Records a 3-second audio snippet (300 samples at 100 Hz) upon sound detection. [cite: 246, 374]
* [cite_start]**Local Storage (SPIFFS):** Saves recorded audio as raw binary (`.raw`) files to the ESP32's SPIFFS flash memory. [cite: 246, 195]
* [cite_start]**Wi-Fi Connectivity:** Connects to a specified Wi-Fi network to enable MQTT communication. [cite: 247, 370]
* [cite_start]**MQTT Alerting:** Publishes JSON-formatted messages to a public MQTT broker (`broker.hivemq.com`) with details about the detected sound event, including timestamp, amplitude, and filename. [cite: 248, 170, 377]
* [cite_start]**Detailed Serial Logging:** Provides real-time feedback on threshold crossings, recording progress, file operations, and MQTT communication. [cite: 372, 373, 196, 180]

## Hardware Requirements

* **ESP32 Development Board:** Any standard ESP32 development board (e.g., ESP32-DevKitC, ESP32 WROOM-32).
* **Analog Microphone Module:** An electret condenser microphone breakout board with analog output (e.g., KY-038, MAX4466 module).

## Software and Toolchain

* **ESP-IDF (Espressif IoT Development Framework):** Version 5.3.1 (recommended) or compatible.
* **PlatformIO IDE (Recommended) or ESP-IDF Command Line:** For building, flashing, and monitoring.
* **MQTT Client:** An MQTT client application (e.g., MQTT Explorer, Mosquitto command-line client) to subscribe to the alert topic.

## Setup Instructions

### 1. ESP-IDF Installation

Follow the official [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html) for setting up your development environment. This typically involves:
* Installing prerequisites (Python, Git).
* Cloning the ESP-IDF repository.
* Running the install and export scripts.

### 2. Hardware Wiring

Connect your analog microphone module to the ESP32 as follows:

* **Microphone VCC / +3V3:** Connect to ESP32's 3.3V pin.
* **Microphone GND:** Connect to ESP32's GND pin.
* **Microphone Analog OUT / SIG:** Connect to **GPIO34** on your ESP32 board. [cite_start]This corresponds to `ADC_CHANNEL_6` in the firmware. [cite: 246]

### 3. Project Setup

1.  **Clone this repository:**
    ```bash
    git clone [YOUR_REPOSITORY_URL_HERE]
    cd [YOUR_REPOSITORY_NAME]
    ```
2.  **Configure ESP32 Target:**
    ```bash
    idf.py set-target esp32
    ```
3.  **Open Menuconfig:**
    ```bash
    idf.py menuconfig
    ```
    * Navigate to `Component config` -> `SPIFFS` and ensure "SPIFFS Support" is enabled.
    * Navigate to `Component config` -> `Wi-Fi` and ensure "Wi-Fi NVS Flash" is enabled.
    * Navigate to `Component config` -> `mbedTLS` -> `TLS Method` and ensure `TLS disabled` is selected (for simplicity with public broker).
    * Save and exit.

4.  **Configure Wi-Fi and MQTT Credentials:**
    * Open the `mediaSDK.txt` (or your main `.c` file).
    * [cite_start]Modify the `WIFI_SSID` and `WIFI_PASSWORD` defines to match your local Wi-Fi network. [cite: 247]
        ```c
        #define WIFI_SSID      "YOUR_WIFI_SSID" // Replace with your Wi-Fi SSID
        #define WIFI_PASSWORD  "YOUR_WIFI_PASSWORD" // Replace with your Wi-Fi password if any
        ```
    * [cite_start]You can also customize the `MQTT_BROKER_URI` and `DEVICE_ID` if needed. [cite: 248]

5.  **Build the Project:**
    ```bash
    idf.py build
    ```
6.  **Flash to ESP32 and Monitor Serial Output:**
    * Replace `/dev/ttyUSB0` with your ESP32's serial port.
    ```bash
    idf.py -p /dev/ttyUSB0 flash monitor
    ```
    (On Windows, this might be `COMx` like `COM3`).

### 4. SPIFFS Partitioning (First-time Flash)

For SPIFFS to work, your ESP32's flash needs to be partitioned correctly. The default `partitions_singleapp.csv` (or similar) in ESP-IDF usually includes a SPIFFS partition. If you encounter SPIFFS mounting errors, you might need to:
* Ensure your `sdkconfig` enables SPIFFS.
* Flash a partition table that includes a SPIFFS partition (e.g., `idf.py erase_flash` followed by `idf.py flash` if you modify `partitions.csv`).

## Usage

1.  **Monitor Serial Output:** Once flashed, observe the serial output using `idf.py monitor`. [cite_start]You should see initialization messages for NVS, SPIFFS, Wi-Fi, and MQTT. [cite: 368, 369, 370, 371]
2.  **Trigger a Sound Event:** Make a loud noise (e.g., clap, shout, whistle) near the microphone connected to your ESP32.
3.  **Observe Recording:**
    * [cite_start]The serial log will show "LOUD SOUND DETECTED!" [cite: 372]
    * [cite_start]Followed by "Recording sample X/300" messages as audio is captured. [cite: 374]
    * [cite_start]Once recording is complete (300 samples), it will show "File saved successfully!" and list the files on SPIFFS. [cite: 200, 169]
4.  **Check MQTT Alerts:** Open your MQTT client (e.g., MQTT Explorer) and subscribe to the topic `esp32/audio_alerts/esp32-audio-01` (or your chosen `DEVICE_ID`). You should receive a JSON message similar to this:
    ```json
    {
      "event": "sound_detected",
      "device": "esp32-audio-01",
      "timestamp": "1970-01-01T00:00:14Z", // Note: Timestamp will be epoch time without SNTP
      "amplitude": 0.91,
      "audio_filename": "recording_1.raw"
    }
    ```
    [cite_start]You will see the message details, including `msg_id`, printed in the serial log as well. [cite: 180]

## Known Issues / Future Enhancements

* **Inaccurate Timestamp:** The current timestamp in MQTT messages uses `gettimeofday()` but is not synchronized with a Network Time Protocol (NTP) server. This results in an epoch time (e.g., `1970-01-01T00:00:14Z`). For accurate timestamps, integrate SNTP client functionality.
* **Direct ADC Use:** The project directly uses the ESP32's internal ADC for microphone input. While effective, if a strict "ADC (I2C)" interface was intended, it would require an external I2C ADC chip and a corresponding driver. The current setup simplifies hardware and achieves the core functionality.
* **Basic Amplitude Metric:** The `amplitude` in the MQTT payload is a simple normalized maximum deviation. For more advanced sound analysis, consider implementing FFT (Fast Fourier Transform) or decibel (dB) calculations.
* **Security:** For production environments, Wi-Fi should use WPA2-Enterprise (if applicable) and MQTT communication should be secured with TLS/SSL.
* **Power Management:** Implement ESP-IDF's power management features for low-power operation if battery-powered.
* **Web Interface/OTA:** Add a simple web interface for configuration or Over-The-Air (OTA) updates.

---
