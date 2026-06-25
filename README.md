# ⚙️ Dual-Sensor Operational Modal Analysis (Digital Twin)

A high-speed, real-time Structural Health Monitoring (SHM) dashboard and data pipeline. This project acts as a "Digital Twin" for a physical cantilever beam, using dual I2C accelerometers to capture, visualize, and mathematically prove the 1st and 2nd bending modes (resonance frequencies) of the structure.

## 🚀 Features

* **High-Speed Data Acquisition:** Streams 6 axes of acceleration data at 1,000 Hz (1ms loop) over a 500,000 baud serial connection.
* **Dual-Sensor Architecture:** Utilizes an MPU6050 at the free end (tip) and an MPU6500 at the beam's belly/node.
* **Phase Shift Visualization:** Real-time Time-Domain graphing isolates the Z-axis to physically show the sensors moving in/out of phase (proving Mode Shapes).
* **Dual FFT Processing:** Runs independent Fast Fourier Transforms on both sensor data streams to detect dominant resonance frequencies (e.g., isolating a ~357 Hz 2nd Bending Mode).
* **Modern UI:** Built with PyQt5 and PyQtGraph, featuring a custom high-contrast, low-latency dark mode dashboard.

## 🛠️ Hardware Setup

**Microcontroller:** Arduino Uno (or ESP32 for enhanced 1kHz stability)
**Sensors:** 1x MPU6050, 1x MPU6500 (or MPU9250)

### I2C Wiring Guide (Crucial Address Override)
To prevent the sensors from crashing on the shared I2C bus, their hardware addresses must be staggered:
1. **Tip Sensor (MPU6050):** * `SDA` -> Arduino A4
   * `SCL` -> Arduino A5
   * `AD0` -> Leave disconnected or wire to GND **(Address: 0x68)**
2. **Middle Sensor (MPU6500):**
   * `SDA` -> Arduino A4
   * `SCL` -> Arduino A5
   * `AD0` -> Wire to **3.3V** **(Address: 0x69)**

## 💻 Software Installation

### 1. The Firmware (C++)

#### For Arduino:
1. Open `vibration_streamer/dual_vibration_streamer.ino` in the Arduino IDE.
2. Flash the code to your microcontroller.
3. **CRITICAL:** Completely close the Arduino IDE after uploading to free the COM port.

#### For ESP32:
1. Open `vibration_streamer/esp_dual_vibration_streamer/esp32_6_dof_streamer.ino` in the Arduino IDE.
2. Ensure you have the ESP32 board manager installed.
3. Select your ESP32 board, flash the code, and close the Arduino IDE after uploading.

### 2. The Python Dashboard
Ensure you have Python 3.8+ installed. Navigate to the project directory and install the required dependencies:

```bash
pip install pyserial numpy pyqtgraph PyQt5
```

## 🎮 Usage Instructions

1. Clamp your aluminum beam rigidly.
2. Launch the Digital Twin dashboard:

   - **For Arduino firmware:**
     ```bash
     python digital_twin_app.py
     ```
   
   - **For ESP32 firmware:**
     ```bash
     python dual_sensor_digital_twin.py
     ```

3. Select your microcontroller's COM Port from the top-left dropdown and click **CONNECT DATALINK**.
4. **1st Mode Test:** Pluck the free tip of the beam. The Time-Domain graph will show both sensors moving in phase, and the FFT will spike at your fundamental frequency (e.g., ~60 Hz).
5. **2nd Mode Test (Node Forcing):** Lightly pin the 2nd mode node (approx. 78% from the free end) with your finger. Strike the beam at its belly. The Time-Domain graph will instantly invert (sensors moving out of phase), and the FFT will lock onto the high-frequency harmonic (e.g., ~357 Hz).

## 🗂️ Project Structure

* `/vibration_streamer/dual_vibration_streamer.ino`: The high-speed C++ I2C polling script for Arduino.
* `/vibration_streamer/esp_dual_vibration_streamer/esp32_6_dof_streamer.ino`: The high-speed C++ I2C polling script for ESP32.
* `/digital_twin_app.py`: The main PyQt5 dashboard and DSP engine for Arduino.
* `/dual_sensor_digital_twin.py`: The PyQt5 dashboard and DSP engine for ESP32.

## 📝 License

This project is open-source and available under the MIT License.