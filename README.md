# ESP32 Rotator Controller

An ESP32-based dual-axis (Azimuth/Elevation) antenna rotator controller.  
Designed for satellite tracking, radio astronomy, and other applications requiring precise pointing.

## ✨ Features
- **Dual-axis control** (Azimuth & Elevation)
- **Stepper motor drivers** (DM556T or compatible)
- **FastAccelStepper** library for smooth motion
- **Magnetometer/accelerometer (LSM303)** feedback for position sensing
- **Optical limit switches** for safe homing
- **Web UI** for monitoring and manual control
  - Displays live AZ/EL readings
  - Absolute and Relative move commands
  - ROTCL Connection
  - Emergency Stop button
  - Reset & Homing controls
  - OTA Updates
- **rotctl protocol support** over Wi-Fi (Hamlib compatible on port 4533)
- **FreeRTOS task-based design** (separate cores for motor control, sensors, networking, etc.)

## 🛠️ Hardware Requirements
- ESP32 development board (tested on LOLIN32 D32)
- NEMA 34 stepper motors        
- DM556T stepper drivers
- Planetary gearboxes (50:1 or 72:1)
- LSM303 accelerometer + magnetometer module
- Magnetic Hall Effect endstop switches for AZ/EL
- Power supply sized for motors


## 🚀 Getting Started

### Build & Upload (PlatformIO)
1. Clone this repository:
   ```bash
   git clone https://github.com/<your-username>/ESP32_rotator.git
   cd ESP32_rotator
2. Open in VS Code + PlatformIO
3. Connect your ESP32 board.
4. Build & upload:

##Web UI
Connect to ESP32’s Wi-Fi network or your LAN.
Open the ESP32’s IP in a browser (default port 80).

📖 TODO / Roadmap
 Improve sensor filtering (smoothing on LSM303 data)
 Add configurable calibration offsets
 Enhance error handling & watchdog recovery
 Document FreeRTOS task layout and core assignments

📝 License
MIT License – feel free to use, modify, and share.
