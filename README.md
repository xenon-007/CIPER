#🚗 Collision Identification and Prevention System with Enhanced Response

An **affordable, camera-only ADAS solution** designed for non-ADAS vehicles. This system replaces expensive and failure-prone sensors with a single camera and real-time object detection using **YOLOv8**. It identifies obstacles and automatically recommends or takes evasive actions to prevent collisions — all while being cost-effective and simple to deploy.

> ⚙️ **Economical** | 🛡️ **Increased Safety** | 🌱 **Sustainable Innovation**

## 💡 Why This Matters

Modern ADAS systems rely heavily on expensive sensors like LiDAR, radar, and ultrasonic modules — making them cost-prohibitive for most existing vehicles.

This project uses just a **USB camera**, AI-based vision, and a Raspberry Pi to offer:

- ✅ **Affordable retrofitting** for non-ADAS cars
- ✅ **Improved road safety** for drivers and passengers
- ✅ **Low energy footprint**, aligning with a **renewable future**


## 🔧 Hardware Requirements for Algorithm Testing

- Raspberry Pi (e.g., Raspberry Pi 4)
- USB Camera (acts as the only sensor)
- Motor Driver (e.g., L298N)
- DC Motors
- OLED Display (SSD1306, I2C)
- Power supply and basic wiring

> ❌ No LiDAR. No radar. No ultrasonic sensors.


## 🧠 Software Stack

- Python 3
- YOLOv8 (via Ultralytics)
- OpenCV
- RPi.GPIO
- Adafruit SSD1306 Library
- Tkinter GUI
- Pillow (PIL)


## 📦 Installation

```bash
git clone https://github.com/your-username/collision-avoidance-yolo.git
cd collision-avoidance-yolo
pip install -r requirements.txt
sudo raspi-config  # Enable I2C and camera interfaces
python main.py
```

## 🚘 System Overview

- Real-time object detection using YOLOv8
- Calculates object distance from bounding box size
- Determines movement direction from object position
- GPIO-driven motors simulate autonomous navigation
- Real-time updates on OLED display and GUI

## 🌍 Toward a Renewable & Safer Future

This project is a **step toward democratizing ADAS**:
- Makes safety features accessible for all vehicle types
- Reduces dependency on rare-earth sensor components
- Supports **low-cost, sustainable transportation innovation**

> A vision for safer roads and a cleaner planet — powered by open-source and AI.

