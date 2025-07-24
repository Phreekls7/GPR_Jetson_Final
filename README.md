

---

# GPR Jetson Project

This ROS 2-based project provides tools for collecting, visualizing, and saving Ground Penetrating Radar (GPR) and drone telemetry data on NVIDIA Jetson platforms.

---

## Table of Contents

* [Overview](#overview)
* [Components](#components)
* [Requirements](#requirements)
* [Installation](#installation)
* [How to Use](#how-to-use)
* [ROS Topics](#ros-topics)
* [PX4 Setup](#px4-setup)
* [Notes](#notes)
* [Contributing](#contributing)

---

## Overview

This system connects to a COBRA GPR sensor and a PX4-based drone running ROS 2.
It includes:

* A node to stream GPR data to ROS
* A real-time GUI to visualize radar and telemetry
* A SEG-Y saver to record radar scans
* A test node to plot GPS path on a map using OpenStreetMap

---

## Components

| Name                   | Description                                     |
| ---------------------- | ----------------------------------------------- |
| `gpr_ros2_streamer.py` | Streams live GPR traces as grayscale ROS images |
| `telemetry_gui.py`     | GUI showing live B-scan, drone path, and range  |
| `gpr_sgy_saver.py`     | Saves B-scan image columns as SEG-Y traces      |
| `flight_path_post.py`  | Test script: plots GPS path on OSM basemap      |

---

## Requirements

* ROS 2 Foxy or newer
* Python 3.6+
* Jetson Linux or Ubuntu
* COBRA GPR device
* PX4-compatible drone with odometry + distance sensor

### Python dependencies:

* `rclpy`, `cv_bridge`, `px4_msgs`
* `numpy`, `opencv-python`, `pandas`, `matplotlib`
* `pyqt5`, `pyqtgraph`, `segyio`, `requests`, `Pillow`, `geographiclib`

---

## Installation

```bash
# Clone the repo
git clone https://github.com/Phreekls7/GPR_Jetson.git
cd GPR_Jetson

# Install required Python packages
pip install numpy opencv-python pandas matplotlib pyqt5 pyqtgraph segyio requests Pillow geographiclib

# Build ROS 2 workspace (if needed)
colcon build --packages-select gpr_ros2_node
source install/setup.bash
```

---

## How to Use

### 1. Stream GPR Data

```bash
ros2 run gpr_ros2_node gpr_ros2_streamer.py
```

Publishes live B-scan image from COBRA GPR to `/gpr/bscan`.

---

### 2. Launch the GUI

```bash
ros2 run gpr_ros2_node telemetry_gui.py
```

Opens the PyQt5 interface with:

* Real-time B-scan
* Flight path (XY)
* Live altitude label

---

### 3. Save Radar to SEG-Y

```bash
ros2 run gpr_ros2_node gpr_sgy_saver.py
```

Records each trace and saves a `.sgy` file on shutdown.

---

### 4. (Optional) Run GPS Flight Path Test

```bash
ros2 run gpr_ros2_node flight_path_post.py
```

Shows the drone's GPS path over a static OpenStreetMap tile.
Test/demo only — not meant for deployment.

---

## ROS Topics

| Topic                       | Type                       | Description                     |
| --------------------------- | -------------------------- | ------------------------------- |
| `/gpr/bscan`                | `sensor_msgs/Image`        | Grayscale B-scan image from GPR |
| `/fmu/out/vehicle_odometry` | `px4_msgs/VehicleOdometry` | Drone position and orientation  |
| `/fmu/out/distance_sensor`  | `px4_msgs/DistanceSensor`  | Drone range from ground (LiDAR) |

---

## PX4 Setup

To enable PX4 to publish odometry and distance sensor data to ROS 2, follow the PX4 ROS 2 user guide:
[https://docs.px4.io/main/en/ros2/user\_guide.html#setup-the-agent](https://docs.px4.io/main/en/ros2/user_guide.html#setup-the-agent)

### 1. Install ROS 2 bridge tools

Follow the PX4 documentation to set up ROS 2 integration.

### 2. Enable `DistanceSensor` in the firmware

Edit the PX4 file:

```
PX4-Autopilot/src/modules/micrortps_bridge/micrortps_bridge_topics.yaml
```

Add `DistanceSensor.msg` to the list of outgoing topics:

```yaml
outgoing:
  - DistanceSensor
  - VehicleOdometry
```

### 3. Rebuild PX4 firmware

```bash
make px4_fmu-v5x_default
```

Then flash the firmware to the drone.

### 4. Start the Micro XRCE-DDS Agent

On the Jetson or ROS 2 computer:

```bash
MicroXRCEAgent udp4 -p 8888
```

This connects PX4's RTPS bridge to ROS 2 DDS.

### 5. Verify topics

Check that these topics appear in ROS 2:

```bash
ros2 topic list
```

You should see:

* `/fmu/out/vehicle_odometry`
* `/fmu/out/distance_sensor`

---

## Notes

* Make sure the GPR is connected via Ethernet and reachable at `192.168.0.10`.
* GUI and SEG-Y saver can run at the same time.
* Test script (`flight_path_post.py`) runs separately.
* PX4 firmware must publish both odometry and distance sensor data.

---

## Contributing

To contribute:

1. Fork the repo
2. Create a branch
3. Make changes
4. Open a pull request

---


