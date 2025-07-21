
---

# GPR ROS 2 GUI + SEG-Y Saver

2 parts are present in the system, one that is responsible for the PyQT representation of the data, and a second one that is responsible for saving the GPR data in a .sgy file

## Table of Contents

* [Description](#description)
* [Features](#features)
* [Requirements](#requirements)
* [Installation](#installation)
* [Usage](#usage)
* [ROS Topic](#ros-topic)
* [How It Works](#how-it-works)
* [Output](#output)
* [Contributing](#contributing)

---

## Description

The GUI visualizes the drone’s 2D flight path, live LiDAR range, and the GPR B-scan image aligned to the ground.
The SEG-Y saver listens to the same GPR stream and saves each new radar trace to memory, then writes everything to a GeoLitix-compatible `.sgy` file when the node shuts down.

---

## Features

* Real-time PyQt5 GUI
* GPR image auto-shifted by altitude
* Plots flight path using PX4 odometry
* Displays LiDAR range
* Collects radar data column-by-column
* Saves GPR data in SEG-Y format on shutdown

---

## Requirements

* ROS 2 Foxy or newer
* Python 3.6+
* `rclpy`
* `cv_bridge`
* `numpy`
* `opencv-python`
* `pyqt5`
* `pyqtgraph`
* `segyio`
* PX4 flight stack (publishing odometry and distance sensor topics)

---

## Installation

```bash
# Clone the repository
git clone https://github.com/Phreekls7/GPR_Jetson.git
cd GPR_Jetson

# Install Python packages
pip install numpy opencv-python pyqt5 pyqtgraph segyio

# Build the ROS 2 package
colcon build --packages-select gpr_ros2_node
source install/setup.bash
```

---

## Usage

### Start GUI:

```bash
ros2 run gpr_ros2_node telemetry_gui.py
```

This opens the live telemetry viewer.

### Start SEG-Y saver:

```bash
ros2 run gpr_ros2_node gpr_sgy_saver.py
```

This node runs in the background and writes a `.sgy` file when stopped (Ctrl+C).

---

## ROS Topics

Both nodes subscribe to:

* `/gpr/bscan` — `sensor_msgs/Image` (grayscale B-scan image)
* `/fmu/out/vehicle_odometry` — `px4_msgs/VehicleOdometry` (drone position)
* `/fmu/out/distance_sensor` — `px4_msgs/DistanceSensor` (LiDAR)

---

## How It Works

* The GUI shows:

  * A scrolling GPR image with altitude-based alignment
  * Real-time drone path (XY) based on odometry
  * Range label updated from LiDAR

* The SEG-Y saver:

  * Extracts the latest GPR trace (rightmost column)
  * Converts it from `uint8` to `int16`
  * Appends to memory
  * On shutdown, builds a SEG-Y file with headers and writes to disk

---

## Output

When the SEG-Y saver exits, it creates a file like:

```
gpr_output_20250721_1530.sgy
```

This file is compatible with GeoLitix and other SEG-Y readers.

---

## Contributing

To contribute:

1. Fork the repo
2. Create a new branch
3. Make your changes
4. Submit a pull request

---

Let me know if you also want a combined launch file or startup script.
