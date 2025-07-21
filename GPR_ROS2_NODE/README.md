
---

# GPR ROS 2 Streamer

A ROS 2 node that connects to a COBRA Ground Penetrating Radar (GPR) device over TCP, receives radar traces, builds a rolling B-scan image, and publishes it to a ROS topic.

## Table of Contents

* [Description](#description)
* [Features](#features)
* [Requirements](#requirements)
* [Installation](#installation)
* [Usage](#usage)
* [ROS Topic](#ros-topic)
* [How It Works](#how-it-works)
* [Contributing](#contributing)

---

## Description

This script connects to a COBRA GPR device at IP `192.168.0.10` and port `23`.
It configures the device, reads radar traces continuously, and builds a rolling image of the scan.
The image is converted to 8-bit grayscale and published as a ROS 2 `sensor_msgs/Image` on the `/gpr/bscan` topic.

---

## Features

* Connects to GPR over TCP
* Sends device setup command
* Receives 1024-sample traces
* Builds a 1000-frame rolling image
* Publishes as `mono8` image at \~30 Hz
* Uses background thread for I/O
* Publishes to `/gpr/bscan`

---

## Requirements

* ROS 2 Foxy or newer
* Python 3.6+
* `rclpy`
* `cv_bridge`
* `numpy`
* `opencv-python`

---

## Installation

```bash
# Clone the repository
git clone https://github.com/Phreekls7/GPR_Jetson.git
cd GPR_Jetson

# Install Python dependencies
pip install numpy opencv-python

# Make sure cv_bridge is sourced from your ROS workspace
# Then build the ROS 2 workspace (if applicable)
colcon build --packages-select gpr_ros2_node
source install/setup.bash
```

---

## Usage

```bash
# Run the streamer node
ros2 run gpr_ros2_node gpr_ros2_streamer.py
```

Make sure your COBRA GPR is powered on and connected at `192.168.0.10`.

---

## ROS Topic

This node publishes:

* `/gpr/bscan` — `sensor_msgs/Image`
  A grayscale B-scan image where each column is one radar trace.
  The image scrolls left as new data arrives.

---

## How It Works

* Opens TCP socket to COBRA GPR at `192.168.0.10:23`
* Sends setup message for 1024-sample traces with 100 ns range
* Reads and parses raw traces (skipping service channel)
* Appends each trace as a new column to a fixed-width 2D NumPy array
* Converts that array into 8-bit grayscale (`mono8`)
* Publishes images via ROS 2 at \~30 Hz using a timer callback

---

## Contributing

Pull requests are welcome.

To contribute:

1. Fork the repo
2. Create a new branch
3. Make your changes
4. Submit a pull request
