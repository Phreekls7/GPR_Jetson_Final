````markdown
# GPR ROS 2 Streamer

A ROS 2 node that connects to a COBRA Ground Penetrating Radar (GPR) device over TCP, receives radar traces, builds a rolling B-scan image, and publishes it to a ROS topic.

---

## Table of Contents

- [Description](#description)  
- [Features](#features)  
- [Requirements](#requirements)  
- [Installation](#installation)  
- [Usage](#usage)  
- [ROS Topic](#ros-topic)  
- [How It Works](#how-it-works)  
- [Contributing](#contributing)  

---

## Description

This script connects to a COBRA GPR device at IP `192.168.0.10` and port `23`. It configures the device, reads radar traces continuously, and builds a rolling image of the scan. The image is converted to 8-bit grayscale and published as a ROS 2 `sensor_msgs/Image` on the `/gpr/bscan` topic.

---

## Features

- Connects to GPR over TCP  
- Sends device setup command  
- Receives 1024-sample traces  
- Builds a 1000-frame rolling image  
- Publishes as `mono8` image at ~30 Hz  
- Runs a background thread for I/O  
- Publishes image on `/gpr/bscan`

---

## Requirements

- ROS 2 Foxy or newer  
- Python 3.6+  
- `rclpy`  
- `cv_bridge`  
- `numpy`  
- `opencv-python`

---

## Installation

1. Clone the repository:

```bash
git clone https://github.com/Phreekls7/GPR_Jetson.git
cd GPR_Jetson
````

2. Install dependencies (if not already):

```bash
pip install numpy opencv-python
```

3. Make sure `cv_bridge` is available (usually part of your ROS 2 workspace).

4. Build the package (if used inside a ROS 2 workspace):

```bash
colcon build --packages-select gpr_ros2_node
source install/setup.bash
```

---

## Usage

Start the node:

```bash
ros2 run gpr_ros2_node gpr_ros2_streamer.py
```

Make sure the COBRA GPR is connected and powered on with IP `192.168.0.10`.

---

## ROS Topic

This node publishes:

* `/gpr/bscan` — `sensor_msgs/Image`
  A grayscale image where each column is a GPR trace. The image scrolls left with new incoming data.

---

## How It Works

* Connects to the GPR device using TCP.
* Sends a setup message based on trace quantity and range.
* Reads raw trace data from the socket.
* Each trace is placed into a 2D buffer.
* Converts that buffer into an 8-bit image.
* A timer publishes the image via ROS at \~30 Hz.

---

## Contributing

To contribute:

1. Fork the repo
2. Create a new branch
3. Make your changes
4. Submit a pull request
```
