
---

# GPR ROS 2 – Flight Path Map (Test Code)

A ROS 2 test node that plots the drone's GPS path in real-time on an OpenStreetMap tile using PX4 odometry data.

**Note:** This script is for testing only. It’s not fully functional or production-ready.

---

## Table of Contents

* [Description](#description)
* [Features](#features)
* [Requirements](#requirements)
* [Installation](#installation)
* [Usage](#usage)
* [ROS Topic](#ros-topic)
* [How It Works](#how-it-works)
* [Limitations](#limitations)
* [Contributing](#contributing)

---

## Description

This test script uses drone odometry data to:

* Reconstruct the flight path in real-world GPS coordinates
* Overlay the path on an OpenStreetMap tile
* Save flight data to a CSV file on shutdown

---

## Features

* Listens to `/fmu/out/vehicle_odometry`
* Converts local NED coordinates to latitude/longitude
* Fetches a background OSM tile
* Plots the flight path in real time using `matplotlib`
* Saves positions to a CSV file on exit

---

## Requirements

* ROS 2 Foxy or newer
* Python 3.6+
* `rclpy`
* `px4_msgs`
* `numpy`
* `pandas`
* `matplotlib`
* `Pillow`
* `requests`
* `geographiclib`

---

## Installation

```bash
# Clone the repository
git clone https://github.com/Phreekls7/GPR_Jetson.git
cd GPR_Jetson

# Install Python dependencies
pip install numpy pandas matplotlib Pillow requests geographiclib

# Make sure PX4 odometry is publishing to /fmu/out/vehicle_odometry
```

---

## Usage

```bash
ros2 run gpr_ros2_node flight_path_post.py
```

This will:

* Start the map viewer in a pop-up window
* Begin logging GPS and NED data
* Save a CSV file to `/tmp/flight_odometry.csv` by default

---

## ROS Topic

* `/fmu/out/vehicle_odometry` — `px4_msgs/VehicleOdometry`
  Used for position tracking and timestamp logging

---

## How It Works

* Sets a fixed takeoff GPS location (`home_lat`, `home_lon`)
* Converts incoming local (x, y) odometry to global coordinates
* Fetches one OSM tile using HTTP request
* Displays the real-time drone path over the map using `matplotlib`
* Saves all data to a CSV on shutdown

---

## Limitations

* Not tested with moving GPS origin — fixed home location is hardcoded
* Only fetches one OSM tile — map doesn't move or pan
* No error handling if network request for map fails
* CSV output is basic and unfiltered
* Only supports `POSE_FRAME_NED` from PX4

This is intended for **test/demo purposes only**.

---

## Contributing

To contribute:

1. Fork the repo
2. Create a new branch
3. Make your changes
4. Submit a pull request

---

