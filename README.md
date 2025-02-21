# SensorCompute

## Overview
SensorCompute is a ROS 2 package that contains two nodes: **SensorNode** and **ComputeNode**. The package simulates sensor data generation and computation using sine wave-based values. A launch file is provided to start the nodes efficiently.

## System
ROS2 Iron middleware was used to do this task and was executed on Ubuntu 22.04. The architecture is x86_64

## Nodes
### 1. SensorNode
- Generates simulated sine wave sensor data.
- Publishes the data to a ROS 2 topic.

### 2. ComputeNode
- Subscribes to sensor data from `SensorNode`.
- Computes latency and processes received data.
- Saves sensor data and computed latencies to CSV and text files.

## Dependencies
Ensure you have ROS 2 installed and sourced. This was built on ROS2 iron ,To build the package:
```sh
colcon build --packages-select sensor_compute
```
## Launch File
This launch file starts the `SensorNode` and `ComputeNode` together.

#### Usage:
```sh
ros2 launch sensor_compute launch_sensor.launch.py
```

## Visualizing Data with rqt_plot
You can visualize the sensor data in real-time using `rqt_plot`. Run the following command:
```sh
ros2 run rqt_plot rqt_plot
```
Then, select the topics:
- `/sensor1_topic/data`
- `/sensor2_topic/data`
For plotting data using matlab you can run the node and after sensor_data.csv is generated use the plot.py to get the plots

## Logging and Output
- Sensor data is saved in `sensor_data.csv`
- Computed latency values are logged in `latency_measurements.txt`
- ROS 2 logs provide additional runtime details.

## Future Enhancements
- latencies too high since 2ms is too low on linux maybe need RT_PREMPT 
- better ways to log this information or plot 
