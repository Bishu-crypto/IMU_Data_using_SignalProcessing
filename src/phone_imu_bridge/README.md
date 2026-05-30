# phone_imu_bridge

ROS2 package that streams IMU data from an Android phone (Sensor Logger app)
to a ROS2 node via HTTP Push, applies Madgwick sensor fusion, and publishes
orientation as `sensor_msgs/Imu`.

## Prerequisites
- ROS2 Humble on Ubuntu 22.04
- Phone and PC on same Wi-Fi network (or phone hotspot)
- Sensor Logger app (Android) with HTTP Push enabled

## Setup
```bash
cd ~/phone_imu_ws
colcon build --symlink-install
source install/setup.bash
```

## Run
```bash
ros2 launch phone_imu_bridge imu_bridge.launch.py
```

## Phone Configuration (Sensor Logger)
- HTTP Push URL: `http://<your-pc-ip>:8000/data`
- Batch Period: 100ms
- Enable: Accelerometer + Gyroscope

## Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/phone/imu/data_raw` | `sensor_msgs/Imu` | Raw IMU from phone |
| `/phone/imu/data_fused` | `sensor_msgs/Imu` | Madgwick fused orientation |

## Author
Vaibhav (23f3000074), BS Electronic Systems, IIT Madras
