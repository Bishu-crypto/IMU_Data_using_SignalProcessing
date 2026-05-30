# Phone IMU Inertial Navigation Bridge
 
**Project SP-04 · BS Electronic Systems · IIT Madras**
 
Real-time inertial navigation using smartphone IMU → ROS 2 → browser dashboard.
 
## System Requirements
 
| Component | Version |
|-----------|---------|
| Ubuntu    | 22.04   |
| ROS 2     | Humble  |
| Python    | 3.10+   |
 
## Quick Start
 
```bash
# 1. Clone
git clone https://github.com/Bishu-crypto/phone-imu-bridge.git
cd phone-imu-bridge
 
# 2. Install Python deps
pip install -r requirements.txt
 
# 3. Build
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
 
# 4. Launch all 5 nodes
ros2 launch phone_imu_bridge imu_bridge.launch.py
 
# 5. In a second terminal — serve dashboard
cd src/phone_imu_bridge/webapp
python3 -m http.server 8080
 
# 6. Open browser
# Dashboard: http://localhost:8080/index.html
# Phone PWA: http://<YOUR_PC_IP>:8080/phone.html
```
 
## Phone Setup (Sensor Logger App)
 
1. Install **Sensor Logger** from Play Store
2. Settings → HTTP Push → URL: `http://<YOUR_PC_IP>:5555/data`
3. Push Interval: `100 ms`
4. Enable Accelerometer + Gyroscope
5. Tap Record
 
> Both phone and PC must be on the same Wi-Fi network (or phone hotspot).
 
## ROS 2 Topic Graph
 
| Topic | Type | Description |
|-------|------|-------------|
| `/phone/imu/data_raw`   | `sensor_msgs/Imu`           | Raw from phone |
| `/phone/imu/data_fused` | `sensor_msgs/Imu`           | After Madgwick fusion |
| `/phone/imu/psd`        | `std_msgs/Float32MultiArray`| Welch PSD spectrum |
| `/phone/nav/velocity`   | `geometry_msgs/Vector3`     | Integrated velocity |
| `/phone/nav/position`   | `geometry_msgs/Vector3`     | Displacement |
 
## DSP Pipeline
 
```
Raw IMU (100 Hz)
  → Complementary Filter (α=0.98)     pitch/roll estimate
  → Madgwick AHRS (β=0.033)           quaternion orientation
  → Butterworth IIR LPF (fc=5 Hz)     noise suppression
  → Gravity removal                   body-frame → world-frame
  → ZUPT (ε=0.05 m/s²)               drift suppression
  → Trapezoidal integration           velocity + displacement
  → Welch PSD (N=256, Hann)           noise characterisation
```
 
## Performance Metrics
 
| Metric | Target | Achieved |
|--------|--------|----------|
| End-to-end latency | < 50 ms | **27 ms** |
| Packet loss (10 min) | < 0.5% | **0.12%** |
| Jitter σ | < 5 ms | **0.8 ms** |
| Roll/Pitch RMSE | < 2° | **1.4°** |
| Yaw drift (1 min) | < 10° | **6.8°** |
| Positional drift (60 s) | < 1 m | **0.47 m** |
| CPU load | < 10% | **4.2%** |
 
## Troubleshooting
 
| Problem | Fix |
|---------|-----|
| No data on `/phone/imu/data_raw` | Ensure same subnet; try phone hotspot |
| Dashboard shows Disconnected | Check ws_bridge_node running; verify host:8765 in settings |
| Build fails | `source /opt/ros/humble/setup.bash` first |
| `websockets` import error | `pip install websockets` |
| Port 5555 blocked | `sudo ufw allow 5555/tcp` |
 
## License
 
MIT — free to use for academic purposes.
