# Agri Eye ROS2

### build
```bash
cd ~/ros2_ws/src
git clone git@github.com:AgriSwarm/agri_eye_ros2.git
cd ~/ros2_ws
colcon build --symlink-install
. install/local_setup.bash
```
### Camera demo
flash済みのesp32-camを起動
```bash
. install/local_setup.bash
ros2 run local_sensing esp32cam_server.py --ros-args -p visualize:=true
```
### Sendpose demo

```bash
ros2 run visual_odom send_pose.py
```

visualizationしたい場合はcrazyflieを起動
```bash
ros2 launch crazyflie launch.py backend:=cflib
```

### Aggregate imu&image&odom data
```bash
ros2 run visual_odom data_aggregator --ros-args -p freq:=10
```