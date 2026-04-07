# 732RoomberGoomber

### Env setup
```bash
source ~/732RoomberGoomber/ros2_ws/install/setup.bash &&
source /opt/ros/humble/setup.bash
```

### Prereq
```bash
set-turtlebot 23
sanity
ros2 action send_goal /T23/undock irobot_create_msgs/action/Undock {}
```

Check after 20 sec
```bash
ros2 topic list | grep T23
```

### Building
```bash
cd ~/732RoomberGoomber/ros2_ws &&
colcon build --packages-select tb4_sensor_reader &&
source ~/732RoomberGoomber/ros2_ws/install/setup.bash
```

### Running odometry logger

Logger
```bash
source ~/732RoomberGoomber/ros2_ws/install/setup.bash &&
source /opt/ros/humble/setup.bash &&
~/ros2_venv/bin/python3 ~/ros2_ws/src/lab_scripts/odom_logger.py --namespace /T23 --mode square --duration 60 --trial 1
```

Straight line test
```bash
source ~/732RoomberGoomber/ros2_ws/install/setup.bash &&
source /opt/ros/humble/setup.bash &&
~/ros2_venv/bin/python3 ~/732RoomberGoomber/ros2_ws/src/lab_scripts/odom_logger.py --namespace /T23 --mode linear --target 1.0 --duration 30 --trial 1
```

Square test
```bash
source ~/732RoomberGoomber/ros2_ws/install/setup.bash &&
source /opt/ros/humble/setup.bash &&
ros2 service call /T23/reset_pose irobot_create_msgs/srv/ResetPose &&
ros2 run tb4_sensor_reader test_node
```


```bash
ros2 service call /T23/reset_pose irobot_create_msgs/srv/ResetPose
```