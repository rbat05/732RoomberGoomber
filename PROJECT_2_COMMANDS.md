# 732RoomberGoomber

# ODOMETRY ONLY METHOD

---

# Phase 1: SLAM Mapping (Target ≤ 2 Minutes)

## 1. Environment Setup

```bash
source ~/732RoomberGoomber/ros2_ws/install/setup.bash && \
source /opt/ros/humble/setup.bash
```

---

## 2. Robot Setup / Sanity Check

```bash
set-turtlebot 21
```

```bash
sanity
```

Wait ~20 seconds, then confirm topics are visible:

```bash
ros2 topic list | grep T10
```

---

## 3. Undock Robot

```bash
ros2 action send_goal /T10/undock irobot_create_msgs/action/Undock {}
```

Place robot on tape mark facing correct orientation.

---

## Reset odom
```bash
ros2 service call /T10/reset_pose irobot_create_msgs/srv/ResetPose {}
```

## 4. Launch SLAM

Terminal 1:

```bash
source ~/732RoomberGoomber/ros2_ws/install/setup.bash && \
source /opt/ros/humble/setup.bash && \
ros2 launch turtlebot4_navigation slam.launch.py namespace:=/T10
```

---

## 5. Launch RViz

Terminal 2:

```bash
source ~/732RoomberGoomber/ros2_ws/install/setup.bash && \
source /opt/ros/humble/setup.bash && \
ros2 launch turtlebot4_viz view_robot.launch.py namespace:=/T10
```

---

## 6. Launch Teleop

Terminal 3:

```bash
source ~/732RoomberGoomber/ros2_ws/install/setup.bash && \
source /opt/ros/humble/setup.bash && \
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/T10/cmd_vel
```

---

## 7. Mapping Procedure

- Drive full C-shape around arena
- Ensure all three cylinders appear clearly in SLAM map
- Avoid collisions
- Return robot to tape mark
- Align robot to original orientation

---

## 8. Dock Robot

```bash
ros2 action send_goal /T10/dock irobot_create_msgs/action/Dock {}
```

---

## 9. Save Map (while SLAM is still running)

Open new terminal:

```bash
source ~/732RoomberGoomber/ros2_ws/install/setup.bash && \
source /opt/ros/humble/setup.bash && \
cd ~/Desktop && \
ros2 run nav2_map_server map_saver_cli -f "lab_map" --ros-args -p map_subscribe_transient_local:=true -r __ns:=/T10
```

Creates:

- `lab_map.pgm`
- `lab_map.yaml`

---

## 10. Save Posegraph

```bash
source ~/732RoomberGoomber/ros2_ws/install/setup.bash && \
source /opt/ros/humble/setup.bash && \
ros2 service call /T10/slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$HOME/Desktop/lab_map'}"
```

---

## 11. Required Report Evidence

Take screenshots of:

- RViz map
- Generated `.pgm` map
- Robot final pose

---

# Phase 2: Autonomous Run (~8 Minutes)

---

# Pre-Run Setup

## 1. Undock Robot

```bash
ros2 action send_goal /T10/undock irobot_create_msgs/action/Undock {}
```

Place robot exactly on tape mark with correct orientation.

---

## 2. Reset Odometry

```bash
ros2 service call /T10/reset_pose irobot_create_msgs/srv/ResetPose {}
```

---

## 3. Verify Odometry Reset

```bash
ros2 topic echo /T10/odom
```

Verify:

- x ≈ 0
- y ≈ 0
- yaw ≈ 0

Press `CTRL+C` once confirmed.

---

## 4. Build Autonomous Package

```bash
cd ~/732RoomberGoomber/ros2_ws && \
colcon build --packages-select tb4_sensor_reader && \
source ~/732RoomberGoomber/ros2_ws/install/setup.bash
```

---

## 5. Launch Autonomous Node (AFTER GO SIGNAL)

```bash
source ~/732RoomberGoomber/ros2_ws/install/setup.bash && \
source /opt/ros/humble/setup.bash && \
~/ros2_venv/bin/python3 -m tb4_sensor_reader.proj2_script_simple
```

# Useful Commands

## Reset Odometry

```bash
ros2 service call /T10/reset_pose irobot_create_msgs/srv/ResetPose {}
```

---

## Manual Dock

```bash
ros2 action send_goal /T10/dock irobot_create_msgs/action/Dock {}
```

---

## Manual Undock

```bash
ros2 action send_goal /T10/undock irobot_create_msgs/action/Undock {}
```

---

## View Topics

```bash
ros2 topic list | grep T10
```

---

## Echo Odometry

```bash
ros2 topic echo /T10/odom
```

---

## Launch RViz

```bash
ros2 launch turtlebot4_viz view_robot.launch.py namespace:=/T10
```

---

## Launch SLAM

```bash
ros2 launch turtlebot4_navigation slam.launch.py namespace:=/T10
```

---

## Teleop

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/T10/cmd_vel
```


t : Toggle Teleoperation Mode (Switches manual driving mode ON or OFF). When turned on, it halts autonomous movement and listens for the driving keys below.

s : Force Search State (WALL_FOLLOWING). Resumes tracking the waypoint list extracted from your map.

c : Force Cube Scan State (CUBE_FINDING). Resets the spin parameters and immediately forces the robot to start spinning in place to look for the red cube.

r : Force Return State (RETURNING). Aborts the search and orders the robot to navigate back to the origin coordinate (0.0, 0.0).