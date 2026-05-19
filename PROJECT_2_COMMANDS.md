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
ros2 topic list | grep T21
```

---

## 3. Undock Robot

```bash
ros2 action send_goal /T21/undock irobot_create_msgs/action/Undock {}
```

Place robot on tape mark facing correct orientation.

---

## Reset odom
```bash
ros2 service call /T21/reset_pose irobot_create_msgs/srv/ResetPose {}
```

## 4. Launch SLAM

Terminal 1:

```bash
source ~/732RoomberGoomber/ros2_ws/install/setup.bash && \
source /opt/ros/humble/setup.bash && \
ros2 launch turtlebot4_navigation slam.launch.py namespace:=/T21
```

---

## 5. Launch RViz

Terminal 2:

```bash
source ~/732RoomberGoomber/ros2_ws/install/setup.bash && \
source /opt/ros/humble/setup.bash && \
ros2 launch turtlebot4_viz view_robot.launch.py namespace:=/T21
```

---

## 6. Launch Teleop

Terminal 3:

```bash
source ~/732RoomberGoomber/ros2_ws/install/setup.bash && \
source /opt/ros/humble/setup.bash && \
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/T21/cmd_vel
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
ros2 action send_goal /T21/dock irobot_create_msgs/action/Dock {}
```

---

## 9. Save Map (while SLAM is still running)

Open new terminal:

```bash
source ~/732RoomberGoomber/ros2_ws/install/setup.bash && \
source /opt/ros/humble/setup.bash && \
cd ~/Desktop && \
ros2 run nav2_map_server map_saver_cli -f "lab_map" --ros-args -p map_subscribe_transient_local:=true -r __ns:=/T21
```

Creates:

- `lab_map.pgm`
- `lab_map.yaml`

---

## 10. Save Posegraph

```bash
source ~/732RoomberGoomber/ros2_ws/install/setup.bash && \
source /opt/ros/humble/setup.bash && \
ros2 service call /T21/slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$HOME/Desktop/lab_map'}"
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
ros2 action send_goal /T21/undock irobot_create_msgs/action/Undock {}
```

Place robot exactly on tape mark with correct orientation.

---

## 2. Reset Odometry

```bash
ros2 service call /T21/reset_pose irobot_create_msgs/srv/ResetPose {}
```

---

## 3. Verify Odometry Reset

```bash
ros2 topic echo /T21/odom
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
~/ros2_venv/bin/python3 -m tb4_sensor_reader.proj2_script
```

---

# Autonomous Logic

## Load Map

Program loads:

- `lab_map.pgm`
- `lab_map.yaml`

---

## Feature Extraction

Extract cylinder coordinates from map:

- Cylinder 1 → `(x1, y1)`
- Cylinder 2 → `(x2, y2)`
- Cylinder 3 → `(x3, y3)`

Optional override:

- Manual hardcoded waypoint input

---

# State Machine

---

# SEARCHING

Robot behavior:

- Navigate to cylinder waypoints sequentially
- Use odometry only for localization
- Continuously monitor:
  - `/scan`
  - camera feed

At each waypoint:

1. Stop
2. Rotate/scan
3. Search for red cube

Obstacle avoidance:

- Reactive LiDAR avoidance around cylinders

Transitions:

- Red cube detected → `REPORTING`
- Time limit exceeded (~7 min) → `RETURNING`

---

# REPORTING

Actions:

1. Stop immediately
2. Save:
   - `cube_x`
   - `cube_y`
3. Save image:

```bash
detection_snapshot.jpg
```

4. Verify image write success:

```python
cv2.imwrite(...)
```

5. Log warning if save failed

Transition:

```text
REPORTING → RETURNING
```

---

# RETURNING

Actions:

1. Read current odometry pose
2. Compute heading to origin `(0,0)`
3. Rotate toward origin
4. Drive forward until:

```text
distance_to_origin < 0.25m
```

Transition:

```text
RETURNING → DONE
```

---

# DONE

Actions:

- Stop robot
- Hold position

Print final summary:

```text
Detected cube position
Final return position
Total runtime
```

---

# Useful Commands

## Reset Odometry

```bash
ros2 service call /T21/reset_pose irobot_create_msgs/srv/ResetPose {}
```

---

## Manual Dock

```bash
ros2 action send_goal /T21/dock irobot_create_msgs/action/Dock {}
```

---

## Manual Undock

```bash
ros2 action send_goal /T21/undock irobot_create_msgs/action/Undock {}
```

---

## View Topics

```bash
ros2 topic list | grep T21
```

---

## Echo Odometry

```bash
ros2 topic echo /T21/odom
```

---

## Launch RViz

```bash
ros2 launch turtlebot4_viz view_robot.launch.py namespace:=/T21
```

---

## Launch SLAM

```bash
ros2 launch turtlebot4_navigation slam.launch.py namespace:=/T21
```

---

## Teleop

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/T21/cmd_vel
```