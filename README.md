# 732RoomberGoomber

## Environment setup

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

## Building

```bash
cd ~/732RoomberGoomber/ros2_ws
colcon build --packages-select tb4_sensor_reader
source ~/732RoomberGoomber/ros2_ws/install/setup.bash
```

## Running test logger

### Terminal 1

```bash
~/ros2_venv/bin/python3 ~/732RoomberGoomber/ros2_ws/src/lab_scripts/odom_logger.py --namespace /TXX --mode linear --target 1.0 --duration 30 --trial 1
```

### Terminal 2

```bash
source ~/732RoomberGoomber/ros2_ws/install/setup.bash
ros2 run tb4_sensor_reader test_node
```
