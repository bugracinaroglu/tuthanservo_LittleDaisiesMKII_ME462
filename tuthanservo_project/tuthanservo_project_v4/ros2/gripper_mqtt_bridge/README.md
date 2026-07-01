# ROS 2 MQTT Bridge

This package is optional. The Pico firmware does not need to change.

## Build

Copy the package into a ROS 2 workspace:

```bash
mkdir -p ~/gripper_ros2_ws/src
cp -r ros2/gripper_mqtt_bridge \
  ~/gripper_ros2_ws/src/
cd ~/gripper_ros2_ws

source /opt/ros/$ROS_DISTRO/setup.bash
python3 -m pip install paho-mqtt
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 run gripper_mqtt_bridge gripper_bridge \
  --ros-args \
  -p broker:=localhost
```

## Command

```bash
ros2 topic pub --once \
  /gripper/command \
  std_msgs/msg/String \
  "{data: 'open'}"
```

Supported values:

```text
open
close
neutral
stop
status
```

The bridge publishes raw MQTT JSON strings on:

```text
/gripper/state
/gripper/event
/gripper/availability
```
