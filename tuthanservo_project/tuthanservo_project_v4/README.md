# Pico W Wi-Fi Gripper

Two servos and two FSR sensors are controlled locally by a Pico W.  
Remote commands are delivered through MQTT.

## Commands

```text
open
close
neutral
stop
status
```

During `close`, each servo stops independently when its own FSR average reaches 2.0 V.

## First-time Raspberry Pi setup

From the project directory:

```bash
chmod +x setup_host.sh run_gripper.sh check_system.sh
./setup_host.sh
```

This installs Mosquitto, enables it at boot, prepares the Python environment, and installs the host requirements.

## Pico upload

Connect the Pico by USB only when uploading code:

```bash
./upload_to_pico.sh
```

After upload, the Pico can run from its external power supply and communicate through Wi-Fi.

## Daily use

Only run:

```bash
./run_gripper.sh
```

The script:

1. Checks Mosquitto.
2. Starts it if necessary.
3. Activates `~/python_envs/tuthanservo_environment`.
4. Runs the interactive gripper client.

Optional telemetry:

```bash
./run_gripper.sh --show-telemetry
```

Optional broker override:

```bash
BROKER=bugra.local ./run_gripper.sh
```

## Quick system check

```bash
./check_system.sh
```

## Important MQTT topics

```text
robot/gripper/command
robot/gripper/state
robot/gripper/event
robot/gripper/telemetry
robot/gripper/availability
```

## ROS 2

An optional ROS 2 bridge package is included:

```text
ros2/gripper_mqtt_bridge
```

The Pico remains responsible for servo motion and FSR stopping. ROS 2 only sends high-level commands and receives state/event messages.

See:

```text
ros2/gripper_mqtt_bridge/README.md
```

## Hardware note

Do not power the servos from the Pico 3.3 V pin. Use a suitable external servo supply and connect the Pico, servo supply, and FSR grounds together.



## Normal Usage Sequence

For the initial setup:

```bash
./setup_host.sh
```

To check the system:

```bash
./check_system.sh
```

For daily use:

```bash
./run_gripper.sh
```

When you change the Pico code:

```bash
./upload_to_pico.sh
```



# To initialize directly

cd ~/tuthanservo_LittleDaisiesMKII_ME462/tuthanservo_project/tuthanservo_project_v4
source ~/python_envs/tuthanservo_environment/bin/activate
./run_gripper.sh