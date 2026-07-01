# Pico W MQTT Gripper

This project controls a two-servo gripper from a Raspberry Pi over Wi-Fi/MQTT.

## State behavior

- `open`: both servos move to their full configured open angles.
- `close`: each finger closes independently and stops when its own FSR average reaches 2.0 V.
- `closed`: reported only when both FSR thresholds are reached.
- `partial_contact`: one FSR reached the threshold, while the other finger reached its configured close limit.
- `close_failed`: both configured close limits were reached without FSR contact.
- `neutral`: both servos move to their own mean angles.
- `stop`: both servos hold their current angles.

## Pin and angle configuration

Configured in `pico/config.py`:

- Left servo: GP16
- Right servo: GP17
- Left FSR ADC: GP26
- Right FSR ADC: GP27
- Left mean: 85 degrees
- Right mean: 100 degrees
- Open offset: 30 degrees
- Close offset: 10 degrees
- FSR thresholds: 2.0 V

Therefore:

- Open: left 55°, right 130°
- Close limits: left 95°, right 90°
- Neutral: left 85°, right 100°

## 1. Hardware requirement

Use a Raspberry Pi Pico W or Pico 2 W. A standard Pico has no built-in Wi-Fi.

Do not power the servos from the Pico 3.3 V pin. Use a suitable external servo supply and connect:

- Servo supply GND
- Pico GND
- FSR divider GND

to a common ground.

Typical active-high FSR divider:

```text
3.3 V --- FSR ---+--- Pico ADC pin
                 |
                10 kOhm
                 |
                GND
```

## 2. Install Mosquitto on Raspberry Pi

```bash
sudo apt update
sudo apt install -y mosquitto mosquitto-clients python3-venv
```

Copy the included first-test configuration:

```bash
sudo cp mosquitto/gripper.conf /etc/mosquitto/conf.d/gripper.conf
sudo systemctl restart mosquitto
sudo systemctl enable mosquitto
```

Find the Raspberry Pi IP:

```bash
hostname -I
```

The included broker config permits anonymous connections on the local network for initial testing. Do not expose MQTT port 1883 to the internet. Add authentication/TLS before using an untrusted network.

## 3. Configure the Pico

Open:

```text
pico/secrets.py
```

The Wi-Fi credentials are already entered. Replace:

```python
MQTT_BROKER = "SET_RASPBERRY_PI_IP"
```

with the Raspberry Pi IP, for example:

```python
MQTT_BROKER = "192.168.1.50"
```

Using a DHCP reservation/static IP for the Raspberry Pi is recommended.

## 4. Create a host Python environment

From the project directory:

```bash
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install -r requirements.txt
```

On Windows PowerShell:

```powershell
python -m venv .venv
.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

## 5. Install the Pico MQTT package and upload files

Connect the Pico by USB, then run:

```bash
chmod +x upload_to_pico.sh
./upload_to_pico.sh
```

Manual equivalent:

```bash
mpremote mip install umqtt.simple

mpremote fs cp pico/config.py :
mpremote fs cp pico/secrets.py :
mpremote fs cp pico/servo_controller.py :
mpremote fs cp pico/fsr.py :
mpremote fs cp pico/gripper_controller.py :
mpremote fs cp pico/wifi_manager.py :
mpremote fs cp pico/mqtt_transport.py :
mpremote fs cp pico/main.py :

mpremote reset
```

To view Pico output:

```bash
mpremote
```

## 6. Remote-control test

Run on the Raspberry Pi or another computer on the same network:

```bash
python3 raspberry_pi/gripper_client.py --broker RASPBERRY_PI_IP
```

Example:

```bash
python3 raspberry_pi/gripper_client.py --broker 192.168.1.50
```

Then enter:

```text
open
close
neutral
stop
status
```

You can also use Mosquitto command-line tools:

```bash
mosquitto_sub -h RASPBERRY_PI_IP -t 'robot/gripper/#' -v
```

```bash
mosquitto_pub \
  -h RASPBERRY_PI_IP \
  -t robot/gripper/command \
  -m '{"id":"test-1","target":"close"}'
```

## MQTT topics

Commands:

```text
robot/gripper/command
```

Retained state:

```text
robot/gripper/state
```

Periodic sensor/angle data:

```text
robot/gripper/telemetry
```

State and FSR events:

```text
robot/gripper/event
```

Connection state:

```text
robot/gripper/availability
```

## Future ROS 2 integration

Keep the Pico firmware unchanged and add a ROS 2-to-MQTT bridge on the Raspberry Pi:

```text
ROS 2 command/action
        |
ROS 2 MQTT bridge
        |
robot/gripper/command
        |
      Pico W
```

The Pico owns real-time servo/FSR safety behavior. ROS 2 sends high-level goals (`open`, `close`, `neutral`) and receives state/telemetry. This is easier to test now and can later be replaced by micro-ROS if required.
