from time import ticks_ms, ticks_diff, sleep_ms

import config
import secrets

from servo_controller import ServoController
from fsr import FSR
from gripper_controller import GripperController
from wifi_manager import WiFiManager
from mqtt_transport import MQTTTransport


def validate_settings():
    if secrets.MQTT_BROKER == "SET_RASPBERRY_PI_IP":
        raise ValueError(
            "Edit pico/secrets.py and set MQTT_BROKER "
            "to the Raspberry Pi IP address."
        )


validate_settings()


# =====================================================
# Hardware objects
# =====================================================

left_servo = ServoController(
    pin=config.LEFT_SERVO_PIN,
    initial_angle=config.LEFT_MEAN_ANGLE,
    freq=config.SERVO_FREQUENCY,
    min_us=config.SERVO_MIN_US,
    max_us=config.SERVO_MAX_US,
    max_angle=config.SERVO_MAX_ANGLE
)

right_servo = ServoController(
    pin=config.RIGHT_SERVO_PIN,
    initial_angle=config.RIGHT_MEAN_ANGLE,
    freq=config.SERVO_FREQUENCY,
    min_us=config.SERVO_MIN_US,
    max_us=config.SERVO_MAX_US,
    max_angle=config.SERVO_MAX_ANGLE
)

left_fsr = FSR(
    pin=config.LEFT_FSR_PIN,
    threshold_voltage=(
        config.LEFT_FSR_THRESHOLD_VOLTAGE
    ),
    window_size=config.FSR_WINDOW_SIZE,
    sample_interval_ms=(
        config.FSR_SAMPLE_INTERVAL_MS
    ),
    vref=config.FSR_VREF,
    active_high=config.FSR_ACTIVE_HIGH
)

right_fsr = FSR(
    pin=config.RIGHT_FSR_PIN,
    threshold_voltage=(
        config.RIGHT_FSR_THRESHOLD_VOLTAGE
    ),
    window_size=config.FSR_WINDOW_SIZE,
    sample_interval_ms=(
        config.FSR_SAMPLE_INTERVAL_MS
    ),
    vref=config.FSR_VREF,
    active_high=config.FSR_ACTIVE_HIGH
)

gripper = GripperController(
    left_servo=left_servo,
    right_servo=right_servo,
    left_fsr=left_fsr,
    right_fsr=right_fsr,

    left_mean_angle=config.LEFT_MEAN_ANGLE,
    right_mean_angle=config.RIGHT_MEAN_ANGLE,

    open_offset=config.OPEN_OFFSET,
    close_offset=config.CLOSE_OFFSET,

    initial_position=config.INITIAL_POSITION,

    step_deg=config.STEP_DEG,
    move_interval_ms=config.MOVE_INTERVAL_MS,
    close_settle_ms=config.CLOSE_SETTLE_MS
)


# =====================================================
# Network objects
# =====================================================

wifi = WiFiManager(
    ssid=secrets.WIFI_SSID,
    password=secrets.WIFI_PASSWORD,
    retry_interval_ms=(
        config.WIFI_RETRY_INTERVAL_MS
    ),
    connect_timeout_ms=(
        config.WIFI_CONNECT_TIMEOUT_MS
    )
)

mqtt = MQTTTransport(
    broker=secrets.MQTT_BROKER,
    port=secrets.MQTT_PORT,
    base_topic=config.MQTT_BASE_TOPIC,
    username=secrets.MQTT_USERNAME,
    password=secrets.MQTT_PASSWORD,
    keepalive_seconds=(
        config.MQTT_KEEPALIVE_SECONDS
    ),
    reconnect_interval_ms=(
        config.MQTT_RECONNECT_INTERVAL_MS
    ),
    ping_interval_ms=(
        config.MQTT_PING_INTERVAL_MS
    )
)


# =====================================================
# Main loop
# =====================================================

print("Pico gripper controller started.")
print("Command topic:", config.MQTT_BASE_TOPIC + "/command")

last_telemetry_time = ticks_ms()

try:
    while True:
        wifi.update()

        mqtt_event = mqtt.update(
            wifi_connected=wifi.is_connected()
        )

        # Re-publish current state after each MQTT reconnect.
        if mqtt_event == "connected":
            mqtt.publish_state(gripper.get_status())
            mqtt.publish_telemetry(
                gripper.get_status()
            )

        command = mqtt.pop_command()

        if command is not None:
            target = command["target"]
            command_id = command["id"]

            if target == "status":
                mqtt.publish_state(gripper.get_status())

            else:
                accepted = gripper.request_state(
                    target=target,
                    command_id=command_id
                )

                if not accepted:
                    mqtt.publish_event({
                        "event": "command_rejected",
                        "target": target,
                        "command_id": command_id
                    })

                mqtt.publish_state(gripper.get_status())

        gripper.update()

        events = gripper.pop_events()

        for event in events:
            mqtt.publish_event(event)
            mqtt.publish_state(gripper.get_status())

        now = ticks_ms()

        if ticks_diff(
            now,
            last_telemetry_time
        ) >= config.TELEMETRY_INTERVAL_MS:
            last_telemetry_time = now

            mqtt.publish_telemetry(
                gripper.get_status()
            )

        sleep_ms(config.MAIN_LOOP_SLEEP_MS)

except KeyboardInterrupt:
    print("Controller stopped from USB terminal.")

finally:
    mqtt.disconnect()
    gripper.deinit()
    print("PWM signals disabled.")
