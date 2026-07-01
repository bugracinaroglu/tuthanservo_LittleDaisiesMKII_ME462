from machine import Pin
from time import ticks_ms, ticks_diff, sleep_ms

import config
import secrets

from servo_controller import ServoController
from fsr import FSR
from gripper_controller import GripperController
from wifi_manager import WiFiManager
from mqtt_transport import MQTTTransport


# =====================================================
# Onboard LED Controller
# =====================================================

class StatusLED:
    def __init__(self, blink_duration_ms=150):
        try:
            # Pico W / Pico 2 W
            self.led = Pin("LED", Pin.OUT)
        except Exception:
            # Eski Pico kartları için fallback
            self.led = Pin(25, Pin.OUT)

        self.blink_duration_ms = blink_duration_ms

        self.wifi_connected = False
        self.blinking = False
        self.blink_start_time = ticks_ms()

        self.led.off()

    def set_wifi_connected(self, connected):
        """
        Wi-Fi bağlıysa LED normalde açık,
        bağlantı yoksa kapalı tutulur.
        """

        self.wifi_connected = connected

        if self.blinking:
            return

        if connected:
            self.led.on()
        else:
            self.led.off()

    def command_received(self):
        """
        MQTT komutu geldiğinde LED kısa süre söner.
        """

        self.blinking = True
        self.blink_start_time = ticks_ms()
        self.led.off()

    def update(self):
        """
        Bloklamayan LED blink kontrolü.
        sleep kullanılmadığı için servo ve MQTT akışı durmaz.
        """

        if not self.blinking:
            return

        now = ticks_ms()

        if ticks_diff(
            now,
            self.blink_start_time
        ) >= self.blink_duration_ms:

            self.blinking = False

            if self.wifi_connected:
                self.led.on()
            else:
                self.led.off()

    def off(self):
        self.blinking = False
        self.led.off()


# =====================================================
# Settings Validation
# =====================================================

def validate_settings():
    if secrets.MQTT_BROKER == "SET_RASPBERRY_PI_IP":
        raise ValueError(
            "pico/secrets.py içindeki MQTT_BROKER "
            "değerini Raspberry Pi IP adresi veya "
            "hostname ile değiştir."
        )


validate_settings()


# =====================================================
# Status LED
# =====================================================

status_led = StatusLED(
    blink_duration_ms=150
)


# =====================================================
# Servo Objects
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


# =====================================================
# FSR Objects
# =====================================================

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


# =====================================================
# Gripper Controller
# =====================================================

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
# Wi-Fi Manager
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


# =====================================================
# MQTT Transport
# =====================================================

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
# Main
# =====================================================

print("Pico gripper controller started.")
print(
    "Command topic:",
    config.MQTT_BASE_TOPIC + "/command"
)

last_telemetry_time = ticks_ms()


try:
    while True:

        # ---------------------------------------------
        # Wi-Fi connection
        # ---------------------------------------------

        wifi.update()

        wifi_connected = wifi.is_connected()

        status_led.set_wifi_connected(
            wifi_connected
        )


        # ---------------------------------------------
        # MQTT connection and incoming messages
        # ---------------------------------------------

        mqtt_event = mqtt.update(
            wifi_connected=wifi_connected
        )

        # MQTT yeniden bağlandığında mevcut durumu yayınla.
        if mqtt_event == "connected":
            mqtt.publish_state(
                gripper.get_status()
            )

            mqtt.publish_telemetry(
                gripper.get_status()
            )


        # ---------------------------------------------
        # Process remote command
        # ---------------------------------------------

        command = mqtt.pop_command()

        if command is not None:
            # Komut geldiğinde LED kısa süre söner.
            status_led.command_received()

            target = command["target"]
            command_id = command["id"]

            print(
                "Processing command:",
                target,
                "| ID:",
                command_id
            )

            if target == "status":
                mqtt.publish_state(
                    gripper.get_status()
                )

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

                mqtt.publish_state(
                    gripper.get_status()
                )


        # ---------------------------------------------
        # Update gripper state machine
        # ---------------------------------------------

        gripper.update()


        # ---------------------------------------------
        # Publish gripper events
        # ---------------------------------------------

        events = gripper.pop_events()

        for event in events:
            mqtt.publish_event(event)

            mqtt.publish_state(
                gripper.get_status()
            )


        # ---------------------------------------------
        # Periodic telemetry
        # ---------------------------------------------

        now = ticks_ms()

        if ticks_diff(
            now,
            last_telemetry_time
        ) >= config.TELEMETRY_INTERVAL_MS:

            last_telemetry_time = now

            mqtt.publish_telemetry(
                gripper.get_status()
            )


        # ---------------------------------------------
        # Update onboard LED
        # ---------------------------------------------

        status_led.update()


        # ---------------------------------------------
        # Small CPU delay
        # ---------------------------------------------

        sleep_ms(
            config.MAIN_LOOP_SLEEP_MS
        )


except KeyboardInterrupt:
    print("Controller stopped from USB terminal.")


finally:
    status_led.off()

    mqtt.disconnect()
    gripper.deinit()

    print("PWM signals disabled.")
    print("Status LED disabled.")