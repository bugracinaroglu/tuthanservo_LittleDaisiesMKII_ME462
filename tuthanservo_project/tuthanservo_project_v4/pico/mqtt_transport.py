from time import ticks_ms, ticks_diff
from machine import unique_id

try:
    import ujson as json
except ImportError:
    import json

try:
    import ubinascii as binascii
except ImportError:
    import binascii

from umqtt.simple import MQTTClient


class MQTTTransport:
    def __init__(
        self,
        broker,
        port,
        base_topic,
        username=None,
        password=None,
        keepalive_seconds=30,
        reconnect_interval_ms=3000,
        ping_interval_ms=15000
    ):
        unique = binascii.hexlify(unique_id()).decode()

        self.client_id = "pico-gripper-" + unique
        self.broker = broker
        self.port = port
        self.username = username
        self.password = password
        self.keepalive_seconds = keepalive_seconds
        self.reconnect_interval_ms = reconnect_interval_ms
        self.ping_interval_ms = ping_interval_ms

        self.base_topic = base_topic.rstrip("/")

        self.command_topic = (
            self.base_topic + "/command"
        ).encode()

        self.state_topic = (
            self.base_topic + "/state"
        ).encode()

        self.telemetry_topic = (
            self.base_topic + "/telemetry"
        ).encode()

        self.event_topic = (
            self.base_topic + "/event"
        ).encode()

        self.availability_topic = (
            self.base_topic + "/availability"
        ).encode()

        self.client = None
        self.connected = False

        self.last_connect_attempt = None
        self.last_ping_time = ticks_ms()

        # Only the latest remote command is retained locally.
        self.pending_command = None

    def _on_message(self, topic, message):
        if topic != self.command_topic:
            return

        try:
            text = message.decode().strip()

            if text.startswith("{"):
                data = json.loads(text)
                target = data.get("target")
                command_id = data.get("id")
            else:
                target = text
                command_id = None

            if target is None:
                raise ValueError("target is missing")

            self.pending_command = {
                "target": str(target).lower(),
                "id": command_id
            }

            print(
                "MQTT command received:",
                self.pending_command
            )

        except Exception as error:
            print("Invalid MQTT command:", error)

    def _connect(self):
        self.last_connect_attempt = ticks_ms()

        print(
            "Connecting to MQTT broker:",
            self.broker,
            self.port
        )

        client = MQTTClient(
            client_id=self.client_id,
            server=self.broker,
            port=self.port,
            user=self.username,
            password=self.password,
            keepalive=self.keepalive_seconds
        )

        client.set_callback(self._on_message)

        client.set_last_will(
            self.availability_topic,
            b"offline",
            retain=True,
            qos=0
        )

        client.connect(clean_session=True)
        client.subscribe(self.command_topic, qos=0)

        self.client = client
        self.connected = True
        self.last_ping_time = ticks_ms()

        self.client.publish(
            self.availability_topic,
            b"online",
            retain=True,
            qos=0
        )

        print("MQTT connected.")
        return "connected"

    def _mark_disconnected(self):
        if self.connected:
            print("MQTT disconnected.")

        self.connected = False
        self.client = None

    def update(self, wifi_connected):
        now = ticks_ms()

        if not wifi_connected:
            self._mark_disconnected()
            return None

        if not self.connected:
            reconnect_due = (
                self.last_connect_attempt is None
                or ticks_diff(
                    now,
                    self.last_connect_attempt
                ) >= self.reconnect_interval_ms
            )

            if reconnect_due:
                try:
                    return self._connect()
                except Exception as error:
                    print("MQTT connection failed:", error)
                    self._mark_disconnected()

            return None

        try:
            # Non-blocking MQTT receive operation.
            self.client.check_msg()

            if ticks_diff(
                now,
                self.last_ping_time
            ) >= self.ping_interval_ms:
                self.client.ping()
                self.last_ping_time = now

        except Exception as error:
            print("MQTT communication error:", error)
            self._mark_disconnected()
            return "disconnected"

        return None

    def pop_command(self):
        command = self.pending_command
        self.pending_command = None
        return command

    def _publish_json(self, topic, data, retain=False):
        if not self.connected or self.client is None:
            return False

        try:
            payload = json.dumps(data).encode()

            self.client.publish(
                topic,
                payload,
                retain=retain,
                qos=0
            )

            return True

        except Exception as error:
            print("MQTT publish failed:", error)
            self._mark_disconnected()
            return False

    def publish_state(self, status):
        return self._publish_json(
            self.state_topic,
            status,
            retain=True
        )

    def publish_telemetry(self, status):
        return self._publish_json(
            self.telemetry_topic,
            status,
            retain=False
        )

    def publish_event(self, event):
        return self._publish_json(
            self.event_topic,
            event,
            retain=False
        )

    def disconnect(self):
        if self.client is None:
            return

        try:
            self.client.publish(
                self.availability_topic,
                b"offline",
                retain=True,
                qos=0
            )
            self.client.disconnect()
        except Exception:
            pass

        self._mark_disconnected()
