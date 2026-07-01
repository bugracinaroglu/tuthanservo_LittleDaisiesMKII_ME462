#!/usr/bin/env python3

import json
import threading
import time
import uuid
from collections import deque

import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


VALID_COMMANDS = {
    "open",
    "close",
    "neutral",
    "stop",
    "status"
}


def create_mqtt_client(client_id):
    try:
        return mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id=client_id
        )
    except AttributeError:
        return mqtt.Client(client_id=client_id)


class GripperMQTTBridge(Node):
    def __init__(self):
        super().__init__("gripper_mqtt_bridge")

        self.declare_parameter("broker", "localhost")
        self.declare_parameter("port", 1883)
        self.declare_parameter("base_topic", "robot/gripper")

        self.broker = (
            self.get_parameter("broker")
            .get_parameter_value()
            .string_value
        )
        self.port = (
            self.get_parameter("port")
            .get_parameter_value()
            .integer_value
        )
        self.base_topic = (
            self.get_parameter("base_topic")
            .get_parameter_value()
            .string_value
            .rstrip("/")
        )

        self.command_publisher = self.create_subscription(
            String,
            "/gripper/command",
            self._on_ros_command,
            10
        )

        self.state_publisher = self.create_publisher(
            String,
            "/gripper/state",
            10
        )
        self.event_publisher = self.create_publisher(
            String,
            "/gripper/event",
            10
        )
        self.availability_publisher = self.create_publisher(
            String,
            "/gripper/availability",
            10
        )

        self.message_queue = deque()
        self.queue_lock = threading.Lock()

        client_id = (
            "ros2-gripper-bridge-"
            + uuid.uuid4().hex[:8]
        )

        self.mqtt_client = create_mqtt_client(client_id)
        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_message = self._on_mqtt_message

        self.mqtt_client.connect(
            self.broker,
            self.port,
            30
        )
        self.mqtt_client.loop_start()

        self.create_timer(0.05, self._publish_queued_messages)

        self.get_logger().info(
            "Connecting to MQTT broker "
            + self.broker
            + ":"
            + str(self.port)
        )

    def _on_mqtt_connect(
        self,
        client,
        userdata,
        flags,
        reason_code,
        properties=None
    ):
        self.get_logger().info(
            "MQTT connected: " + str(reason_code)
        )

        client.subscribe(self.base_topic + "/state")
        client.subscribe(self.base_topic + "/event")
        client.subscribe(self.base_topic + "/availability")

    def _on_mqtt_message(self, client, userdata, message):
        payload = message.payload.decode(
            errors="replace"
        )

        with self.queue_lock:
            self.message_queue.append(
                (message.topic, payload)
            )

    def _on_ros_command(self, message):
        command = message.data.strip().lower()

        if command not in VALID_COMMANDS:
            self.get_logger().warning(
                "Rejected command: " + command
            )
            return

        payload = {
            "id": "ros2-" + str(int(time.time() * 1000)),
            "target": command
        }

        self.mqtt_client.publish(
            self.base_topic + "/command",
            json.dumps(payload),
            qos=0,
            retain=False
        )

        self.get_logger().info(
            "Sent gripper command: " + command
        )

    def _publish_queued_messages(self):
        queued = []

        with self.queue_lock:
            while self.message_queue:
                queued.append(
                    self.message_queue.popleft()
                )

        for topic, payload in queued:
            message = String()
            message.data = payload

            if topic.endswith("/state"):
                self.state_publisher.publish(message)

            elif topic.endswith("/event"):
                self.event_publisher.publish(message)

            elif topic.endswith("/availability"):
                self.availability_publisher.publish(message)

    def destroy_node(self):
        try:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = GripperMQTTBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
