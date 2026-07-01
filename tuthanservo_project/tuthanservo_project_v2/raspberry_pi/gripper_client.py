#!/usr/bin/env python3

import argparse
import json
import time
import uuid

import paho.mqtt.client as mqtt


def create_client(client_id):
    try:
        return mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id=client_id
        )
    except AttributeError:
        # Compatibility with older paho-mqtt.
        return mqtt.Client(client_id=client_id)


def main():
    parser = argparse.ArgumentParser(
        description="Interactive MQTT client for the Pico gripper."
    )

    parser.add_argument(
        "--broker",
        required=True,
        help="Raspberry Pi / Mosquitto IP address"
    )

    parser.add_argument(
        "--port",
        type=int,
        default=1883
    )

    parser.add_argument(
        "--topic",
        default="robot/gripper"
    )

    args = parser.parse_args()

    client_id = "gripper-cli-" + uuid.uuid4().hex[:8]
    client = create_client(client_id)

    def on_connect(
        client,
        userdata,
        flags,
        reason_code,
        properties=None
    ):
        print("Connected to broker:", reason_code)
        client.subscribe(args.topic + "/#")

    def on_message(client, userdata, message):
        payload = message.payload.decode(
            errors="replace"
        )

        print()
        print("[MQTT]", message.topic)

        try:
            parsed = json.loads(payload)
            print(json.dumps(parsed, indent=2))
        except json.JSONDecodeError:
            print(payload)

        print("Command [open/close/neutral/stop/status/q]: ", end="", flush=True)

    client.on_connect = on_connect
    client.on_message = on_message

    client.connect(args.broker, args.port, 30)
    client.loop_start()

    print("Command topic:", args.topic + "/command")

    try:
        while True:
            command = input(
                "Command [open/close/neutral/stop/status/q]: "
            ).strip().lower()

            if command == "q":
                break

            if command not in (
                "open",
                "close",
                "neutral",
                "stop",
                "status"
            ):
                print("Invalid command.")
                continue

            payload = {
                "id": str(int(time.time() * 1000)),
                "target": command
            }

            result = client.publish(
                args.topic + "/command",
                json.dumps(payload),
                qos=0,
                retain=False
            )

            result.wait_for_publish()

    finally:
        client.loop_stop()
        client.disconnect()


if __name__ == "__main__":
    main()
