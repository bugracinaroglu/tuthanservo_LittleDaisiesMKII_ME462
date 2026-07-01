#!/usr/bin/env python3

import argparse
import json
import time
import uuid

import paho.mqtt.client as mqtt


PROMPT = "Command [open/close/neutral/stop/status/q]: "


def create_client(client_id):
    try:
        return mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id=client_id
        )
    except AttributeError:
        return mqtt.Client(client_id=client_id)


def print_prompt():
    print(PROMPT, end="", flush=True)


def main():
    parser = argparse.ArgumentParser(
        description="Interactive MQTT client for the Pico gripper."
    )

    parser.add_argument(
        "--broker",
        required=True,
        help="MQTT broker hostname or IP address"
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

    parser.add_argument(
        "--show-telemetry",
        action="store_true",
        help="Also print continuous telemetry messages"
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
        print()
        print("Connected to broker:", reason_code)

        # Important messages only.
        client.subscribe(args.topic + "/availability")
        client.subscribe(args.topic + "/state")
        client.subscribe(args.topic + "/event")

        # Telemetry is optional because it is continuous.
        if args.show_telemetry:
            client.subscribe(args.topic + "/telemetry")

        print_prompt()

    def on_message(client, userdata, message):
        payload = message.payload.decode(errors="replace")

        # Ignore telemetry unless explicitly requested.
        if (
            message.topic.endswith("/telemetry")
            and not args.show_telemetry
        ):
            return

        print()

        if message.topic.endswith("/availability"):
            print("[AVAILABILITY]", payload)

        else:
            try:
                data = json.loads(payload)
            except json.JSONDecodeError:
                print("[MQTT]", message.topic, payload)
                print_prompt()
                return

            if message.topic.endswith("/state"):
                print(
                    "[STATE]",
                    data.get("state"),
                    "| Left:", data.get("left_angle"),
                    "| Right:", data.get("right_angle"),
                    "| Left contact:", data.get("left_contact"),
                    "| Right contact:", data.get("right_contact"),
                    "| Reason:", data.get("reason")
                )

            elif message.topic.endswith("/event"):
                event_name = data.get("event")

                if event_name == "fsr_contact":
                    details = data.get("details", {})
                    fsr = details.get("fsr", {})

                    print(
                        "[FSR CONTACT]",
                        details.get("side"),
                        "| Angle:", details.get("stopped_angle"),
                        "| Voltage:",
                        fsr.get("average_voltage"),
                        "V"
                    )
                else:
                    print(
                        "[EVENT]",
                        event_name,
                        "| State:", data.get("state")
                    )

            else:
                print(
                    "[MQTT]",
                    message.topic,
                    json.dumps(data)
                )

        print_prompt()

    client.on_connect = on_connect
    client.on_message = on_message

    client.connect(args.broker, args.port, 30)
    client.loop_start()

    print("Command topic:", args.topic + "/command")

    try:
        while True:
            command = input(PROMPT).strip().lower()

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
