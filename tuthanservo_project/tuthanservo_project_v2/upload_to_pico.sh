#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PICO_DIR="$PROJECT_DIR/pico"

echo "Installing MicroPython MQTT dependency..."
mpremote mip install umqtt.simple

echo "Uploading Pico files..."

# Copy main.py last.
for file in \
    config.py \
    secrets.py \
    servo_controller.py \
    fsr.py \
    gripper_controller.py \
    wifi_manager.py \
    mqtt_transport.py
do
    mpremote fs cp "$PICO_DIR/$file" :
done

mpremote fs cp "$PICO_DIR/main.py" :

echo "Resetting Pico..."
mpremote reset

echo "Upload completed."
