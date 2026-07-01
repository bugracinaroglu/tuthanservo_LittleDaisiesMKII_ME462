#!/usr/bin/env bash
set -u

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="${VENV_DIR:-$HOME/python_envs/tuthanservo_environment}"

echo "=== Gripper system check ==="

if systemctl is-active --quiet mosquitto; then
    echo "[OK] Mosquitto service is active."
else
    echo "[FAIL] Mosquitto service is not active."
fi

if ss -ltn 2>/dev/null | grep -qE '(:|\])1883[[:space:]]'; then
    echo "[OK] MQTT port 1883 is listening."
else
    echo "[FAIL] MQTT port 1883 is not listening."
fi

if [ -x "$VENV_DIR/bin/python3" ]; then
    echo "[OK] Python environment exists: $VENV_DIR"

    if "$VENV_DIR/bin/python3" -c \
        "import paho.mqtt.client" >/dev/null 2>&1; then
        echo "[OK] paho-mqtt is installed."
    else
        echo "[FAIL] paho-mqtt is not installed."
    fi
else
    echo "[FAIL] Python environment is missing: $VENV_DIR"
fi

if [ -f "$PROJECT_DIR/pico/secrets.py" ]; then
    echo "[OK] pico/secrets.py exists."
else
    echo "[FAIL] pico/secrets.py is missing."
fi
